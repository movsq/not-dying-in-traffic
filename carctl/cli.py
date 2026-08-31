from __future__ import annotations
import argparse, json, math, os, re, signal, subprocess, sys
from . import (blame as blamemod, lineage as lineagemod, loop as loopmod,
               publish as pubmod, retain as retainmod, safety,
               stash as stashmod)
from .plant import Plant
from .stash import Preconditions

# Filled in by main(), per command, from the working directory. It used to be
# derived from __file__, which answers "where is carctl installed?" -- a
# question with nothing to do with which record this invocation is about.
# After `pip install .` that answer was site-packages, and a drive crashed on
# the first git call; run from a venv living inside some other checkout it was
# that checkout, and the drive quietly wrote its frames into a repository
# nobody was looking at, which is the worse of the two by a distance. The
# README's contract is "whatever repo you run it from", and the working
# directory is the only thing that says which one that is.
REPO = ""


def _find_repo() -> str:
    """The repository this invocation is about: $CARCTL_REPO, else cwd's.

    The env var comes first because unattended units have no meaningful
    working directory -- systemd starts a service in /, cron in $HOME -- and
    "the repo you ran it from" is advice with no addressee there. The
    __file__-derived answer such a unit may once have leaned on is gone for
    the reason at the top of this module: it names where carctl is installed,
    which is not a repository, or is the wrong one. A variable a unit file can
    set is the replacement.
    """
    override = os.environ.get("CARCTL_REPO", "").strip()
    try:
        r = subprocess.run(["git", "rev-parse", "--show-toplevel"],
                           cwd=override or os.getcwd(), capture_output=True,
                           text=True, encoding="utf-8")
    except OSError as exc:
        # cwd= on a path that is not a directory fails here rather than in
        # git, and $CARCTL_REPO is the only way that path gets to be wrong.
        if override:
            sys.exit(f"carctl: $CARCTL_REPO={override!r} cannot be entered: "
                     f"{exc}")
        sys.exit(f"carctl: cannot run git: {exc}")
    if r.returncode != 0 or not r.stdout.strip():
        # One line and no traceback. Standing in the wrong directory is an
        # ordinary thing to get wrong at a shell prompt, and a stack trace
        # through subprocess says nothing an operator can act on.
        #
        # A garbled $CARCTL_REPO is loud and fatal, the same way a garbled
        # $CARCTL_WINDOW_DAYS is, and specifically does NOT fall back to cwd:
        # falling back is how a drive ends up writing its frames into a
        # repository nobody is looking at, which is the failure this whole
        # module comment is about.
        if override:
            sys.exit(f"carctl: $CARCTL_REPO={override!r} is not a git "
                     "repository")
        sys.exit("carctl: not inside a git repository (run from the repo the "
                 "record should live in)")
    return os.path.abspath(r.stdout.strip())


class GitError(RuntimeError):
    """A git command this tool needed did not succeed."""


def _run_git(*a) -> subprocess.CompletedProcess:
    # `replay` is dispatched without resolving a repository, so REPO is "" for
    # the whole of it, and cwd="" is not "no directory" to subprocess -- it is
    # a path that does not exist, or on some platforms the current one. Either
    # way the answer would be about a repository nobody chose. Nothing on the
    # replay path calls git today; this is what makes the first one that does
    # fail with a sentence instead of with cwd='' garbage or, worse, an answer.
    if not REPO:
        raise GitError("no repository resolved for this command, so there is "
                       "nothing here to ask git about")
    return subprocess.run(["git", *a], cwd=REPO, capture_output=True,
                          text=True, encoding="utf-8")


def _git(*a) -> str:
    """Run git and return its stdout, or raise.

    The exit code used to be discarded, so "failed" and "had nothing to say"
    were the same empty string. `carctl log` in a repository with no history
    printed one blank line and exited 0: a report of nothing having happened,
    manufactured out of not being able to look. Every caller here is asking
    the repository a question, and a question that could not be put is not
    answered by silence.
    """
    r = _run_git(*a)
    if r.returncode != 0:
        raise GitError(f"`git {' '.join(a)}` failed: "
                       f"{r.stderr.strip() or f'exit {r.returncode}'}")
    return r.stdout


def _rev(rev: str) -> str:
    """The sha `rev` resolves to, or "" if it does not resolve.

    The one place a non-zero exit is an answer rather than a failure, which is
    why it does not go through _git.
    """
    return _run_git("rev-parse", "--verify", "--quiet", rev).stdout.strip()


def _commit_for_seq(seq: int) -> tuple[str, str]:
    """Resolve a frame's seq to its commit in the most recent drive.

    Returns (sha, error). Indexing `git log --max-count=<seconds/0.1>`
    positionally assumed every drive was exactly as long as the flag passed to
    THIS command. Real drive lengths on this repo run 280, 170, 60, so the
    index landed inside a previous drive -- and that wrong sha was then
    blamed, and record-reverted, against a frame that had no incident. The
    Seq: trailer is already in every message; read it.

    The two ways this fails are different problems and get different words: a
    repository with no frame history at all needs a drive, while one whose
    last drive was too short needs a longer one. "Run a drive at least 111
    frames long" is unhelpful advice for a clone that has never driven.
    """
    if not _rev("refs/heads/main"):
        return "", ("refs/heads/main does not exist here, so there are no "
                    "frames to resolve a seq against; run `carctl drive` "
                    "first")
    out = _git("log", "--format=%H%x1f%B%x1e", "refs/heads/main")
    for entry in out.split("\x1e"):
        sha, _, body = entry.strip().partition("\x1f")
        m = re.search(r"^Seq: (\d+)$", body, re.M)
        if not m:
            continue
        found = int(m.group(1))
        if found == seq:
            return sha.strip(), ""
        if found < seq:
            # Walking back from the tip, seq descends within a drive. Dropping
            # below the target without matching means this drive ended before
            # that frame, so it is not the drive we are looking at.
            break
    return "", (f"seq {seq} is not in the most recent drive on "
                f"refs/heads/main; run a drive at least {seq + 1} frames long")


def _interrupt_on_sigterm(signum, frame):
    """Make SIGTERM arrive as a Ctrl-C.

    Default SIGTERM tears the interpreter down where it stands: the drain
    thread never writes `done`, fast-import exits "stream ends early", every
    frame since the last checkpoint -- up to CHECKPOINT_EVERY, so 5 s of
    driving -- is lost, and retain's drive lock is left on disk for its full
    expiry with no drive behind it. KeyboardInterrupt already has all of that
    handled. `kill` is how an operator stops a drive on a vehicle with no
    terminal attached, and it should not be the expensive way to do it.
    """
    raise KeyboardInterrupt


def cmd_drive(args):
    seen = []
    def on_frame(f, inc):
        if inc:
            seen.append((f, inc))
    # Before the drive, not before the first publish. The raw frames this
    # command is about to write are exactly what the guard protects -- 10 Hz
    # poses on refs/heads/main, the branch this repo is usually sitting on --
    # so installing it at publish time left every repo that had driven and not
    # yet published in the window where one habitual `git push` sends the lot.
    # Warnings rather than failures: a guard that could not be installed is
    # worth a line on the way past, and is never a reason to stop a vehicle
    # from recording what it is doing.
    for line in pubmod.ensure_push_safety(REPO):
        print(line)
    # Installed for the drive and restored after it. A signal handler is
    # process-global state, so leaving one behind would change how anything
    # later in this process dies. Guarded twice over: SIGTERM does not exist
    # on every platform CPython runs on, and signal() refuses to install
    # anything off the main thread.
    previous = None
    if hasattr(signal, "SIGTERM"):
        try:
            previous = signal.signal(signal.SIGTERM, _interrupt_on_sigterm)
        except (OSError, ValueError):
            previous = None
    try:
        rep = loopmod.drive(REPO, args.seconds, realtime=not args.fast,
                            on_frame=on_frame)
    except (RuntimeError, retainmod.MaintenanceError, OSError) as exc:
        # gitstore refuses outright to append frames onto a tip that is not a
        # frame history, and reports a dead fast-import the same way. The other
        # two are the same shape from further out: drive_lock resolves the git
        # dir and raises MaintenanceError when that fails, and OSError is the
        # git binary having vanished between one command and the next. All
        # three are conditions an operator fixes by running the command
        # somewhere else, or by fixing their PATH; none is served by a
        # traceback out of the middle of a drive.
        sys.exit(f"carctl drive: {exc}")
    except KeyboardInterrupt:
        # The last net. drive() records an interrupt on every path it owns and
        # returns a report, so reaching this means the signal landed outside
        # it -- between the handler going in and the drive starting, say, or
        # while this function was still printing. There is no report to print
        # by then, but there is still an exit code to get right, and it is the
        # same 130 an interrupted drive uses below.
        print("\ncarctl drive: interrupted before the drive could report; "
              "any frames it committed are on refs/heads/main")
        sys.exit(130)
    finally:
        if previous is not None:
            signal.signal(signal.SIGTERM, previous)
    print(f"ticks           {rep.ticks}")
    print(f"committed       {rep.committed}")
    print(f"dropped frames  {rep.dropped}")
    print(f"promotions      {rep.promotions}   <- commits on {lineagemod.REF}")
    if rep.realtime:
        print(f"deadline overruns {rep.overruns}")
        print(f"max jitter      {rep.max_jitter_ms:.2f} ms")
    else:
        # Not zeros. --fast schedules nothing against a deadline, so there was
        # no deadline to miss and no jitter to measure -- and "0 overruns,
        # 0.00 ms" is a clean bill of health on precisely the two numbers this
        # loop is written the way it is in order to produce. A CI run printing
        # it would look like the strongest possible evidence for the claim it
        # never tested.
        print("deadline overruns not measured (--fast)")
        print("max jitter        not measured (--fast)")
    print(f"max submit cost {rep.max_submit_us:.1f} us   <- the loop's entire git bill")
    print(f"tagged as      {rep.tag or 'UNTAGGED'}")
    kinds = {}
    for i in rep.incidents:
        kinds.setdefault(i.kind, []).append(i)
    print(f"\nincidents: {len(rep.incidents)} across {len(kinds)} kind(s)")
    for k, v in kinds.items():
        print(f"  {k:15s} first at seq {v[0].seq:4d}  {v[0].detail}")
        print(f"  {'':15s} owner: {v[0].subsystem}")
    if rep.interrupted:
        print(f"\ndrive cut short by a signal after {rep.ticks} tick(s) of a "
              f"{args.seconds:g} s drive. Everything above is true of the "
              "drive that happened, and it is tagged like any other.")
        # 128 + SIGINT, the shell's convention for "ended by a signal".
        # Exiting 0 here would tell a script that a drive it asked for ran to
        # length when it did not, and the frames it is about to read are a
        # prefix, not the drive.
        sys.exit(130)


def cmd_incident(args):
    """Full incident walk-through: find it, blame it, decide the revert."""
    # Re-run the drive deterministically to recover the frame + incident.
    plant = Plant()
    hit = None
    now = None
    # `f` outlives the loop, and with anything under 0.05 s the body never
    # runs at all, so the `now = f` below raised UnboundLocalError where "no
    # incident" was the honest answer.
    f = None
    # The previous frame, threaded through so detect() can see a range
    # shrinking. This command re-drives the scenario to explain an incident
    # that a drive reported, so it has to be handed the same inputs the drive
    # handed detect() -- one missing argument here and it explains a drive
    # nobody took, or fails to find the incident it was asked about.
    prev = None
    # round, not int, for the reason loop.py gives: int(2.9 / 0.1) is 28.
    for _ in range(round(args.seconds / 0.1)):
        # Ground truth for THIS tick, read before step() advances the clock
        # past it. Asked afterwards it answers for the next tick, and detect()
        # gets (frame_i, truth_{i+1}). loop.drive and cmd_replay sample in the
        # same order for the same reason; all three have to agree or this tool
        # ends up explaining a drive that differs from the one that ran.
        truth = plant.true_light()
        f = plant.step()
        for inc in safety.detect(f, truth, prev):
            if inc.kind == args.kind and hit is None:
                hit = (f, inc)
        prev = f
        if hit and f.seq == hit[0].seq + 8:
            now = f
            break
    else:
        now = f
    if not hit:
        # Exit 0, deliberately. A drive with no incident of this kind is a
        # good drive, and asking about one is what this command is for: the
        # question was put and answered. Only a question that could not be put
        # is a failure, which is every branch below.
        print(f"no {args.kind} in this drive"); return
    frame, inc = hit
    csha, err = _commit_for_seq(frame.seq)
    if err:
        # The incident is real and we cannot say which commit holds it, so
        # nothing after this line -- blame, the revert verdict, the record
        # revert -- can run at all. Printing the reason and exiting 0 put an
        # unanswerable question and a clean answer on the same exit code.
        sys.exit(f"carctl incident: {err}")

    print(f"INCIDENT  {inc.kind}  seq {inc.seq}  commit {csha[:10]}")
    print(f"  {inc.detail}")
    print(f"  subject: {_git('log', '-1', '--format=%s', csha).strip()}")
    print()
    print("git blame ->")
    for k, v in blamemod.attribute(REPO, csha, inc.subsystem).items():
        print(f"  {k:28s} {v}")
    print()
    verdict = safety.physical_revert(REPO, csha, now)
    print(f"physical revert ({(now.t_mono_ns-frame.t_mono_ns)/1e9:.1f}s later) ->")
    print(f"  allowed: {verdict.allowed}")
    print(f"  reason:  {verdict.reason}")
    print(f"  return path: {verdict.cost_m:.1f} m")
    print()
    wt = os.path.join(REPO, ".control")
    rr = safety.record_revert(REPO, csha, wt,
                              f"Incident: {inc.kind} ({inc.detail})")
    status = "ok" if rr.ok else "FAILED"
    print(f"record revert -> {status}  {rr.sha[:10]}  {rr.detail}")
    if rr.ok and not rr.already:
        print("  (record plane only; the physical world was not walked back)")


def cmd_park(args):
    # ParkingStash raises RuntimeError when it cannot anchor a stash -- no
    # refs/heads/main to hang it off, or update-ref refusing the name. That is
    # a repository this command cannot run in, which is one line of advice,
    # not a traceback out of the middle of a demo.
    try:
        _park(args)
    except RuntimeError as exc:
        sys.exit(f"carctl park: {exc}")


def _park(args):
    st = stashmod.ParkingStash(REPO)
    plant = Plant()
    for _ in range(125):
        f = plant.step()
    swept = st.sweep(f)
    if swept:
        print(f"swept {len(swept)} stash entr(ies) past TTL")
    pre = Preconditions(gap_length_m=6.1, lead_vehicle_x=f.pose.x + 7.0,
                        follow_vehicle_x=f.pose.x - 1.2, clearance_m=0.55)
    entry = st.push(f, pre, attempt=1)
    print(f"git stash push -> {entry.ref}  (seq {entry.seq}, "
          f"return pose {entry.return_pose['x']:.1f},{entry.return_pose['y']:.1f})")
    print(f"  betting on: gap {pre.gap_length_m} m, clearance {pre.clearance_m} m")
    print()
    for _ in range(20):
        f2 = plant.step()
    worse = Preconditions(gap_length_m=5.4, lead_vehicle_x=pre.lead_vehicle_x,
                          follow_vehicle_x=pre.follow_vehicle_x - 0.9,
                          clearance_m=0.22)
    ok, conflicts = st.pop(entry, f2, worse)
    print(f"git stash pop  -> {'applied' if ok else 'CONFLICT'}")
    for c in conflicts:
        print(f"  conflict: {c}")
    if not conflicts:
        print("  trajectory replayed onto current world state")
    else:
        print("  -> stash kept, replanning attempt 2 (the gap is not the gap "
              "it was)")
    print()
    print("stash list:")
    for e in st.list():
        print(f"  {e.ref}  attempt {e.attempt}  seq {e.seq}")
    if args.drop:
        st.drop(entry); print("dropped")


def cmd_lineage(args):
    """The ref that outlives the frames: one commit per checkpoint promotion."""
    if args.backfill or args.rebuild:
        n, msg = lineagemod.backfill(REPO, rebuild=args.rebuild)
        print(msg)
        if not n:
            return
    if not _rev(lineagemod.REF):
        # Non-zero: asked for the promotion history and there is none to show.
        # The advice is the useful half, but a script that treats exit 0 as
        # "here is the lineage" must not get one from a ref that is absent.
        sys.exit(f"carctl lineage: {lineagemod.REF} does not exist. Seed it "
                 "from the promotions already on main with:  "
                 "carctl lineage --backfill")
    n = _git("rev-list", "--count", lineagemod.REF).strip()
    print(f"{lineagemod.REF}  {n} promotion(s), kept indefinitely")
    print()
    # One Promoted trailer per line. Run together on one line, a commit that
    # moved four checkpoints at once reads as one very long checkpoint id.
    print(_git("log", f"-n{args.n}", "--format=%h  %ad  %s%n"
               "            %(trailers:key=Promoted,valueonly,"
               "separator=%x0A            )"
               "%n            during %(trailers:key=Drive,valueonly)",
               "--date=format:%Y-%m-%d %H:%M", lineagemod.REF), end="")


def cmd_maintain(args):
    """Retention and repack. Runs only while the vehicle is stopped."""
    try:
        report = retainmod.maintain(REPO, args.days, args.dry_run)
    except retainmod.MaintenanceError as exc:
        # The window is resolved inside maintain() -- flag, then
        # $CARCTL_WINDOW_DAYS, then the default -- so a garbage env value
        # surfaces here rather than at parse time. Exit non-zero: a
        # maintenance pass that did not run must not look like one that found
        # nothing to do.
        #
        # The report travels with the exception, and for a destructive
        # partial failure it is the only account of how far the prune got --
        # refs already deleted, boundary already written. Swallowing it here
        # reduced "half pruned" to a one-line "maintain: ..." that read like a
        # refusal, which is the exact confusion MaintenanceError.destructive
        # exists to prevent.
        for line in exc.report:
            print(line)
        sys.exit(f"maintain: {exc}")
    for line in report:
        print(line)


def cmd_bisect(args):
    # Numerically, via the same helper retention picks the previous drive
    # with. `for-each-ref` sorts lexically, which puts drive-0010 before
    # drive-0009 and hands bisect two endpoints in the wrong order once a
    # tenth drive exists.
    tags = [name for _, name in retainmod._drive_tags(REPO)]
    if len(tags) < 2:
        # A bisect range is a pair of drive boundaries, so one drive is not a
        # range and no script was emitted. Exit 0 said "here is your bisect"
        # to a caller that got nothing.
        sys.exit(f"carctl bisect: need at least two tagged drives to bisect "
                 f"between; this repository has {len(tags)}")
    good, bad = args.good or tags[0], args.bad or tags[-1]
    print(f"{len(tags)} tagged drives, {good} .. {bad}")
    print()
    print(blamemod.bisect_script(REPO, good, bad), end="")
    print()
    print("emitted, not run. bisecting a moving vehicle is not a thing.")
    print("the script calls `carctl`, so it needs the package installed and "
          "on PATH (pip install .); `python -m carctl` works the same way if "
          "you would rather not.")
    print("`--kind red_light_run` narrows the assert to one incident kind; "
          "the bare form works too, because the obstacles are places in the "
          "world, and a replay that stops for the light never reaches them.")


class ReplayError(Exception):
    """The checkout does not describe a frame we can replay."""


def _frame_input(directory: str, name: str):
    """One of the checked-out frame's files, as JSON.

    Read from the working directory, never from REPO. `git bisect run` runs
    its command in a checkout of the commit under test, and the frame being
    replayed is the one in that tree; REPO is wherever this package happens to
    be installed and its main is at whatever tip the last drive left.
    """
    path = os.path.join(directory, name)
    try:
        with open(path, encoding="utf-8") as fh:
            return json.load(fh)
    except (OSError, ValueError) as exc:
        raise ReplayError(f"{name}: {exc}") from exc


def cmd_replay(args):
    """Re-drive up to the checked-out frame with its checkpoint set pinned.

    The question this answers is the one bisect is asking: would this drive
    have gone wrong under the models that were loaded HERE? So the checked-out
    models.json is held in force for the whole replay rather than swapped
    mid-drive by the OTA script -- otherwise every replay ends up running the
    promoted checkpoint and every commit in the range answers the same way,
    which is a bisect with no signal in it.

    Deterministic, so the seq in state.json is enough to say how far to run:
    the plant is a fixed script and the frame at tick n is the frame at tick n.
    """
    directory = os.path.abspath(args.dir)
    try:
        state = _frame_input(directory, "state.json")
        seq = state.get("seq") if isinstance(state, dict) else None
        if not isinstance(seq, int) or isinstance(seq, bool) or seq < 0:
            raise ReplayError("state.json carries no usable seq")
        checkpoints = _frame_input(directory, "models.json")
        if (not isinstance(checkpoints, dict) or not checkpoints
                or not all(isinstance(v, str) for v in checkpoints.values())):
            raise ReplayError("models.json is not a checkpoint set")
    except ReplayError as exc:
        print(f"replay: {exc}", file=sys.stderr)
        print(f"replay: reads a frame checkout, and {directory} is not one. "
              "git bisect leaves one in the working directory; --dir points "
              "somewhere else.", file=sys.stderr)
        # Not 125. To `git bisect run` that means "skip this commit", and a
        # frame we cannot read is a broken invocation, not an untestable
        # commit: skipping every commit in the range would end in a confident
        # bisect result drawn from nothing.
        sys.exit(2)

    plant = Plant(checkpoints)
    found = []
    # Same reason as cmd_incident: detect() judges a frame against the one
    # before it, and a bisect step that fed it None every tick would be
    # answering with a weaker detector than the drive used -- every commit in
    # the range coming back clean is a bisect result, and a wrong one.
    prev = None
    for _ in range(seq + 1):
        # Before the step, matching loop.drive and cmd_incident: step() ends
        # by advancing plant.t, so the oracle asked afterwards describes the
        # next tick. A bisect that judged frames against a world one tick
        # ahead of them would be answering a question nobody asked.
        truth = plant.true_light()
        f = plant.step()
        # Nothing past the frame under test counts. The loop bound already
        # says that, and the filter is what keeps it true of the answer rather
        # than of the loop: a bisect step must judge the commit it was handed,
        # not the crash that came four seconds after it.
        found += [i for i in safety.detect(f, truth, prev)
                  if i.seq <= seq and args.kind in ("", i.kind)]
        prev = f
    what = args.kind or "incident"

    print(f"replaying seq 0..{seq} with the checked-out checkpoint set held "
          "in force:")
    for k, v in sorted(checkpoints.items()):
        print(f"  {k:12s} {v}")
    if not found:
        print(f"clean: no {what} through seq {seq}")
        return
    kinds = {}
    for i in found:
        kinds.setdefault(i.kind, []).append(i)
    across = "" if args.kind else f" across {len(kinds)} kind(s)"
    print(f"{len(found)} {what} tick(s){across}:")
    for k, v in kinds.items():
        print(f"  {k:15s} first at seq {v[0].seq:4d}  {v[0].detail}")
        print(f"  {'':15s} owner: {v[0].subsystem}")
    if args.assert_no_incident:
        # 1, not 2 and not 125: `git bisect run` reads 1..124 as "bad", 125 as
        # "skip", and anything above 127 aborts the bisect outright.
        sys.exit(1)


def cmd_publish(args):
    try:
        sha = pubmod.build_public_ref(REPO)
    except (pubmod.Unscrubbable, pubmod.AuditError,
            subprocess.CalledProcessError, RuntimeError) as exc:
        # A scrubber that could not scrub is the one failure this command
        # must never round off: the ref it would have built is the thing
        # about to leave the vehicle.
        sys.exit(f"carctl publish: {exc}")
    n = _git("rev-list", "--count", "refs/heads/public").strip()
    print(f"refs/heads/public  {sha[:10]}  {n} commits")
    print("  poses snapped to 25 m, commit times rounded to the minute")
    before = _git("show", "refs/heads/main:state.json").splitlines()
    after = _git("show", "refs/heads/public:state.json").splitlines()
    print("")
    print("  main   " + " ".join(l.strip() for l in before if '"x"' in l or '"y"' in l))
    print("  public " + " ".join(l.strip() for l in after if '"x"' in l or '"y"' in l))
    print("")
    # The gate runs whether or not we are pushing. --push decides where the
    # ref goes, not whether anyone checked it: the bare form prints a sha, a
    # commit count and a scrubbed pose, which reads as a clean result, and
    # then said nothing at all about the audit. Someone reading that before
    # copying the ref somewhere by hand has been shown every reassuring thing
    # except the one that was actually checked. push() audits again before it
    # pushes -- this does not stand in for that, it stops the plain form from
    # being silent about it.
    try:
        problems = pubmod.audit(REPO)
    except pubmod.AuditError as exc:
        sys.exit(f"carctl publish: the audit could not complete, so nothing "
                 f"about this ref has been established: {exc}")
    if problems:
        for problem in problems:
            print(f"audit: {problem}")
        sys.exit("carctl publish: audit failed; refs/heads/public is not "
                 "publishable and was not pushed")
    print("audit: clean")
    # Lineage goes out with every push, so the plain form checks it too --
    # same reasoning as above, applied to the ref whose "publishable by
    # construction" story is the one that already failed once. Only when the
    # ref exists: a repo that has never driven has nothing to check, and
    # push() skips the refspec for it the same way.
    if _rev(lineagemod.REF):
        try:
            lineage_problems = pubmod.audit_lineage(REPO)
        except pubmod.AuditError as exc:
            sys.exit(f"carctl publish: the lineage audit could not complete, "
                     f"so nothing about that ref has been established: {exc}")
        if lineage_problems:
            for problem in lineage_problems:
                print(f"audit: {problem}")
            sys.exit("carctl publish: lineage audit failed; `carctl lineage "
                     "--rebuild` rewrites the ref from main with the public "
                     "identity")
        print("audit: lineage clean")
    if not args.push:
        print("not pushed. to publish:  carctl publish --push")
        return
    # audited=True: this command has just run audit() and audit_lineage()
    # above, on the same refs, in the same process, with nothing in between
    # that could move them. push() re-auditing would be a second full pass
    # over every blob on both refs for no new information. The flag says "a
    # gate was passed", not "skip the gate" -- push() still audits by default,
    # because a library caller has not necessarily run one.
    ok, msg = pubmod.push(REPO, audited=True)
    if not ok:
        # Non-zero. A push that did not happen is the one outcome of this
        # command a caller most needs to be able to detect without reading
        # the words.
        sys.exit(f"carctl publish: push failed, nothing left the vehicle: "
                 f"{msg}")
    print("pushed: " + msg)


def cmd_log(args):
    print(_git("log", f"-n{args.n}", "--format=%h  %ad  %s",
               "--date=format:%H:%M:%S"))


def _seconds(text: str) -> float:
    """A drive length argparse will accept.

    `type=float` took nan and inf, which float() parses without complaint, and
    took 0 and -5. Each of those reached the loop and failed there or, worse,
    did not: round(nan / DT) raises ValueError out of the middle of drive(),
    inf raises OverflowError, and a non-positive length produced a drive of no
    ticks that went on to tag the previous drive's tip as though it were its
    own. All four are the same slip at a shell prompt, and argparse's own
    error is where a bad flag value belongs -- before the drive lock is taken
    and before fast-import is started.
    """
    try:
        value = float(text)
    except ValueError:
        raise argparse.ArgumentTypeError(f"{text!r} is not a number")
    if not math.isfinite(value):
        raise argparse.ArgumentTypeError(
            f"{text!r} is not a finite number of seconds")
    if value <= 0:
        raise argparse.ArgumentTypeError(
            f"{value:g} is not a drive length; give a positive number of "
            "seconds")
    return value


def main(argv=None):
    # Street names are UTF-8. A cp1252 console would otherwise raise
    # UnicodeEncodeError mid-report.
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except (AttributeError, ValueError):
            pass
    p = argparse.ArgumentParser(prog="carctl")
    sub = p.add_subparsers(dest="cmd", required=True)

    d = sub.add_parser("drive")
    # 17, not 14. The scripted stop is at t=14.0, so a 14 s drive ends with
    # the car still rolling at ~9 km/h and retain.stationary(), which reads
    # the last committed frame's speed, can never pass after a default drive.
    # By 17 s it has settled to ~0.4 km/h.
    d.add_argument("--seconds", type=_seconds, default=17.0,
                   help="drive length; the default runs past the scripted "
                        "stop so the vehicle is stationary at the end, which "
                        "is what `carctl maintain` requires")
    d.add_argument("--fast", action="store_true",
                   help="skip real-time pacing (CI / replay)")
    d.set_defaults(func=cmd_drive)

    i = sub.add_parser("incident")
    # Validated against the kinds that exist, rather than taken as free text.
    # A typo matched nothing, and this command's honest answer for "nothing
    # matched" is `no <kind> in this drive`, exit 0 -- so `--kind
    # red_light_runn` reported a clean drive, confidently, on the tool whose
    # entire job is finding the thing that went wrong. Being wrong quietly is
    # bad enough anywhere; here it is the failure mode the command exists to
    # rule out.
    i.add_argument("--kind", default="red_light_run",
                   choices=sorted(safety.OWNER))
    i.add_argument("--seconds", type=_seconds, default=14.0)
    i.set_defaults(func=cmd_incident)

    k = sub.add_parser("park"); k.add_argument("--drop", action="store_true")
    k.set_defaults(func=cmd_park)

    b = sub.add_parser("bisect")
    b.add_argument("--good"); b.add_argument("--bad")
    b.set_defaults(func=cmd_bisect)

    rp = sub.add_parser(
        "replay",
        help="re-drive up to the checked-out frame with its checkpoint set "
             "pinned; the test `carctl bisect` emits")
    rp.add_argument("--assert-no-incident", action="store_true",
                    help="exit 1 if the replay hits one, 0 if it is clean; "
                         "what `git bisect run` reads")
    # Same validation as `incident --kind` and for the same reason, with one
    # more consumer: an unmatched kind reports every frame clean and hands
    # `git bisect run` a confident answer built out of a misspelling, which it
    # then repeats across the whole range.
    rp.add_argument("--kind", default="", choices=sorted(safety.OWNER),
                    help="judge this incident kind only; red_light_run is "
                         "the one the checkpoint set decides, while collision "
                         "and curb_strike are scripted into the world and "
                         "fire either way")
    rp.add_argument("--dir", default=".",
                    help="the frame checkout to read state.json and "
                         "models.json from (default: the working directory, "
                         "which is where git bisect leaves them)")
    rp.set_defaults(func=cmd_replay)

    ln = sub.add_parser("lineage")
    ln.add_argument("-n", type=int, default=20)
    ln.add_argument("--backfill", action="store_true",
                    help="seed an empty ref from the promotions already on main")
    ln.add_argument("--rebuild", action="store_true",
                    help="replace an existing ref from main, keeping the old "
                         "tip under refs/backup/")
    ln.set_defaults(func=cmd_lineage)

    m = sub.add_parser("maintain")
    # Defaulted in retain.maintain(), not here, because the fallback is a
    # chain rather than a value: the flag wins, then $CARCTL_WINDOW_DAYS, then
    # MAIN_WINDOW_DAYS. Filling the constant in here would make every
    # invocation look like an explicit --days and the env var would never be
    # consulted.
    m.add_argument("--days", type=float, default=None,
                   help="how much full-frame history main keeps, in days; "
                        f"default $CARCTL_WINDOW_DAYS, else "
                        f"{retainmod.MAIN_WINDOW_DAYS:g}")
    m.add_argument("--dry-run", action="store_true",
                   help="say what would be dropped, change nothing")
    m.set_defaults(func=cmd_maintain)

    pb = sub.add_parser("publish")
    pb.add_argument("--push", action="store_true",
                    help="actually push refs/heads/public to origin")
    pb.set_defaults(func=cmd_publish)

    l = sub.add_parser("log"); l.add_argument("-n", type=int, default=20)
    l.set_defaults(func=cmd_log)

    a = p.parse_args(argv)

    global REPO
    # Resolved per command, after parsing and before dispatch, so a bad flag
    # is reported by argparse rather than by a repository lookup the command
    # was never going to survive anyway.
    #
    # `replay` is the exception and has to stay one: `git bisect run` invokes
    # it inside a checkout that git made, wherever it pleased, and it reads
    # its whole input out of --dir. Requiring a repository would make it fail
    # in exactly the situation it was written for, and the failure would be
    # read as "bad commit".
    if a.cmd != "replay":
        REPO = _find_repo()
    try:
        a.func(a)
    except GitError as exc:
        # One line, non-zero. Every _git call is this tool asking the
        # repository a question; a question that could not be put has no
        # answer, and the exit code has to say so.
        sys.exit(f"carctl {a.cmd}: {exc}")


if __name__ == "__main__":
    main()
