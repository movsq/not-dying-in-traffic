from __future__ import annotations
import argparse, json, os, re, subprocess, sys
from . import (blame as blamemod, lineage as lineagemod, loop as loopmod,
               publish as pubmod, retain as retainmod, safety,
               stash as stashmod)
from .plant import Plant
from .stash import Preconditions

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _git(*a):
    return subprocess.run(["git", *a], cwd=REPO, capture_output=True, text=True, encoding="utf-8").stdout


def _commit_for_seq(seq: int) -> tuple[str, str]:
    """Resolve a frame's seq to its commit in the most recent drive.

    Returns (sha, error). Indexing `git log --max-count=<seconds/0.1>`
    positionally assumed every drive was exactly as long as the flag passed to
    THIS command. Real drive lengths on this repo run 280, 140, 60, 30, so the
    index landed inside a previous drive -- and that wrong sha was then
    blamed, and record-reverted, against a frame that had no incident. The
    Seq: trailer is already in every message; read it.
    """
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


def cmd_drive(args):
    seen = []
    def on_frame(f, inc):
        if inc:
            seen.append((f, inc))
    rep = loopmod.drive(REPO, args.seconds, realtime=not args.fast,
                        on_frame=on_frame)
    print(f"ticks           {rep.ticks}")
    print(f"committed       {rep.committed}")
    print(f"dropped frames  {rep.dropped}")
    print(f"promotions      {rep.promotions}   <- commits on {lineagemod.REF}")
    print(f"deadline overruns {rep.overruns}")
    print(f"max jitter      {rep.max_jitter_ms:.2f} ms")
    print(f"max submit cost {rep.max_submit_us:.1f} us   <- the loop's entire git bill")
    print(f"tagged as      {rep.tag or 'UNTAGGED'}")
    kinds = {}
    for i in rep.incidents:
        kinds.setdefault(i.kind, []).append(i)
    print(f"\nincidents: {len(rep.incidents)} across {len(kinds)} kind(s)")
    for k, v in kinds.items():
        print(f"  {k:15s} first at seq {v[0].seq:4d}  {v[0].detail}")
        print(f"  {'':15s} owner: {v[0].subsystem}")


def cmd_incident(args):
    """Full incident walk-through: find it, blame it, decide the revert."""
    # Re-run the drive deterministically to recover the frame + incident.
    plant = Plant()
    hit = None
    now = None
    # `f` outlives the loop, and with --seconds 0 (anything under 0.05) the
    # body never runs at all, so the `now = f` below raised UnboundLocalError
    # where "no incident" was the honest answer.
    f = None
    # round, not int, for the reason loop.py gives: int(2.9 / 0.1) is 28.
    for _ in range(round(args.seconds / 0.1)):
        f = plant.step()
        for inc in safety.detect(f, plant.true_light()):
            if inc.kind == args.kind and hit is None:
                hit = (f, inc)
        if hit and f.seq == hit[0].seq + 8:
            now = f
            break
    else:
        now = f
    if not hit:
        print(f"no {args.kind} in this drive"); return
    frame, inc = hit
    csha, err = _commit_for_seq(frame.seq)
    if err:
        print(err); return

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
    if _git("rev-parse", "--verify", "--quiet", lineagemod.REF).strip() == "":
        print(f"{lineagemod.REF} does not exist. Seed it from the promotions "
              f"already on main with:  carctl lineage --backfill")
        return
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
        print("need at least two tagged drives"); return
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
    for _ in range(seq + 1):
        f = plant.step()
        # Nothing past the frame under test counts. The loop bound already
        # says that, and the filter is what keeps it true of the answer rather
        # than of the loop: a bisect step must judge the commit it was handed,
        # not the crash that came four seconds after it.
        found += [i for i in safety.detect(f, plant.true_light())
                  if i.seq <= seq and args.kind in ("", i.kind)]
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
    sha = pubmod.build_public_ref(REPO)
    n = _git("rev-list", "--count", "refs/heads/public").strip()
    print(f"refs/heads/public  {sha[:10]}  {n} commits")
    print("  poses snapped to 25 m, commit times rounded to the minute")
    before = _git("show", "refs/heads/main:state.json").splitlines()
    after = _git("show", "refs/heads/public:state.json").splitlines()
    print("")
    print("  main   " + " ".join(l.strip() for l in before if '"x"' in l or '"y"' in l))
    print("  public " + " ".join(l.strip() for l in after if '"x"' in l or '"y"' in l))
    if not args.push:
        print("")
        print("not pushed. to publish:  carctl publish --push")
        return
    ok, msg = pubmod.push(REPO)
    print(("pushed" if ok else "push failed") + ": " + msg)


def cmd_log(args):
    print(_git("log", f"-n{args.n}", "--format=%h  %ad  %s",
               "--date=format:%H:%M:%S"))


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
    d.add_argument("--seconds", type=float, default=17.0,
                   help="drive length; the default runs past the scripted "
                        "stop so the vehicle is stationary at the end, which "
                        "is what `carctl maintain` requires")
    d.add_argument("--fast", action="store_true",
                   help="skip real-time pacing (CI / replay)")
    d.set_defaults(func=cmd_drive)

    i = sub.add_parser("incident")
    i.add_argument("--kind", default="red_light_run")
    i.add_argument("--seconds", type=float, default=14.0)
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
    # Validated against the kinds that exist, rather than taken as free text.
    # A typo would otherwise match nothing, report every frame clean, and hand
    # `git bisect run` a confident answer built out of a misspelling.
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
    a.func(a)


if __name__ == "__main__":
    main()
