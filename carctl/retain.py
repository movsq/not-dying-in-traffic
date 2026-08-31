"""Retention and repack: what stops being kept, and when we may spend disk.

At 10 Hz the car commits 864,000 times a day. Across the 1020 commits already
on main a frame costs 255.1 bytes packed, so a drive-day is about 220 MB and a
year about 80 GB. Treat that as a floor: these are short scripted drives whose
poses delta extremely well, and it is measured after a repack.

Blame has to reach back to the promotion of the oldest checkpoint still in
service, which today is ckpt-controller-2026.03.01-0b12, about six months old.
Eighteen months of full frames to satisfy that would be ~121 GB, so retention
splits by file rather than by time:

  * Full frames on main: 14 days. Enough for incident forensics plus upload
    lag, ~3.1 GB at the floor, budget 10 GB for real delta ratios.
  * models.json: kept indefinitely on refs/heads/lineage, which commits that
    one file, one commit per promotion. See lineage.py.

The window is a forensics judgement, not a derived number: it is how long
after an incident somebody might still want 10 Hz poses on the vehicle. The
lineage half is derived and firm, and it is the half that makes the window
safe to shorten. It is the single knob here, settable per run with --days or
per environment with CARCTL_WINDOW_DAYS; every pass reports which of the three
it used, because the window is what decides that frames stop existing.

Pruning is done with a shallow boundary rather than a rewrite, because a
rewrite changes every surviving commit's sha, and a sha is a frame's identity
here: refs/reverts/<sha> is what anchors an incident to the frame it happened
on, and every incident report ever printed names one. Rewriting main daily
would invalidate yesterday's incident report. A graft was the other candidate
and does not work: git disables replace refs while packing, on purpose, so the
commit objects survive, and commits are over half the pack.

The price is the commit-graph. Git declines to write one in a shallow
repository, because the parent of the boundary commit is not there to record,
so once main has been pruned there is no commit-graph for anything. That is a
cache, not the record, and it is the right way round: blame is the operation
that has to reach furthest back and it now runs on the lineage ref, which is
small and does not need one. If walking main ever becomes the binding cost,
the answer is a shorter window, not a rewritten record.

Repacking is the other half. fast-import writes one pack per session and
nothing collapses them, and lookup cost grows with the pack count, so packs
bite before the disk does. Both halves run only while the vehicle is
stationary. A repack competing with the committer for disk is exactly the
stalled-disk scenario the bounded queue drops frames on, so repacking during a
drive would manufacture the failure the architecture exists to survive.
"""
from __future__ import annotations
import contextlib, json, math, os, stat, subprocess, time, uuid
from . import lineage, publish

MAIN_WINDOW_DAYS = 14
# The one knob, and where it can be turned from without passing --days.
WINDOW_ENV = "CARCTL_WINDOW_DAYS"
# km/h. Below this the wheels are not turning and maintenance may run.
STATIONARY_KMH = 1.0
LOCK_NAME = "carctl-drive.lock"
# Grace on the declared end of a drive before its lock stops counting. A drive
# knows its own length up front, so the lock can expire by itself: a power cut
# mid-drive must not leave a file behind that blocks maintenance forever on a
# vehicle with nobody in it.
LOCK_GRACE_S = 60.0
# Ceiling on how far ahead a lock may claim the disk, whatever it declares. A
# typo'd `--seconds 1e9` plus a crash otherwise blocks every maintenance pass
# until 2058, on a vehicle nobody is sitting in, and the record plane must not
# be able to lock itself out of its own housekeeping on one bad digit. Four
# hours is longer than any drive this thing does; a genuinely longer one is
# still covered, because stationary() reads the last frame's speed as a second
# and independent signal and a moving car fails that gate on its own.
LOCK_MAX_S = 4 * 3600.0
# `git repack --geometric` landed in 2.32. Everything else here works on the
# git that ships with a 2020 distribution, so this is the one version gate,
# and it degrades rather than refusing.
MIN_GEOMETRIC_GIT = (2, 32)


class MaintenanceError(Exception):
    """A step could not be completed. Never reported as "nothing to do": a
    gate that answers the same way when it passes and when it never ran is
    worse than no gate.

    `report` carries whatever the failing pass had already accumulated, and
    `destructive` says it had already changed the repository when it failed.
    An exception that arrives with neither leaves the caller unable to tell a
    refusal from a half-finished prune, and those need opposite responses.
    """

    def __init__(self, msg: str, report: list[str] | None = None,
                 destructive: bool = False):
        super().__init__(msg)
        self.report = list(report or [])
        self.destructive = destructive


def _git(repo: str, *args: str) -> subprocess.CompletedProcess:
    return subprocess.run(["git", *args], cwd=repo, capture_output=True,
                          text=True, encoding="utf-8")


def _out(repo: str, *args: str) -> str:
    r = _git(repo, *args)
    if r.returncode != 0:
        raise MaintenanceError(f"`git {' '.join(args)}` failed: "
                               f"{r.stderr.strip()}")
    return r.stdout


# ---------------------------------------------------------------------------
# The drive interlock.

def lock_path(repo: str) -> str:
    return os.path.join(_out(repo, "rev-parse", "--absolute-git-dir").strip(),
                        LOCK_NAME)


@contextlib.contextmanager
def drive_lock(repo: str, seconds: float):
    """Advertise a drive in progress, for the length it says it will take.

    Advisory in one direction only. It tells maintenance to stay off the disk;
    it never stops a drive from starting. The record plane must not be able to
    stand between an operator and the vehicle moving, and a lock left behind
    by a crash would do exactly that.
    """
    path = lock_path(repo)
    # `seconds` is what the drive declared and stays in the file as written --
    # drive_in_progress reports it and old locks carry it -- but the horizon it
    # buys is capped. The two numbers disagreeing is the honest reading: this
    # is what you asked for, this is how long we will hold the disk for it.
    token = uuid.uuid4().hex
    body = json.dumps({"pid": os.getpid(),
                       "until": time.time() + min(seconds, LOCK_MAX_S)
                                + LOCK_GRACE_S,
                       "seconds": seconds,
                       # Additive. drive_in_progress reads pid/until/seconds
                       # and never looks here, so a lock written by an older
                       # build still reads and still blocks correctly.
                       "token": token})
    try:
        with open(path, "w", encoding="utf-8", newline="\n") as fh:
            fh.write(body + "\n")
    except OSError:
        # A drive that cannot write its lock still drives. Maintenance is
        # gated on the vehicle being stopped as well, so this degrades to one
        # gate rather than none.
        path = ""
    try:
        yield
    finally:
        if path:
            _release_lock(path, token)


def _release_lock(path: str, token: str) -> None:
    """Remove the lock only if it is still ours.

    Two drives overlapping both write this path, and the second one wins. The
    unconditional remove that used to be here then let whichever drive
    finished FIRST delete the lock the still-running one is protected by, so
    maintenance was free to start a full repack under a moving vehicle -- the
    exact stalled-disk scenario the bounded queue drops frames on.

    Read-compare-delete, which is not atomic, and does not need to be: the
    only way to lose the race is for the winner's own release to run between
    our read and our unlink, and it deletes the same file we were about to.
    Every failure mode is a lock left behind, and a lock left behind expires
    on its own `until`.
    """
    with contextlib.suppress(OSError, ValueError):
        with open(path, encoding="utf-8") as fh:
            if json.load(fh).get("token") != token:
                return      # somebody else's drive owns this file now
        os.remove(path)


def drive_in_progress(repo: str) -> str:
    """A reason to stay off the disk, or "" if there is none."""
    path = lock_path(repo)
    try:
        with open(path, encoding="utf-8") as fh:
            lock = json.load(fh)
    except (OSError, ValueError):
        return ""
    left = float(lock.get("until", 0)) - time.time()
    if left <= 0:
        return ""     # expired by itself; a crashed drive does not block us
    return (f"a drive is in progress (pid {lock.get('pid', '?')}, "
            f"{left:.0f}s left on its declared {lock.get('seconds', '?')}s)")


def stationary(repo: str) -> tuple[bool, str]:
    """May we do disk-heavy work right now?

    Two independent signals, because either alone is weak. The lock says a
    drive is running; the last committed frame says whether the vehicle was
    moving when the record last saw it. A frame reporting 32 km/h means the
    car is either still moving or the record is stale, and neither is a green
    light for taking the disk away from the committer.
    """
    why = drive_in_progress(repo)
    if why:
        return False, why
    r = _git(repo, "show", "refs/heads/main:state.json")
    if r.returncode != 0:
        return True, "no frames on refs/heads/main yet"
    try:
        kmh = json.loads(r.stdout)["pose"]["v"] * 3.6
    except (ValueError, KeyError, TypeError) as exc:
        return False, f"cannot read the last frame's speed: {exc}"
    if kmh >= STATIONARY_KMH:
        return False, (f"the last frame on main reports {kmh:.1f} km/h; the "
                       "vehicle is moving, or the record is stale")
    return True, f"stopped, last frame reports {kmh:.1f} km/h"


# ---------------------------------------------------------------------------
# Where the window falls.

def resolve_window_days(days: float | None) -> tuple[float, str]:
    """The retention window in days, and where the number came from.

    Three sources in descending order: --days, the environment, the built-in
    default. The source is returned because the window decides what is
    destroyed, and a report that says "14 days" without saying who chose 14
    cannot be audited after the frames are gone.

    A malformed environment value is an error, not a fallback. Silently
    reverting to 14 would be the one outcome nobody notices, and the failure it
    hides -- a window shortened or garbled by ambient config -- destroys frames
    that were meant to be kept.

    The flag is checked too, and used not to be: argparse's `type=float` takes
    `nan` and `-1e9` as perfectly good floats, so `--days -1e9` set a horizon
    before the epoch and `--days nan` made every comparison below false and
    tracebacked somewhere further in. A number that decides what is destroyed
    does not get to arrive unchecked from either source.

    The two sources are checked to different rules, deliberately. Zero is
    rejected from the environment and accepted from the flag: a standing
    CARCTL_WINDOW_DAYS=0 is a deployment-wide instruction to wipe every drive
    on every pass, which is the kind of thing that gets set once and forgotten,
    while `--days 0` is an operator typing a one-shot decision at a prompt --
    and window_cut holds it back to the start of the most recent drive anyway.
    """
    if days is not None:
        if not math.isfinite(days) or days < 0:
            raise MaintenanceError(
                f"--days {days:g} is not a usable retention window; it must "
                "be a finite number of days, zero or more. A window decides "
                "which frames are destroyed, so a garbled one is refused here "
                "rather than turned into a horizon nothing survives")
        return days, "--days"
    raw = os.environ.get(WINDOW_ENV)
    if raw is None or not raw.strip():
        return MAIN_WINDOW_DAYS, "MAIN_WINDOW_DAYS default"
    try:
        value = float(raw)
    except ValueError:
        value = float("nan")
    if not math.isfinite(value) or value <= 0:
        raise MaintenanceError(
            f"{WINDOW_ENV}={raw!r} is not a usable retention window; it must "
            "be a positive number of days. Refusing rather than falling back "
            f"to {MAIN_WINDOW_DAYS}: a window this pass reads out of the "
            "environment decides which frames are destroyed, and one that is "
            "garbled has to be loud")
    return value, WINDOW_ENV


def _drive_tags(repo: str) -> list[tuple[int, str]]:
    """Drive tags sorted by number. Lexical order puts drive-0010 before
    drive-0009, which is the wrong order for picking the previous drive."""
    names = _out(repo, "for-each-ref", "--format=%(refname:short)",
                 "refs/tags/drive-*").split()
    return sorted((int(n.rsplit("-", 1)[-1]), n) for n in names
                  if n.rsplit("-", 1)[-1].isdigit())


def _is_ancestor(repo: str, a: str, b: str) -> bool:
    return _git(repo, "merge-base", "--is-ancestor", a, b).returncode == 0


def git_version(repo: str) -> tuple[int, ...]:
    """The leading numeric fields of `git version`, or () if it will not say."""
    parts = _out(repo, "version").split()
    if len(parts) < 3:
        return ()
    out = []
    for field in parts[2].split("."):
        if not field.isdigit():
            break
        out.append(int(field))
    return tuple(out)


def _is_shallow(repo: str) -> bool:
    return _out(repo, "rev-parse", "--is-shallow-repository").strip() == "true"


def _remove_cache(path: str) -> None:
    """Delete a git-written cache file or tree of them.

    Git marks graph and pack files read-only, and Windows refuses to unlink a
    read-only file: shutil.rmtree(ignore_errors=True) therefore left the stale
    commit-graph exactly where it was and said nothing, which leaves a
    repository `git fsck` calls broken. Clearing the bit first is what makes
    the unlink possible there.

    On POSIX the chmod decides nothing -- the directory's write bit is what
    permits an unlink -- so it is best-effort. Insisting on it turned a file
    owned by another uid into a PermissionError on a delete that would have
    succeeded, which is the chmod failing the deletion it exists to enable.
    It also narrows the mode as a side effect, and a mode we changed on a file
    we then failed to remove is a change nobody asked for.
    """
    if not os.path.lexists(path):
        return
    if os.path.isdir(path) and not os.path.islink(path):
        for name in os.listdir(path):
            _remove_cache(os.path.join(path, name))
        os.rmdir(path)
        return
    with contextlib.suppress(OSError):
        os.chmod(path, stat.S_IWRITE | stat.S_IREAD)
    os.remove(path)     # the real operation, and the one that reports


def refresh_commit_graph(repo: str) -> str:
    """Rebuild the commit-graph, or remove it and say why it cannot be rebuilt.

    A graph left over from before a prune names commits that are gone, and
    `git fsck` reports that as a broken repository. Removing it is safe: it is
    a regenerable cache, not history. Rebuilding it is what a shallow main
    costs. Note the split chain as well as the single file: `git commit-graph
    write --reachable` does not replace a chain, so writing over one leaves
    the stale entries in place.
    """
    info = os.path.join(_out(repo, "rev-parse", "--absolute-git-dir").strip(),
                        "objects", "info")
    stale = [os.path.join(info, n) for n in ("commit-graph", "commit-graphs")]
    for path in stale:
        try:
            _remove_cache(path)
        except OSError as exc:
            # Loud, because the alternative is a repository that reports
            # itself broken from now on for a reason nobody watched happen.
            raise MaintenanceError(
                f"the stale commit-graph at {path} could not be deleted: "
                f"{exc}. Left in place it names commits that are gone, and "
                "git fsck reports that as a broken repository") from exc
    if _is_shallow(repo):
        return ("commit-graph dropped, not rebuilt: git does not write one "
                "for a shallow repository")
    _out(repo, "commit-graph", "write", "--reachable")
    return "commit-graph rewritten"


def _oversized(repo: str, cut: str, keeping: int,
               handled: list[str]) -> list[tuple[str, int]]:
    """Refs outside the retention path holding more frames than main keeps.

    Retention governs refs/heads/main and whatever reaches into it. A ref
    carrying an independent copy of the record, a backup of an earlier public
    ref among them, shares no commits with main at all, so nothing above
    notices it, and the disk it costs is just as real.
    """
    skip = tuple(handled) + ("refs/reverts/", "refs/parking",
                             "refs/heads/main", "refs/heads/src",
                             # Rebuilt from the pruned main a few lines below.
                             "refs/heads/public", lineage.REF)
    out = []
    for name in _out(repo, "for-each-ref", "--format=%(refname)",
                     "refs/").split():
        if name.startswith(skip) or _is_ancestor(repo, cut, name):
            # Descends from the cut, so its history is main's kept history
            # and its commit count here is the pre-prune one. A drive tag on
            # the current drive lands in this branch.
            continue
        r = _git(repo, "rev-list", "--count", name)
        n = r.stdout.strip()
        if r.returncode == 0 and n.isdigit() and int(n) > keeping:
            out.append((name, int(n)))
    return out


def window_cut(repo: str, days: float = MAIN_WINDOW_DAYS,
               ref: str = "refs/heads/main") -> tuple[str, str]:
    """The oldest commit to keep on `ref`, and a sentence about why.

    Never cuts into the most recent drive. The unit of forensic value is a
    drive, not a frame: half a drive on disk answers no question that the
    whole drive would not have answered better, and it takes the bisect
    endpoint with it.
    """
    horizon = time.time() - days * 86400
    kept = _out(repo, "rev-list", f"--max-age={int(horizon)}", ref).split()
    note = f"{len(kept)} frame(s) inside the {days:g} day window"
    cut = kept[-1] if kept else ""
    tags = _drive_tags(repo)
    if tags:
        # Everything after the previous drive's tag is the current drive. With
        # only one tag left, which is what the ref looks like after a prune,
        # that is the whole ref: over-retaining is the safe direction to be
        # wrong in, and the alternative is cutting into the only drive on
        # disk.
        span = f"{tags[-2][1]}..{ref}" if len(tags) >= 2 else ref
        first = _out(repo, "rev-list", span).split()
        floor = first[-1] if first else ""
        if floor and (not cut or _is_ancestor(repo, floor, cut)):
            cut = floor
            note += "; held back to the start of the most recent drive"
    if not cut:
        cut = _out(repo, "rev-list", "--max-count=1", ref).strip()
        note += "; nothing else survives, keeping the tip alone"
    return cut, note


def unrecorded_promotions(repo: str, cut: str,
                          ref: str = "refs/heads/main") -> list[str]:
    """Checkpoint sets among the frames about to go that lineage cannot explain.

    Exactly the lookup blame.attribute() does, run ahead of time: for every
    models.json in the range being dropped, is there a lineage entry carrying
    that same blob, dated no later than the frames that ran under it? A
    lineage entry that is newer does not explain them, it just happens to
    match, and blaming an old incident with a promotion that had not happened
    yet is the confident wrong answer this whole split exists to avoid.
    """
    # --min-parents, not --max-parents. A root commit has zero parents, which
    # is <= 1, so the max form matched every cut there has ever been and the
    # guard below it could never fire.
    has_parent = _out(repo, "rev-list", "--min-parents=1", "--max-count=1",
                      cut).strip()
    if not has_parent:
        return []                      # cut is the root; nothing is dropped
    raw = _git(repo, "log", "--format=%x01%H %ct", "--raw", "--no-abbrev",
               "--no-renames", f"{cut}^", "--", "models.json")
    if raw.returncode != 0:
        return []                      # `{cut}^` does not resolve; nothing below
    known = lineage.index(repo)
    missing, ct = [], 0
    for line in raw.stdout.splitlines():
        if line.startswith("\x01"):
            ct = int(line[1:].split()[1])
        elif line.startswith(":"):
            blob = line.split()[3]
            if not any(b == blob and lct <= ct for _, lct, b in known):
                missing.append(blob)
    return sorted(set(missing))


# ---------------------------------------------------------------------------
# Doing it.

def _reaches_dropped(repo: str, cut: str, pattern: str) -> list[str]:
    """Refs matching `pattern` that reach commits below `cut`.

    A ref is what keeps objects alive, so pruning main reclaims nothing while
    a drive tag, a revert anchor or a leftover parking entry still points into
    the range being dropped. The test is the merge base with the cut: a ref
    off to the side reaches the dropped range through its own parents, which
    is how refs/reverts/<sha> holds a whole chain of frames alive while
    pointing at a commit that is on no branch at all. A ref whose merge base
    is the cut itself only reaches kept history, and one with no merge base is
    a separate history like refs/heads/src.
    """
    out = []
    for name in _out(repo, "for-each-ref", "--format=%(refname)",
                     pattern).split():
        r = _git(repo, "merge-base", name, cut)
        base = r.stdout.strip()
        # A ref pointing at a blob, refs/parking-meta/* among them, fails here
        # rather than needing a type test: it reaches no commits at all.
        if r.returncode == 0 and base and base != cut:
            out.append(name)
    return out


def _is_remote_tracking(ref: str) -> bool:
    return ref.startswith("refs/remotes/")


def _stray_note(ref: str) -> str:
    """How a ref that still reaches below the cut gets described.

    "left alone: somebody's safety copy" is true of refs/backup/* and false of
    refs/remotes/*, and filing the second under the first is the difference
    between an operator decision and this clone's own bookkeeping. A clone
    fetched origin's main and its tracking ref pins every object main just
    dropped, so the pack barely moves and the report says the pass respected
    somebody's choice -- a choice nobody made. Named accurately, with the
    command, because reclaiming the disk here is a decision about the remote
    and retention has no business making it silently either way.
    """
    if _is_remote_tracking(ref):
        return ("still holds dropped frames (remote-tracking; retention "
                "cannot reclaim these objects -- `git update-ref -d <ref>` or "
                f"repoint the remote if this clone's disk matters): {ref}")
    return f"left alone, still holds dropped frames: {ref}"


def prune(repo: str, days: float = MAIN_WINDOW_DAYS,
          dry_run: bool = False) -> tuple[list[str], bool]:
    """Drop frames older than the window off refs/heads/main.

    Returns the report and whether it did a full repack, which is the one
    thing the caller cannot see from the text and must not repeat.
    """
    report = []
    if _git(repo, "rev-parse", "--verify", "--quiet", lineage.REF).returncode:
        raise MaintenanceError(
            f"{lineage.REF} does not exist, so the promotions on main are "
            "recorded nowhere else and pruning would destroy them. Seed it "
            "with `carctl lineage --backfill` first", report=report)
    cut, note = window_cut(repo, days)
    report.append(f"cut at {cut[:10]} ({note})")
    total = int(_out(repo, "rev-list", "--count", "refs/heads/main").strip())
    keeping = int(_out(repo, "rev-list", "--count", f"{cut}..refs/heads/main")
                  .strip()) + 1
    if keeping >= total:
        report.append("nothing on main is outside the window")
        return report, False
    report.append(f"dropping {total - keeping} of {total} frame(s)")

    gaps = unrecorded_promotions(repo, cut)
    if gaps:
        raise MaintenanceError(
            f"{len(gaps)} checkpoint set(s) in the range being dropped have "
            f"no lineage entry dated at or before the frames that ran under "
            f"them (e.g. blob {gaps[0][:10]}). Pruning would destroy the only "
            "record of when those models shipped. `carctl lineage "
            "--backfill` seeds an empty ref from them; `carctl lineage "
            "--rebuild` replaces one that a drive has already written to",
            # The lines above -- where the cut fell and how many frames were
            # at stake -- are the context that makes this refusal actionable.
            # Reporting the gate alone told the operator that something was
            # not pruned and nothing about what nearly was.
            report=report)

    doomed = (_reaches_dropped(repo, cut, "refs/tags/drive-*")
              + _reaches_dropped(repo, cut, "refs/reverts/*")
              + _reaches_dropped(repo, cut, "refs/parking/*"))
    report.append(f"dropping {len(doomed)} ref(s) that reach below the cut")
    # Anything else still reaching into the dropped range keeps it on disk.
    # Reported rather than deleted: refs/backup/* is somebody's safety copy,
    # and a retention pass is not the thing that gets to decide about that.
    # public is excluded because it is rebuilt from the pruned main below.
    strays = [r for r in _reaches_dropped(repo, cut, "refs/")
              if r not in doomed
              and r not in ("refs/heads/main", "refs/heads/public")]
    for r in strays:
        report.append("  " + _stray_note(r))
    for name, n in _oversized(repo, cut, keeping, doomed + strays):
        report.append(f"  outside retention, {n} commits of its own: {name}"
                      + (" (remote-tracking)" if _is_remote_tracking(name)
                         else ""))
    if dry_run:
        report.append("dry run, nothing changed")
        return report, False

    # From here on the repository is being changed, and a failure partway
    # leaves it in a state the caller has to be told about rather than handed
    # as a bare "not pruned". The report accumulated above is the only account
    # of how far it got, so it travels with the exception.
    try:
        for name in doomed:
            _out(repo, "update-ref", "-d", name)
        # The shallow boundary. `git repack` honours it, unlike a replace-ref
        # graft, so the dropped commits actually leave the pack.
        shallow = os.path.join(
            _out(repo, "rev-parse", "--absolute-git-dir").strip(), "shallow")
        # Added, not replaced. A repo can already be shallow for a reason
        # that is not ours, a --depth clone among them, and dropping
        # somebody else's boundary leaves git expecting parents that are not
        # there. A stale entry for a commit that is gone is ignored, so
        # keeping one costs nothing.
        have = []
        try:
            with open(shallow, encoding="utf-8") as fh:
                have = [line.strip() for line in fh if line.strip()]
        except OSError:
            pass
        if cut not in have:
            have.append(cut)
        with open(shallow, "w", encoding="utf-8", newline="\n") as fh:
            fh.write("".join(line + "\n" for line in have))
        # public is a scrubbed copy of main and inherits its retention. Rebuilt
        # rather than deleted, because leaving a full-length copy of the frames
        # we just dropped on another ref reclaims nothing.
        logs = ["HEAD", "refs/heads/main"]
        if _git(repo, "rev-parse", "--verify", "--quiet",
                "refs/heads/public").returncode == 0:
            publish.build_public_ref(repo)
            report.append("rebuilt refs/heads/public from the pruned main")
            # Only after that rebuild succeeded. publish.py keeps the previous
            # public ref reachable through its reflog precisely so a failed
            # rebuild has something to fall back to, and every one of those
            # older entries is a full-length copy of the frames this pass is
            # dropping. There is a good public ref now, so the fallback's job
            # is done.
            logs.append("refs/heads/public")
        # These reflogs and no others. `--all` would take refs/heads/src with
        # it, which is source history's safety net and has nothing to do with
        # the retention window. HEAD is here because it is checked out on main
        # and its reflog pins every frame we are dropping.
        report.append("expiring the reflogs of " + ", ".join(logs))
        _out(repo, "reflog", "expire", "--expire=now",
             "--expire-unreachable=now", *logs)
        before = _disk(repo)
        # The one place a full repack is right: it is what actually drops the
        # objects, and it runs at most once per window rather than after every
        # drive.
        _out(repo, "repack", "-a", "-d", "--unpack-unreachable=now")
        _out(repo, "prune", "--expire=now")
        report.append(f"pack {before} -> {_disk(repo)}")
    except Exception as exc:
        # Exception, not MaintenanceError. Everything in this block that is
        # not a git call raises something else entirely: build_public_ref
        # raises Unscrubbable for a path nobody has decided about and
        # RuntimeError when fast-import fails, and the shallow file is plain
        # open(). Catching only our own type let those out as raw tracebacks
        # AFTER the doomed refs were deleted and the boundary was written --
        # no report, and a full-length refs/heads/public left standing beside
        # a pruned main, which is the one outcome that reclaims nothing and
        # looks like a crash rather than a half-done job.
        #
        # Not BaseException: a KeyboardInterrupt or a SystemExit here is
        # somebody stopping the process, and swallowing that into a report is
        # how a Ctrl-C stops meaning stop.
        raise MaintenanceError(
            str(exc) if isinstance(exc, MaintenanceError)
            else f"{type(exc).__name__}: {exc}",
            report=report, destructive=True) from exc
    return report, True


def repack(repo: str, dry_run: bool = False) -> list[str]:
    """Collapse the per-session packs, without rewriting the whole store.

    fast-import writes one pack per drive and nothing merges them, so the pack
    count grows with the drive count and every object lookup pays for it. A
    geometric repack keeps that count logarithmic while only rewriting the
    small packs, which is what makes this affordable often enough to matter.
    """
    n = len(_packs(repo))
    if not n:
        # A repo whose objects are all loose, which is every repo before its
        # first fast-import session. `git repack` succeeds and reports
        # "0 pack(s) 0.0 MB -> 0 pack(s) 0.0 MB", which reads as a failure to
        # do anything rather than as nothing to do.
        return ["no packs yet, nothing to repack"]
    geometric = git_version(repo) >= MIN_GEOMETRIC_GIT
    how = "geometrically" if geometric else "in full, git is older than 2.32"
    if dry_run:
        return [f"{n} pack(s), {_disk(repo)}; would repack {how}"]
    before = _disk(repo)
    if geometric:
        _out(repo, "repack", "-d", "--geometric=2")
    else:
        # No --geometric means no cheap way to collapse the per-session packs,
        # so take the expensive one. More work than it needs to be, which is
        # the right way round for the half of this that is only about lookup
        # cost. The multi-pack-index is written by maintain() either way.
        _out(repo, "repack", "-a", "-d")
    return [f"{n} pack(s) {before} -> {len(_packs(repo))} pack(s) "
            f"{_disk(repo)} ({how})"]


def _packs(repo: str) -> list[str]:
    d = os.path.join(_out(repo, "rev-parse", "--absolute-git-dir").strip(),
                     "objects", "pack")
    try:
        return [f for f in os.listdir(d) if f.endswith(".pack")]
    except OSError:
        return []


def _disk(repo: str) -> str:
    d = os.path.join(_out(repo, "rev-parse", "--absolute-git-dir").strip(),
                     "objects", "pack")
    total = 0
    try:
        for name in os.listdir(d):
            with contextlib.suppress(OSError):
                total += os.path.getsize(os.path.join(d, name))
    except OSError:
        return "?"
    return f"{total / 1e6:.1f} MB"


def maintain(repo: str, days: float | None = None,
             dry_run: bool = False) -> list[str]:
    """The whole stationary-only maintenance pass.

    `days` of None means "whatever resolve_window_days decides", which is the
    environment or the built-in default. Its MaintenanceError is deliberately
    not caught here: an unusable window is a reason not to start, and the
    caller reports it.

    Returns the report when the pass finished, whatever it refused along the
    way. Raises MaintenanceError with the FULL report attached when the prune
    got past its point of no return, because a half-pruned repository that
    exits 0 is the failure this whole module is written not to have: refs are
    gone, the boundary is on disk, and the next thing to read that exit code
    is a cron line that will never mention it again.
    """
    days, source = resolve_window_days(days)
    ok, why = stationary(repo)
    report = [f"stationary check: {why}"]
    # Stated, not implied. The window is what decides which frames stop
    # existing, and a report that has to be read months later cannot go and ask
    # what the environment held at the time.
    report.append(f"window: {days:g} day(s) ({source})")
    if not ok:
        report.append("refusing to touch the object store while the vehicle "
                      "is not stopped")
        return report
    partway = None
    try:
        pruned, repacked = prune(repo, days, dry_run)
    except MaintenanceError as exc:
        # A retention gate that refuses is not a reason to skip the pack
        # maintenance. Packs bite before the disk does, and the two halves
        # fail independently.
        repacked = False
        if exc.destructive:
            # It got past the point of no return. Saying only "not pruned"
            # here would be a lie in the direction that matters: refs are gone
            # and the boundary is written, so what the operator needs is the
            # account of how far it got and the fact that finishing the job is
            # a re-run, not a repair. Held, not raised yet: the rest of the
            # pass still has to run and still has lines worth reading, and
            # they belong in the same report as this.
            partway = exc
            pruned = exc.report + [
                f"prune failed PARTWAY: {exc}",
                "the doomed refs are already deleted and the shallow boundary "
                "is already written, so the objects below the cut are "
                "unreachable but still on disk; once the cause above is "
                "cleared, running maintain again finishes the job"]
        else:
            # exc.report first: it holds where the cut fell and how many
            # frames were at stake, which is what makes a refusal something an
            # operator can act on instead of a sentence about a gate.
            pruned = exc.report + [f"not pruned: {exc}"]
    report += pruned
    if not repacked:
        try:
            report += repack(repo, dry_run)
        except MaintenanceError as exc:
            # Same independence, one level down: a repack that cannot run is a
            # line in the report, not a traceback out of a maintenance pass
            # that has already done destructive work worth reading about.
            report.append(f"repack failed: {exc}")
    if not dry_run:
        # Last, and after both halves: either can invalidate them.
        try:
            report.append(refresh_commit_graph(repo))
            # `git multi-pack-index write` exits non-zero with "no pack files
            # to index" on a repo whose objects are all still loose. That is
            # git declining to write an index of nothing, not a cache refresh
            # failing, and reporting it as one puts a scary line in the report
            # of a pass where nothing was wrong.
            if _packs(repo):
                _out(repo, "multi-pack-index", "write")
                report.append("multi-pack-index rewritten")
        except MaintenanceError as exc:
            report.append(f"cache refresh failed: {exc}")
    if partway is not None:
        # Everything the pass did is in `report` now, including the repack and
        # cache lines above. The caller prints it and exits non-zero: a
        # half-pruned repository must not be reported the way a clean one is.
        raise MaintenanceError(str(partway), report=report,
                               destructive=True) from partway
    return report
