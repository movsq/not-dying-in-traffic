"""Retention and repack: what stops being kept, and when we may spend disk.

At 10 Hz the car commits 864,000 times a day. Across the 1210 commits already
on main a frame costs 238.8 bytes packed, so a drive-day is about 197 MB and a
year about 70 GB. Treat that as a floor: these are short scripted drives whose
poses delta extremely well, and it is measured after a repack.

Blame has to reach back to the promotion of the oldest checkpoint still in
service, which today is ckpt-controller-2026.03.01-0b12, about six months old.
Eighteen months of full frames to satisfy that would be ~105 GB, so retention
splits by file rather than by time:

  * Full frames on main: 14 days. Enough for incident forensics plus upload
    lag, ~2.8 GB at the floor, budget 10 GB for real delta ratios.
  * models.json: kept indefinitely on refs/heads/lineage, which commits that
    one file, one commit per promotion. See lineage.py.

The window is a forensics judgement, not a derived number: it is how long
after an incident somebody might still want 10 Hz poses on the vehicle. The
lineage half is derived and firm, and it is the half that makes the window
safe to shorten.

Pruning is done with a shallow boundary rather than a rewrite, because a
rewrite changes every surviving commit's sha, and a sha is a frame's identity
here: refs/reverts/<sha> is what anchors an incident to the frame it happened
on, and every incident report ever printed names one. Rewriting main daily
would invalidate yesterday's incident report. A graft was the other candidate
and does not work: git disables replace refs while packing, on purpose, so the
commit objects survive, and commits are 70% of the pack.

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
import contextlib, json, os, stat, subprocess, time
from . import lineage, publish

MAIN_WINDOW_DAYS = 14
# km/h. Below this the wheels are not turning and maintenance may run.
STATIONARY_KMH = 1.0
LOCK_NAME = "carctl-drive.lock"
# Grace on the declared end of a drive before its lock stops counting. A drive
# knows its own length up front, so the lock can expire by itself: a power cut
# mid-drive must not leave a file behind that blocks maintenance forever on a
# vehicle with nobody in it.
LOCK_GRACE_S = 60.0
# `git repack --geometric` landed in 2.32. Everything else here works on the
# git that ships with a 2020 distribution, so this is the one version gate,
# and it degrades rather than refusing.
MIN_GEOMETRIC_GIT = (2, 32)


class MaintenanceError(Exception):
    """A step could not be completed. Never reported as "nothing to do": a
    gate that answers the same way when it passes and when it never ran is
    worse than no gate."""


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
    body = json.dumps({"pid": os.getpid(),
                       "until": time.time() + seconds + LOCK_GRACE_S,
                       "seconds": seconds})
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
            with contextlib.suppress(OSError):
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
    """(major, minor) of the git on PATH, or () if it will not say."""
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
    repository `git fsck` calls broken. Clearing the bit first is a no-op on
    POSIX, where the directory's write permission is what decides.
    """
    if not os.path.lexists(path):
        return
    if os.path.isdir(path) and not os.path.islink(path):
        for name in os.listdir(path):
            _remove_cache(os.path.join(path, name))
        os.rmdir(path)
        return
    os.chmod(path, stat.S_IWRITE | stat.S_IREAD)
    os.remove(path)


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
                f"the commit-graph at {path} names commits this prune "
                f"removed and could not be deleted: {exc}") from exc
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
    if len(tags) >= 2:
        previous = tags[-2][1]
        first = _out(repo, "rev-list", f"{previous}..{ref}").split()
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
    parents = _out(repo, "rev-list", "--max-parents=1", "--max-count=1",
                   cut).strip()
    if not parents:
        return []                      # cut is the root; nothing is dropped
    raw = _git(repo, "log", "--format=%x01%H %ct", "--raw", "--no-abbrev",
               "--no-renames", f"{cut}^", "--", "models.json")
    if raw.returncode != 0:
        return []                      # no parent, so nothing below the cut
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
            "with `carctl lineage --backfill` first")
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
            "--rebuild` replaces one that a drive has already written to")

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
        report.append(f"  left alone, still holds dropped frames: {r}")
    for name, n in _oversized(repo, cut, keeping, doomed):
        report.append(f"  outside retention, {n} commits of its own: {name}")
    if dry_run:
        report.append("dry run, nothing changed")
        return report, False

    for name in doomed:
        _out(repo, "update-ref", "-d", name)
    # The shallow boundary. `git repack` honours it, unlike a replace-ref
    # graft, so the dropped commits actually leave the pack.
    with open(os.path.join(_out(repo, "rev-parse", "--absolute-git-dir").strip(),
                           "shallow"), "w", encoding="utf-8",
              newline="\n") as fh:
        fh.write(cut + "\n")
    # public is a scrubbed copy of main and inherits its retention. Rebuilt
    # rather than deleted, because leaving a full-length copy of the frames we
    # just dropped on another ref reclaims nothing.
    logs = ["HEAD", "refs/heads/main"]
    if _git(repo, "rev-parse", "--verify", "--quiet",
            "refs/heads/public").returncode == 0:
        publish.build_public_ref(repo)
        report.append("rebuilt refs/heads/public from the pruned main")
        # Only after that rebuild succeeded. publish.py keeps the previous
        # public ref reachable through its reflog precisely so a failed
        # rebuild has something to fall back to, and every one of those older
        # entries is a full-length copy of the frames this pass is dropping.
        # There is a good public ref now, so the fallback's job is done.
        logs.append("refs/heads/public")
    # These reflogs and no others. `--all` would take refs/heads/src with it,
    # which is source history's safety net and has nothing to do with the
    # retention window. HEAD is here because it is checked out on main and its
    # reflog pins every frame we are dropping.
    report.append("expiring the reflogs of " + ", ".join(logs))
    _out(repo, "reflog", "expire", "--expire=now", "--expire-unreachable=now",
         *logs)
    before = _disk(repo)
    # The one place a full repack is right: it is what actually drops the
    # objects, and it runs at most once per window rather than after every
    # drive.
    _out(repo, "repack", "-a", "-d", "--unpack-unreachable=now")
    _out(repo, "prune", "--expire=now")
    report.append(f"pack {before} -> {_disk(repo)}")
    return report, True


def repack(repo: str, dry_run: bool = False) -> list[str]:
    """Collapse the per-session packs, without rewriting the whole store.

    fast-import writes one pack per drive and nothing merges them, so the pack
    count grows with the drive count and every object lookup pays for it. A
    geometric repack keeps that count logarithmic while only rewriting the
    small packs, which is what makes this affordable often enough to matter.
    """
    n = len(_packs(repo))
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


def maintain(repo: str, days: float = MAIN_WINDOW_DAYS,
             dry_run: bool = False) -> list[str]:
    """The whole stationary-only maintenance pass."""
    ok, why = stationary(repo)
    report = [f"stationary check: {why}"]
    if not ok:
        report.append("refusing to touch the object store while the vehicle "
                      "is not stopped")
        return report
    try:
        pruned, repacked = prune(repo, days, dry_run)
    except MaintenanceError as exc:
        # A retention gate that refuses is not a reason to skip the pack
        # maintenance. Packs bite before the disk does, and the two halves
        # fail independently.
        pruned, repacked = [f"not pruned: {exc}"], False
    report += pruned
    report += [] if repacked else repack(repo, dry_run)
    if not dry_run:
        # Last, and after both halves: either can invalidate them.
        report.append(refresh_commit_graph(repo))
        _out(repo, "multi-pack-index", "write")
        report.append("multi-pack-index rewritten")
    return report
