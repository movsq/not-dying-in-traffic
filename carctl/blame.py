"""Attribute an incident to a model checkpoint.

models.json is written one subsystem per line specifically so that a line-range
blame resolves to the commit that last changed *that* subsystem's checkpoint --
which, for a car that hot-swaps checkpoints over the air mid-drive, is the
commit where the offending model was promoted.

That blame runs on refs/heads/lineage, not on main. main has a retention
window and the promotion being looked for is routinely older than it, so a
main sha here is an answer with an expiry date on it: once the window moves
past the promotion, `git blame` walks off the end of what survives and lands
on the oldest remaining commit, which is a confident wrong answer. The lineage
ref carries the same file with one commit per promotion and is never pruned.
See lineage.py.
"""
from __future__ import annotations
import json, pathlib, shlex, subprocess
from . import lineage


def _git(repo: str, *args: str) -> str:
    return subprocess.run(["git", *args], cwd=repo, capture_output=True,
                          text=True, encoding="utf-8").stdout


def line_of(repo: str, sha: str, subsystem: str) -> int | None:
    blob = _git(repo, "show", f"{sha}:models.json").splitlines()
    for i, line in enumerate(blob, start=1):
        if line.strip().startswith(f'"{subsystem}"'):
            return i
    return None


def _where_to_blame(repo: str, sha: str) -> tuple[str, str]:
    """(rev to blame, why it is not the lineage ref).

    Finds the lineage entry that put the incident frame's exact models.json in
    force. Matching on the blob rather than on the timestamp is what makes the
    two refs line up: the frame and the lineage entry commit the same file, so
    they are the same git object, and line numbers agree without anything
    having to assume they do.
    """
    blob = _git(repo, "rev-parse", f"{sha}:models.json").strip()
    when = _git(repo, "log", "-1", "--format=%ct", sha).strip()
    if not blob or not when.isdigit():
        return sha, f"cannot resolve models.json at {sha[:10]}"
    entry = lineage.entry_for(repo, blob, int(when))
    if not entry:
        # Frames written before the lineage ref existed, or a repo where it
        # was never seeded. Answering from main is still right today; it stops
        # being right the moment retention moves past the promotion, so say
        # which of the two answers this is. retain.prune refuses to prune
        # frames in this state for exactly the same reason.
        return sha, (f"no {lineage.REF} entry for this checkpoint set; "
                     "blamed against main, which retention will prune")
    return entry, ""


def attribute(repo: str, sha: str, subsystem: str) -> dict:
    n = line_of(repo, sha, subsystem)
    if n is None:
        return {"error": f"no line for subsystem {subsystem!r} at {sha[:10]}"}

    rev, degraded = _where_to_blame(repo, sha)
    porcelain = _git(repo, "blame", "-L", f"{n},{n}", "--porcelain", rev,
                     "--", "models.json").splitlines()
    if not porcelain:
        return {"error": f"blame returned nothing for {rev[:10]}"}

    culprit = porcelain[0].split()[0]
    meta = {"subsystem": subsystem, "models_json_line": n,
            "promoted_on_ref": "refs/heads/main" if degraded else lineage.REF,
            "promoted_in": culprit}
    if degraded:
        meta["promoted_in_is_provisional"] = degraded
    for line in porcelain[1:]:
        if line.startswith("summary "):
            meta["promoted_by_commit_subject"] = line[len("summary "):]
        if line.startswith("committer-time "):
            meta["promoted_at_unix"] = int(line.split()[1])
        if line.startswith("\t"):
            meta["checkpoint"] = line.strip().split('"')[3]
    # The drive is a name in the body, not a ref: retention deletes the tag
    # with the frames it bounds, and the promotion outlives both.
    for line in _git(repo, "log", "-1", "--format=%B", culprit).splitlines():
        if line.startswith("Drive: "):
            meta["promoted_during_drive"] = line[len("Drive: "):]

    reg = pathlib.Path(repo, "models", "registry.json")
    if reg.exists() and "checkpoint" in meta:
        registry = json.loads(reg.read_text())
        meta["provenance"] = registry.get(meta["checkpoint"], "not in registry")
    return meta


def bisect_script(repo: str, good: str, bad: str) -> str:
    """`git bisect` across a fleet-wide history finds the first drive where a
    checkpoint regressed. Emitted as a script rather than run, because bisect
    on a live vehicle is not a thing.

    Everything interpolated is quoted, and the path is emitted POSIX style.
    The repo path used to go in raw, so on Windows a shell ate the backslashes
    and `git -C` received C:Usersfixednot-dying-in-traffic; a path containing a
    space broke it the same way. A rev like `x; curl ...` became a second
    command in a script whose entire purpose is to be pasted and run.
    """
    r = shlex.quote(pathlib.PurePath(repo).as_posix())
    return (f"git -C {r} bisect start {shlex.quote(bad)} {shlex.quote(good)}\n"
            f"git -C {r} bisect run carctl replay --assert-no-incident\n")
