"""Attribute an incident to a model checkpoint.

models.json is written one subsystem per line specifically so that a line-range
blame resolves to the commit that last changed *that* subsystem's checkpoint --
which, for a car that hot-swaps checkpoints over the air mid-drive, is the
commit where the offending model was promoted.
"""
from __future__ import annotations
import json, pathlib, shlex, subprocess


def _git(repo: str, *args: str) -> str:
    return subprocess.run(["git", *args], cwd=repo, capture_output=True,
                          text=True, encoding="utf-8").stdout


def line_of(repo: str, sha: str, subsystem: str) -> int | None:
    blob = _git(repo, "show", f"{sha}:models.json").splitlines()
    for i, line in enumerate(blob, start=1):
        if line.strip().startswith(f'"{subsystem}"'):
            return i
    return None


def attribute(repo: str, sha: str, subsystem: str) -> dict:
    n = line_of(repo, sha, subsystem)
    if n is None:
        return {"error": f"no line for subsystem {subsystem!r} at {sha[:10]}"}

    porcelain = _git(repo, "blame", "-L", f"{n},{n}", "--porcelain", sha,
                     "--", "models.json").splitlines()
    if not porcelain:
        return {"error": "blame returned nothing"}

    culprit = porcelain[0].split()[0]
    meta = {"subsystem": subsystem, "models_json_line": n,
            "promoted_in": culprit}
    for line in porcelain[1:]:
        if line.startswith("summary "):
            meta["promoted_by_commit_subject"] = line[len("summary "):]
        if line.startswith("committer-time "):
            meta["promoted_at_unix"] = int(line.split()[1])
        if line.startswith("\t"):
            meta["checkpoint"] = line.strip().split('"')[3]

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
