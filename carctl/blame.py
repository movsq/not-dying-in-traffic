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


# The order the fields read in, ahead of anything the registry grows later.
# Not a schema, and not a requirement: an entry carries whatever whoever
# promoted the model bothered to record, so every field here is optional and
# an unknown one still gets printed rather than dropped.
_FIELDS = ("trained", "dataset", "shadow_km", "gate_km", "status", "notes")


def _provenance(entry) -> str:
    """A registry entry as one line a person reads at 3am.

    This used to be the raw dict, printed straight into the incident report:
    the single line of that report that answers "why was this model in the
    car", rendered as `{'trained': '2026-07-14', 'dataset': ...}` with the two
    numbers that matter, shadow kilometres against the gate they were supposed
    to clear, sitting unrelated somewhere in the middle of it.
    """
    if not isinstance(entry, dict):
        return str(entry)          # "not in registry", or a bare note
    shadow, gate = entry.get("shadow_km"), entry.get("gate_km")
    # Only a pair says anything. A shadow figure with no gate to hold it
    # against is a number, and a gate with nothing measured against it is a
    # policy; either alone gets reported as the plain field it is.
    paired = all(isinstance(v, (int, float)) and not isinstance(v, bool)
                 for v in (shadow, gate))
    bits = []
    for key, value in sorted(entry.items(), key=_field_order):
        if key == "gate_km" and paired:
            continue               # said in the shadow_km field
        if key == "shadow_km" and paired:
            if shadow < gate:
                bits.append(f"shadow_km {_km(shadow)} of a {_km(gate)} km "
                            "gate, promoted anyway")
            else:
                bits.append(f"shadow_km {_km(shadow)} over a {_km(gate)} km "
                            "gate")
            continue
        bits.append(f"{key} {value}")
    return ", ".join(bits) if bits else "registry entry is empty"


def _km(n) -> str:
    """A distance, written the way the registry writes it. `f"{1180000:g}"`
    comes out 1.18e+06, and an exponent in the middle of a sentence about
    shadow coverage is a number nobody reads."""
    return f"{n:.0f}" if float(n) == int(n) else f"{n}"


def _field_order(item) -> tuple[int, str]:
    key = item[0]
    return (_FIELDS.index(key) if key in _FIELDS else len(_FIELDS), key)


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
            # The porcelain content line is the models.json line itself,
            # `  "perception": "ckpt-...",` behind a tab, so parse it as the
            # JSON it is. Splitting on quote characters and taking [3] threw
            # away exactly the escaping state.py's models_json() exists to
            # produce: a checkpoint id containing a quote is one field to a
            # JSON reader and four fragments to that split, and the answer it
            # returned was the fragment before the quote.
            frag = line[1:].strip().rstrip(",")
            try:
                pair = json.loads("{" + frag + "}")
            except ValueError:
                # Not our line format at all. The raw fragment is a worse
                # answer than a parsed one and a much better answer than none.
                meta["checkpoint"] = frag
            else:
                meta["checkpoint"] = next(iter(pair.values()), frag)
    # The drive is a name in the body, not a ref: retention deletes the tag
    # with the frames it bounds, and the promotion outlives both.
    for line in _git(repo, "log", "-1", "--format=%B", culprit).splitlines():
        if line.startswith("Drive: "):
            meta["promoted_during_drive"] = line[len("Drive: "):]

    reg = pathlib.Path(repo, "models", "registry.json")
    if reg.exists() and "checkpoint" in meta:
        # encoding="utf-8" explicitly: read_text() otherwise decodes with the
        # locale codec, which is cp1252 on this machine, and a registry note
        # carrying anything outside it raises UnicodeDecodeError in the middle
        # of an incident report. A registry that cannot be read is a missing
        # explanation, not a failed attribution -- everything above it is
        # already correct -- so it is reported in the field it belongs to
        # rather than taking the report down with it.
        try:
            registry = json.loads(reg.read_text(encoding="utf-8"))
            if not isinstance(registry, dict):
                raise ValueError("not a JSON object of checkpoint entries")
            entry = registry.get(meta["checkpoint"], "not in registry")
        except (OSError, ValueError) as exc:
            meta["provenance"] = f"registry unreadable: {exc}"
        else:
            meta["provenance"] = _provenance(entry)
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
