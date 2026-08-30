"""Build the public ref, then push it.

refs/heads/main holds exact poses at 10 Hz. That is a location feed for one
named person, and git history outlives any later deletion, so main never
leaves the car.

refs/heads/public is rebuilt from main by streaming `git fast-export` through
a filter and back into `git fast-import`. A commit carries location in more
places than its tree, so every channel gets an explicit decision:

  blob      state.json poses snap to a 25 m grid; sensors.json loses the
            landmark-relative distance and its full-precision lane offset
  message   the `Pose:` trailer snaps to the same grid
  identity  the committer becomes an anonymous address, timestamp to the minute

Every path in the tree is dispatched through _SCRUBBERS by name, and a path
with no entry aborts the build. An unrecognised file is a channel nobody has
decided about yet, and defaulting those to "publish" is exactly how
sensors.json reached the public ref byte-identical to main: it is not
state.json, so the old shape-sniffing scrubber returned it untouched, and
`light_distance` -- the signed distance to a fixed landmark on a named street
-- shipped at full double precision beside a pose claiming 25 m quantisation.

Street names, manoeuvres, speeds and the reversible flag survive untouched.
Those are the parts worth reading.

No git-filter-repo dependency. The same fast-import the commit loop uses does
the rewrite.
"""
from __future__ import annotations
import json
import re
import subprocess

GRID_M = 25.0    # pose quantisation
TIME_Q = 60      # commit timestamp quantisation, seconds

# The car's own identity stays on main. Nothing personal reaches the public ref.
PUBLIC_IDENT = b"not-dying-in-traffic <fleet@not-dying-in-traffic.invalid>"

_POSE_LINE = re.compile(
    rb"^Pose: (-?\d+(?:\.\d+)?),(-?\d+(?:\.\d+)?) @ (-?\d+(?:\.\d+)?) rad$",
    re.MULTILINE)

# Same shape as _POSE_LINE, for reading messages back out of a built ref. The
# looser `[\d.]+` this replaced also matched "1.2.3", which float() then threw
# on -- inside the gate, where an exception reads as "did not finish".
_POSE_TEXT = re.compile(r"^Pose: (-?\d+(?:\.\d+)?),(-?\d+(?:\.\d+)?)", re.M)


class Unscrubbable(Exception):
    """A path or payload the filter has no decision for. Always fatal: the
    build stops rather than publishing something nobody has looked at."""


class AuditError(Exception):
    """The audit could not complete. Never reported as "no problems found" --
    a gate that answers the same way when it passes and when it never ran is
    worse than no gate, because only one of those readings is true."""


def _snap(v: float) -> float:
    return round(v / GRID_M) * GRID_M


def _load(raw: bytes, path: str) -> dict:
    """Parse a blob we are about to publish. A payload we cannot read is a
    payload we cannot check, so it stops the build instead of passing."""
    try:
        d = json.loads(raw)
    except ValueError as exc:
        raise Unscrubbable(f"{path} is not JSON: {exc}") from exc
    if not isinstance(d, dict):
        raise Unscrubbable(f"{path} is not a JSON object")
    return d


def _dump(d: dict) -> bytes:
    return (json.dumps(d, indent=1, sort_keys=True) + "\n").encode()


def scrub_state(raw: bytes) -> bytes:
    """Coarsen one state.json blob."""
    d = _load(raw, "state.json")
    missing = {"pose", "t_mono_ns"} - d.keys()
    if missing:
        raise Unscrubbable(f"state.json is missing {sorted(missing)}")
    p = d["pose"]
    p["x"] = _snap(p["x"])
    p["y"] = _snap(p["y"])
    p["heading"] = round(p["heading"], 1)
    p["v"] = round(p["v"], 1)
    p["steer"] = round(p["steer"], 1)
    d["t_mono_ns"] = int(round(d["t_mono_ns"] / 1e9 / TIME_Q) * TIME_Q * 1e9)
    return _dump(d)


def scrub_sensors(raw: bytes) -> bytes:
    """sensors.json is a location channel, and used to be published raw.

    `light_distance` is the signed distance to a fixed landmark on a named
    street, so at full precision it is an exact position along that street --
    strictly sharper than the grid state.json spends all its effort on. It
    snaps to the same grid. `lateral_offset` is position across the lane: a
    decimetre of it is worth reading, fourteen decimal places is a
    fingerprint. The lidar ranges carry the same trailing-digit fingerprint.
    """
    d = _load(raw, "sensors.json")
    if "light_distance" in d:
        d["light_distance"] = _snap(d["light_distance"])
    for key, places in (("lateral_offset", 1), ("lidar_min_range", 1),
                        ("lidar_min_range_rear", 1), ("imu_accel_z", 1),
                        ("lidar_min_bearing", 2), ("wheel_slip", 2)):
        if key in d:
            d[key] = round(d[key], places)
    return _dump(d)


def scrub_actuators(raw: bytes) -> bytes:
    """Commanded throttle, brake and steering. Not a position channel, but
    steer_cmd integrates into heading, so it gets the rounding heading gets
    rather than full float precision."""
    d = _load(raw, "actuators.json")
    for key in ("throttle", "brake", "steer_cmd"):
        if key in d:
            d[key] = round(d[key], 2)
    return _dump(d)


def scrub_models(raw: bytes) -> bytes:
    """Checkpoint identifiers. No location, and the one-key-per-line layout is
    load-bearing for `git blame -L n,n models.json`, so this passes through
    byte for byte -- deliberately, not by falling off the end of a lookup."""
    _load(raw, "models.json")        # parse to prove it is what we think
    return raw


# Every path that reaches the public ref needs an entry here. Adding a file to
# Frame.tree() without adding it here fails the build rather than publishing
# an unreviewed channel.
_SCRUBBERS = {
    "state.json":     scrub_state,
    "sensors.json":   scrub_sensors,
    "actuators.json": scrub_actuators,
    "models.json":    scrub_models,
}


def scrub_message(raw: bytes) -> bytes:
    """Commit messages carry the pose too. Scrubbing only the tree leaves the
    exact trajectory sitting in the body of every commit, which defeats the
    whole exercise."""
    def repl(m: re.Match) -> bytes:
        x, y, h = (float(g) for g in m.groups())
        return b"Pose: %.2f,%.2f @ %.1f rad" % (_snap(x), _snap(y), round(h, 1))
    return _POSE_LINE.sub(repl, raw)


def _public_ident(line: bytes) -> bytes:
    """`committer Name <mail> 1788056467 +0200` loses its name, address and
    second hand. The keyword and timezone stay."""
    parts = line.rstrip(b"\n").rsplit(b" ", 2)
    if len(parts) != 3 or not parts[1].isdigit():
        return line
    keyword = parts[0].split(b" ", 1)[0]
    ts = int(parts[1]) // TIME_Q * TIME_Q
    return b"%s %s %d %s\n" % (keyword, PUBLIC_IDENT, ts, parts[2])


_M_LINE = re.compile(rb"(?m)^M \d+ (:\d+|[0-9a-f]{40}) (.+)$")


def _mark_paths(stream: bytes) -> dict[bytes, str]:
    """Map each blob mark to the path it is committed at.

    fast-export emits every blob up front carrying only a mark, and names the
    path later in the commit's `M <mode> :<mark> <path>` line -- so which
    scrubber applies is not knowable at the moment the payload streams past.
    One cheap scan up front resolves it. Before this, every blob was fed to
    scrub_state regardless of path, which is precisely why sensors.json came
    out untouched: no "pose" key, so the old scrubber handed it straight back.
    """
    paths: dict[bytes, str] = {}
    for m in _M_LINE.finditer(stream):
        ref = m.group(1)
        path = m.group(2).rstrip(b"\r").decode("utf-8", "replace")
        if not ref.startswith(b":"):
            raise Unscrubbable(
                f"{path} is committed by raw oid {ref.decode()[:12]}, which "
                "would republish the original unsnapped blob from the same "
                "object store")
        if path not in _SCRUBBERS:
            raise Unscrubbable(
                f"no scrubber for {path!r}: every path published to the "
                "public ref needs an explicit decision")
        prev = paths.setdefault(ref, path)
        if prev != path:
            raise Unscrubbable(
                f"mark {ref.decode()} is committed at both {prev!r} and "
                f"{path!r}, so no single scrubber applies")
    return paths


def _filter(stream: bytes, src: bytes, dst: bytes) -> bytes:
    """Rewrite a fast-export stream. Length-prefixed `data` payloads mean we
    cannot work line by line the whole way, so track what we are inside of."""
    out = bytearray()
    i, n, ctx = 0, len(stream), b""
    mark = b""
    paths = _mark_paths(stream)
    while i < n:
        eol = stream.find(b"\n", i)
        if eol == -1:
            out += stream[i:]
            break
        line = stream[i:eol + 1]
        i = eol + 1

        if line.startswith(b"data "):
            size = int(line[5:])
            payload = stream[i:i + size]
            i += size
            if ctx == b"blob":
                if mark not in paths:
                    raise Unscrubbable(
                        f"blob {mark.decode() or '<unmarked>'} is never "
                        "committed at any path, so no scrubber applies")
                payload = _SCRUBBERS[paths[mark]](payload)
            elif ctx == b"commit":
                payload = scrub_message(payload)
            out += b"data %d\n" % len(payload) + payload
            ctx = b""
            continue

        if line.rstrip() == b"blob":
            ctx, mark = b"blob", b""
        elif ctx == b"blob" and line.startswith(b"mark :"):
            # Only a blob's mark. Commits carry marks too, and capturing one
            # of those would scrub a commit message as if it were a tree.
            mark = line.split()[1]
        elif line.startswith(b"M ") and b" inline " in line:
            # The inline form names its path on the line itself and the
            # payload follows immediately, so it needs no mark lookup.
            parts = line.rstrip(b"\n").split(b" ", 3)
            path = parts[3].decode("utf-8", "replace")
            if path not in _SCRUBBERS:
                raise Unscrubbable(
                    f"no scrubber for {path!r}: every path published to the "
                    "public ref needs an explicit decision")
            mark = b"inline:" + parts[3]
            ctx, paths[mark] = b"blob", path
        elif line.startswith((b"commit ", b"reset ", b"tag ")):
            # `tag ` lands here so an annotated tag's message goes through
            # scrub_message like any other body. It used to fall through
            # every branch and emit its payload verbatim, while the `tagger`
            # line right above it was being anonymised -- so tags looked
            # handled. Lightweight tags carry no payload and are unaffected.
            ctx = b"commit"
            line = line.replace(src, dst)
        elif line.startswith((b"committer ", b"author ", b"tagger ")):
            line = _public_ident(line)
        out += line
    return bytes(out)


def build_public_ref(repo: str, src: str = "refs/heads/main",
                     dst: str = "refs/heads/public") -> str:
    exported = subprocess.run(["git", "fast-export", src], cwd=repo,
                              capture_output=True, check=True).stdout
    rewritten = _filter(exported, src.encode(), dst.encode())
    # No `update-ref -d dst` first. Deleting the ref took the previously
    # published public ref -- and its reflog -- with it, so a rebuild that
    # then failed left nothing to fall back to. It was never needed either:
    # --force is exactly the flag that lets fast-import move a ref
    # non-fast-forward.
    r = subprocess.run(["git", "fast-import", "--quiet", "--force"],
                       cwd=repo, input=rewritten, capture_output=True)
    if r.returncode != 0:
        raise RuntimeError(r.stderr.decode("utf-8", "replace")[:2000])
    rev = subprocess.run(["git", "rev-parse", dst], cwd=repo,
                         capture_output=True, text=True, encoding="utf-8")
    if rev.returncode != 0:
        raise RuntimeError(f"fast-import reported success but {dst} does not "
                           f"resolve: {rev.stderr.strip()}")
    return rev.stdout.strip()


def _git_out(repo: str, *args: str) -> str:
    """Run git for the audit, or raise. Any git failure inside the gate is a
    failure OF the gate: swallowing it left empty output, which filtered down
    to zero problems and read exactly like a clean ref."""
    r = subprocess.run(["git", *args], cwd=repo, capture_output=True,
                       text=True, encoding="utf-8")
    if r.returncode != 0:
        raise AuditError(f"`git {' '.join(args)}` failed: {r.stderr.strip()}")
    return r.stdout


def _check_blob(path: str, raw: bytes) -> list[str]:
    """A published blob must be a fixed point of its own scrubber.

    Checking idempotence rather than re-listing every field means a new key
    added to a frame cannot slip past: if the scrubber would change what is
    already published, it was not scrubbed.
    """
    try:
        return [] if _SCRUBBERS[path](raw) == raw else [
            f"{path} in the built ref is not in its scrubbed form"]
    except Unscrubbable as exc:
        return [f"{path} in the built ref cannot be scrubbed: {exc}"]


def _blob_problems(repo: str, ref: str) -> list[str]:
    """Re-read what actually landed in the trees.

    The scrubbers run at build time; this is the independent check that they
    ran and did their job. An audit reading only `git log` cannot see a blob
    at all, which is how sensors.json shipped byte-identical to main with the
    gate reporting clean.
    """
    wanted: dict[str, str] = {}
    for line in _git_out(repo, "rev-list", "--objects", ref).splitlines():
        oid, _, name = line.partition(" ")
        if name in _SCRUBBERS:
            wanted[oid] = name
    if not wanted:
        raise AuditError(f"{ref} exposes none of {sorted(_SCRUBBERS)}; "
                         "refusing to bless a ref this does not understand")
    # One cat-file for the whole ref, not one `git show` per commit.
    batch = subprocess.run(["git", "cat-file", "--batch"], cwd=repo,
                           input=("\n".join(wanted) + "\n").encode(),
                           capture_output=True)
    if batch.returncode != 0:
        raise AuditError("`git cat-file --batch` failed: "
                         + batch.stderr.decode("utf-8", "replace").strip())
    # Aggregated per path, not per blob. A ref with 280 unscrubbed frames is
    # one defect reported once, not 280 lines nobody reads to the end of.
    counts: dict[str, list] = {}
    buf, pos = batch.stdout, 0
    while pos < len(buf):
        eol = buf.find(b"\n", pos)
        if eol == -1:
            break
        header = buf[pos:eol].split()
        pos = eol + 1
        if len(header) != 3:
            raise AuditError(f"unexpected cat-file header {buf[:eol]!r}")
        oid, size = header[0].decode(), int(header[2])
        for why in _check_blob(wanted[oid], buf[pos:pos + size]):
            seen = counts.setdefault(why, [0, oid])
            seen[0] += 1
        pos += size + 1
    return [f"{why} ({n} of {sum(1 for p in wanted.values() if p == why.split()[0])}"
            f" blobs, e.g. {oid[:12]})" for why, (n, oid) in sorted(counts.items())]


def audit(repo: str, ref: str = "refs/heads/public") -> list[str]:
    """Fail loudly if anything personal survived. Cheap enough to run before
    every push, and a push is the one step you cannot take back.

    Raises AuditError if it cannot complete. Returning [] in that case made
    "clean" and "never looked" the same answer.
    """
    problems = []
    idents = _git_out(repo, "log", "--format=%an <%ae>%n%cn <%ce>", ref)
    for who in sorted({i for i in idents.split("\n") if i.strip()}):
        if who.encode() != PUBLIC_IDENT:
            problems.append(f"identity {who!r} is not the public identity")
    bodies = _git_out(repo, "log", "--format=%B", ref)
    unsnapped = sorted({
        (float(m.group(1)), float(m.group(2)))
        for m in _POSE_TEXT.finditer(bodies)
        if float(m.group(1)) != _snap(float(m.group(1)))
        or float(m.group(2)) != _snap(float(m.group(2)))})
    for x, y in unsnapped[:5]:
        problems.append(f"unsnapped pose in a commit message: {x},{y}")
    if len(unsnapped) > 5:
        problems.append(f"...and {len(unsnapped) - 5} further distinct "
                        "unsnapped poses in commit messages")
    problems += _blob_problems(repo, ref)
    return problems


def push(repo: str, remote: str = "origin") -> tuple[bool, str]:
    """Push the public ref only. The refspec is pinned in .git/config too, so
    a bare `git push` cannot reach main by accident."""
    try:
        problems = audit(repo)
    except AuditError as exc:
        return False, f"audit could not complete, nothing pushed: {exc}"
    if problems:
        return False, "audit failed, nothing pushed:\n  " + "\n  ".join(problems)
    r = subprocess.run(
        ["git", "push", remote, "refs/heads/public:refs/heads/main"],
        cwd=repo, capture_output=True, text=True, encoding="utf-8", timeout=180)
    return r.returncode == 0, (r.stderr or r.stdout).strip()
