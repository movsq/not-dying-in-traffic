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
import os
import re
import stat
import subprocess

from . import lineage

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
    # lane_ref names a street and a lane, which state.json already publishes
    # as `road`, so it passes through: it is a decision, not an omission.
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


def _ensure_push_guard(repo: str) -> list[str]:
    """Make a bare `git push` from this repo fail instead of guessing.

    `push.default` decides what `git push` with no refspec sends. Every value
    except `nothing` picks branches for you, and the branch this repo is on is
    main -- exact 10 Hz poses. One habitual `git push` in the wrong directory
    is all it takes, and a push is the one step you cannot take back.

    Set locally and only when the local config does not already say something.
    An operator who wrote `push.default = simple` into THIS repo made a
    decision about this repo and keeps it; a value merely inherited from
    ~/.gitconfig is a global habit, and a repo that publishes a scrubbed copy
    of a location feed must not inherit "push whatever matches".

    What this deliberately does NOT do is pin `remote.<name>.push` refspecs.
    Pinned refspecs would make a bare `git push` succeed -- publishing public
    and lineage -- while BYPASSING audit() and audit_lineage() entirely. That
    turns the guard into a shortcut around the gate. The job here is the
    opposite: a bare push fails loudly, and `carctl publish --push`, which
    audits first, stays the only path that reaches the remote.

    Returns warning lines, and raises nothing. A guard that cannot be armed is
    a fact the caller has to carry into its own report; raising here made a
    config.lock collision -- a `git config` running anywhere else in this repo
    at the same instant -- into a failure of whatever operation happened to be
    asking, which is how this ended up aborting a retention pass partway.
    """
    have = subprocess.run(["git", "config", "--local", "--get", "push.default"],
                          cwd=repo, capture_output=True, text=True,
                          encoding="utf-8")
    if have.returncode == 0 and have.stdout.strip():
        value = have.stdout.strip()
        if value == "nothing":
            return []
        return [f"push.default is {value!r} in this repo's config, left as the "
                "operator set it; a bare `git push` here picks branches on its "
                "own, and the branch this repo is on is main"]
    r = subprocess.run(["git", "config", "--local", "push.default", "nothing"],
                       cwd=repo, capture_output=True, text=True,
                       encoding="utf-8")
    if r.returncode != 0:
        return ["could not set push.default=nothing, so a bare `git push` here "
                f"is not guarded: {r.stderr.strip()}"]
    return []


# Deliberately /bin/sh and deliberately tiny: this file is read by whoever is
# about to push, and a guard nobody can read in ten seconds is a guard nobody
# trusts. It is also compared byte for byte against what is on disk, which is
# how ensure_push_safety tells its own work from somebody else's hook.
_PRE_PUSH_HOOK = """#!/bin/sh
# carctl push guard -- installed by carctl.
#
# refs/heads/main HERE is exact 10 Hz poses for one named person. The remote's
# main carries refs/heads/public, the scrubbed rebuild. So the only local ref
# allowed on the left of a refspec ending at the remote's main is public.
#
# push.default=nothing only blocks the BARE `git push`, and git's own error
# for that helpfully suggests naming a refspec -- `git push origin main` --
# which is precisely the push that must never happen. This hook is the guard
# on the spelling git itself recommends.
while read -r local_ref local_sha remote_ref remote_sha
do
	case "$remote_ref" in
	refs/heads/main)
		if [ "$local_ref" != "refs/heads/public" ]; then
			echo "carctl: refusing to push $local_ref -> $remote_ref" >&2
			echo "carctl: the remote's main carries refs/heads/public, the scrubbed ref;" >&2
			echo "carctl: this repo's refs/heads/main is exact 10 Hz poses and stays in the car." >&2
			echo "carctl: publish with:  carctl publish --push" >&2
			exit 1
		fi
		;;
	esac
done
exit 0
"""


def ensure_push_safety(repo: str) -> list[str]:
    """Arm both push guards. Returns warning lines; never raises.

    Two guards, because they cover different spellings of the same mistake.
    `push.default=nothing` covers the bare `git push`. The hook covers
    `git push origin main` -- the refspec git's own error message recommends
    when the bare form is refused, which makes it the likeliest next thing
    typed by the person who just read that error.

    Never raises, and returns lines rather than printing them, because the
    callers are a publish command and a drive: neither has any business
    failing because a guard could not be armed, and both have a report to put
    the warning in. A guard that aborts the operation it protects is how this
    used to poison a retention pass partway through its destructive window.

    The hook lives in .git/hooks, which does not clone and does not travel in
    any ref, so every working copy has to install its own. That is why this
    runs from `carctl drive` -- the command that creates the sensitive data,
    and therefore the first moment a clone has anything to protect -- as well
    as from publish.
    """
    warnings = _ensure_push_guard(repo)
    hooks = subprocess.run(["git", "rev-parse", "--git-path", "hooks"],
                           cwd=repo, capture_output=True, text=True,
                           encoding="utf-8")
    if hooks.returncode != 0 or not hooks.stdout.strip():
        return warnings + ["could not locate this repo's hooks directory, so "
                           "the pre-push guard is not installed: "
                           + (hooks.stderr.strip() or "git said nothing")]
    # --git-path answers relative to the repo root, and honours core.hooksPath,
    # which is the whole reason for asking git instead of joining ".git/hooks".
    path = os.path.join(repo, hooks.stdout.strip(), "pre-push")
    try:
        with open(path, encoding="utf-8") as fh:
            existing = fh.read()
    except OSError:
        existing = None
    # Honouring core.hooksPath also means following it out of the repo. Set in
    # ~/.gitconfig it names one directory every repository on the machine
    # runs its hooks from, and this hook installed there refused
    # `git push <remote> main` in all of them, each refusal claiming that
    # repo's main was somebody's 10 Hz poses. The guard is about this repo, so
    # it is only written inside this repo's own git dir. Anywhere else is the
    # operator's to decide, and not armed until they do.
    common = subprocess.run(["git", "rev-parse", "--git-common-dir"],
                            cwd=repo, capture_output=True, text=True,
                            encoding="utf-8")
    gitdir = (os.path.realpath(os.path.join(repo, common.stdout.strip()))
              if common.returncode == 0 and common.stdout.strip() else None)
    hookdir = os.path.realpath(os.path.dirname(path))
    try:
        inside = (gitdir is not None
                  and os.path.commonpath([gitdir, hookdir]) == gitdir)
    except ValueError:          # two drives on Windows: certainly not inside
        inside = False
    if not inside:
        where = os.path.dirname(path)
        line = (f"core.hooksPath points at {where}, outside this repo, so the "
                "pre-push guard is not installed: a hook there would refuse "
                "pushes to main in every repository that shares it. "
                "push.default refuses only the bare `git push`; "
                "`git push origin main` from this repo is not guarded")
        if existing == _PRE_PUSH_HOOK:
            line += (f". {path} is a copy an earlier carctl installed there, "
                     "and it applies to every repository using that "
                     "directory; move it aside")
        return warnings + [line]
    if existing is not None:
        if existing == _PRE_PUSH_HOOK:
            return warnings            # already ours, nothing to say
        # Not overwritten. A pre-push hook is somebody's decision about this
        # repo, and silently replacing it would be this tool deciding for them
        # -- possibly deleting the very check they wrote. Named loudly instead,
        # because until it is dealt with the second guard is not armed.
        return warnings + [
            f"{path} already exists and was left alone, so the pre-push guard "
            "is not armed; push.default refuses only the bare `git push`, "
            "and `git push origin main` from this repo is not guarded. Move "
            "it aside and re-run, or add the refusal to it by hand"]
    try:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8", newline="\n") as fh:
            fh.write(_PRE_PUSH_HOOK)
        # 0755, not the umask's guess: git runs a hook only if it is
        # executable, and a hook that silently does not run is worse than none.
        os.chmod(path, stat.S_IRWXU | stat.S_IRGRP | stat.S_IXGRP
                 | stat.S_IROTH | stat.S_IXOTH)
    except OSError as exc:
        return warnings + [f"could not install the pre-push guard at {path}: "
                           f"{exc}"]
    return warnings


def build_public_ref(repo: str, src: str = "refs/heads/main",
                     dst: str = "refs/heads/public") -> str:
    # No push guard here. It used to be armed on the way in, on the reasoning
    # that the window between a scrubbed copy existing and the push being
    # guarded should be zero -- but retain.prune() calls this from inside its
    # destructive window, after the doomed refs are deleted and the shallow
    # boundary is written, and `git config` there can lose a config.lock race
    # and take the whole retention pass down half-finished. Retention has no
    # business writing config. push() and ensure_push_safety() are the guard
    # sites, and ensure_push_safety runs from `carctl drive` -- before any of
    # this exists, which closes the window from the other end.
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


def _batch_blobs(repo: str, oids: list[str]):
    """Yield (oid, payload) for every oid, through one `git cat-file --batch`.

    One cat-file for a whole ref, not one `git show` per commit. Both audits
    need exactly this and both used to carry their own copy of the length-
    prefixed parser -- near-verbatim, down to the same header check -- so a
    fix to one was a fix to one. There is one copy now and two checks on top
    of it.

    Every parse failure is an AuditError rather than a skipped blob, for the
    reason the whole module keeps repeating: a gate that answers the same way
    when it passes and when it could not look is worse than no gate. A missing
    object comes back as `<oid> missing`, which is two fields, not three.
    """
    batch = subprocess.run(["git", "cat-file", "--batch"], cwd=repo,
                           input=("\n".join(oids) + "\n").encode(),
                           capture_output=True)
    if batch.returncode != 0:
        raise AuditError("`git cat-file --batch` failed: "
                         + batch.stderr.decode("utf-8", "replace").strip())
    buf, pos = batch.stdout, 0
    while pos < len(buf):
        eol = buf.find(b"\n", pos)
        if eol == -1:
            break
        header = buf[pos:eol].split()
        start, pos = pos, eol + 1
        if len(header) != 3 or header[1] != b"blob":
            # The header line itself, not buf[:eol]: sliced from the start of
            # the buffer that quotes every blob read so far into the message.
            raise AuditError(f"unexpected cat-file header {buf[start:eol]!r}")
        size = int(header[2])
        yield header[0].decode(), buf[pos:pos + size]
        pos += size + 1


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
    # Aggregated per path, not per blob. A ref with 280 unscrubbed frames is
    # one defect reported once, not 280 lines nobody reads to the end of.
    counts: dict[str, list] = {}
    for oid, payload in _batch_blobs(repo, list(wanted)):
        for why in _check_blob(wanted[oid], payload):
            seen = counts.setdefault(why, [0, oid])
            seen[0] += 1
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


def audit_lineage(repo: str, ref: str = lineage.REF) -> list[str]:
    """The same gate, for the ref that is pushed alongside public.

    "Publishable by construction" was an argument, not a check. Lineage was
    built by a different code path from public -- gitstore and lineage.backfill
    write it directly, no fast-export filter anywhere near it -- so nothing
    ever confirmed the construction held, and the personal committer address
    rode out on every lineage commit that has ever been pushed.

    audit() cannot be pointed at this ref: it requires the four frame paths and
    refuses a ref it does not recognise, and lineage carries exactly one file.
    So the three channels get checked here in the shape lineage actually has:

      identity  every author and committer is PUBLIC_IDENT
      tree      nothing but models.json, and each blob parses as a checkpoint
                dict -- the same proof-of-shape scrub_models demands, because
                an unreadable payload on this ref is a payload nobody checked
      message   no Pose: trailer anywhere; this ref is kept forever, so a pose
                on it is the one thing that must never be here

    Raises AuditError if it cannot complete, for the reason audit() does:
    "clean" and "never looked" must not be the same answer.
    """
    _git_out(repo, "rev-parse", "--verify", ref)
    problems = []
    idents = _git_out(repo, "log", "--format=%an <%ae>%n%cn <%ce>", ref)
    for who in sorted({i for i in idents.split("\n") if i.strip()}):
        if who.encode() != PUBLIC_IDENT:
            problems.append(f"identity {who!r} is not the public identity")
    bodies = _git_out(repo, "log", "--format=%B", ref)
    # Any line opening with the trailer keyword, not _POSE_TEXT's shape. A
    # malformed Pose: line is still a pose that got here, and the ref this
    # gate protects is the one nothing ever prunes.
    if any(line.startswith("Pose:") for line in bodies.splitlines()):
        problems.append("a commit message on this ref carries a Pose: line, "
                        "on the one ref that is kept forever")
    # rev-list --objects names every object with the path it appears at; root
    # trees come back with an empty name. Anything else named is a second file
    # or a subdirectory, i.e. a channel nobody decided about.
    blobs = []
    for line in _git_out(repo, "rev-list", "--objects", ref).splitlines():
        oid, _, name = line.partition(" ")
        if not name:
            continue
        if name != "models.json":
            problems.append(f"{name!r} is in a tree on this ref, which is "
                            "supposed to carry models.json and nothing else")
        else:
            blobs.append(oid)
    if not blobs:
        raise AuditError(f"{ref} exposes no models.json at all; refusing to "
                         "bless a ref this does not understand")
    # Aggregated, like _blob_problems: a ref with 300 unreadable entries is one
    # defect reported once, not 300 lines nobody reads to the end of. Same
    # reader as _blob_problems, different check on top of it.
    bad: dict[str, list] = {}
    for oid, payload in _batch_blobs(repo, blobs):
        try:
            scrub_models(payload)
        except Unscrubbable as exc:
            seen = bad.setdefault(str(exc), [0, oid])
            seen[0] += 1
    problems += [f"models.json on this ref is not a checkpoint set: {why} "
                 f"({n} of {len(blobs)} blobs, e.g. {oid[:12]})"
                 for why, (n, oid) in sorted(bad.items())]
    return sorted(set(problems))


# What git says when the remote has history the pushed ref does not contain.
# Matched on three spellings because the wording moved between git versions
# and the remedy is the same for all of them.
_REJECTED = ("non-fast-forward", "[rejected]", "fetch first")


def _push_one(repo: str, remote: str, refspec: str) -> tuple[bool, str]:
    r = subprocess.run(
        ["git", "push", remote, refspec],
        cwd=repo, capture_output=True, text=True, encoding="utf-8", timeout=180)
    return r.returncode == 0, (r.stderr or r.stdout).strip()


def _with(warnings: list[str], text: str) -> str:
    """Warnings first, then the outcome. A guard that could not be armed is
    not a reason to fail a push, but it is not something to drop either."""
    return "\n".join([f"warning: {w}" for w in warnings] + [text]).strip()


def push(repo: str, remote: str = "origin",
         audited: bool = False) -> tuple[bool, str]:
    """Push the public ref, and lineage alongside it.

    Lineage goes because it is publishable by construction: it was designed to
    carry no pose precisely so it could be kept forever, and a clone without
    it answers every blame from the provisional main path -- shipping the
    scrubbed frames while withholding the one ref that explains them would
    publish the puzzle and keep the answer.

    Both refs are audited, by their own gates. Auditing only public and
    pushing lineage verbatim was the same "publishable by construction"
    reasoning the whole public ref exists to distrust, and it shipped the car
    owner's address on every lineage commit.

    `audited=True` says the caller already ran audit() and audit_lineage() in
    this invocation and stopped if either had anything to say -- which is
    exactly what `carctl publish` does before it gets here, so the default
    made the plain form audit the same two refs twice. It is opt-in, not the
    default, because a library caller that has run no gate at all must not get
    an unaudited push by forgetting an argument.

    Two pushes, not one. Sending both refspecs in a single `git push` meant a
    remote that refused lineage refused the whole command, and this reported
    total failure -- while public had in fact landed, because git pushes what
    it can. "Nothing was published" and "half of it was" need opposite next
    actions from whoever reads it.
    """
    warnings = ensure_push_safety(repo)
    if not audited:
        try:
            problems = audit(repo)
        except AuditError as exc:
            return False, _with(warnings,
                                f"audit could not complete, nothing pushed: {exc}")
        if problems:
            return False, _with(warnings, "audit failed, nothing pushed:\n  "
                                + "\n  ".join(problems))
    have_lineage = subprocess.run(
        ["git", "rev-parse", "--verify", "--quiet", lineage.REF],
        cwd=repo, capture_output=True).returncode == 0
    if have_lineage and not audited:
        try:
            lineage_problems = audit_lineage(repo)
        except AuditError as exc:
            return False, _with(warnings, "the lineage audit could not "
                                f"complete, nothing pushed: {exc}")
        if lineage_problems:
            # The whole push, not just the lineage half. Pushing public alone
            # and reporting the lineage failure would publish the puzzle and
            # keep the answer, which is the thing this pair exists to avoid --
            # and it would do it while calling itself a partial success. Both
            # gates therefore run before either push starts.
            return False, _with(warnings,
                                "lineage audit failed, nothing pushed:\n  "
                                + "\n  ".join(lineage_problems)
                                + "\n  `carctl lineage --rebuild` rewrites the "
                                  "ref from main with the public identity; "
                                  "entries written before that identity was "
                                  "adopted still carry the car's own address")
    ok, out = _push_one(repo, remote, "refs/heads/public:refs/heads/main")
    if not ok:
        return False, _with(warnings, f"public was refused, nothing pushed:\n{out}")
    if not have_lineage:
        return True, _with(warnings, out)
    ok, lineage_out = _push_one(repo, remote, f"{lineage.REF}:{lineage.REF}")
    if ok:
        return True, _with(warnings, "\n".join(x for x in (out, lineage_out) if x))
    if any(mark in lineage_out for mark in _REJECTED):
        # The expected failure, once, on every vehicle that has run `carctl
        # lineage --rebuild`. The rebuild re-derives the ref from main under
        # the public identity, so its commits are new objects with new shas
        # and the remote's copy is not an ancestor of them.
        return False, _with(warnings, (
            f"public is pushed and current on {remote}; lineage was REFUSED "
            "and is the only thing outstanding:\n" + lineage_out
            + f"\n\n  The remote still carries the pre-rebuild {lineage.REF}: "
              "the history written before the public identity was adopted, "
              "with the car owner's own address on every commit. The rebuilt "
              "ref does not descend from it, so a fast-forward is impossible "
              "and git is right to refuse.\n"
              "  Force it, deliberately, once:\n\n"
              f"      git push {remote} +{lineage.REF}:{lineage.REF}\n\n"
              "  Forcing is the correct answer here and not a way around the "
              "gate: the rewrite exists precisely to take that address off a "
              "ref that is kept forever, so the history being discarded is "
              "the one nobody wants kept. Nothing else is lost -- the shas "
              "were re-derived from the same promotions on main, entry for "
              "entry, and the previous tip is still on this vehicle at "
              "refs/backup/lineage-before-<sha>."))
    return False, _with(warnings, f"public is pushed and current on {remote}; "
                        f"the lineage push failed:\n{lineage_out}")
