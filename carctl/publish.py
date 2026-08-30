"""Build the public ref, then push it.

refs/heads/main holds exact poses at 10 Hz. That is a location feed for one
named person, and git history outlives any later deletion, so main never
leaves the car.

refs/heads/public is rebuilt from main by streaming `git fast-export` through
a filter and back into `git fast-import`. Three things get rewritten, and it
has to be all three, because a commit carries location in more places than the
tree:

  blob      state.json poses snap to a 25 m grid
  message   the `Pose:` trailer snaps to the same grid
  identity  the committer becomes an anonymous address, timestamp to the minute

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


def _snap(v: float) -> float:
    return round(v / GRID_M) * GRID_M


def scrub_state(raw: bytes) -> bytes:
    """Coarsen one state.json blob. Anything else passes through."""
    try:
        d = json.loads(raw)
    except ValueError:
        return raw
    if not isinstance(d, dict) or "pose" not in d:
        return raw
    p = d["pose"]
    p["x"] = _snap(p["x"])
    p["y"] = _snap(p["y"])
    p["heading"] = round(p["heading"], 1)
    p["v"] = round(p["v"], 1)
    p["steer"] = round(p["steer"], 1)
    d["t_mono_ns"] = int(round(d["t_mono_ns"] / 1e9 / TIME_Q) * TIME_Q * 1e9)
    return (json.dumps(d, indent=1, sort_keys=True) + "\n").encode()


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


def _filter(stream: bytes, src: bytes, dst: bytes) -> bytes:
    """Rewrite a fast-export stream. Length-prefixed `data` payloads mean we
    cannot work line by line the whole way, so track what we are inside of."""
    out = bytearray()
    i, n, ctx = 0, len(stream), b""
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
                payload = scrub_state(payload)
            elif ctx == b"commit":
                payload = scrub_message(payload)
            out += b"data %d\n" % len(payload) + payload
            ctx = b""
            continue

        if line.startswith(b"blob"):
            ctx = b"blob"
        elif b" inline" in line and line.startswith(b"M "):
            ctx = b"blob"          # inline file data follows, same as a blob
        elif line.startswith((b"commit ", b"reset ")):
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
    subprocess.run(["git", "update-ref", "-d", dst], cwd=repo,
                   capture_output=True)
    r = subprocess.run(["git", "fast-import", "--quiet", "--force"],
                       cwd=repo, input=rewritten, capture_output=True)
    if r.returncode != 0:
        raise RuntimeError(r.stderr.decode("utf-8", "replace")[:2000])
    return subprocess.run(["git", "rev-parse", dst], cwd=repo,
                          capture_output=True, text=True, encoding="utf-8").stdout.strip()


def audit(repo: str, ref: str = "refs/heads/public") -> list[str]:
    """Fail loudly if anything personal survived. Cheap enough to run before
    every push, and a push is the one step you cannot take back."""
    problems = []
    idents = subprocess.run(
        ["git", "log", "--format=%an <%ae>%n%cn <%ce>", ref], cwd=repo,
        capture_output=True, text=True, encoding="utf-8").stdout.split("\n")
    for who in set(i for i in idents if i.strip()):
        if who.encode() != PUBLIC_IDENT:
            problems.append(f"identity {who!r} is not the public identity")
    bodies = subprocess.run(["git", "log", "--format=%B", ref], cwd=repo,
                            capture_output=True, text=True, encoding="utf-8").stdout
    for m in re.finditer(r"^Pose: (-?[\d.]+),(-?[\d.]+)", bodies, re.M):
        x, y = float(m.group(1)), float(m.group(2))
        if x != _snap(x) or y != _snap(y):
            problems.append(f"unsnapped pose in a commit message: {x},{y}")
            break
    return problems


def push(repo: str, remote: str = "origin") -> tuple[bool, str]:
    """Push the public ref only. The refspec is pinned in .git/config too, so
    a bare `git push` cannot reach main by accident."""
    problems = audit(repo)
    if problems:
        return False, "audit failed, nothing pushed:\n  " + "\n  ".join(problems)
    r = subprocess.run(
        ["git", "push", remote, "refs/heads/public:refs/heads/main"],
        cwd=repo, capture_output=True, text=True, encoding="utf-8", timeout=180)
    return r.returncode == 0, (r.stderr or r.stdout).strip()
