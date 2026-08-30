"""Incident detection and the two kinds of revert.

`git revert` on a source tree is total: any diff can be inverted. Physics is
not. So a revert here splits into two independent operations that are allowed
to disagree, and the whole design hangs on keeping them separate:

  RECORD REVERT   is a real `git revert` in the control worktree. Always
                    succeeds. Produces the public, auditable statement "this
                    frame was wrong". Costs nothing physical.

  PHYSICAL REVERT reads the parent commit's state.json, treats it as a goal
                    pose, and ask the planner whether it is inside the
                    reachable set from where the car is *now*. If yes, drive
                    there. If no, the revert is refused and the car runs a
                    minimal-risk manoeuvre instead.

The reachability check is the load-bearing part. A revert that is attempted
without it is just an unplanned manoeuvre with a reassuring name.
"""
from __future__ import annotations
from dataclasses import dataclass
import json, math, os, subprocess
from .state import Frame

MAX_LAT_OFFSET = 1.75   # m from lane centre before we call it off-road
MIN_CLEARANCE  = 1.5    # m
CURB_ACCEL_Z   = 20.0   # m/s^2

# One definition of a one-way door, imported by plant.py (which decides the
# flag at capture time) and msgen.py (which names the reason). These lived as
# three separate literals that disagreed: plant tested `curb == 0.0` against a
# 9.81 gravity offset while msgen tested imu_accel_z > 20, so any curb impulse
# in 0 < curb <= 10.19 marked a frame irreversible with the reason "unknown".
IRREVERSIBLE_CLEARANCE = 2.0   # m; closer than this and the frame is one-way

# A parallel park legitimately closes on the car behind. plant.py's own
# comment calls that van "what the parking manoeuvre is squeezing in behind",
# and MIN_CLEARANCE flagged the successful park as a collision.
PARK_CLEARANCE = 0.5    # m; the floor even a parking manoeuvre must not cross

LIDAR_MAX_RANGE  = 40.0   # m; a return at exactly max range means "nothing seen"
PATH_BEARING_TOL = 0.35   # rad; a return outside this cone is not on our path

# Which subsystem owns which failure. This mapping is what turns `git blame`
# from a party trick into an incident tool.
OWNER = {
    "red_light_run":  "perception",
    "collision":      "prediction",
    "curb_strike":    "controller",
    "off_road":       "planner",
}


@dataclass
class Incident:
    kind: str
    seq: int
    detail: str

    @property
    def subsystem(self) -> str:
        return OWNER.get(self.kind, "planner")


def detect(f: Frame, true_light: str) -> list[Incident]:
    """Every incident true of this frame, most severe first.

    This used to return on the first match. `stop_line_crossed` is true for
    exactly one tick, so a red-light run that coincided with any lidar or IMU
    trigger was not merely deprioritised -- it was erased, with no later tick
    able to catch it. blame.attribute() was then handed "controller" or
    "prediction" instead of "perception", and the OTA-promoted checkpoint that
    actually caused the run was never blamed. Several things can be wrong at
    once, and a frame where two are wrong is not less interesting than one.
    """
    found = []
    if f.sensors.imu_accel_z > CURB_ACCEL_Z:
        found.append(Incident("curb_strike", f.seq,
                              f"az={f.sensors.imu_accel_z:.1f} m/s^2"))
    # Parking gets a tighter floor, not an exemption. A bare static threshold
    # reported the successful park at seq 139 (1.40 m from the parked van at
    # 2.6 m/s) as a collision and blamed prediction for a manoeuvre that went
    # right; in the shipped drive that was invisible only because curb_strike
    # happened to outrank it.
    limit = PARK_CLEARANCE if f.maneuver == "park" else MIN_CLEARANCE
    if f.sensors.lidar_min_range < limit:
        found.append(Incident("collision", f.seq,
                              f"clearance={f.sensors.lidar_min_range:.2f} m"))
    if true_light == "red" and f.sensors.stop_line_crossed and f.pose.v > 1.0:
        found.append(Incident(
            "red_light_run", f.seq,
            f"crossed stop line at {f.pose.v * 3.6:.0f} km/h "
            f"while perception reported '{f.sensors.light_state}'"))
    if abs(f.sensors.lateral_offset) > MAX_LAT_OFFSET:
        found.append(Incident("off_road", f.seq,
                              f"lateral offset {f.sensors.lateral_offset:.2f} m"))
    return found


# ---------------------------------------------------------------------------

@dataclass
class RevertVerdict:
    allowed: bool
    reason: str
    goal: dict | None = None
    cost_m: float = 0.0


def _arc_length(dist: float, bearing: float, behind: bool) -> float:
    """Chord to driven arc.

    A car cannot translate sideways, so reaching a goal offset from straight
    ahead (or straight behind) means driving a curve, and the straight line
    understates it. Reverting seq 55 back to seq 25 across the scripted left
    turn is 16.12 m of chord against 18.42 m of path, so the clearance test
    was asking for 2.3 m less room than the manoeuvre actually needs.
    """
    off = (math.pi - abs(bearing)) if behind else abs(bearing)
    off = min(off, math.pi / 2 - 1e-3)
    return dist if off < 1e-6 else dist * off / math.sin(off)


def reachable(now: Frame, goal: dict) -> RevertVerdict:
    """Is the parent commit's pose still inside our reachable set?"""
    gp = goal["pose"]
    dx, dy = gp["x"] - now.pose.x, gp["y"] - now.pose.y
    dist = math.hypot(dx, dy)

    if not goal.get("reversible", True):
        return RevertVerdict(False, "parent frame is itself marked irreversible")

    # The goal is behind us. Reversing on a public road is a manoeuvre, not an
    # undo, and it is only admissible below a crawl with the space to do it.
    bearing = math.atan2(dy, dx) - now.pose.heading
    bearing = (bearing + math.pi) % (2 * math.pi) - math.pi
    behind = abs(bearing) > math.pi / 2

    if behind and now.pose.v > 2.0:
        return RevertVerdict(False,
            f"goal is {dist:.1f} m behind at {now.pose.v * 3.6:.0f} km/h; "
            "reversing is inadmissible above 7 km/h", cost_m=dist)
    # The forward cone says nothing about a path behind the car. Picking the
    # wrong sensor here refuses clear reverses and, worse, clears occupied ones.
    side = "rear" if behind else "forward"
    clearance = (now.sensors.lidar_min_range_rear if behind
                 else now.sensors.lidar_min_range)
    path_len = _arc_length(dist, bearing, behind)
    need = path_len + MIN_CLEARANCE

    # A scalar min-range says nothing about WHERE the return is.
    # lidar_min_bearing was written every frame and read by nothing, so an
    # obstacle 0.4 rad off the path refused a clear return. The rear channel
    # carries no bearing, so it counts as on-path, which is the conservative
    # reading of not knowing.
    on_path = behind or abs(now.sensors.lidar_min_bearing) <= PATH_BEARING_TOL

    if need > LIDAR_MAX_RANGE:
        # Previously this came back as "occupied at 40.0 m", reporting the
        # sensor's own range limit as an occupancy fact. Not seeing anything
        # as far as you can see is not the same as seeing that it is clear.
        return RevertVerdict(
            False, f"{side} return path of {path_len:.1f} m runs past the "
            f"{LIDAR_MAX_RANGE:.0f} m sensor horizon", cost_m=path_len)
    if on_path and clearance < LIDAR_MAX_RANGE and clearance < need:
        return RevertVerdict(
            False, f"{side} return path is occupied at {clearance:.1f} m",
            cost_m=path_len)
    return RevertVerdict(True, f"reachable, {path_len:.1f} m of return path",
                         goal=goal, cost_m=path_len)


def _state_at(repo: str, rev: str) -> tuple[dict | None, str]:
    """Read one commit's state.json. Returns (state, error). A malformed blob
    used to raise straight out of the gate, and an exception escaping a safety
    check is not a refusal -- it is an unhandled crash where a `no` belonged.
    """
    r = subprocess.run(["git", "show", f"{rev}:state.json"], cwd=repo,
                       capture_output=True, text=True, encoding="utf-8")
    if r.returncode != 0:
        return None, f"cannot read state.json at {rev}"
    try:
        d = json.loads(r.stdout)
    except ValueError as exc:
        return None, f"state.json at {rev} is not JSON: {exc}"
    if not isinstance(d, dict) or "pose" not in d:
        return None, f"state.json at {rev} has no pose"
    return d, ""


def physical_revert(repo: str, sha: str, now: Frame) -> RevertVerdict:
    """Evaluate reverting `sha` physically. Does not touch the repo."""
    # The one-way door is recorded on the frame being reverted, not on the
    # frame we would return to. Reading only the parent asked "was the world
    # reversible one tick BEFORE the curb strike?", which is true right up
    # until the instant it stops being the question -- so `reversible: false`
    # never once blocked the revert of the very frame that set it, and the
    # `!` marker msgen derives from it was decorative here.
    frame, err = _state_at(repo, sha)
    if err:
        return RevertVerdict(False, err)
    if not frame.get("reversible", True):
        return RevertVerdict(
            False, f"frame {frame.get('seq', '?')} is itself marked "
            "irreversible; the world cannot be walked back through it")
    goal, err = _state_at(repo, f"{sha}^")
    if err:
        return RevertVerdict(False, "no parent commit to revert to")
    return reachable(now, goal)


@dataclass
class RecordRevert:
    ok: bool
    sha: str = ""
    detail: str = ""
    already: bool = False


def _rev(repo: str, rev: str) -> str:
    return subprocess.run(["git", "rev-parse", "--verify", "--quiet", rev],
                          cwd=repo, capture_output=True, text=True,
                          encoding="utf-8").stdout.strip()


def record_revert(repo: str, sha: str, worktree_root: str,
                  note: str) -> RecordRevert:
    """The record-plane half. Runs in a throwaway detached worktree so it can
    never race the fast-import stream writing to refs/heads/main.

    Every step is checked. The earlier version discarded three exit codes in a
    row, so a second run against an already-reverted tree printed the old sha
    and looked like a fresh revert. It also left the revert commit on a
    detached HEAD in a scratch directory, reachable from nothing, so anchor it
    under refs/reverts/<original> before the worktree goes away.
    """
    full = _rev(repo, sha)
    if not full:
        return RecordRevert(False, detail=f"cannot resolve {sha}")
    ref = f"refs/reverts/{full}"
    existing = _rev(repo, ref)
    if existing:
        return RecordRevert(True, existing, f"already recorded at {ref}",
                            already=True)

    wt = os.path.join(worktree_root, full[:12])
    if os.path.exists(wt):
        # Debris from a crashed or un-removable earlier attempt. `-f` below
        # overrides the branch-already-checked-out check, NOT an existing
        # path, so without this every later revert of this sha collides with
        # the leftover and fails forever.
        subprocess.run(["git", "worktree", "remove", "--force", wt], cwd=repo,
                       capture_output=True)
        subprocess.run(["git", "worktree", "prune"], cwd=repo,
                       capture_output=True)
    add = subprocess.run(["git", "worktree", "add", "--detach", "-f", wt, full],
                         cwd=repo, capture_output=True, text=True,
                         encoding="utf-8")
    if add.returncode != 0:
        return RecordRevert(False, detail=f"worktree: {add.stderr.strip()[:300]}")
    keep = False
    try:
        rv = subprocess.run(["git", "revert", "--no-edit", "-n", full], cwd=wt,
                            capture_output=True, text=True, encoding="utf-8")
        if rv.returncode != 0:
            return RecordRevert(False,
                                detail=f"conflicted: {rv.stderr.strip()[:300]}")
        msg = "\n\n".join([f'revert: "{_subject(repo, full)}"', note,
                           f"This reverts commit {full}."])
        cm = subprocess.run(["git", "commit", "-m", msg], cwd=wt,
                            capture_output=True, text=True, encoding="utf-8")
        if cm.returncode != 0:
            return RecordRevert(
                False, detail=f"commit: {(cm.stderr or cm.stdout).strip()[:300]}")
        new = _rev(wt, "HEAD")
        if not new:
            return RecordRevert(
                False, detail="revert was committed but HEAD does not resolve")
        anchor = subprocess.run(["git", "update-ref", ref, new], cwd=repo,
                                capture_output=True, text=True,
                                encoding="utf-8")
        if anchor.returncode != 0:
            # This was the one unchecked exit code left, and it was the one
            # that mattered: the ref is the ONLY thing referencing `new`, so
            # the removal below would have left the revert commit unreachable
            # and gc-able while we returned ok=True. Keep the worktree -- it
            # is now the sole reference -- and say so.
            keep = True
            return RecordRevert(
                False, new,
                f"revert commit {new[:12]} was made but could not be anchored "
                f"at {ref}: {anchor.stderr.strip()[:200]}. Keeping the "
                f"worktree at {wt}, which is the only thing referencing it.")
        return RecordRevert(True, new, f"recorded at {ref}")
    finally:
        # The checkout is a regenerable artifact ONLY once the commit it
        # produced is anchored by the ref above; `keep` is what tells the two
        # cases apart.
        if not keep:
            rm = subprocess.run(["git", "worktree", "remove", "--force", wt],
                                cwd=repo, capture_output=True, text=True,
                                encoding="utf-8")
            if rm.returncode != 0:
                # A file held open (AV scanners do this on Windows) leaves
                # both the directory and its admin entry behind, and
                # `worktree add -f` does not overwrite an existing path -- so
                # every future revert of this sha would fail. Prune the admin
                # entry so the next attempt can at least diagnose the
                # leftover directory instead of colliding with a stale one.
                subprocess.run(["git", "worktree", "prune"], cwd=repo,
                               capture_output=True)


def _subject(repo: str, sha: str) -> str:
    return subprocess.run(["git", "log", "-1", "--format=%s", sha], cwd=repo,
                          capture_output=True, text=True, encoding="utf-8").stdout.strip()
