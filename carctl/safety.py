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
# so MIN_CLEARANCE is not the right question to ask of a park frame; this is
# the floor even a parking manoeuvre must not cross.
PARK_CLEARANCE = 0.5    # m

# A near miss is a kinematic event: it is about closing speed, not about
# proximity, and closing speed is what this constant finally measures --
# nearest-return delta over the frames' own clock, which is an approach rate
# whether the ego is the thing moving or not.
#
# Ego speed alone was standing in for it, and it blinded the detector in two
# directions at once. A parallel park closes on the car behind at a crawl, so
# a 0.3 m/s creep straight through PARK_CLEARANCE fired nothing: the 0.5 m
# floor was unreachable at the speeds parks actually happen at, which is the
# only regime it was ever written for. And a stationary ego being closed on by
# somebody else was silent for the same reason -- the gate asked the one car
# in the scene that was not moving.
COLLISION_CLOSING_SPEED = 0.2   # m/s of range being eaten

# The ego-speed arm is kept as the other half of an OR, not replaced. Matching
# another vehicle's speed inside the clearance eats no range at all -- closing
# is ~0 while the gap stays dangerous -- so following at speed has to keep
# counting, and the recorded seq 108/109 pair is exactly that frame: drop this
# arm and those two stop meaning what they have always meant.
#
# The same 1.0 m/s red_light_run already uses as its floor for "moving",
# deliberately: two different answers to "is this car in motion" inside one
# detect() would be a defect of its own. It is also comfortably above the
# 0.86 m/s the shipped drive is still doing at the first of its stop frames,
# which is what the gate has to clear to work at all.
COLLISION_MIN_SPEED = 1.0   # m/s (3.6 km/h)

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

# Worst first, by how close the frame is to hurting someone: imminent contact
# with another road user outranks entering a junction against a red, which
# outranks striking a curb, which outranks drifting out of the lane. Detection
# order is a function of which sensor is cheapest to test, which is no ranking
# at all -- and callers that take found[0] as "the worst thing here", cli's
# per-tick line among them, were reading exactly that.
SEVERITY = ("collision", "red_light_run", "curb_strike", "off_road")


@dataclass
class Incident:
    kind: str
    seq: int
    detail: str

    @property
    def subsystem(self) -> str:
        return OWNER.get(self.kind, "planner")


def detect(f: Frame, true_light: str, prev: Frame | None = None) -> list[Incident]:
    """Every incident true of this frame, most severe first, by SEVERITY.

    `prev` is the frame one tick back, or None on the first tick of a run. It
    is the only way to know a range is shrinking: a single frame carries a
    distance, and no single distance is an approach. Callers that cannot
    supply it get the old ego-speed-only collision gate, which is a weaker
    detector rather than a broken one.

    "Most severe first" used to describe the order the checks happen to be
    written in, which is not a ranking of anything; sorting is what makes the
    sentence true, and found[0] the worst thing rather than the earliest test.

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
    # Parking gets a tighter floor, not an exemption: a manoeuvre whose whole
    # point is closing on the car behind must not be judged by the road's
    # clearance, but it still has a limit, and PARK_CLEARANCE is it. It is a
    # guard rather than an observed trigger -- the shipped parking manoeuvre
    # never gets closer than 2.92 m in the forward cone (its minimum, at seq
    # 139, doing 2.47 m/s), so nothing in this scenario can reach 0.5 m and
    # the constant has never once fired here.
    #
    # Which floor applies is decided by lane_ref, not by the manoeuvre label.
    # An empty lane_ref means the planner deliberately left the lane
    # reference, and that emptiness already suspends off_road below for
    # exactly this reason: proximity is the job out there, not a fault. The
    # manoeuvre label got it wrong twice on the same van. The car FINISHES
    # parking, the script rolls over to "stop", and the settling frames --
    # still creeping the last half-metre toward the bay van, inside 1.5 m --
    # took the road limit: first as 21 stationary phantoms, then, once the
    # closing-speed arm below existed, as a phantom at seq 149 for closing on
    # a van it was deliberately parking behind. The same signal, one decision:
    # off the lane reference on purpose, judged by the parking floor.
    limit = MIN_CLEARANCE if f.sensors.lane_ref else PARK_CLEARANCE
    # Measured over the frames' own clock, not over an assumed DT. A tick that
    # overran, or a caller stepping the plant at its own rate, would otherwise
    # be handed a fabricated approach rate computed from a period that did not
    # happen. A dt of zero or less means the clock did not advance -- a
    # repeated or reordered frame -- and there is no rate to be had from it,
    # so that reads as "no previous frame" rather than as a division.
    closing = 0.0
    if prev is not None:
        dt = (f.t_mono_ns - prev.t_mono_ns) / 1e9
        if dt > 0:
            closing = (prev.sensors.lidar_min_range
                       - f.sensors.lidar_min_range) / dt
    if (f.sensors.lidar_min_range < limit
            and (closing > COLLISION_CLOSING_SPEED
                 or f.pose.v > COLLISION_MIN_SPEED)):
        found.append(Incident("collision", f.seq,
                              f"clearance={f.sensors.lidar_min_range:.2f} m"))
    if true_light == "red" and f.sensors.stop_line_crossed and f.pose.v > 1.0:
        found.append(Incident(
            "red_light_run", f.seq,
            f"crossed stop line at {f.pose.v * 3.6:.0f} km/h "
            f"while perception reported '{f.sensors.light_state}'"))
    # No reference path means lateral_offset is a placeholder, so there is
    # nothing to compare against and a 0.0 must not be read as a held lane.
    # The only manoeuvre without one is parking, which leaves the lane
    # deliberately. This is a gap by construction, so it is recorded in every
    # frame rather than inferred: lane_ref is committed in sensors.json, and
    # an empty one is visible in the history as the reason off_road was not
    # evaluated.
    if (f.sensors.lane_ref
            and abs(f.sensors.lateral_offset) > MAX_LAT_OFFSET):
        found.append(Incident("off_road", f.seq,
                              f"lateral offset {f.sensors.lateral_offset:.2f} m "
                              f"from {f.sensors.lane_ref}"))
    # A kind missing from SEVERITY sorts last rather than raising: a new
    # detector that nobody ranked is still an incident.
    found.sort(key=lambda inc: SEVERITY.index(inc.kind)
               if inc.kind in SEVERITY else len(SEVERITY))
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
    turn is 15.93 m of chord against 18.45 m of driven path, so the clearance
    test was asking for 2.5 m less room than the manoeuvre actually needs.

    This returns 17.18 m for that case, not 18.45. The model is a single
    constant-radius arc and the real return is an S: out of the turn, then
    back into the lane. So it recovers about half the deficit and still
    understates the path by ~1.3 m. That residual is deliberately left to
    MIN_CLEARANCE rather than papered over with a fudge factor -- a made-up
    multiplier tuned on this one turn would be wrong on every other geometry,
    and wrong in the unsafe direction on a tighter one. The number this
    returns is a lower bound on the path, which is the only honest thing a
    one-arc model can be.
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
        # _state_at already distinguishes a missing parent from a parent whose
        # state.json is unreadable or poseless. Replacing all three with "no
        # parent commit" told an operator to look for a root commit while the
        # real fault was a corrupt frame.
        return RevertVerdict(
            False, f"cannot read the parent frame to revert to: {err}")
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
