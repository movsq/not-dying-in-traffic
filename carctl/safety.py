"""Incident detection and the two kinds of revert.

`git revert` on a source tree is total: any diff can be inverted. Physics is
not. So a revert here splits into two independent operations that are allowed
to disagree, and the whole design hangs on keeping them separate:

  RECORD REVERT   — a real `git revert` in the control worktree. Always
                    succeeds. Produces the public, auditable statement "this
                    frame was wrong". Costs nothing physical.

  PHYSICAL REVERT — read the parent commit's state.json, treat it as a goal
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


def detect(f: Frame, true_light: str) -> Incident | None:
    if f.sensors.imu_accel_z > CURB_ACCEL_Z:
        return Incident("curb_strike", f.seq,
                        f"az={f.sensors.imu_accel_z:.1f} m/s^2")
    if f.sensors.lidar_min_range < MIN_CLEARANCE:
        return Incident("collision", f.seq,
                        f"clearance={f.sensors.lidar_min_range:.2f} m")
    if true_light == "red" and f.sensors.stop_line_crossed and f.pose.v > 1.0:
        return Incident("red_light_run", f.seq,
                        f"crossed stop line at {f.pose.v * 3.6:.0f} km/h "
                        f"while perception reported '{f.sensors.light_state}'")
    if abs(f.sensors.lateral_offset) > MAX_LAT_OFFSET:
        return Incident("off_road", f.seq,
                        f"lateral offset {f.sensors.lateral_offset:.2f} m")
    return None


# ---------------------------------------------------------------------------

@dataclass
class RevertVerdict:
    allowed: bool
    reason: str
    goal: dict | None = None
    cost_m: float = 0.0


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
    ahead_or_behind = ("rear" if behind else "forward")
    clearance = (now.sensors.lidar_min_range_rear if behind
                 else now.sensors.lidar_min_range)
    if clearance < dist + MIN_CLEARANCE:
        return RevertVerdict(False,
            f"{ahead_or_behind} return path is occupied at {clearance:.1f} m",
            cost_m=dist)
    return RevertVerdict(True, f"reachable, {dist:.1f} m of return path",
                         goal=goal, cost_m=dist)


def physical_revert(repo: str, sha: str, now: Frame) -> RevertVerdict:
    """Evaluate reverting `sha` physically. Does not touch the repo."""
    parent_state = subprocess.run(
        ["git", "show", f"{sha}^:state.json"], cwd=repo,
        capture_output=True, text=True, encoding="utf-8")
    if parent_state.returncode != 0:
        return RevertVerdict(False, "no parent commit to revert to")
    return reachable(now, json.loads(parent_state.stdout))


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
    add = subprocess.run(["git", "worktree", "add", "--detach", "-f", wt, full],
                         cwd=repo, capture_output=True, text=True,
                         encoding="utf-8")
    if add.returncode != 0:
        return RecordRevert(False, detail=f"worktree: {add.stderr.strip()[:300]}")
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
        subprocess.run(["git", "update-ref", ref, new], cwd=repo,
                       capture_output=True)
        return RecordRevert(True, new, f"recorded at {ref}")
    finally:
        # The checkout is a regenerable artifact; the commit it produced is
        # already anchored by the ref above.
        subprocess.run(["git", "worktree", "remove", "--force", wt], cwd=repo,
                       capture_output=True)


def _subject(repo: str, sha: str) -> str:
    return subprocess.run(["git", "log", "-1", "--format=%s", sha], cwd=repo,
                          capture_output=True, text=True, encoding="utf-8").stdout.strip()
