"""Commit message generation.

Conventional Commits, because the history is public and people will grep it.
The subject is derived from the manoeuvre classifier; the trailers are what
make the history machine-readable (bisect, blame, incident tooling).
"""
from __future__ import annotations
from .state import Frame
from . import safety

_VERB = {
    "cruise":      ("feat",  "continued along {road} at {kmh:.0f} km/h"),
    "brake":       ("fix",   "slowed to {kmh:.0f} km/h on {road}"),
    "turn_left":   ("feat",  "turned left onto {road}"),
    "turn_right":  ("feat",  "turned right onto {road}"),
    "lane_change": ("refactor", "moved to the {side} lane of {road}"),
    "park":        ("chore", "parallel parking on {road}"),
    "stop":        ("chore", "came to a stop on {road}"),
}


def subject(f: Frame) -> str:
    kind, tmpl = _VERB.get(f.maneuver, ("chore", "held state on {road}"))
    # Positive steer increases heading, and heading 0 is east, so positive is
    # a left turn. The template said "left" unconditionally, so the scripted
    # lane change (steer_cmd -0.06, heading 1.5892 -> 1.4614, a move to the
    # right) committed the opposite of what the car did, into a history whose
    # entire point is that people will grep it.
    side = "left" if f.actuators.steer_cmd > 0 else "right"
    body = tmpl.format(road=f.road, kmh=f.pose.v * 3.6, side=side)
    # `!` is the Conventional Commits breaking-change marker. Here it means
    # exactly that: this frame cannot be reverted, so it breaks the history's
    # otherwise-invertible property. Downstream tooling filters on it.
    bang = "" if f.reversible else "!"
    return f"{kind}{bang}: {body}"


def message(f: Frame) -> str:
    lines = [subject(f), ""]
    lines.append(f"Seq: {f.seq}")
    # Six decimals, not two. publish scrubs this trailer independently of the
    # blob, so a 2 dp value could snap into a different 25 m cell than the
    # full-precision pose did: a true x of 37.4999 prints as "37.50", which
    # snaps to 50.0 in the message and 25.0 in the tree. Both are multiples of
    # 25, so nothing downstream complained, but publishing two different cells
    # for one frame pins x to a 1 cm window, three orders of magnitude finer
    # than the grid. main is private and public snaps this either way, so the
    # extra precision here costs nothing.
    lines.append(f"Pose: {f.pose.x:.6f},{f.pose.y:.6f} @ {f.pose.heading:.4f} rad")
    lines.append(f"Speed: {f.pose.v * 3.6:.1f} km/h")
    lines.append(f"Reversible: {'yes' if f.reversible else 'no'}")
    if not f.reversible:
        lines.append(f"Irreversible-Reason: {_why(f)}")
    lines.append(f"Checkpoint-Planner: {f.checkpoints.get('planner', '?')}")
    return "\n".join(lines) + "\n"


def _why(f: Frame) -> str:
    # The same constants plant.py decides the flag with. As separate literals
    # the two disagreed, and "unknown" was the visible symptom: a frame marked
    # irreversible for a reason this function could not name.
    if f.sensors.imu_accel_z > safety.CURB_ACCEL_Z:
        return "vertical-accel-spike"
    if f.sensors.lidar_min_range <= safety.IRREVERSIBLE_CLEARANCE:
        return "object-inside-braking-distance"
    if f.sensors.stop_line_crossed:
        return "stop-line-crossed"
    return "unknown"
