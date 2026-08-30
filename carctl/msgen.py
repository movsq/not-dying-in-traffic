"""Commit message generation.

Conventional Commits, because the history is public and people will grep it.
The subject is derived from the manoeuvre classifier; the trailers are what
make the history machine-readable (bisect, blame, incident tooling).
"""
from __future__ import annotations
from .state import Frame

_VERB = {
    "cruise":      ("feat",  "continued along {road} at {kmh:.0f} km/h"),
    "brake":       ("fix",   "slowed to {kmh:.0f} km/h on {road}"),
    "turn_left":   ("feat",  "turned left onto {road}"),
    "turn_right":  ("feat",  "turned right onto {road}"),
    "lane_change": ("refactor", "moved to the left lane of {road}"),
    "park":        ("chore", "parallel parking on {road}"),
    "stop":        ("chore", "came to a stop on {road}"),
}


def subject(f: Frame) -> str:
    kind, tmpl = _VERB.get(f.maneuver, ("chore", "held state on {road}"))
    body = tmpl.format(road=f.road, kmh=f.pose.v * 3.6)
    # `!` is the Conventional Commits breaking-change marker. Here it means
    # exactly that: this frame cannot be reverted, so it breaks the history's
    # otherwise-invertible property. Downstream tooling filters on it.
    bang = "" if f.reversible else "!"
    return f"{kind}{bang}: {body}"


def message(f: Frame) -> str:
    lines = [subject(f), ""]
    lines.append(f"Seq: {f.seq}")
    lines.append(f"Pose: {f.pose.x:.2f},{f.pose.y:.2f} @ {f.pose.heading:.4f} rad")
    lines.append(f"Speed: {f.pose.v * 3.6:.1f} km/h")
    lines.append(f"Reversible: {'yes' if f.reversible else 'no'}")
    if not f.reversible:
        lines.append(f"Irreversible-Reason: {_why(f)}")
    lines.append(f"Checkpoint-Planner: {f.checkpoints.get('planner', '?')}")
    return "\n".join(lines) + "\n"


def _why(f: Frame) -> str:
    if f.sensors.imu_accel_z > 20:
        return "vertical-accel-spike"
    if f.sensors.lidar_min_range <= 2.0:
        return "object-inside-braking-distance"
    if f.sensors.light_distance <= 0:
        return "stop-line-crossed"
    return "unknown"
