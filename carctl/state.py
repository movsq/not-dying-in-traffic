"""Vehicle state. This is what gets committed every 100 ms."""
from __future__ import annotations
from dataclasses import dataclass, field, asdict
import json

# Frozen for a reason: a frame is an immutable observation. The control loop
# builds a new one each tick; the committer only ever reads.
@dataclass(frozen=True)
class Pose:
    x: float      # m, map frame
    y: float      # m, map frame
    heading: float  # rad, 0 = east
    v: float      # m/s
    steer: float  # rad, front wheel


@dataclass(frozen=True)
class Sensors:
    lidar_min_range: float   # m, nearest return in the forward cone
    lidar_min_bearing: float # rad
    light_state: str         # green | amber | red | none
    light_distance: float    # m, negative once past the stop line
    lateral_offset: float    # m from lane centre, + is left
    wheel_slip: float        # 0..1
    imu_accel_z: float       # m/s^2, spikes on curb strikes
    lidar_min_range_rear: float = 40.0
    # True only on the tick the stop line passes under the car. light_distance
    # stays negative for the rest of the drive, so anything derived from its
    # sign latches and never recovers.
    stop_line_crossed: bool = False
    # Which reference path lateral_offset was measured against: a lane centre
    # ("Ječná:1") or a path through a junction ("Vinohradská:0>Ječná:0").
    # Empty means there was no reference, so lateral_offset is a placeholder
    # rather than a reading. The number alone cannot say which of those it is,
    # and a fabricated 0.0 reads as a perfectly held lane, which is how a
    # whole junction went by with off_road unable to fire.
    lane_ref: str = ""


@dataclass(frozen=True)
class Actuators:
    throttle: float  # 0..1
    brake: float     # 0..1
    steer_cmd: float # rad


# Every path a frame commit's tree is allowed to contain, and the only place
# that list exists. gitstore has to recognise a tree somebody else wrote and
# used to restate the four names on its own side; two literals that have to
# agree forever agree only until one of them is edited, and the side that gets
# left behind is the side that refuses -- or worse, accepts -- every frame of
# a drive. Frame.tree() below is built from this tuple, so they cannot drift.
FRAME_PATHS = ("actuators.json", "models.json", "sensors.json", "state.json")


@dataclass(frozen=True)
class Frame:
    seq: int
    t_mono_ns: int        # monotonic, the only clock control logic trusts
    t_wall_s: int         # wall clock, for the commit timestamp only
    pose: Pose
    sensors: Sensors
    actuators: Actuators
    road: str             # current street name, for the commit message
    maneuver: str         # classifier output, drives the message verb
    reversible: bool      # can the physical world be walked back from here?
    checkpoints: dict[str, str] = field(default_factory=dict)

    def __post_init__(self):
        # frozen=True stops rebinding, not mutation, and this dict arrives by
        # reference from the plant. The committer thread can read it up to
        # QUEUE_DEPTH * DT = 51 s after capture, so an in-place edit would
        # retroactively rewrite frames already queued and attribute pre-swap
        # frames to the post-swap checkpoint, destroying the one piece of
        # evidence models.json exists to carry. Copy at the boundary.
        object.__setattr__(self, "checkpoints", dict(self.checkpoints))

    def state_json(self) -> str:
        return json.dumps(
            {"seq": self.seq, "t_mono_ns": self.t_mono_ns,
             "pose": asdict(self.pose), "road": self.road,
             "maneuver": self.maneuver, "reversible": self.reversible},
            indent=1, sort_keys=True) + "\n"

    def sensors_json(self) -> str:
        return json.dumps(asdict(self.sensors), indent=1, sort_keys=True) + "\n"

    def actuators_json(self) -> str:
        return json.dumps(asdict(self.actuators), indent=1, sort_keys=True) + "\n"

    def models_json(self) -> str:
        # One subsystem per line, deliberately. `git blame -L n,n models.json`
        # then resolves to the commit that last swapped that checkpoint, which
        # is the whole point of the file existing.
        # json.dumps per key and value, not f-string interpolation: a
        # checkpoint id containing a quote or a backslash used to emit a file
        # that is not JSON, which every reader downstream of blame parses.
        lines = ["{"]
        items = sorted(self.checkpoints.items())
        for i, (k, v) in enumerate(items):
            comma = "," if i < len(items) - 1 else ""
            lines.append(f'  {json.dumps(k)}: {json.dumps(v)}{comma}')
        lines.append("}")
        return "\n".join(lines) + "\n"

    def tree(self) -> dict[str, str]:
        """The complete tree of a frame commit: FRAME_PATHS, nothing else.

        Derived from FRAME_PATHS rather than spelled out again, because the
        two are one fact: `sensors.json` is written by sensors_json(), and a
        fifth file has to be added to the tuple and to a method with the
        matching name before it can appear in a tree at all. A hand-written
        dict here would let the writer and the checker disagree about what a
        frame is, silently, in the direction of whichever one was edited last.
        """
        return {path: getattr(self, path[:-len(".json")] + "_json")()
                for path in FRAME_PATHS}
