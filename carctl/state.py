"""Vehicle state — the thing that gets committed every 100 ms."""
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


@dataclass(frozen=True)
class Actuators:
    throttle: float  # 0..1
    brake: float     # 0..1
    steer_cmd: float # rad


@dataclass(frozen=True)
class Frame:
    seq: int
    t_mono_ns: int        # monotonic clock — the only clock control logic trusts
    t_wall_s: int         # wall clock, for the commit timestamp only
    pose: Pose
    sensors: Sensors
    actuators: Actuators
    road: str             # current street name, for the commit message
    maneuver: str         # classifier output, drives the message verb
    reversible: bool      # can the physical world be walked back from here?
    checkpoints: dict[str, str] = field(default_factory=dict)

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
        lines = ["{"]
        items = sorted(self.checkpoints.items())
        for i, (k, v) in enumerate(items):
            comma = "," if i < len(items) - 1 else ""
            lines.append(f'  "{k}": "{v}"{comma}')
        lines.append("}")
        return "\n".join(lines) + "\n"

    def tree(self) -> dict[str, str]:
        return {"state.json": self.state_json(),
                "sensors.json": self.sensors_json(),
                "actuators.json": self.actuators_json(),
                "models.json": self.models_json()}
