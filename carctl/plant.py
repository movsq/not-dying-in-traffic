"""Kinematic bicycle model + a toy world, so the loop has something to commit."""
from __future__ import annotations
import math, time
from .state import Pose, Sensors, Actuators, Frame
from . import safety

WHEELBASE = 2.7  # m
DT = 0.1         # s, one tick, one commit
DT_NS = 100_000_000   # the same tick as an exact integer of nanoseconds

# The scripted drive. (t_start, road, maneuver, steer_cmd, target_speed)
SCRIPT = [
    (0.0,  "Vinohradská", "cruise",      0.00, 13.9),
    (1.5,  "Vinohradská", "brake",       0.00,  6.0),
    (2.5,  "Hlavní"     , "turn_left",   0.30,  5.0),
    (4.5,  "Hlavní",      "turn_left",   0.10,  6.0),
    (5.5,  "Hlavní",      "cruise",      0.00, 12.0),
    (7.0,  "Hlavní",      "cruise",      0.00, 13.5),
    (9.0,  "Hlavní",      "lane_change", -0.06, 12.0),
    (10.0, "Hlavní",      "cruise",      0.02, 12.0),
    (11.5, "Hlavní",      "brake",       0.00,  2.0),
    (12.5, "Hlavní",      "park",       -0.35,  1.5),
]

# The stop line the car is going to blow through, because the perception
# checkpoint below mis-classifies amber as green under low sun.
STOP_LINE_S = 108.0   # m along Hlavní
LIGHT_RED_AT = 10.2    # s

CHECKPOINTS = {
    "controller": "ckpt-controller-2026.03.01-0b12",
    "perception": "ckpt-perception-2026.05.30-1e4d",
    "planner":    "ckpt-planner-2026.08.19-3de0",
    "prediction": "ckpt-prediction-2026.06.02-77c1",
}

# Shadow-mode checkpoint promoted mid-drive by the OTA agent. This is the
# event `git blame` has to find later.
OTA_SWAP_AT = 6.0
OTA_SWAP = ("perception", "ckpt-perception-2026.07.14-a91f")


def _script_at(t: float):
    row = SCRIPT[0]
    for r in SCRIPT:
        if t >= r[0]:
            row = r
    return row


class Plant:
    """Integrates the vehicle forward. Owns no git state whatsoever."""

    def __init__(self, checkpoints: dict[str, str] | None = None):
        self.pose = Pose(x=0.0, y=0.0, heading=0.0, v=13.9, steer=0.0)
        self.t = 0.0
        self.seq = 0
        self.s_along = 0.0  # arc length, used to place the stop line
        self.prev_light_dist = STOP_LINE_S
        self.checkpoints = dict(checkpoints or CHECKPOINTS)

    def _light(self) -> tuple[str, float]:
        true_state = "red" if self.t >= LIGHT_RED_AT else "green"
        dist = STOP_LINE_S - self.s_along
        # The bug lives here: this perception checkpoint reports the light as
        # green for 1.2 s after it turns red. Everything downstream is correct.
        if self.checkpoints["perception"] == "ckpt-perception-2026.07.14-a91f":
            if true_state == "red" and self.t < LIGHT_RED_AT + 1.2:
                return "green", dist
        return true_state, dist

    def true_light(self) -> str:
        return "red" if self.t >= LIGHT_RED_AT else "green"

    def step(self) -> Frame:
        if self.t >= OTA_SWAP_AT and self.checkpoints[OTA_SWAP[0]] != OTA_SWAP[1]:
            self.checkpoints = dict(self.checkpoints)
            self.checkpoints[OTA_SWAP[0]] = OTA_SWAP[1]

        t0, road, maneuver, steer_cmd, v_target = _script_at(self.t)

        # Longitudinal: crude P controller onto the scripted target speed.
        err = v_target - self.pose.v
        throttle = max(0.0, min(1.0, err / 6.0))
        brake = max(0.0, min(1.0, -err / 6.0))
        a = 3.0 * throttle - 6.0 * brake

        # Lateral: first-order steering actuator lag.
        steer = self.pose.steer + (steer_cmd - self.pose.steer) * 0.35

        v = max(0.0, self.pose.v + a * DT)
        heading = self.pose.heading + (v / WHEELBASE) * math.tan(steer) * DT
        x = self.pose.x + v * math.cos(heading) * DT
        y = self.pose.y + v * math.sin(heading) * DT
        self.s_along += v * DT
        self.pose = Pose(x=x, y=y, heading=heading, v=v, steer=steer)

        light_state, light_dist = self._light()
        # A parked van appears at 12.0 s; that is what the parking manoeuvre
        # is squeezing in behind.
        lidar = 40.0
        if self.t >= 11.8:
            lidar = max(1.4, 14.0 - (self.t - 11.8) * 6.0)
        curb = 0.0
        if maneuver == "park" and self.t >= 13.1:
            curb = 41.0  # curb strike: irreversible, and the IMU says so

        # The crossing is one tick, not every tick after it.
        crossed_now = self.prev_light_dist > 0 >= light_dist
        self.prev_light_dist = light_dist

        # Something is behind us during the parking manoeuvre. The forward
        # cone cannot see it, which is the whole reason this reading exists.
        lidar_rear = 40.0
        if maneuver == "park":
            lidar_rear = max(0.8, 5.0 - (self.t - 12.5) * 3.0)

        sensors = Sensors(
            lidar_min_range=lidar,
            lidar_min_bearing=0.0 if lidar > 20 else -0.4,
            light_state=light_state,
            light_distance=light_dist,
            lateral_offset=math.sin(self.t * 1.7) * 0.12,
            wheel_slip=0.02 if brake < 0.5 else 0.11,
            imu_accel_z=9.81 + curb,
            lidar_min_range_rear=lidar_rear,
            stop_line_crossed=crossed_now and self.true_light() == "red",
        )
        actuators = Actuators(throttle=round(throttle, 3),
                              brake=round(brake, 3),
                              steer_cmd=round(steer_cmd, 3))

        # Reversibility is decided here, at capture time, not at revert time.
        # Anything that dissipated energy into the world is a one-way door.
        # Read from the same constants msgen names the reason with: testing
        # `curb == 0.0` here against `imu_accel_z > 20` there disagreed for
        # every impulse in 0 < curb <= 10.19, which committed a frame as
        # irreversible with the reason "unknown", and the float equality made
        # any filtered or noisy accel reading irreversible outright.
        reversible = (sensors.imu_accel_z <= safety.CURB_ACCEL_Z
                      and lidar > safety.IRREVERSIBLE_CLEARANCE
                      and not sensors.stop_line_crossed)

        frame = Frame(
            seq=self.seq,
            t_mono_ns=self.seq * DT_NS,
            t_wall_s=int(time.time()),
            pose=self.pose, sensors=sensors, actuators=actuators,
            road=road, maneuver=maneuver, reversible=reversible,
            checkpoints=self.checkpoints,
        )
        self.seq += 1
        # Derived from the tick count, never accumulated. `self.t += DT`
        # drifts low, because 0.1 is not representable in binary: at nominal
        # tick 60 t was 5.999999999999995, so every scripted threshold fired
        # one tick late (the OTA swap at 61, the light at 103, the park at
        # 126) and t_mono_ns came out 13899999999 where seq 139 should be
        # 13900000000 -- in the clock state.py calls the only one control
        # logic trusts.
        self.t = self.seq * DT
        return frame
