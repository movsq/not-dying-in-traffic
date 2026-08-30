"""Kinematic bicycle model + a toy world, so the loop has something to commit."""
from __future__ import annotations
import math, time
from .state import Pose, Sensors, Actuators, Frame
from . import safety

WHEELBASE = 2.7  # m
DT = 0.1         # s, one tick, one commit
DT_NS = 100_000_000   # the same tick as an exact integer of nanoseconds

LANE_WIDTH = 3.2   # m
N_LANES = 2        # per road, numbered rightward from 0

# Each street is a straight centreline: a point on it and the heading of
# travel. `lateral_offset` is the signed distance from the centre of the
# nearest lane the car is allowed to be in, positive to the left. Before this
# it was a decorative sine of amplitude 0.12 m against an off-road threshold
# of 1.75 m, so `off_road` was one of four incident kinds that nothing could
# trigger, and the planner was the one subsystem blame never got asked about.
ROADS = {
    "Vinohradská": ((0.0, 0.0), 0.0),
    "Hlavní":      ((41.8, 0.0), math.pi / 2),
}

# A junction is where a straight centreline per street runs out. The two turn
# rows of the script used to carry no path at all, so the plant reported a
# lateral offset of 0.0 through the whole turn, and off_road, the only
# incident kind that reads that number, could not fire in the one place a
# lane departure is most likely and most expensive. A junction gets its own
# reference path: an arc tangent to the centre of the lane the car enters on
# and to the centre of the lane it leaves on.
#
#   name -> (entry road, entry lane, exit road, exit lane)
JUNCTIONS = {
    "Vinohradská>Hlavní": ("Vinohradská", 0, "Hlavní", 0),
}

# Radius of that arc at the lane centre. A turn has to be driven at some
# radius, and this one is picked to sit inside the box the two streets cross
# in: at 10 m the arc leaves Vinohradská 10 m short of the intersection and
# joins Hlavní 10 m past it.
JUNCTION_RADIUS = 10.0   # m

# The scripted drive.
#   (t_start, road, maneuver, steer_cmd, target_speed, path)
# `path` is the reference the planner is asking the car to hold, and the plant
# steers to hold it: a number is that lane of `road`, a string is that entry
# in JUNCTIONS. `None` hands steering back to steer_cmd and reports no lateral
# offset at all, which now means exactly one manoeuvre, parking, which leaves
# the lane on purpose and has no reference left to depart from.
SCRIPT = [
    (0.0,  "Vinohradská", "cruise",      0.00, 13.9, 0),
    (1.5,  "Vinohradská", "brake",       0.00,  6.0, 0),
    (2.5,  "Hlavní"     , "turn_left",   0.00,  5.0, "Vinohradská>Hlavní"),
    (4.5,  "Hlavní",      "turn_left",   0.00,  6.0, "Vinohradská>Hlavní"),
    (5.5,  "Hlavní",      "cruise",      0.00, 12.0, 0),
    (7.0,  "Hlavní",      "cruise",      0.00, 13.5, 0),
    # The planner asks for lane 2.3 on a two lane road. This is the planner
    # fault the drive exists to exercise, and it is the mirror of the
    # perception one: a bad target, held long enough to leave the roadway,
    # then corrected. `off_road` fires while the car is out there and
    # OWNER maps it to the planner.
    (9.0,  "Hlavní",  "lane_change_right", 0.00, 12.0, 2.6),
    (10.6, "Hlavní",      "cruise",      0.00, 12.0, 1),
    (11.5, "Hlavní",      "brake",       0.00,  2.0, 1),
    (12.5, "Hlavní",      "park",       -0.35,  1.5, None),
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


def _wrap(a: float) -> float:
    """An angle folded into (-pi, pi]."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def line_offset(px: float, py: float, th: float, x: float, y: float) -> float:
    """Signed distance left of the line through (px, py) with heading th."""
    return -(x - px) * math.sin(th) + (y - py) * math.cos(th)


def lane_offset(road: str, x: float, y: float) -> float:
    """Signed distance left of the road centreline, in metres."""
    (px, py), th = ROADS[road]
    return line_offset(px, py, th, x, y)


def lane_centre(road: str, lane: float) -> tuple[tuple[float, float], float]:
    """A point on the centre of `lane`, and the heading of travel there.

    Lane k sits k * LANE_WIDTH to the right of the road centreline, so at a
    left offset of -k * LANE_WIDTH.
    """
    (px, py), th = ROADS[road]
    d = -lane * LANE_WIDTH
    return (px - d * math.sin(th), py + d * math.cos(th)), th


def nearest_lane_error(road: str, x: float, y: float) -> float:
    """Signed distance from the centre of the nearest lane we may be in.

    Measuring against the nearest of them is what makes the number mean "how
    far out of a lane are you" rather than "how far from the middle of the
    road", which a car correctly in lane 1 is always a full lane width from.
    """
    off = lane_offset(road, x, y)
    return min((off + k * LANE_WIDTH for k in range(N_LANES)), key=abs)


class LanePath:
    """The reference path on an ordinary street: one straight lane centre."""

    def __init__(self, road: str, lane: float):
        self.road, self.lane = road, lane
        self.name = f"{road}:{lane:g}"
        (self.px, self.py), self.th = lane_centre(road, lane)

    def at(self, x: float, y: float) -> tuple[float, float, float]:
        """(signed offset left of the path, path heading, signed curvature)."""
        return line_offset(self.px, self.py, self.th, x, y), self.th, 0.0

    def departure(self, x: float, y: float) -> float:
        """What off_road is measured against.

        Not the offset from the commanded lane: a car that has drifted a lane
        over is in the wrong place, but it is not off the road, and the
        threshold this feeds is a road-edge threshold.
        """
        return nearest_lane_error(self.road, x, y)


class JunctionPath:
    """The reference path through a junction.

    An arc tangent to the centre of the entry lane and the centre of the exit
    lane, with those two lane centres serving as its own extensions past the
    tangent points. That last part is what makes the offset continuous across
    the whole manoeuvre: the arc is tangent to both lines, so a car that has
    not reached the turn yet, or has finished it, is measured against exactly
    the straight lane it is on.
    """

    def __init__(self, entry_road: str, entry_lane: float,
                 exit_road: str, exit_lane: float,
                 radius: float = JUNCTION_RADIUS):
        self.name = f"{entry_road}:{entry_lane:g}>{exit_road}:{exit_lane:g}"
        self.r = radius
        (ax, ay), th0 = lane_centre(entry_road, entry_lane)
        (bx, by), th1 = lane_centre(exit_road, exit_lane)
        d0 = (math.cos(th0), math.sin(th0))
        d1 = (math.cos(th1), math.sin(th1))
        denom = d0[0] * d1[1] - d0[1] * d1[0]
        if abs(denom) < 1e-9:
            raise ValueError(f"{self.name}: the two lane centres never meet, "
                             "so there is no arc joining them")
        # Where the two lane centres cross, then back off along each of them
        # by the tangent length an arc of this radius needs to meet both.
        k = ((bx - ax) * d1[1] - (by - ay) * d1[0]) / denom
        ix, iy = ax + k * d0[0], ay + k * d0[1]
        self.turn = _wrap(th1 - th0)
        self.sense = 1.0 if self.turn > 0 else -1.0
        t = radius * math.tan(abs(self.turn) / 2)
        self.entry = (ix - t * d0[0], iy - t * d0[1], th0)
        self.exit = (ix + t * d1[0], iy + t * d1[1], th1)
        # The centre of the arc is one radius to the left of the entry tangent
        # point on a left turn, and to the right on a right turn.
        self.cx = self.entry[0] - self.sense * radius * math.sin(th0)
        self.cy = self.entry[1] + self.sense * radius * math.cos(th0)
        self.phi0 = math.atan2(self.entry[1] - self.cy, self.entry[0] - self.cx)
        phi1 = math.atan2(self.exit[1] - self.cy, self.exit[0] - self.cx)
        self.sweep = (self.sense * (phi1 - self.phi0)) % (2 * math.pi)

    def at(self, x: float, y: float) -> tuple[float, float, float]:
        """(signed offset left of the path, path heading, signed curvature)."""
        phi = math.atan2(y - self.cy, x - self.cx)
        # How far round the arc we are, measured in the direction of travel.
        u = (self.sense * (phi - self.phi0)) % (2 * math.pi)
        if u <= self.sweep:
            d = math.hypot(x - self.cx, y - self.cy)
            # On a left turn the inside of the curve is to the left, so
            # closing on the centre is drifting left; on a right turn it is
            # the other way round. Hence the sense factor rather than a bare
            # r - d, which reports a right turn's departures backwards.
            return (self.sense * (self.r - d),
                    _wrap(phi + self.sense * math.pi / 2),
                    self.sense / self.r)
        # Short of one end or past the other, so we are on a lane centre.
        # Wrapping makes those two the same interval, so pick whichever end
        # is nearer in angle.
        px, py, th = (self.entry if (2 * math.pi - u) < (u - self.sweep)
                      else self.exit)
        return line_offset(px, py, th, x, y), th, 0.0

    def departure(self, x: float, y: float) -> float:
        """What off_road is measured against.

        A junction has one legal path for a given manoeuvre, not a set of
        parallel lanes to take the nearest of, so this is the offset from the
        arc itself.
        """
        return self.at(x, y)[0]


_JUNCTION_PATHS = {name: JunctionPath(*spec)
                   for name, spec in JUNCTIONS.items()}
_LANE_PATHS: dict[tuple[str, float], LanePath] = {}


def path_for(road: str, spec):
    """Resolve a SCRIPT row's path column. None means there is no reference.

    Cached, because the arc costs a dozen trig calls to build and the control
    loop asks for its path ten times a second.
    """
    if spec is None:
        return None
    if isinstance(spec, str):
        return _JUNCTION_PATHS[spec]
    key = (road, spec)
    if key not in _LANE_PATHS:
        _LANE_PATHS[key] = LanePath(road, spec)
    return _LANE_PATHS[key]


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

    def _path_steer(self, path) -> float:
        """Steer to hold the reference path the planner asked for."""
        off, th, kappa = path.at(self.pose.x, self.pose.y)
        head = _wrap(th - self.pose.heading)
        # Curvature feedforward, zero on a straight lane and so a no-op there.
        # Without it the error terms have to manufacture the whole steady
        # state steer angle out of tracking error, and holding a 10 m radius
        # arc needs atan(L/R) = 0.26 rad of it: the car would corner
        # permanently wide by roughly the amount of error that buys, which is
        # a lane departure the plant would then have reported as the truth.
        return max(-0.45, min(0.45, math.atan(WHEELBASE * kappa)
                              + 0.05 * -off + 0.9 * head))

    def true_light(self) -> str:
        return "red" if self.t >= LIGHT_RED_AT else "green"

    def step(self) -> Frame:
        if self.t >= OTA_SWAP_AT and self.checkpoints[OTA_SWAP[0]] != OTA_SWAP[1]:
            self.checkpoints = dict(self.checkpoints)
            self.checkpoints[OTA_SWAP[0]] = OTA_SWAP[1]

        t0, road, maneuver, steer_cmd, v_target, spec = _script_at(self.t)
        path = path_for(road, spec)

        # Longitudinal: crude P controller onto the scripted target speed.
        err = v_target - self.pose.v
        throttle = max(0.0, min(1.0, err / 6.0))
        brake = max(0.0, min(1.0, -err / 6.0))
        a = 3.0 * throttle - 6.0 * brake

        # Lateral: hold the reference path where there is one, otherwise take
        # the scripted steer. Open loop steering cannot hold a lane, and the
        # old script did not try: it wandered 3.4 m of x across a street that
        # is meant to be straight, which a lane model makes impossible to
        # ignore. The turn is closed loop on the junction arc for the same
        # reason, and a stronger one: an open loop turn has no reference, so
        # the offset it was measured against would have been invented to
        # match it.
        cmd = steer_cmd if path is None else self._path_steer(path)
        # First-order steering actuator lag.
        steer = self.pose.steer + (cmd - self.pose.steer) * 0.35

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
        # Something is already occupying the space the planner steered into.
        # The excursion is what brings the car inside MIN_CLEARANCE of it, so
        # the near miss is downstream of the planner fault and prediction owns
        # it. Without this, `collision` is the incident kind nothing triggers:
        # a successful parallel park used to stand in for one, which is the
        # false positive PARK_CLEARANCE exists to stop.
        if 10.0 <= self.t < 11.1:
            lidar = max(1.2, 12.0 - (self.t - 10.0) * 14.0)
        if self.t >= 11.8:
            lidar = max(1.4, 14.0 - (self.t - 11.8) * 6.0)
        curb = 0.0
        if maneuver == "park" and self.t >= 13.1:
            curb = 41.0  # curb strike: irreversible, and the IMU says so

        # With no reference path there is nothing honest to report, and that
        # is now only the parking manoeuvre, which leaves the lane on purpose.
        # The 0.0 is a placeholder, not a measurement of being centred, so it
        # travels with an empty lane_ref and safety.detect() declines to judge
        # it rather than reading it as a perfectly held lane.
        lat = 0.0 if path is None else path.departure(x, y)

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
            lateral_offset=lat,
            wheel_slip=0.02 if brake < 0.5 else 0.11,
            imu_accel_z=9.81 + curb,
            lidar_min_range_rear=lidar_rear,
            stop_line_crossed=crossed_now and self.true_light() == "red",
            lane_ref="" if path is None else path.name,
        )
        actuators = Actuators(throttle=round(throttle, 3),
                              brake=round(brake, 3),
                              steer_cmd=round(cmd, 3))

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
