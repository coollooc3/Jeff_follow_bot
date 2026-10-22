"""LEAP planar stop checks. No navigation or obstacle bypass is implemented."""
from dataclasses import dataclass
import math

# Same Stage1 base-frame envelope; laser/axle poses agree with leap_ros.urdf.
# These are NOT complete wheel collision meshes or measured stopping distances.
X_MIN, X_MAX, HALF_WIDTH = -.08325, .07489, .081
LASER_X, LASER_Y, AXLE_X = -.0227, .000062, -.05075
TURN_RADIUS = max(math.hypot(x-c, HALF_WIDTH) for x in (X_MIN, X_MAX)
                  for c in (0., AXLE_X)) + .05


@dataclass
class InputAge:
    good: float = None
    stamp: int = None
    issue: str = 'missing'

    def accept(self, now, stamp, publication_age, valid, timeout):
        if not valid or stamp <= 0 or not -.05 <= publication_age <= timeout:
            self.issue = 'invalid'
            return False
        if self.stamp is not None and stamp <= self.stamp:
            return False  # Repeated publications cannot refresh a stopped source.
        self.good, self.stamp, self.issue = now, stamp, ''
        return True

    def reason(self, now, timeout):
        if self.issue:
            return self.issue
        return '' if self.good is not None and 0 <= now-self.good <= timeout else 'timeout'


def scan_points(ranges, angle_min, angle_max, increment, minimum, maximum, frame):
    if (frame != 'laser_frame' or len(ranges) < 3
            or not all(math.isfinite(v) for v in (angle_min, angle_max, increment, minimum, maximum))
            or increment == 0 or not 0 <= minimum < maximum
            or abs(angle_max-angle_min) < math.pi*1.9
            or abs(angle_max-angle_min-(len(ranges)-1)*increment) > 1.5*abs(increment)):
        return None
    usable = front_usable = front_total = 0
    points = []
    for i, distance in enumerate(ranges):
        angle = angle_min+i*increment
        in_front = abs(math.atan2(math.sin(angle), math.cos(angle))) <= math.pi/6
        valid = distance == math.inf or math.isfinite(distance) and minimum <= distance <= maximum
        usable += int(valid)
        front_total += int(in_front)
        front_usable += int(in_front and valid)
        if valid and math.isfinite(distance):
            points.append((LASER_X+distance*math.cos(angle), LASER_Y+distance*math.sin(angle)))
    if usable < .8*len(ranges) or front_total == 0 or front_usable < .8*front_total:
        return None
    return points


def obstacle_blocked(points, linear, angular, front_margin, side_margin):
    for x, y in points:
        if (linear > 0 and X_MIN-side_margin <= x <= X_MAX+front_margin
                and abs(y) <= HALF_WIDTH+side_margin):
            return True
        if abs(angular) > 1e-6 and min(math.hypot(x, y), math.hypot(x-AXLE_X, y)) <= TURN_RADIUS:
            return True
    return False


def odom_valid(values, frame, child):
    return (frame == 'odom' and child == 'base_link' and len(values) == 13
            and all(math.isfinite(v) for v in values)
            and .95 <= math.sqrt(sum(v*v for v in values[3:7])) <= 1.05)
