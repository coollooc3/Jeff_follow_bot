"""Continuous image-space following policy. No ROS, clocks, or hardware I/O."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Settings:
    max_forward_mps: float = .12
    max_turn_rps: float = .60
    forward_gain: float = .50
    turn_gain: float = 1.20
    target_timeout: float = .25

    def __post_init__(self):
        if (not all(math.isfinite(v) and v > 0 for v in vars(self).values())
                or self.max_forward_mps > .20 or self.max_turn_rps > 1.
                or not .10 <= self.target_timeout <= .40):
            raise ValueError('Invalid control settings (speed <= .20m/s, turn <= 1rad/s)')


@dataclass(frozen=True)
class Command:
    linear: float = 0.
    angular: float = 0.
    reason: str = 'disabled'


def slew(current, desired, amount):
    return current + max(-amount, min(amount, desired-current))


class Follower:
    def __init__(self, settings=Settings()):
        self.settings = settings
        self.enabled = False
        self.reference = None
        self.last_seen = self.last_step = None
        self.count = 0
        self.side = self.offset = self.raw_side = None
        self.visible = False
        self.approaching = False
        self.command = Command()

    def set_reference(self, side):
        if not math.isfinite(side) or not 0 < side < 1:
            raise ValueError('Reference side must be normalized by image width')
        self.reference = side
        self.approaching = False

    def set_enabled(self, enabled):
        self.enabled = bool(enabled)
        if not enabled:
            self.approaching = False
            self.command = Command()

    def observe(self, now, side=None, offset=None):
        """Missing frames slow to zero; they do not reset an established identity."""
        self.visible = False
        if (side is None or offset is None or not all(math.isfinite(v) for v in (now, side, offset))
                or not 0 < side < 1 or not -1 <= offset <= 1):
            return
        gap = None if self.last_seen is None else now-self.last_seen
        associated = (gap is not None and 0 < gap <= self.settings.target_timeout
                      and abs(offset-self.offset) <= .45 and .5 <= side/self.side <= 2.)
        if associated:
            alpha = 1-math.exp(-gap/.08)
            self.side += alpha*(side-self.side)
            self.offset += alpha*(offset-self.offset)
            self.count = min(3, self.count+1)
        else:
            self.side, self.offset, self.count = side, offset, 1
        self.raw_side, self.last_seen, self.visible = side, now, True

    def step(self, now, blocked=''):
        def stop(reason):
            self.command = Command(reason=reason)
            return self.command

        if not math.isfinite(now) or self.last_step is not None and now < self.last_step:
            self.set_enabled(False)
            self.count = 0
            return stop('control_clock_invalid')
        dt = .05 if self.last_step is None else min(.1, now-self.last_step)
        self.last_step = now
        if not self.enabled:
            return stop('disabled')
        if self.reference is None:
            return stop('reference_missing')
        if blocked:
            return stop(blocked)
        if self.last_seen is None or not 0 <= now-self.last_seen <= self.settings.target_timeout:
            self.count = 0
            self.approaching = False
            return stop('target_lost')
        if self.count < 3:
            return stop('acquiring')
        if not self.visible:
            self.command = Command(max(0., self.command.linear-.8*dt), 0., 'target_missing')
            return self.command
        # Raw size stops promptly; filtered size avoids noisy forward restarts.
        if self.raw_side >= .98*self.reference:
            self.approaching = False
        elif max(self.raw_side, self.side) < .92*self.reference:
            self.approaching = True
        heading = max(0., abs(self.offset)-.03)
        angular = -math.copysign(min(self.settings.max_turn_rps,
                                    self.settings.turn_gain*heading), self.offset)
        desired = 0.
        if self.approaching:
            desired = min(self.settings.max_forward_mps,
                          max(min(.05, self.settings.max_forward_mps),
                              self.settings.forward_gain*(self.reference/self.side-1)))
            desired *= max(0., 1-(abs(self.offset)/.65)**2)
        linear = slew(self.command.linear, desired, (.35 if desired > self.command.linear else .8)*dt)
        if not self.approaching:
            linear = 0.
        angular = slew(self.command.angular, angular, 2.*dt)
        self.command = Command(linear, angular, 'following' if linear > 0 else 'holding')
        return self.command
