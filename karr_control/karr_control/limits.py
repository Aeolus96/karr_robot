"""Speed and turn limits between cmd_vel and the steering kinematics. No ROS in here (unit tested in test/).

The limiter has two kinds of motion and never mixes them:

DRIVING (|linear.x| requested >= swing_speed_threshold)
  The turn is handled as a curvature k = wz / vx (1 / turning radius), so speeding up or slowing down never changes
  the path's shape or the steering angles. Every cycle:
    1. Speed:        vx clamped to [-max_reverse_speed, max_speed], then moved toward it at most max_accel (speeding
                     up) or max_decel (slowing down, and on cmd_vel timeout) per second. Reversing stops first.
    2. Curvature:    |k| <= 1 / min_turn_radius            tightest turn while driving (default 1.0 m)
                     |k| <= max_lateral_accel / vx^2       <- the "no 50 deg steering at 6 mph" rule: sideways
                                                              acceleration in a turn is v^2 * k
                     |k| <= max_angular_speed / |vx|
    3. Curvature change: k moves toward its target at most max_curvature_rate per second, so the steering swings in
                     gradually. It drops to a new, lower limit at once if speed went up.
  Output: vx and wz = vx * k.

SWING TURN (|linear.x| requested < swing_speed_threshold and angular.z != 0)
  Turning on the spot: in front mode about the middle of the rear axle (front wheels at ~72 deg), in four_wheel mode
  about the robot centre. Only from standstill: if the robot is driving, it first slows to a stop on its current path.
  wz then ramps at max_angular_accel up to max_angular_speed. Asking to drive again stops the swing first.
  allow_swing_turn: false disables it.

Steering alignment (applied by the drive node): while a steering motor lags far behind its target, the wheels slow
down (alignment_scale) and the limiter doesn't speed up.
"""
from dataclasses import dataclass

STOPPED = 1e-3   # m/s or rad/s


@dataclass
class Limits:
    max_speed: float = 2.68          # m/s forward (6 mph)
    max_reverse_speed: float = 1.0   # m/s
    max_accel: float = 0.5           # m/s^2 when speeding up
    max_decel: float = 1.0           # m/s^2 when slowing down / stopping on timeout
    max_angular_speed: float = 1.0   # rad/s
    max_angular_accel: float = 1.0   # rad/s^2, swing turns
    max_lateral_accel: float = 1.0   # m/s^2 (v^2 * k). Tip-over is ~5 m/s^2 for this robot; keep a big margin.
    min_turn_radius: float = 1.0     # m, tightest turn while driving
    max_curvature_rate: float = 1.0  # 1/m per s: from straight to the tightest (1 m) turn takes 1 s
    swing_speed_threshold: float = 0.05  # m/s: a linear.x request below this (with angular.z) means "swing turn"
    allow_swing_turn: bool = True


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


def _toward(x, target, step):
    return x + _clamp(target - x, -step, step)


class CommandLimiter:
    def __init__(self, limits: Limits):
        self.limits = limits
        self.vx = 0.0     # m/s
        self.k = 0.0      # 1/m, curvature while driving
        self.spin = 0.0   # rad/s, swing turn rate

    @property
    def wz(self):
        return self.vx * self.k + self.spin

    def max_curvature(self, vx):
        """Largest |k| allowed while driving at speed vx."""
        lim = self.limits
        k = 1.0 / lim.min_turn_radius if lim.min_turn_radius > 0 else float('inf')
        if abs(vx) > STOPPED:
            k = min(k, lim.max_lateral_accel / vx ** 2, lim.max_angular_speed / abs(vx))
        return k

    def step(self, vx_cmd, wz_cmd, dt, speed_up=1.0):
        """Advance one cycle of length dt (s) toward the requested command. Returns the limited (vx, wz).

        speed_up in [0, 1] scales how fast the limiter may increase speed or swing rate (0 = hold; slowing down and
        steering changes are never held). The drive node lowers it while the steering hasn't caught up.
        """
        lim = self.limits
        swing = lim.allow_swing_turn and abs(vx_cmd) < lim.swing_speed_threshold and abs(wz_cmd) > STOPPED

        if swing:
            # stop on the current path, then turn on the spot
            self.vx = _toward(self.vx, 0.0, lim.max_decel * dt)
            if abs(self.vx) <= STOPPED:
                self.vx, self.k = 0.0, 0.0
                target = _clamp(wz_cmd, -lim.max_angular_speed, lim.max_angular_speed)
                rate = lim.max_angular_accel * (speed_up if abs(target) > abs(self.spin) else 1.0)
                self.spin = _toward(self.spin, target, rate * dt)
            return self.vx, self.wz

        # driving (or stopping): end any swing turn before moving off
        if abs(self.spin) > STOPPED:
            self.spin = _toward(self.spin, 0.0, lim.max_angular_accel * dt)
            return self.vx, self.wz
        self.spin = 0.0

        target = _clamp(vx_cmd, -lim.max_reverse_speed, lim.max_speed)
        if self.vx * target < 0:                 # reversing: stop first
            target = 0.0
        rate = lim.max_accel * speed_up if abs(target) > abs(self.vx) else lim.max_decel
        self.vx = _toward(self.vx, target, rate * dt)

        k_max = self.max_curvature(self.vx)
        if abs(vx_cmd) >= lim.swing_speed_threshold:
            k_target = _clamp(wz_cmd / vx_cmd, -k_max, k_max)
        else:
            k_target = self.k                    # stopping: keep the current path
        self.k = _clamp(_toward(self.k, k_target, lim.max_curvature_rate * dt), -k_max, k_max)
        return self.vx, self.wz

    def reset(self):
        self.vx = self.k = self.spin = 0.0


def alignment_scale(errors, full_speed_below=0.1, stop_above=0.5):
    """Wheel speed factor from steering errors (rad, |target - measured|): 1 below full_speed_below, 0 above
    stop_above, linear in between. Driving with misaligned wheels scrubs tyres and pushes the robot off its path."""
    e = max((abs(x) for x in errors), default=0.0)
    if e <= full_speed_below:
        return 1.0
    if e >= stop_above:
        return 0.0
    return (stop_above - e) / (stop_above - full_speed_below)
