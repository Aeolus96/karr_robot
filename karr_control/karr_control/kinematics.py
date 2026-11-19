"""Ackermann steering kinematics for K.A.R.R.: four independently steered and driven wheels, tied together in software.

No ROS in here, so it is easy to test:  python3 -m pytest karr_control/test

Input is a forward speed vx and a turn rate wz (cmd_vel linear.x, angular.z). The robot turns about a point on a
line across the robot (the ICR), at distance vx / wz to the side. Each wheel's ground velocity is
    u = (vx - wz * y,  wz * (x - rx))
for a wheel at (x, y); rx is where that line crosses the robot. The steering angle is the direction of u and the
wheel speed is its length, so every wheel points at the same ICR (Ackermann). The two modes only differ in rx:

  front       Front Ackermann, rear wheels locked straight. rx = rear axle, so the rear wheels' u has no sideways
              part and their angle is always 0. base_link (the robot centre) sees a small sideways velocity in
              turns (wz * distance from base_link to the rear axle).
  four_wheel  Double Ackermann. rx = 0 (robot centre): the rear wheels steer opposite to the front, which roughly
              halves the turning radius.

There is no sideways (crab) mode: cmd_vel linear.y is ignored.
"""
import math
from dataclasses import dataclass

import numpy as np

MODES = ('front', 'four_wheel')


@dataclass
class Module:
    name: str     # 'fl', 'fr', 'bl', 'br'
    x: float      # steering pivot in base_link (m)
    y: float


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def module_commands(vx, wz, modules, mode, wheel_radius, steer_limit=math.pi / 2,
                    max_wheel_speed=None, prev_angles=None, eps=1e-4):
    """Return {name: (steering_angle_rad, wheel_speed_rad_s)} for a forward speed vx (m/s) and turn rate wz (rad/s).

    In 'front' mode rear modules (x below the robot centre) always get angle 0.
    When a wheel would not move, it keeps its previous angle (no snapping back to 0 when stopping).
    If an angle is outside +-steer_limit (90 deg), the wheel turns the other way and spins backwards instead, so
    commands stay well inside the +-165 deg hard stop.
    If max_wheel_speed is set, all wheels are scaled down together so the path shape is kept.
    """
    if mode not in MODES:
        raise ValueError(f'steering mode must be one of {MODES}, got {mode!r}')
    prev_angles = prev_angles or {}
    if mode == 'front':
        rear = [m for m in modules if m.x < 0]
        rx = sum(m.x for m in rear) / len(rear)
    else:
        rx = 0.0

    out = {}
    for m in modules:
        ux = vx - wz * m.y
        uy = wz * (m.x - rx)
        speed = math.hypot(ux, uy)
        if mode == 'front' and m.x < 0:
            angle, speed = 0.0, math.copysign(speed, ux)
        elif speed < eps:
            angle, speed = prev_angles.get(m.name, 0.0), 0.0
        else:
            angle = math.atan2(uy, ux)
            if abs(angle) > steer_limit:
                angle, speed = _wrap(angle + math.pi), -speed
            angle = max(-steer_limit, min(steer_limit, angle))
        out[m.name] = (angle, speed / wheel_radius)

    if max_wheel_speed:
        peak = max(abs(w) for _, w in out.values())
        if peak > max_wheel_speed:
            k = max_wheel_speed / peak
            out = {n: (a, w * k) for n, (a, w) in out.items()}
    return out


def body_twist(modules, angles, wheel_speeds, wheel_radius):
    """Least-squares body twist (vx, vy, wz) at base_link from measured steering angles and wheel speeds (rad/s).
    vy is measured, not commanded: it is non-zero in front-mode turns (see module docstring) or when wheels slip."""
    A, b = [], []
    for m in modules:
        v = wheel_speeds[m.name] * wheel_radius
        a = angles[m.name]
        A += [[1.0, 0.0, -m.y], [0.0, 1.0, m.x]]
        b += [v * math.cos(a), v * math.sin(a)]
    sol, *_ = np.linalg.lstsq(np.array(A), np.array(b), rcond=None)
    return tuple(float(s) for s in sol)
