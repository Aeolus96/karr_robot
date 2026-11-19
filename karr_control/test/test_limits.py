import math

import pytest

from karr_control.kinematics import Module, module_commands
from karr_control.limits import CommandLimiter, Limits, alignment_scale

DT = 0.02
R = 0.1225
MODS = [Module('fl', 0.3556, 0.2286), Module('fr', 0.3556, -0.2286),
        Module('bl', -0.3556, 0.2286), Module('br', -0.3556, -0.2286)]


def run(lim, vx, wz, seconds, **kw):
    out = (lim.vx, lim.wz)
    for _ in range(int(round(seconds / DT))):
        out = lim.step(vx, wz, DT, **kw)
    return out


def front_angles(vx, wz):
    c = module_commands(vx, wz, MODS, 'front', R)
    return math.degrees(c['fl'][0]), math.degrees(c['fr'][0])


def test_speed_limits():
    lim = CommandLimiter(Limits())
    assert run(lim, 10.0, 0, 20)[0] == pytest.approx(2.68)
    assert run(lim, -10.0, 0, 20)[0] == pytest.approx(-1.0)


def test_acceleration_is_limited():
    vx, _ = run(CommandLimiter(Limits(max_accel=0.5)), 2.0, 0, 1.0)
    assert vx == pytest.approx(0.5, abs=0.02)


def test_reversing_stops_first():
    lim = CommandLimiter(Limits())
    run(lim, 1.0, 0, 5)
    seen = [lim.step(-1.0, 0, DT)[0] for _ in range(200)]
    i = next(i for i, v in enumerate(seen) if v < 0)
    assert seen[i - 1] == pytest.approx(0.0)


def test_teleop_forward_turn_is_not_a_swing_turn():
    """teleop_twist_keyboard 'u' = 0.5 m/s + 1.0 rad/s (0.5 m radius): clamped to the 1 m driving minimum."""
    lim = CommandLimiter(Limits())
    for t in range(1, 151):
        vx, wz = lim.step(0.5, 1.0, DT)
        if vx > 0.01:
            assert vx / wz >= 1.0 - 1e-9
            assert max(map(abs, front_angles(vx, wz))) < 45
    assert vx / wz == pytest.approx(1.0)


def test_curvature_swings_in_gradually():
    lim = CommandLimiter(Limits(max_curvature_rate=1.0))
    run(lim, 0.5, 0, 2)
    run(lim, 0.5, 0.5, 0.5)
    assert lim.k == pytest.approx(0.5, abs=0.03)


def test_speeding_up_keeps_the_path():
    lim = CommandLimiter(Limits())
    vx1, wz1 = run(lim, 0.3, 0.3, 3)          # 1 m radius, settled
    vx2, wz2 = run(lim, 0.6, 0.6, 0.4)        # same radius, faster
    assert vx1 / wz1 == pytest.approx(1.0) and vx2 / wz2 == pytest.approx(1.0)


def test_no_sharp_steering_at_full_speed():
    lim = CommandLimiter(Limits())
    run(lim, 2.68, 0, 10)
    vx, wz = run(lim, 2.68, 1.0, 3)
    assert vx * wz <= 1.0 + 1e-9
    assert max(map(abs, front_angles(vx, wz))) < 10


def test_swing_turn_from_standstill():
    lim = CommandLimiter(Limits())
    vx, wz = run(lim, 0.0, 1.0, 3)
    assert vx == 0 and wz == pytest.approx(1.0)
    fl, fr = front_angles(1e-9, 1.0)
    assert abs(fl) == pytest.approx(72.2, abs=0.5)


def test_swing_request_while_driving_stops_first():
    lim = CommandLimiter(Limits())
    run(lim, 1.0, 0, 4)
    vx, wz = lim.step(0.0, 1.0, DT)
    assert vx > 0.9 and wz == pytest.approx(0.0)     # still on its straight path, slowing
    vx, wz = run(lim, 0.0, 1.0, 1.2)
    assert vx == 0                                    # stopped, then swing starts
    assert run(lim, 0.0, 1.0, 2)[1] == pytest.approx(1.0)


def test_drive_after_swing_ends_swing_first():
    lim = CommandLimiter(Limits())
    run(lim, 0.0, 1.0, 3)
    vx, wz = lim.step(0.5, 0.0, DT)
    assert vx == 0 and 0 < wz < 1.0
    vx, wz = run(lim, 0.5, 0.0, 2)
    assert vx > 0 and wz == pytest.approx(0.0)


def test_swing_can_be_disabled():
    assert run(CommandLimiter(Limits(allow_swing_turn=False)), 0.0, 1.0, 3) == (0.0, 0.0)


def test_timeout_stop_keeps_the_path():
    lim = CommandLimiter(Limits())
    vx, wz = run(lim, 0.5, 0.25, 4)
    r = vx / wz
    vx, wz = run(lim, 0.0, 0.0, 0.2)
    assert 0 < vx < 0.5 and vx / wz == pytest.approx(r)


def test_hold_while_steering_catches_up():
    lim = CommandLimiter(Limits())
    run(lim, 1.0, 0, 1.0)
    v_before = lim.vx
    assert run(lim, 2.0, 0, 1.0, speed_up=0.0)[0] == pytest.approx(v_before)
    assert run(lim, 0.0, 0, 5.0, speed_up=0.0)[0] == pytest.approx(0.0)   # slowing down still works


@pytest.mark.parametrize('errs,expected', [([0.0, 0.05], 1.0), ([0.3], 0.5), ([0.6, 0.0], 0.0), ([], 1.0)])
def test_alignment_scale(errs, expected):
    assert alignment_scale(errs) == pytest.approx(expected)
