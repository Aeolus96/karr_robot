import math

import pytest

from karr_control.kinematics import Module, body_twist, module_commands

R = 0.1225
MODS = [Module('fl', 0.3556, 0.2286), Module('fr', 0.3556, -0.2286),
        Module('bl', -0.3556, 0.2286), Module('br', -0.3556, -0.2286)]


@pytest.mark.parametrize('mode', ['front', 'four_wheel'])
def test_straight(mode):
    c = module_commands(1.0, 0, MODS, mode, R)
    for a, w in c.values():
        assert a == pytest.approx(0)
        assert w == pytest.approx(1.0 / R)


def test_front_left_turn_is_ackermann():
    c = module_commands(1.0, 0.5, MODS, 'front', R)
    assert c['bl'][0] == 0 and c['br'][0] == 0           # rear locked
    assert c['fl'][0] > c['fr'][0] > 0                   # inner (left) wheel steers more
    assert c['br'][1] > c['bl'][1]                       # outer rear wheel spins faster
    # all wheel normals meet on the rear axle line at y = v / w = 2 m
    for n in ('fl', 'fr'):
        m = next(m for m in MODS if m.name == n)
        y_icr = m.y + (m.x + 0.3556) / math.tan(c[n][0])
        assert y_icr == pytest.approx(2.0)


def test_four_wheel_double_ackermann_is_symmetric():
    c = module_commands(1.0, 0.5, MODS, 'four_wheel', R)
    assert c['fl'][0] == pytest.approx(-c['bl'][0])
    assert c['fr'][0] == pytest.approx(-c['br'][0])


def test_four_wheel_turns_tighter_than_front():
    front = module_commands(1.0, 0.5, MODS, 'front', R)
    four = module_commands(1.0, 0.5, MODS, 'four_wheel', R)
    assert abs(four['fl'][0]) < abs(front['fl'][0])      # same turn rate needs less steering angle


def test_turn_in_place_stays_inside_hard_stop():
    for mode in ('front', 'four_wheel'):
        for a, w in module_commands(0, 1.0, MODS, mode, R).values():
            assert abs(a) <= math.pi / 2


def test_reverse_flips_instead_of_turning_around():
    for a, w in module_commands(-1.0, 0, MODS, 'four_wheel', R).values():
        assert a == pytest.approx(0) and w == pytest.approx(-1.0 / R)


def test_speed_limit_scales_all_wheels():
    c = module_commands(5.0, 1.0, MODS, 'front', R, max_wheel_speed=21.9)
    assert max(abs(w) for _, w in c.values()) == pytest.approx(21.9)


def test_stopped_wheels_keep_angle():
    prev = {'fl': 0.3, 'fr': 0.2, 'bl': -0.3, 'br': -0.2}
    c = module_commands(0, 0, MODS, 'four_wheel', R, prev_angles=prev)
    assert {n: a for n, (a, _) in c.items()} == prev


@pytest.mark.parametrize('mode,vx,wz', [('front', 1.0, 0.5), ('four_wheel', 0.8, -0.4)])
def test_odometry_inverts_commands(mode, vx, wz):
    c = module_commands(vx, wz, MODS, mode, R)
    got = body_twist(MODS, {n: a for n, (a, _) in c.items()}, {n: w for n, (_, w) in c.items()}, R)
    vy = wz * 0.3556 if mode == 'front' else 0.0   # base_link slides sideways in front-steer turns
    assert got == pytest.approx((vx, vy, wz))
