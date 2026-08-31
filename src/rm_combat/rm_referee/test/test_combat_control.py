import math

from rm_referee.combat_control import MAX_PITCH, clamp_aim


def test_clamp_aim_inside_limits():
    assert clamp_aim(0.4, -0.2) == (0.4, -0.2)


def test_clamp_aim_at_limits():
    assert clamp_aim(10.0, 2.0) == (math.pi, MAX_PITCH)
    assert clamp_aim(-10.0, -2.0) == (-math.pi, -MAX_PITCH)
