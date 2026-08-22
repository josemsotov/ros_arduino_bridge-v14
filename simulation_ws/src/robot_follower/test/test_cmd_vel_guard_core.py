from robot_follower.cmd_vel_guard_core import evaluate_velocity_guard


def check(**kwargs):
    defaults = dict(effective_mode='STADIA', linear=0.0, angular=0.0,
                    cmd_fresh=True, publisher_count=1, motor_active=False)
    return evaluate_velocity_guard(**{**defaults, **kwargs})


def test_nominal_command_is_safe():
    assert check(linear=0.2)['safe']


def test_detects_multiple_publishers():
    result = check(publisher_count=4)
    assert 'multiple_cmd_vel_publishers' in result['reasons']


def test_detects_command_during_pause():
    result = check(effective_mode='PAUSE', linear=0.1)
    assert 'motion_command_while_stopped' in result['reasons']


def test_ignores_stale_nonzero_command_but_detects_active_motor():
    assert check(effective_mode='PAUSE', linear=0.1, cmd_fresh=False)['safe']
    result = check(effective_mode='EMERGENCY_STOP', cmd_fresh=False,
                   motor_active=True)
    assert 'motor_active_while_stopped' in result['reasons']


def test_detects_velocity_limits():
    result = check(linear=0.5, angular=1.0)
    assert 'linear_limit_exceeded' in result['reasons']
    assert 'angular_limit_exceeded' in result['reasons']
