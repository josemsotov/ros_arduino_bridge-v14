from robot_follower.cmd_vel_mux_core import select_command


def source(linear=0.0, angular=0.0, fresh=True):
    return {'linear': linear, 'angular': angular, 'fresh': fresh}


def test_pause_and_emergency_always_output_zero():
    inputs = {'stadia': source(0.3), 'nav': source(0.4)}
    for mode in ('PAUSE', 'EMERGENCY_STOP'):
        result = select_command(mode, inputs)
        assert (result['linear'], result['angular']) == (0.0, 0.0)
        assert result['selected_source'] is None


def test_mode_selects_only_its_authorized_source():
    inputs = {
        'stadia': source(0.2), 'follower': source(0.3),
        'gesture': source(0.1), 'nav': source(0.4),
    }
    assert select_command('STADIA', inputs)['selected_source'] == 'stadia'
    assert select_command('FOLLOW', inputs)['selected_source'] == 'follower'
    assert select_command('GESTURE', inputs)['selected_source'] == 'gesture'
    assert select_command('GO_TO', inputs)['selected_source'] == 'nav'


def test_stadia_uses_web_only_when_physical_source_is_stale():
    inputs = {'stadia': source(fresh=False), 'web': source(0.2)}
    assert select_command('STADIA', inputs)['selected_source'] == 'web'


def test_stale_source_fails_closed():
    result = select_command('FOLLOW', {'follower': source(0.3, fresh=False)})
    assert result['selected_source'] is None
    assert result['linear'] == 0.0


def test_limits_are_enforced():
    result = select_command('STADIA', {'stadia': source(2.0, -3.0)})
    assert result['linear'] == 0.45
    assert result['angular'] == -0.90
