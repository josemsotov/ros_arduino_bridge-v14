import pytest

from robot_follower.field_supervisor_core import (
    normalize_request,
    select_effective_mode,
)


def decide(request, **kwargs):
    return select_effective_mode(request, **kwargs)


def test_aliases_and_invalid_request():
    assert normalize_request('manual') == 'STADIA'
    assert normalize_request('idle') == 'PAUSE'
    with pytest.raises(ValueError):
        normalize_request('drive')


def test_emergency_stop_has_highest_priority():
    mode, _ = decide(
        'FOLLOW', emergency_latched=True, stadia_fresh=True,
        stadia_connected=True, stadia_mode='stadia')
    assert mode == 'EMERGENCY_STOP'


def test_manual_stadia_overrides_other_requests():
    mode, reason = decide(
        'GO_TO', stadia_fresh=True, stadia_connected=True,
        stadia_mode='stadia', gps_fresh=True, gps_fix=True,
        navigation_ready=True)
    assert (mode, reason) == ('STADIA', 'stadia_manual_override')


def test_follow_requires_fresh_authorized_and_enabled_states():
    ready = dict(stadia_fresh=True, stadia_connected=True,
                 stadia_mode='follower', follower_fresh=True,
                 follower_enabled=True)
    assert decide('FOLLOW', **ready)[0] == 'FOLLOW'
    assert decide('FOLLOW', **{**ready, 'follower_fresh': False})[0] == 'PAUSE'
    assert decide('FOLLOW', **{**ready, 'stadia_mode': 'off'})[0] == 'PAUSE'


def test_gesture_mode_is_explicitly_observable():
    assert decide('GESTURE') == ('GESTURE', 'gesture_requested')


@pytest.mark.parametrize('mode_request', ['GO_TO', 'RETURN_HOME'])
def test_navigation_requires_ready_stack_and_fresh_fix(mode_request):
    ready = dict(navigation_ready=True, gps_fresh=True, gps_fix=True,
                 gps_quality_ok=True)
    assert decide(mode_request, **ready)[0] == mode_request
    assert decide(mode_request, **{**ready, 'gps_fix': False})[0] == 'PAUSE'
    assert decide(
        mode_request, **{**ready, 'navigation_ready': False})[0] == 'PAUSE'


def test_navigation_rejects_unqualified_gps_fix():
    mode, reason = decide(
        'GO_TO', navigation_ready=True, gps_fresh=True, gps_fix=True,
        gps_quality_ok=False)
    assert (mode, reason) == ('PAUSE', 'gps_quality_insufficient')
