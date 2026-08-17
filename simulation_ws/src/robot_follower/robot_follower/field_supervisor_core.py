"""Pure state-selection logic for the field supervisor."""

VALID_REQUESTS = {
    'EMERGENCY_STOP',
    'PAUSE',
    'STADIA',
    'FOLLOW',
    'GESTURE',
    'GO_TO',
    'RETURN_HOME',
}


def normalize_request(value):
    request = str(value).strip().upper()
    if request == 'MANUAL':
        request = 'STADIA'
    if request == 'IDLE':
        request = 'PAUSE'
    if request not in VALID_REQUESTS:
        raise ValueError(f'Unsupported field mode: {request}')
    return request


def select_effective_mode(requested, *, emergency_latched=False,
                          stadia_fresh=False, stadia_connected=False,
                          stadia_mode='', follower_fresh=False,
                          follower_enabled=False, gps_fresh=False,
                          gps_fix=False, gps_quality_ok=None,
                          navigation_ready=False):
    """Return ``(mode, reason)`` using fail-closed field priorities."""
    requested = normalize_request(requested)
    stadia_mode = str(stadia_mode).strip().lower()
    if gps_quality_ok is None:
        gps_quality_ok = gps_fix

    if emergency_latched or requested == 'EMERGENCY_STOP':
        return 'EMERGENCY_STOP', 'emergency_stop_latched'

    # A connected controller actively in manual mode always wins.
    if stadia_fresh and stadia_connected and stadia_mode == 'stadia':
        return 'STADIA', 'stadia_manual_override'

    if requested == 'PAUSE':
        return 'PAUSE', 'pause_requested'

    if requested == 'STADIA':
        if not stadia_fresh:
            return 'PAUSE', 'stadia_state_stale'
        if not stadia_connected:
            return 'PAUSE', 'stadia_disconnected'
        if stadia_mode != 'stadia':
            return 'PAUSE', 'stadia_manual_mode_not_selected'
        return 'STADIA', 'stadia_ready'

    if requested == 'FOLLOW':
        if not stadia_fresh or not stadia_connected:
            return 'PAUSE', 'stadia_authorization_unavailable'
        if stadia_mode != 'follower':
            return 'PAUSE', 'follower_not_authorized_by_stadia'
        if not follower_fresh:
            return 'PAUSE', 'follower_state_stale'
        if not follower_enabled:
            return 'PAUSE', 'follower_not_enabled'
        return 'FOLLOW', 'follower_ready'

    if requested == 'GESTURE':
        return 'GESTURE', 'gesture_requested'

    if requested in ('GO_TO', 'RETURN_HOME'):
        if not navigation_ready:
            return 'PAUSE', 'navigation_not_ready'
        if not gps_fresh:
            return 'PAUSE', 'gps_state_stale'
        if not gps_fix:
            return 'PAUSE', 'gps_fix_unavailable'
        if not gps_quality_ok:
            return 'PAUSE', 'gps_quality_insufficient'
        return requested, 'navigation_ready'

    return 'PAUSE', 'fail_closed'
