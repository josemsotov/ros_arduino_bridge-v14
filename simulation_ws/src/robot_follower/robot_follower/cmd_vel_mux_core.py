"""Source selection and limiting for the single-output velocity arbiter."""


MODE_SOURCE = {
    'STADIA': ('stadia', 'web'),
    'FOLLOW': ('follower',),
    'GESTURE': ('gesture',),
    'GO_TO': ('nav',),
    'RETURN_HOME': ('nav',),
}


def select_command(effective_mode, sources, *, max_linear=0.45,
                   max_angular=0.90):
    mode = str(effective_mode).strip().upper()
    candidates = MODE_SOURCE.get(mode, ())
    selected = None
    command = (0.0, 0.0)
    for name in candidates:
        source = sources.get(name, {})
        if source.get('fresh', False):
            selected = name
            command = (float(source.get('linear', 0.0)),
                       float(source.get('angular', 0.0)))
            break

    linear = max(-max_linear, min(max_linear, command[0]))
    angular = max(-max_angular, min(max_angular, command[1]))
    reason = 'selected_' + selected if selected else (
        'mode_inhibits_motion' if not candidates else 'source_stale')
    return {
        'linear': linear,
        'angular': angular,
        'selected_source': selected,
        'reason': reason,
    }
