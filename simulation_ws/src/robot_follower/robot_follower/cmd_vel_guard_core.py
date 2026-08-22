"""Pure checks used by the monitor-only velocity safety guard."""


def evaluate_velocity_guard(*, effective_mode, linear, angular, cmd_fresh,
                            publisher_count, motor_active,
                            max_linear=0.45, max_angular=0.90):
    reasons = []
    mode = str(effective_mode).strip().upper()
    moving_command = cmd_fresh and (
        abs(float(linear)) > 1e-4 or abs(float(angular)) > 1e-4)

    if publisher_count > 1:
        reasons.append('multiple_cmd_vel_publishers')
    if abs(float(linear)) > float(max_linear):
        reasons.append('linear_limit_exceeded')
    if abs(float(angular)) > float(max_angular):
        reasons.append('angular_limit_exceeded')
    if mode in ('PAUSE', 'EMERGENCY_STOP') and moving_command:
        reasons.append('motion_command_while_stopped')
    if mode in ('PAUSE', 'EMERGENCY_STOP') and motor_active:
        reasons.append('motor_active_while_stopped')

    return {
        'safe': not reasons,
        'reasons': reasons,
        'moving_command': moving_command,
    }
