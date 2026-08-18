"""Pure differential-drive encoder helpers."""


def wheel_encoder_signs(v, w, wheel_base, left_sign=1, right_sign=1):
    """Return encoder signs from differential-drive wheel commands."""
    left_velocity = v - (w * wheel_base / 2.0)
    right_velocity = v + (w * wheel_base / 2.0)
    if abs(left_velocity) > 1.0e-4:
        left_sign = 1 if left_velocity > 0.0 else -1
    if abs(right_velocity) > 1.0e-4:
        right_sign = 1 if right_velocity > 0.0 else -1
    return left_sign, right_sign
