from arduino_bridge_ros2.encoder_math import wheel_encoder_signs


def test_forward_and_reverse_signs():
    assert wheel_encoder_signs(0.2, 0.0, 0.82) == (1, 1)
    assert wheel_encoder_signs(-0.2, 0.0, 0.82) == (-1, -1)


def test_turn_signs():
    assert wheel_encoder_signs(0.0, 0.4, 0.82) == (-1, 1)
    assert wheel_encoder_signs(0.0, -0.4, 0.82) == (1, -1)


def test_stop_preserves_coast_direction():
    assert wheel_encoder_signs(0.0, 0.0, 0.82, -1, -1) == (-1, -1)
