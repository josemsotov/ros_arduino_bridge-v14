from arduino_bridge_ros2.encoder_fusion import WheelEncoderFusion


def feed(fusion, opto, hall, count=10):
    result = None
    for _ in range(count):
        result = fusion.update(opto, hall, moving=True)
    return result


def test_uses_opto_when_ratio_is_consistent():
    result = feed(WheelEncoderFusion(), 4, 3)
    assert result['source'] == 'OPTO'
    assert result['delta'] == 4
    assert result['confidence'] == 1.0


def test_blends_moderate_disagreement():
    result = feed(WheelEncoderFusion(), 4.4, 3)
    assert result['source'] == 'BLEND'
    assert 4.0 < result['delta'] < 4.4


def test_falls_back_to_hall_on_opto_failure():
    result = feed(WheelEncoderFusion(), 0, 3)
    assert result['source'] == 'HALL'
    assert result['delta'] == 4


def test_stopped_wheel_rejects_counts_and_resets_window():
    fusion = WheelEncoderFusion()
    feed(fusion, 4, 3)
    result = fusion.update(9, 9, moving=False)
    assert result['source'] == 'STOP'
    assert result['delta'] == 0