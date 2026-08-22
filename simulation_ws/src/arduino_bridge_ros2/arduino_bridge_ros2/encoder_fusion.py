"""Dual Hall/opto wheel encoder confidence and fallback logic."""
from collections import deque


class WheelEncoderFusion:
    """Fuse incremental counts from colocated 60-PPR opto and 45-PPR Hall sensors."""

    def __init__(self, opto_ppr=60.0, hall_ppr=45.0, window_samples=10,
                 high_threshold=0.05, medium_threshold=0.12):
        self.ratio = float(opto_ppr) / float(hall_ppr)
        self.high_threshold = float(high_threshold)
        self.medium_threshold = float(medium_threshold)
        self._opto = deque(maxlen=int(window_samples))
        self._hall = deque(maxlen=int(window_samples))
        self._source = 'OPTO_PENDING'
        self._candidate = self._source
        self._candidate_streak = 0
        self.switch_samples = 3

    def reset(self):
        self._opto.clear()
        self._hall.clear()
        self._source = 'OPTO_PENDING'
        self._candidate = self._source
        self._candidate_streak = 0

    def _stable_source(self, proposed):
        if proposed == self._source:
            self._candidate = proposed
            self._candidate_streak = 0
            return self._source
        if proposed != self._candidate:
            self._candidate = proposed
            self._candidate_streak = 1
        else:
            self._candidate_streak += 1
        if self._candidate_streak >= self.switch_samples:
            self._source = proposed
            self._candidate_streak = 0
        return self._source

    def update(self, opto_delta, hall_delta, moving=True):
        opto_delta = max(0.0, float(opto_delta))
        hall_delta = max(0.0, float(hall_delta))
        if not moving:
            self.reset()
            return {'delta': 0.0, 'source': 'STOP', 'confidence': 1.0,
                    'error': 0.0, 'opto_window': 0.0, 'hall_window': 0.0}

        self._opto.append(opto_delta)
        self._hall.append(hall_delta)
        opto_window = sum(self._opto)
        hall_window = sum(self._hall)

        if hall_window < 3.0:
            proposed = 'OPTO_PENDING' if opto_window < 4.0 else 'OPTO_ONLY'
            source = self._stable_source(proposed)
            confidence = 0.5 if source == 'OPTO_PENDING' else 0.25
            return {'delta': opto_delta, 'source': source,
                    'confidence': confidence, 'error': -1.0,
                    'opto_window': opto_window, 'hall_window': hall_window}

        hall_as_opto = hall_delta * self.ratio
        expected_window = hall_window * self.ratio
        error = abs(opto_window - expected_window) / expected_window
        if error <= self.high_threshold:
            proposed = 'OPTO'
        elif error <= self.medium_threshold:
            proposed = 'BLEND'
        else:
            proposed = 'HALL'
        source = self._stable_source(proposed)
        if source in ('OPTO', 'OPTO_PENDING', 'OPTO_ONLY'):
            delta = opto_delta
            confidence = 1.0 if source == 'OPTO' else 0.5
        elif source == 'BLEND':
            delta = 0.5 * opto_delta + 0.5 * hall_as_opto
            confidence = 0.65
        else:
            delta = hall_as_opto
            confidence = 0.35
        return {'delta': delta, 'source': source, 'confidence': confidence,
                'error': error, 'opto_window': opto_window,
                'hall_window': hall_window}