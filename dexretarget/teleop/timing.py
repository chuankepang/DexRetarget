"""Bounded, optional host timing samples; no I/O on control paths."""
from collections import deque
import math
import threading

import numpy as np


class TimingStats:
    def __init__(self, capacity=4096):
        self.capacity = capacity
        self._lock = threading.Lock()
        self._values = {}
        self._counts = {}

    def record(self, name, seconds):
        if not math.isfinite(seconds) or seconds < 0:
            return
        with self._lock:
            self._values.setdefault(name, deque(maxlen=self.capacity)).append(seconds * 1000)
            self._counts[name] = self._counts.get(name, 0) + 1

    def snapshot(self):
        with self._lock:
            values = {name: list(samples) for name, samples in self._values.items()}
            counts = self._counts.copy()
        return {name: dict(count=counts[name], window_samples=len(samples),
                          **dict(zip(('p50_ms', 'p95_ms', 'p99_ms', 'max_ms'),
                                     np.percentile(samples, [50, 95, 99, 100]).tolist())))
                for name, samples in values.items()}
