"""Tracking loss policy, independent of keyboard intent and device protocols."""
from __future__ import annotations
import numpy as np


class TrackingHold:
    """Hold on missing input; resume after a continuous fresh interval."""
    def __init__(self, resume_delay=0.01):
        resume_delay = float(resume_delay)
        if not np.isfinite(resume_delay) or resume_delay < 0:
            raise ValueError("tracking_resume_delay_s must be finite and non-negative")
        self.held = False
        self.fresh_since = None
        self.resume_delay = resume_delay

    def update(self, fresh, now):
        if not fresh:
            self.fresh_since = None
            if not self.held:
                self.held = True
                return 'hold'
        elif self.held:
            if self.fresh_since is None:
                self.fresh_since = now
            if now - self.fresh_since >= self.resume_delay:
                self.held = False
                self.fresh_since = None
                return 'resume'
        return None


class HandResumeBlend:
    """Blend from the last sent target after loss, never replay missing samples."""
    def __init__(self, duration=0.5):
        self.duration = duration
        self.last = self.origin = None
        self.started = None

    def resume(self, now):
        self.origin = None if self.last is None else self.last.copy()
        self.started = now

    def target(self, value, now):
        result = np.asarray(value, dtype=float).copy()
        if self.origin is not None:
            weight = min(1.0, max(0.0, (now - self.started) / self.duration))
            result = self.origin + weight * (result - self.origin)
            if weight == 1.0:
                self.origin = None
        self.last = result.copy()
        return result


def fresh_timestamp(timestamp, now, max_age):
    return timestamp is not None and np.isfinite(timestamp) and 0 <= now - timestamp <= max_age
