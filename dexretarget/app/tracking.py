"""Coordinate transient input holds without changing keyboard enable intent."""
from dexretarget.teleop.tracking import TrackingHold, HandResumeBlend


class TrackingRecovery:
    def __init__(self, session):
        self.session = session
        self.policy = TrackingHold()
        self.hand = HandResumeBlend()

    def update(self, fresh, now, *, hold, resume):
        # The input publisher uses this same lock, so callbacks complete before
        # any new arm target or hand target can pass the gate.
        with self.session.lock:
            if not self.session.enabled:
                self.policy = TrackingHold()
                self.session.tracking_held = False
                return
            transition = self.policy.update(fresh, now)
            if transition == 'hold':
                self.session.tracking_held = True
                hold()
                if self.session.collector:
                    self.session.collector.control(False, 'tracking_hold')
                print('Tracking HOLD: arm and hand targets frozen; enable intent retained.', flush=True)
            elif transition == 'resume':
                resume()
                self.hand.resume(now)
                self.session.tracking_held = False
                if self.session.collector:
                    self.session.collector.control(False, 'tracking_recovered_recenter')
                print('Tracking recovered: reference re-centered; following resumes.', flush=True)
