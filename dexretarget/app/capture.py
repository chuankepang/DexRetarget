"""Translate control observations into the unchanged raw-record schema."""
import time
import numpy as np
from anydexretarget.geometry import make_transform
from dexretarget.contracts import EventSink


class CaptureObserver:
    def __init__(self, sink: EventSink):
        self.sink = sink
        self.last_quest = None

    def tracking(self, tracking, sequence):
        wrist = tracking['wrist']
        key = (None if wrist is None else wrist['timestamp'], tracking['landmarks_timestamp'])
        if key == self.last_quest:
            return
        self.last_quest = key
        self.sink.emit('quest', time.monotonic(), sequence, {
            'wrist': np.full((4,4), np.nan) if wrist is None else make_transform(wrist['position'], wrist['rotation']),
            'landmarks': np.full((21,3), np.nan) if tracking['fingers'] is None else tracking['fingers'],
            'wrist_timestamp': np.nan if key[0] is None else key[0],
            'landmarks_timestamp': np.nan if key[1] is None else key[1],
        })

    def arm_target(self, command, accepted_pose, sequence):
        self.sink.emit('arm_command', time.monotonic(), sequence, {
            'requested_tcp': command.base_target_raw, 'accepted_tcp': accepted_pose, 'enabled': True,
        }, command.timestamp)

    def hand_target(self, result, sequence):
        self.sink.emit('hand_command', time.monotonic(), sequence, {
            'angle': result['angle'], 'acknowledged': result['acknowledged'],
        }, result['timestamp'])

    def control(self, enabled, reason=''):
        self.sink.control(enabled, reason)
