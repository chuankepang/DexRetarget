"""Legacy Collector facade. New applications compose Recorder and CaptureSources.

Preserves the historical constructor/start_sources/close API for external users.
Device initialization is confined to this compatibility composition adapter.
"""
from .recorder import Recorder
from .config import DEFAULT_CONFIG
from .cameras import CameraProcesses


class Collector(Recorder):
    def __init__(self, config_path=DEFAULT_CONFIG, context=None):
        super().__init__(config_path, context)
        self.sources = None
        self.cameras = None
        self.threads = []

    def start_sources(self, arm=None, hand=None, cameras=True):
        if self.sources is not None:
            raise RuntimeError('Capture sources can only be started once')
        from dexretarget.app.sources import CaptureSources
        def factory(options, emit, **kwargs):
            return CameraProcesses(self.config_path, self.config, emit)
        self.sources = CaptureSources(self.config, self, read_failures=self.read_failures, camera_factory=factory)
        try:
            self.sources.start(arm, hand, cameras)
        finally:
            self.cameras, self.threads = self.sources.cameras, self.sources.threads

    def check_health(self):
        if self.sources:
            self.sources.check_health()
        super().check_health()

    def close(self):
        try:
            super().close()
        finally:
            if self.sources:
                self.sources.close()
