"""Compatibility adapter. Camera implementation lives in dexretarget.camera."""
from dexretarget.camera.processes import CameraProcesses as _CameraProcesses
from dexretarget.camera.worker import PREFIX, camera_worker
from .config import load_config


class CameraProcesses(_CameraProcesses):
    def __init__(self, config_path, config, emit):
        self.config_path, self.config = config_path, config
        super().__init__(config['cameras'], emit, python=config['camera_python'],
                         warmup_s=config['camera_warmup_s'], jpeg_quality=config['jpeg_quality'])


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--worker', action='store_true', required=True)
    parser.add_argument('--role', choices=['wrist', 'global'], required=True)
    parser.add_argument('--config', required=True)
    args = parser.parse_args()
    camera_worker(load_config(args.config), args.role)
