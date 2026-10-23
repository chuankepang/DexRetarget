"""Shared project paths; no devices, collection, or numerical dependencies."""
import copy
import json
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_COLLECTION_CONFIG = PROJECT_ROOT / "config/collection/collection.json"
DEFAULT_CAMERA_CONFIG = PROJECT_ROOT / "config/cameras/realsense.json"


def project_path(value):
    path = Path(value).expanduser()
    return path if path.is_absolute() else PROJECT_ROOT / path


def validate_camera_config(config, *, require_bound=True):
    config = copy.deepcopy(config)
    for key in ['width', 'height', 'fps']:
        if type(config.get(key)) is not int or config[key] <= 0:
            raise ValueError(f'Camera {key} must be a positive integer')
    cameras = config.get('cameras', [])
    if len(cameras) != 2 or {c.get('role') for c in cameras} != {'wrist', 'global'}:
        raise ValueError('Exactly one wrist and one global camera are required')
    serials = [c.get('serial') for c in cameras]
    if require_bound and (any(not isinstance(s, str) or not s for s in serials) or len(set(serials)) != 2):
        raise ValueError('Bind two distinct camera serials before capture')
    if config.get('color_format', 'rgb8') != 'rgb8' or config.get('depth_format', 'z16') != 'z16':
        raise ValueError('Supported camera formats are RGB8 and Z16')
    for key in ['enable_depth', 'color_auto_exposure', 'color_auto_exposure_priority']:
        if key in config and type(config[key]) is not bool:
            raise ValueError(f'Camera {key} must be boolean')
    return config


def load_camera_config(path=DEFAULT_CAMERA_CONFIG, *, require_bound=True):
    return validate_camera_config(json.loads(Path(path).read_text()), require_bound=require_bound)
