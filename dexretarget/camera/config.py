"""Camera encoding options and shared camera schema exports."""
import math
from dexretarget.config import load_camera_config, validate_camera_config


def validate_encoding(warmup_s, jpeg_quality):
    if type(warmup_s) not in (float, int) or not math.isfinite(warmup_s) or warmup_s < 0:
        raise ValueError('Camera warmup must be finite and nonnegative')
    if type(jpeg_quality) is not int or not 1 <= jpeg_quality <= 100:
        raise ValueError('JPEG quality must be an integer in 1..100')
