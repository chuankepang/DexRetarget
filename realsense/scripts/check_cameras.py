"""Compatibility command; maintained implementation: tools/diagnostics/check_cameras.py."""
from pathlib import Path
import sys

_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from tools.diagnostics.check_cameras import *
from tools.diagnostics.check_cameras import __dict__ as _implementation


def __getattr__(name):
    try:
        return _implementation[name]
    except KeyError:
        raise AttributeError(name) from None


if __name__ == '__main__':
    raise SystemExit(main())
