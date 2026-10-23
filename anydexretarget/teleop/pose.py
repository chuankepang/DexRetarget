"""Compatibility import; implementation lives in :mod:`anydexretarget.geometry`."""
from anydexretarget.geometry import *
from anydexretarget.geometry import __dict__ as _implementation


def __getattr__(name):
    try:
        return _implementation[name]
    except KeyError:
        raise AttributeError(name) from None
