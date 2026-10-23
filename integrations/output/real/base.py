"""Compatibility import; implementation lives in :mod:`dexretarget.teleop.devices.base`."""
from dexretarget.teleop.devices.base import *
from dexretarget.teleop.devices.base import __dict__ as _implementation


def __getattr__(name):
    try:
        return _implementation[name]
    except KeyError:
        raise AttributeError(name) from None
