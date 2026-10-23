"""Compatibility import; implementation lives in :mod:`dexretarget.teleop.devices.inspire`."""
from dexretarget.teleop.devices.inspire import *
from dexretarget.teleop.devices.inspire import __dict__ as _implementation


def __getattr__(name):
    try:
        return _implementation[name]
    except KeyError:
        raise AttributeError(name) from None
