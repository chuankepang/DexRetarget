"""Compatibility import; implementation lives in :mod:`dexretarget.teleop.devices.quest3`."""
from dexretarget.teleop.devices.quest3 import *
from dexretarget.teleop.devices.quest3 import __dict__ as _implementation


def __getattr__(name):
    try:
        return _implementation[name]
    except KeyError:
        raise AttributeError(name) from None
