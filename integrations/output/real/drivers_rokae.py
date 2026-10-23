"""Compatibility import; implementation lives in :mod:`dexretarget.teleop.devices.rokae`."""
from dexretarget.teleop.devices.rokae import *
from dexretarget.teleop.devices.rokae import __dict__ as _implementation


def __getattr__(name):
    try:
        return _implementation[name]
    except KeyError:
        raise AttributeError(name) from None
