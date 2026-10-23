"""Compatibility import; implementation lives in :mod:`dexretarget.app.keyboard`."""
from dexretarget.app.keyboard import *
from dexretarget.app.keyboard import __dict__ as _implementation


def __getattr__(name):
    try:
        return _implementation[name]
    except KeyError:
        raise AttributeError(name) from None
