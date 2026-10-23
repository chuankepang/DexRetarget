"""Compatibility import; implementation lives in :mod:`dexretarget.app.task`."""
from dexretarget.app.task import *
from dexretarget.app.task import __dict__ as _implementation


def __getattr__(name):
    try:
        return _implementation[name]
    except KeyError:
        raise AttributeError(name) from None
