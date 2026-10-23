"""Compatibility import; implementation lives in :mod:`dexretarget.app.session`."""
from dexretarget.app.session import *
from dexretarget.app.session import __dict__ as _implementation


def __getattr__(name):
    try:
        return _implementation[name]
    except KeyError:
        raise AttributeError(name) from None
