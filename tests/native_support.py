"""Discover a binding for this Python ABI/platform without importing its parent."""
from importlib.machinery import PathFinder
from pathlib import Path

NATIVE_DIR = Path(__file__).resolve().parents[1] / 'rokae/python'
HAS_NATIVE = PathFinder.find_spec('_rokae_cpp', [str(NATIVE_DIR)]) is not None
