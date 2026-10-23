"""Self-contained ROKAE xCoreSDK-CPP v0.3.4 integration."""

from typing import Any


def __getattr__(name: str) -> Any:
    """Pure-Python controllers do not require a compiled hardware bridge."""
    if name in {"Rokae", "RealtimeConfig", "SDK_VERSION", "module_file", "monotonic_time"}:
        from .python import rokae as binding

        return getattr(binding, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")

__all__ = [
    "Rokae",
    "RealtimeConfig",
    "SDK_VERSION",
    "module_file",
    "monotonic_time",
]
