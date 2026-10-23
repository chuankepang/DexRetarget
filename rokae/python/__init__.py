"""Public facade, loaded only when requested; native submodules import directly."""

__all__ = [
    "Rokae",
    "RealtimeConfig",
    "SDK_VERSION",
    "module_file",
    "monotonic_time",
]


def __getattr__(name):
    if name in __all__:
        from . import rokae
        return getattr(rokae, name)
    raise AttributeError(f'module {__name__!r} has no attribute {name!r}')


def __dir__():
    return sorted(set(globals()) | set(__all__))
