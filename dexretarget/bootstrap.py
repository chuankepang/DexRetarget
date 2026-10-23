"""Select the active Conda C++ runtime before numerical extensions load."""
import os
import sys
from pathlib import Path

def _ensure_conda_cpp_runtime() -> None:
    """Re-exec with this conda environment's C++ runtime ahead of the OS one.

    Jetson's system libstdc++ can be older than the build used by Pinocchio.
    Activating conda does not always update LD_LIBRARY_PATH, and changing it
    after extension loading is too late.  Re-exec once, before numpy,
    Pinocchio, or the ROKAE extension is imported.
    """
    marker = "DEXRETARGET_CONDA_LIB_REEXEC"
    if os.environ.get(marker) == "1":
        return
    conda_lib = Path(sys.prefix).resolve() / "lib"
    conda_libstdcpp = conda_lib / "libstdc++.so.6"
    if not conda_libstdcpp.exists():
        return
    current = os.environ.get("LD_LIBRARY_PATH", "")
    entries = [entry for entry in current.split(os.pathsep) if entry]
    if entries and Path(entries[0]).resolve() == conda_lib:
        return
    environment = os.environ.copy()
    environment["LD_LIBRARY_PATH"] = os.pathsep.join(
        [str(conda_lib), *entries]
    )
    environment[marker] = "1"
    print(
        f"Restarting with conda C++ runtime: {conda_lib}",
        file=sys.stderr,
        flush=True,
    )
    # Preserve -m/-c and interpreter flags. sys.argv alone turns a module
    # launch into direct __main__.py execution and breaks relative imports.
    original = getattr(sys, "orig_argv", [sys.executable, *sys.argv])
    os.execve(sys.executable, [sys.executable, *original[1:]], environment)
