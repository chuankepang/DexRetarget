"""ROKAE Cartesian drivers with a mock backend and lazy xCoreSDK loading."""

from __future__ import annotations

import ctypes
import os
import threading
import time
from abc import ABC, abstractmethod
from pathlib import Path
from typing import Optional

import numpy as np

from anydexretarget.teleop.pose import validate_transform


class RokaeDriverBase(ABC):
    """Common latest-target interface for mock and real ROKAE backends."""

    @abstractmethod
    def connect(self) -> None:
        """Connect without starting Cartesian motion."""

    @abstractmethod
    def start(self, power_on: bool = False) -> None:
        """Start the backend's periodic Cartesian-position control loop."""

    @abstractmethod
    def get_tcp_pose(self) -> np.ndarray:
        """Return the latest TCP pose relative to the robot base."""

    @abstractmethod
    def set_target_pose(self, base_T_tcp_target: np.ndarray) -> None:
        """Replace the latest target; commands are never queued."""

    @abstractmethod
    def hold(self) -> None:
        """Keep the last safe Cartesian target."""

    @abstractmethod
    def stop(self) -> None:
        """Stop the periodic control loop."""

    @abstractmethod
    def disconnect(self) -> None:
        """Release backend resources."""


class MockRokaeDriver(RokaeDriverBase):
    """Hardware-free latest-value backend with a simulated realtime thread."""

    def __init__(
        self,
        initial_pose: np.ndarray,
        control_hz: float = 1000.0,
        command_timeout: float = 0.20,
    ) -> None:
        if control_hz <= 0.0:
            raise ValueError("control_hz must be positive")
        if command_timeout <= 0.0:
            raise ValueError("command_timeout must be positive")
        pose = validate_transform(initial_pose, "mock initial pose")
        self._current_pose = pose.copy()
        self._latest_target = pose.copy()
        self._control_period = 1.0 / float(control_hz)
        self._command_timeout = float(command_timeout)
        self._last_target_time = time.monotonic()
        self._connected = False
        self._running = False
        self._timed_out = False
        self._command_count = 0
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None

    @property
    def command_count(self) -> int:
        with self._lock:
            return self._command_count

    @property
    def timed_out(self) -> bool:
        with self._lock:
            return self._timed_out

    def connect(self) -> None:
        self._connected = True

    def start(self, power_on: bool = False) -> None:
        del power_on
        if not self._connected:
            raise RuntimeError("Mock ROKAE driver is not connected")
        if self._running:
            return
        self._stop_event.clear()
        self._running = True
        self._last_target_time = time.monotonic()
        self._thread = threading.Thread(
            target=self._control_loop,
            name="mock-rokae-rt",
            daemon=True,
        )
        self._thread.start()

    def _control_loop(self) -> None:
        next_cycle = time.monotonic()
        while not self._stop_event.is_set():
            now = time.monotonic()
            with self._lock:
                self._timed_out = now - self._last_target_time > self._command_timeout
                # A Cartesian position backend holds the last command on timeout.
                self._current_pose = self._latest_target.copy()
                self._command_count += 1
            next_cycle += self._control_period
            delay = next_cycle - time.monotonic()
            if delay > 0.0:
                self._stop_event.wait(delay)
            else:
                next_cycle = time.monotonic()

    def get_tcp_pose(self) -> np.ndarray:
        with self._lock:
            return self._current_pose.copy()

    def set_target_pose(self, base_T_tcp_target: np.ndarray) -> None:
        if not self._running:
            raise RuntimeError("Mock ROKAE realtime loop is not running")
        target = validate_transform(base_T_tcp_target, "ROKAE target")
        with self._lock:
            self._latest_target = target
            self._last_target_time = time.monotonic()
            self._timed_out = False

    def hold(self) -> None:
        with self._lock:
            self._latest_target = self._current_pose.copy()
            self._last_target_time = time.monotonic()

    def stop(self) -> None:
        if not self._running:
            return
        self._stop_event.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=2.0)
        self._thread = None
        self._running = False

    def disconnect(self) -> None:
        self.stop()
        self._connected = False


class RokaeXCoreDriver(RokaeDriverBase):
    """ctypes frontend for the small xCoreSDK C++ realtime bridge.

    Importing or constructing this class does not load the vendor library and
    does not connect to a robot. The bridge is loaded only by ``connect()``.
    """

    _ROBOT_TYPES = {"xmate-6", "xmate-er-pro-7", "standard-6"}

    def __init__(
        self,
        robot_ip: str,
        local_ip: str,
        robot_type: str = "xmate-er-pro-7",
        rt_network_tolerance: int = 20,
        bridge_library: Optional[Path] = None,
    ) -> None:
        if robot_type not in self._ROBOT_TYPES:
            raise ValueError(
                f"robot_type must be one of {sorted(self._ROBOT_TYPES)}, got {robot_type}"
            )
        if not 0 <= rt_network_tolerance <= 100:
            raise ValueError("rt_network_tolerance must be in [0, 100]")
        self.robot_ip = robot_ip
        self.local_ip = local_ip
        self.robot_type = robot_type
        self.rt_network_tolerance = int(rt_network_tolerance)
        default_library = (
            Path(__file__).resolve().parent
            / "rokae_cpp"
            / "build"
            / "libanydex_rokae_bridge.so"
        )
        env_library = os.environ.get("ANYDEX_ROKAE_BRIDGE")
        self.bridge_library = Path(
            bridge_library or env_library or default_library
        ).expanduser()
        self._library: Optional[ctypes.CDLL] = None
        self._handle: Optional[int] = None
        self._connected = False
        self._started = False

    @staticmethod
    def _configure_api(library: ctypes.CDLL) -> None:
        double_pointer = ctypes.POINTER(ctypes.c_double)
        library.anydex_rokae_create.argtypes = [
            ctypes.c_char_p,
            ctypes.c_char_p,
            ctypes.c_size_t,
        ]
        library.anydex_rokae_create.restype = ctypes.c_void_p
        library.anydex_rokae_connect.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_char_p,
            ctypes.c_uint,
        ]
        library.anydex_rokae_connect.restype = ctypes.c_int
        library.anydex_rokae_start.argtypes = [ctypes.c_void_p, ctypes.c_int]
        library.anydex_rokae_start.restype = ctypes.c_int
        library.anydex_rokae_get_tcp_pose.argtypes = [ctypes.c_void_p, double_pointer]
        library.anydex_rokae_get_tcp_pose.restype = ctypes.c_int
        library.anydex_rokae_set_target_pose.argtypes = [
            ctypes.c_void_p,
            double_pointer,
        ]
        library.anydex_rokae_set_target_pose.restype = ctypes.c_int
        library.anydex_rokae_hold.argtypes = [ctypes.c_void_p]
        library.anydex_rokae_hold.restype = ctypes.c_int
        library.anydex_rokae_stop.argtypes = [ctypes.c_void_p]
        library.anydex_rokae_stop.restype = ctypes.c_int
        library.anydex_rokae_disconnect.argtypes = [ctypes.c_void_p]
        library.anydex_rokae_disconnect.restype = ctypes.c_int
        library.anydex_rokae_last_error.argtypes = [ctypes.c_void_p]
        library.anydex_rokae_last_error.restype = ctypes.c_char_p
        library.anydex_rokae_destroy.argtypes = [ctypes.c_void_p]
        library.anydex_rokae_destroy.restype = None

    def _error_text(self) -> str:
        if self._library is None:
            return "ROKAE bridge is not loaded"
        message = self._library.anydex_rokae_last_error(self._handle)
        return message.decode("utf-8", errors="replace") if message else "unknown error"

    def _check(self, return_code: int, operation: str) -> None:
        if return_code != 0:
            raise RuntimeError(f"ROKAE {operation} failed: {self._error_text()}")

    def connect(self) -> None:
        if self._connected:
            return
        if not self.bridge_library.is_file():
            raise FileNotFoundError(
                f"ROKAE bridge not found: {self.bridge_library}. Build it with "
                "`bash example/output/real/rokae_cpp/build_bridge.sh`."
            )
        try:
            library = ctypes.CDLL(str(self.bridge_library))
        except OSError as exc:
            raise RuntimeError(
                "Failed to load the ROKAE bridge/vendor runtime. Check its RPATH, "
                "architecture, and xCoreSDK shared library."
            ) from exc
        self._configure_api(library)
        error_buffer = ctypes.create_string_buffer(1024)
        handle = library.anydex_rokae_create(
            self.robot_type.encode(), error_buffer, len(error_buffer)
        )
        if not handle:
            raise RuntimeError(
                "ROKAE bridge create failed: "
                + error_buffer.value.decode("utf-8", errors="replace")
            )
        self._library = library
        self._handle = int(handle)
        try:
            self._check(
                library.anydex_rokae_connect(
                    self._handle,
                    self.robot_ip.encode(),
                    self.local_ip.encode(),
                    self.rt_network_tolerance,
                ),
                "connect",
            )
        except Exception:
            library.anydex_rokae_destroy(self._handle)
            self._handle = None
            self._library = None
            raise
        self._connected = True

    def start(self, power_on: bool = False) -> None:
        if not self._connected or self._library is None or self._handle is None:
            raise RuntimeError("ROKAE driver is not connected")
        if self._started:
            return
        self._check(
            self._library.anydex_rokae_start(self._handle, int(power_on)),
            "start realtime Cartesian control",
        )
        self._started = True

    def get_tcp_pose(self) -> np.ndarray:
        if not self._connected or self._library is None or self._handle is None:
            raise RuntimeError("ROKAE driver is not connected")
        output = np.empty(16, dtype=np.float64)
        self._check(
            self._library.anydex_rokae_get_tcp_pose(
                self._handle,
                output.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            ),
            "read TCP pose",
        )
        return validate_transform(output.reshape(4, 4), "ROKAE TCP pose")

    def set_target_pose(self, base_T_tcp_target: np.ndarray) -> None:
        if not self._started or self._library is None or self._handle is None:
            raise RuntimeError("ROKAE realtime Cartesian control is not started")
        target = np.ascontiguousarray(
            validate_transform(base_T_tcp_target, "ROKAE target").reshape(-1),
            dtype=np.float64,
        )
        self._check(
            self._library.anydex_rokae_set_target_pose(
                self._handle,
                target.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            ),
            "set target pose",
        )

    def hold(self) -> None:
        if self._started and self._library is not None and self._handle is not None:
            self._check(self._library.anydex_rokae_hold(self._handle), "hold")

    def stop(self) -> None:
        if not self._started or self._library is None or self._handle is None:
            return
        self._check(self._library.anydex_rokae_stop(self._handle), "stop")
        self._started = False

    def disconnect(self) -> None:
        library = self._library
        handle = self._handle
        if library is None or handle is None:
            return
        try:
            if self._started:
                self.stop()
            if self._connected:
                self._check(library.anydex_rokae_disconnect(handle), "disconnect")
        finally:
            library.anydex_rokae_destroy(handle)
            self._handle = None
            self._library = None
            self._connected = False
            self._started = False


__all__ = ["MockRokaeDriver", "RokaeDriverBase", "RokaeXCoreDriver"]
