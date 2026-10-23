"""Keyboard workflow and shared arm/hand gate, independent of device protocols."""
from __future__ import annotations

import threading
from .keyboard import HELP


class TeleopSession:
    def __init__(self, *, collector, task, set_following, ready, home, cancel_home,
                 recenter, quit_event, enabled=False):
        self.lock = threading.RLock()
        self.collector = collector
        self.task = task
        self.enabled = enabled
        self.following_reason = 'startup_enabled' if enabled else 'startup_paused'
        self.tracking_held = False
        self._set_following = set_following
        self._ready = ready
        self._home = home
        self._cancel_home = cancel_home
        self._recenter = recenter
        self.quit_event = quit_event
        self.cancelled = threading.Event()
        self.worker = None
        self.operation = None
        self.fault = None
        if collector is not None:
            collector.stop_for_limit = lambda reason: self.stop_recording(False, reason)

    def following(self, enabled, *, reason=None):
        with self.lock:
            reason = reason or ('keyboard_enable' if enabled else 'keyboard_pause')
            if enabled:
                if self.operation == 'home' or self.fault:
                    raise RuntimeError('Home is running or failed; complete home/restart before enabling')
                self._ready()
            # Publish the shared gate only after both device-side transitions
            # succeed. A failed enable must never leave one output following.
            self.enabled = False
            try:
                self._set_following(bool(enabled))
            except Exception:
                self.following_reason = 'enable_transition_failed'
                try:
                    self._set_following(False)
                except Exception as stop_error:
                    print(f'Enable rollback failed: {stop_error}', flush=True)
                if self.collector:
                    self.collector.control(False, 'enable_transition_failed')
                raise
            self.enabled = bool(enabled)
            self.following_reason = reason
            if self.collector:
                self.collector.control(self.enabled, reason)
            print('臂手跟随: ' + ('ENABLED' if enabled else 'PAUSED') + f' (reason={reason})', flush=True)

    def _background(self, name, operation):
        with self.lock:
            if self.operation is not None:
                raise RuntimeError(f'Wait for {self.operation} to finish')
            self.operation = name
            def run():
                try:
                    operation()
                except Exception as exc:
                    if name == 'home':
                        with self.lock:
                            self.fault = str(exc)
                            already_cancelled = self.cancelled.is_set()
                            self.cancelled.set()
                        if not already_cancelled:
                            try:
                                self._cancel_home()
                            except Exception as stop_error:
                                print(f'Home fault stop failed: {stop_error}', flush=True)
                    print(f'{name} failed: {exc}', flush=True)
                finally:
                    with self.lock:
                        self.operation = None
            self.worker = threading.Thread(target=run, name='teleop-' + name, daemon=False)
            try:
                self.worker.start()
            except BaseException:
                self.worker = None
                self.operation = None
                raise

    def stop_recording(self, success, reason="operator"):
        if self.collector is None:
            raise RuntimeError('Recording is disabled')
        with self.lock:
            if self.operation:
                raise RuntimeError(f'Wait for {self.operation} to finish')
            # Cut at the event on the control thread; drain and align elsewhere.
            detached = self.collector.detach_episode()
            def save():
                print('Saved:', self.collector.finish_episode(detached, success, reason=reason), flush=True)
            try:
                self._background('save', save)
            except BaseException:
                # Do not lose the detached writer when the OS cannot start a
                # worker. Pause control before the synchronous fallback.
                self.following(False)
                save()
                raise

    def handle(self, command):
        if command == 'e':
            self.following(not self.enabled)
        elif command == 'p':
            with self.lock:
                cancel_home = self.operation == 'home'
                if cancel_home:
                    self.cancelled.set()
                self.following(False)
            if cancel_home:
                self._cancel_home()
        elif command == 'c':
            with self.lock:
                if self.operation == 'home' or self.fault:
                    raise RuntimeError('Cannot recenter during/after failed home')
                self._recenter()
                if self.collector:
                    self.collector.control(False, 'keyboard_recenter')
        elif command == 'h':
            with self.lock:
                if self.enabled:
                    raise RuntimeError('Press p to pause arm and hand before h')
                if self.collector and self.collector.writer:
                    raise RuntimeError('Finish recording with s/f before home')
                if self._home is None:
                    raise RuntimeError('No home configured; load --task-config with a home section')
                self.cancelled.clear()
                def move():
                    self._home(self.cancelled)
                    if self.cancelled.is_set():
                        raise RuntimeError('Home interrupted; following remains disabled')
                    self.fault = None
                    print('Home complete; 臂手保持暂停，按 e 使能', flush=True)
                self._background('home', move)
        elif command == 'r':
            if self.collector is None:
                raise RuntimeError('Recording is disabled; use --capture or task collection.enabled')
            with self.lock:
                if not self.enabled or self.tracking_held or self.fault:
                    raise RuntimeError('Press e to enable arm and hand before r')
                self._ready()
                self._background('start', lambda: print('Recording:', self.collector.start_episode(self.task), flush=True))
        elif command in {'s', 'f'}:
            self.stop_recording(command == 's')
        elif command == 'd':
            if self.collector is None:
                raise RuntimeError('Recording is disabled')
            self._background('discard', lambda: print('Moved to trash:', self.collector.discard_last(), flush=True))
        elif command == '?':
            print(HELP + f' | following={self.enabled}, operation={self.operation}, fault={self.fault}', flush=True)
        else:
            raise ValueError(f'Unknown key {command!r}; press ? for help')

    def close(self):
        with self.lock:
            cancel_home = self.operation == 'home'
            if cancel_home:
                self.cancelled.set()
        try:
            self.following(False)
        except Exception as exc:
            self.enabled = False
            print(f'Pause during shutdown failed: {exc}', flush=True)
        if cancel_home:
            try:
                self._cancel_home()
            except Exception as exc:
                print(f'Home stop request failed: {exc}', flush=True)
        if self.worker:
            # A hardware owner must finish before its driver is disconnected.
            self.worker.join()
