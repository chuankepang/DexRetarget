"""Linux local relay: one HTS receiver, bounded nonblocking local subscribers.

Snapshots contain converted coordinates and ORIGINAL host receive timestamps.
The control loop only reads its local cache; it never performs socket I/O.
"""
from __future__ import annotations

import os
import errno
import fcntl
from contextlib import ExitStack
from pathlib import Path
import selectors
import select
import socket
import struct
import stat
import threading

import numpy as np

from .quest3 import Quest3, QuestTrackingState, _HandState
from dexretarget.quest_config import transport_socket_path

# Fixed-size, versioned local wire format; no pickle, JSON or CSV re-parsing.
# Header: magic + source counters. Each hand: timestamps, xyz, xyzw, 21 xyz.
_COUNTERS = ('bytes', 'packets', 'valid_wrist', 'valid_landmarks',
             'valid_left_wrist', 'valid_left_landmarks', 'valid_right_wrist', 'valid_right_landmarks')
_PACKET = struct.Struct('<8s8Q144d')
_MAGIC = b'QUESTv1\0'


def default_socket_path():
    return transport_socket_path()


def encode_snapshot(receiver):
    values = []
    with receiver._lock:
        for side in ('left', 'right'):
            hand = receiver._hands[side]
            values.extend([hand.wrist_last_update or 0., hand.landmarks_last_update or 0.])
            values.extend(hand.wrist_position if hand.wrist_position is not None else [0.]*3)
            values.extend(hand.wrist_quat if hand.wrist_quat is not None else [0.]*4)
            values.extend(hand.landmarks_local.ravel() if hand.landmarks_local is not None else [0.]*63)
        return _PACKET.pack(_MAGIC, *(receiver._stats[key] for key in _COUNTERS), *values)


def apply_snapshot(receiver, packet):
    if len(packet) != _PACKET.size:
        raise ValueError('Invalid Quest relay snapshot size')
    fields = _PACKET.unpack(packet)
    magic, values = fields[0], fields[9:]
    if magic != _MAGIC or not np.all(np.isfinite(values)):
        raise ValueError('Invalid Quest relay snapshot version/values')
    with receiver._lock:
        for index, side in enumerate(('left', 'right')):
            data = values[index*72:(index+1)*72]
            hand = receiver._hands[side]
            # Do not refresh timestamps on relay delivery or heartbeat.
            hand.wrist_last_update = data[0] or None
            hand.landmarks_last_update = data[1] or None
            hand.wrist_position = np.asarray(data[2:5]) if data[0] else None
            hand.wrist_quat = np.asarray(data[5:9]) if data[0] else None
            hand.landmarks_local = np.asarray(data[9:]).reshape(21, 3) if data[1] else None
            hand.last_update = max(data[:2])
        receiver._stats.update(zip(_COUNTERS, fields[1:9]))
        receiver._stats['last_peer'] = 'local Quest relay'


class QuestRelayInput(QuestTrackingState):
    """Duck-compatible input with a background reader and no USB/TCP ownership."""

    def __init__(self, path=None):
        super().__init__()
        self.path = Path(path) if path is not None else default_socket_path()
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._socket = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        try:
            self._socket.settimeout(2.)
            self._socket.connect(str(self.path))
        except BaseException:
            self._socket.close()
            raise
        self._socket.setblocking(False)
        self._thread = threading.Thread(target=self._run, daemon=True, name='quest-local-input')
        self._thread.start()

    def _run(self):
        try:
            while not self._stop.is_set():
                if not select.select([self._socket], [], [], 2.)[0]:
                    raise TimeoutError('Quest communication service is not responding')
                packet = self._socket.recv(_PACKET.size+1)
                if not packet:
                    raise ConnectionError('Quest communication service stopped')
                # Drain the bounded local socket queue; only apply the newest
                # snapshot, even if this process was descheduled for a while.
                while True:
                    try:
                        newer = self._socket.recv(_PACKET.size+1, socket.MSG_DONTWAIT)
                    except BlockingIOError:
                        break
                    if not newer:
                        raise ConnectionError('Quest communication service stopped')
                    packet = newer
                apply_snapshot(self, packet)
                self._ready.set()
        except Exception as exc:
            if not self._stop.is_set():
                with self._lock:
                    self._hands = {side: _HandState(side) for side in ('left', 'right')}
                    self._error = exc
        finally:
            self._ready.set()

    def wait_until_ready(self, timeout=5.):
        if not self._ready.wait(timeout):
            raise TimeoutError('No snapshot from Quest communication service')
        if self.error is not None:
            raise RuntimeError('Quest communication service failed') from self.error

    def stop(self):
        self._stop.set()
        try:
            self._socket.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self._thread.join(timeout=2.5)
        self._socket.close()


class QuestRelayServer:
    """Owns the HTS connection; subscriber exits never close its receiver."""

    def __init__(self, path=None, *, host='127.0.0.1', port=8000, protocol='tcp'):
        self.path = Path(path) if path is not None else default_socket_path()
        self._selector = selectors.DefaultSelector()
        self._listener = None
        self._wake_read = self._wake_write = None
        self.receiver = None
        self._clients = set()
        self.skipped_updates = 0
        self.sent_snapshots = 0
        self.local_bytes = 0
        self._identity = None
        self._lock_fd = None
        self._closed = False
        try:
            self.path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
            self._claim_endpoint()
            self._listener = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
            self._listener.bind(str(self.path))
            info = self.path.lstat()
            self._identity = (info.st_dev, info.st_ino)
            self.path.chmod(0o600)
            self._listener.listen(8)
            self._listener.setblocking(False)
            self._wake_read, self._wake_write = socket.socketpair(socket.AF_UNIX, socket.SOCK_DGRAM)
            self._wake_read.setblocking(False)
            self._wake_write.setblocking(False)
            self._selector.register(self._listener, selectors.EVENT_READ, 'accept')
            self._selector.register(self._wake_read, selectors.EVENT_READ, 'update')
            self.receiver = Quest3(host=host, port=port, protocol=protocol, on_update=self._notify)
            self.receiver.wait_until_ready()
        except BaseException:
            self.close()
            raise

    def _claim_endpoint(self):
        # Keep this small lock file across restarts: unlinking a locked file
        # would let another process lock a different inode at the same path.
        self._lock_fd = os.open(str(self.path)+'.lock',
                                os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
        try:
            fcntl.flock(self._lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise OSError(errno.EADDRINUSE, f'Quest service already running: {self.path}') from exc
        try:
            stale = self.path.lstat()
        except FileNotFoundError:
            return
        if not stat.S_ISSOCK(stale.st_mode) or stale.st_uid != os.getuid():
            raise OSError(errno.EEXIST, f'Refusing to replace a non-owned socket: {self.path}')
        # Probe for old versions which do not acquire our lock. Only a refused
        # connection proves a leftover endpoint; timeout/permission errors do not.
        with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as probe:
            probe.settimeout(.2)
            try:
                probe.connect(str(self.path))
            except ConnectionRefusedError:
                pass
            except FileNotFoundError:
                return
            else:
                raise OSError(errno.EADDRINUSE, f'Quest socket is still active: {self.path}')
        current = self.path.lstat()
        if (current.st_dev, current.st_ino) != (stale.st_dev, stale.st_ino):
            raise OSError(errno.EADDRINUSE, f'Quest socket changed during startup: {self.path}')
        self.path.unlink()

    def _unlink_endpoint(self):
        if self._identity is None:
            return
        try:
            current = self.path.lstat()
        except FileNotFoundError:
            return
        if (current.st_dev, current.st_ino) == self._identity:
            self.path.unlink()

    def _notify(self):
        try:
            self._wake_write.send(b'u')
        except (BlockingIOError, OSError):
            pass  # An existing pending notification covers the latest state.

    def _drop(self, client):
        self._clients.discard(client)
        self._selector.unregister(client)
        client.close()

    def _send(self, client, packet):
        try:
            sent = client.send(packet)
            self.sent_snapshots += 1
            self.local_bytes += sent
        except BlockingIOError:
            self.skipped_updates += 1  # Never block the upstream HTS receiver.
        except OSError:
            self._drop(client)

    def run(self, stop_event=None):
        stop_event = stop_event or threading.Event()
        while not stop_event.is_set():
            if self.receiver.error is not None:
                raise RuntimeError('HTS receiver failed') from self.receiver.error
            for key, _ in self._selector.select(timeout=.5):
                if key.data == 'accept':
                    client, _ = self._listener.accept()
                    client.setblocking(False)
                    client.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4096)
                    self._clients.add(client)
                    self._selector.register(client, selectors.EVENT_READ, 'client')
                elif key.data == 'update':
                    while True:
                        try:
                            self._wake_read.recv(4096)
                        except BlockingIOError:
                            break
                else:
                    # Subscribers are read-only. EOF, errors or unexpected
                    # writes affect only this local connection.
                    self._drop(key.fileobj)
            if self._clients:
                # Also send a heartbeat on idle periods; stale source timestamps
                # remain stale. No subscribers => no binary serialization.
                packet = encode_snapshot(self.receiver)
                for client in list(self._clients):
                    self._send(client, packet)

    def close(self):
        if self._closed:
            return
        self._closed = True
        # Release all local resources even if stopping a receiver fails.
        # The process lock is released last, after unlinking only our endpoint.
        with ExitStack() as cleanup:
            if self._lock_fd is not None:
                cleanup.callback(os.close, self._lock_fd)
            cleanup.callback(self._unlink_endpoint)
            cleanup.callback(self._selector.close)
            for sock in (self._listener, self._wake_read, self._wake_write):
                if sock is not None:
                    cleanup.callback(sock.close)
            for client in list(self._clients):
                cleanup.callback(self._drop, client)
            if self.receiver is not None:
                cleanup.callback(self.receiver.stop)
