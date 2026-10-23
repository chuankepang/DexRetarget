"""Single owner of stdin; single keys in a TTY, newline commands in pipes/SSH fallback."""
from __future__ import annotations

import os
import queue
import select
import sys
import termios
import threading
import tty

HELP = '键盘控制: e=臂手使能/暂停, p=暂停, c=重新定参考, h=回任务home, r=开始录制, s/f=成功/失败保存, d=回收上段, q=退出, ?=帮助'


class Keyboard:
    def __init__(self, quit_event):
        self.quit_event = quit_event
        self.commands = queue.Queue(maxsize=32)
        self.stopping = threading.Event()
        self.thread = None
        self.original = None
        self.fd = None

    def start(self):
        self.fd = sys.stdin.fileno()
        if os.isatty(self.fd):
            self.original = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
        print(HELP, flush=True)
        self.thread = threading.Thread(target=self._read, name='teleop-keyboard', daemon=True)
        self.thread.start()

    def _put(self, command):
        command = command.strip().lower()
        if command in {'q', '\x03'}:
            self.quit_event.set()
            return
        if command:
            try:
                self.commands.put_nowait(command)
            except queue.Full:
                print('键盘队列已满；命令未执行', flush=True)

    def _read(self):
        try:
            self._read_loop()
        except (OSError, ValueError) as exc:
            print(f'键盘输入失效，停止跟随: {exc}', flush=True)
            self.quit_event.set()

    def _read_loop(self):
        pending = b''
        while not self.stopping.is_set():
            if not select.select([self.fd], [], [], .1)[0]:
                continue
            chunk = os.read(self.fd, 256)
            if not chunk:
                # No operator channel remains: stop rather than keep following.
                self.quit_event.set()
                return
            if self.original is not None:
                for char in chunk.decode('utf-8', errors='ignore'):
                    self._put(char)
            else:
                pending += chunk
                while b'\n' in pending:
                    line, pending = pending.split(b'\n', 1)
                    self._put(line.decode('utf-8', errors='ignore'))

    def drain(self):
        while True:
            try:
                yield self.commands.get_nowait()
            except queue.Empty:
                return

    def close(self):
        self.stopping.set()
        try:
            if self.thread and self.thread.ident is not None and self.thread is not threading.current_thread():
                self.thread.join(timeout=2)
        finally:
            if self.original is not None:
                termios.tcsetattr(self.fd, termios.TCSADRAIN, self.original)
                self.original = None
