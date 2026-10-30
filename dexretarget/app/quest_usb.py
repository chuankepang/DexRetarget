"""USB startup orchestration; no robot, camera or recording dependencies."""
from __future__ import annotations

import os
import subprocess
import time


def resolve_usb_transport(args, explicit=()):
    """Keep Wi-Fi overrides usable even when a task selects USB by default."""
    if not getattr(args, 'quest3_usb', False):
        return
    if 'quest3_protocol' in explicit and args.quest3_protocol != 'tcp':
        if 'quest3_usb' in explicit:
            raise ValueError('--quest3-usb requires --quest3-protocol tcp')
        args.quest3_usb = False
        return
    args.quest3_protocol = 'tcp'
    profile = getattr(args, 'task_profile', None) or {}
    if 'quest3_port' not in explicit and 'quest3_port' not in profile.get('teleop', {}):
        args.quest3_port = 8000


def _adb(*arguments):
    try:
        result = subprocess.run(
            ['adb', *arguments], capture_output=True, text=True, timeout=10,
        )
    except FileNotFoundError as exc:
        raise RuntimeError('USB Quest requires adb on PATH') from exc
    except subprocess.TimeoutExpired as exc:
        raise RuntimeError('ADB timed out; check the Quest USB connection') from exc
    if result.returncode:
        raise RuntimeError(f'ADB failed: {(result.stderr or result.stdout).strip()}')
    return result.stdout.strip()


def prepare_usb(port):
    """Select one authorized device and create only this port's reverse rule."""
    rows = [line.split() for line in _adb('devices').splitlines()[1:] if line.strip()]
    devices = {row[0]: row[1] for row in rows if len(row) >= 2}
    serial = os.environ.get('ANDROID_SERIAL')
    if serial is None:
        if len(devices) != 1:
            raise RuntimeError('Connect one Quest USB device, or select it with ANDROID_SERIAL')
        serial = next(iter(devices))
    if devices.get(serial) != 'device':
        raise RuntimeError(f'Quest {serial} is not authorized/online; confirm USB debugging in the headset')
    _adb('-s', serial, 'reverse', f'tcp:{port}', f'tcp:{port}')
    print(f'Quest USB: {serial}; ADB reverse TCP {port} ready.', flush=True)
    return serial


def open_hts(serial):
    """Open the installed app; HTS still owns its Start Streaming button."""
    result = _adb('-s', serial, 'shell', 'am', 'start',
                  '-a', 'android.intent.action.MAIN',
                  '-c', 'android.intent.category.LAUNCHER',
                  '-p', 'com.wengmister.handtrackingstreamer')
    if 'Error:' in result or 'Exception' in result:
        raise RuntimeError(f'Cannot open HTS: {result}')


def wait_for_usb_tracking(receiver, hand, *, require_landmarks):
    """Wait unpowered, without a startup deadline; Ctrl+C uses runtime cleanup.

    Only initial acquisition waits indefinitely. The existing realtime source
    timeout, hold/recovery and post-start fresh-packet barrier remain in force.
    """
    print('等待 Quest HTS 数据（不限时，Ctrl+C 退出；尚未初始化机械臂）。\n'
          '检查 HTS 的模式、目标地址和端口与通信配置一致；按 Start Streaming，保持手部可见。', flush=True)
    next_status = time.monotonic() + 5
    while True:
        if receiver.error is not None:
            raise RuntimeError('Quest receiver failed before robot initialization') from receiver.error
        tracking = receiver.get_hand_tracking_data(hand)
        now = time.monotonic()
        wrist = tracking['wrist']
        fresh_wrist = wrist is not None and 0 <= now - wrist['timestamp'] < .20
        stamp = tracking['landmarks_timestamp']
        fresh_fingers = (tracking['fingers'] is not None and stamp is not None
                         and 0 <= now - stamp < .20)
        if fresh_wrist and (fresh_fingers or not require_landmarks):
            print('Quest 数据已就绪，继续初始化；按任务配置执行回 home。', flush=True)
            return
        if now >= next_status:
            stats = receiver.stats
            print(f'仍在等 {hand}：wrist={fresh_wrist}, landmarks={fresh_fingers}, '
                  f"接收字节={stats['bytes']}（Ctrl+C 退出）", flush=True)
            next_status = now + 5
        time.sleep(.01)
