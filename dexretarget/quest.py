"""Terminal-owned Quest UDP or USB TCP communication service; no robot outputs."""
from __future__ import annotations

import argparse
from pathlib import Path
import signal
import threading


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=Path('config/quest/usb_tcp.yaml'),
                        help='Quest communication YAML: udp.yaml or usb_tcp.yaml')
    parser.add_argument('--port', type=int, default=None, help='Override the configured HTS port')
    parser.add_argument('--socket', help='Override the local Unix socket path')
    parser.add_argument('--no-adb', action='store_true', help='Skip ADB and App launch; for manually prepared input/testing')
    args = parser.parse_args(argv)
    from dexretarget.quest_config import load_quest_config
    try:
        config = load_quest_config(args.config, port=args.port, socket_path=args.socket)
    except (OSError, ValueError, TypeError) as exc:
        parser.error(str(exc))
    from dexretarget.teleop.devices.quest_relay import QuestRelayServer
    from dexretarget.app.quest_usb import prepare_usb, open_hts
    server = None
    stop = threading.Event()
    # Repeated Ctrl+C/SIGTERM requests the same bounded cleanup instead of
    # raising KeyboardInterrupt again in Thread.join and leaving stale sockets.
    previous = {sig: signal.signal(sig, lambda *unused: stop.set())
                for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        # Own listener before ADB/App startup; duplicate instances cannot change
        # another service's reverse rule or take over its socket path.
        server = QuestRelayServer(config['socket_path'], host=config['bind_host'],
                                  port=config['port'], protocol=config['protocol'])
        if stop.is_set():
            return
        if config['mode'] == 'usb_tcp' and not args.no_adb:
            serial = prepare_usb(config['port'])
            if stop.is_set():
                return
            open_hts(serial)
        if stop.is_set():
            return
        print(f"Quest 通信服务已就绪：{config['mode']} {config['bind_host']}:{config['port']} → {server.path}", flush=True)
        if config['mode'] == 'udp':
            print(f"HTS 选择 UDP，目标填电脑 Wi-Fi IPv4，端口 {config['port']}；电脑与 Quest 接同一可互通 Wi-Fi。", flush=True)
        else:
            print(f"HTS 选择 TCP Wired/ADB，目标 127.0.0.1:{config['port']}。", flush=True)
        print('在 HTS 按一次 Start Streaming；此终端保持运行。\n'
              '遥操作可独立启动/退出；本服务 Ctrl+C 停止接收。', flush=True)
        server.run(stop)
    except KeyboardInterrupt:
        pass
    except (OSError, RuntimeError, TimeoutError) as exc:
        if not stop.is_set():
            parser.exit(1, f'Quest 通信服务启动/运行失败：{exc}\n'
                        '检查是否已运行服务、接收端口占用及所选连接配置。\n')
    finally:
        try:
            if server is not None:
                stats = server.receiver.stats
                server.close()
                print(f"Quest 通信已关闭：接收 {stats['packets']} 包 / {stats['bytes']} 字节；"
                      f'慢客户端跳过 {server.skipped_updates} 次更新。', flush=True)
        finally:
            for sig, handler in previous.items():
                signal.signal(sig, handler)


if __name__ == '__main__':
    main()
