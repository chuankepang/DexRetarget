# -*- coding: utf-8 -*-
"""
KUKA IIWA 机械臂控制柜 TCP 通讯最小实现（R2V 读取 / V2R 控制）
协议提取自 GRobotSimulator:
    source/function/adapter/kuka_iiwa/kuka_iiwa_encoder.cpp  (V2R 编码)
    source/function/adapter/kuka_iiwa/kuka_iiwa_decoder.cpp  (R2V 解码)
    source/manager/tcpsocketmanager.cpp                      (连接/收包/分帧)
    source/system/networksystem.cpp                          (文本命令/运动指令封装)
    source/system/transmitsystem.cpp                         (R2V / V2R 数据流向)

与上位机操作的对应关系:
    Network 窗口 connect      -> 本文件 connect()
    Network 窗口 request      -> request_controller()   (Observer -> Controller)
    Network 窗口 async  On/Off-> start_async()/stop_async()
    Network 窗口 mode = R2V   -> 读机械臂: on_data 回调 / get_joints()
    Network 窗口 mode = V2R   -> 控制机械臂: smart_servo 流式发关节角
    Planning 窗口 PTP(输入关节角, 求解规划轨迹)
                              -> move_joints_ptp(): 内部做梯形插值并流式 SmartServo
                                 (真机只支持 SmartServo 流式指令, PTPMotion 的轨迹
                                  在上位机里也是逐帧 SmartServo 发出去的)

用法:
    python kuka_iiwa.py                      # 交互式: 连接->请求控制权->读写
    python kuka_iiwa.py --read               # 仅读取打印机械臂状态
"""

import argparse
import struct
import threading
import time
import socket

# ----------------------------------------------------------------------------
# 协议常量
# ----------------------------------------------------------------------------
END_SYMBOL = b"#"            # 命令结束符
FRAME_SEP = b"\xef\xef"      # R2V 数据帧分隔符(连续两个 0xEF)

# R2V 数据类型 (kuka_iiwa_decoder.h DataType)
DT_JOINT_CUR_POS = 0        # 7 关节当前角, ×0.0001 rad
DT_JOINT_CMD_POS = 1        # 7 关节指令角, ×0.0001 rad
DT_CART_CUR_POS = 2         # 6 笛卡尔位姿, 前3×0.1(mm) 后3×0.0001(rad)
DT_CART_CMD_POS = 3
DT_WRENCH = 4               # 6 力/力矩, ×0.01 (N / Nm)
DT_EXT_J_TORQUE = 5         # 7 关节外部力矩, ×0.01 Nm
DT_RAW_J_TORQUE = 6         # 7 关节原始力矩, ×0.01 Nm

# V2R 命令枚举 (kuka_iiwa_encoder.h)
MSG_ROBOT_MOVE = 0
MSG_GRIPPER_MOVE = 3
MSG_GRIPPER_MODE = 4

MOVE_MOVE = 0        # PTP(编码保留, 真机未使用)
MOVE_ASYNC = 1       # SmartServo 异步运动(真机实际使用)
MOVE_BASIC = 2       # 夹爪 Basic 模式
MOVE_PINCH = 3
MOVE_WIDE = 4
MOVE_SCISSOR = 5

MOTION_PTP = 0
MOTION_SMART_SERVO = 5
MOTION_GRASP = 8
MOTION_RELEASE = 9

GOAL_JOINTS = 0

RANK_MSGS = {
    "MSG:LINK_SUCESS": 1,             # 连接成功, Observer
    "MSG:NOT_OBSERVER_ANYMORE": 2,    # Requester
    "MSG:WAIT_FOR_CONTROLLER": 2,
    "MSG:GET_HIGHEST_AURTHORITY": 3,  # Controller(可控制)
}
RANK_NAMES = {0: "None", 1: "Observer", 2: "Requester", 3: "Controller"}


class KukaIiwa:
    """KUKA IIWA 控制柜 TCP 客户端。一台机械臂一个连接。"""

    def __init__(self):
        self._sock = None
        self._recv_buf = b""
        self._send_lock = threading.Lock()
        self._data_lock = threading.Lock()
        self._running = False
        self._recv_thread = None
        # 最新一帧数据缓存(R2V)
        self._latest = {}
        self.rank = 0
        self.async_on = False
        # 用户回调
        self.on_data = None     # on_data(data_type:int, values:list[float])
        self.on_rank = None     # on_rank(rank:int, text:str)
        self.on_async = None    # on_async(flag:bool)

    # ---------------- 连接管理 ----------------
    def connect(self, ip: str, port: int, timeout: float = 3.0):
        """TCP 连接控制柜(对应 Network 窗口 connect 按钮)"""
        self._sock = socket.create_connection((ip, port), timeout=timeout)
        self._sock.settimeout(None)
        self._running = True
        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._recv_thread.start()

    def disconnect(self):
        """断开(对应 quit 按钮 / 窗口关闭, 发送 quit#)"""
        if self._sock is None:
            return
        self._running = False
        try:
            self._send_text("quit")
        except OSError:
            pass
        try:
            self._sock.close()
        except OSError:
            pass
        self._sock = None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.disconnect()

    # ---------------- V2R 文本命令 ----------------
    def _send_text(self, cmd: str):
        data = cmd.encode("ascii") + END_SYMBOL
        with self._send_lock:
            self._sock.sendall(data)

    def _send_binary(self, data: bytes):
        with self._send_lock:
            self._sock.sendall(data)

    def request_controller(self):
        """请求最高控制权(request 按钮, REQUEST#)。成功后 rank 变 3。"""
        self._send_text("REQUEST")

    def start_async(self):
        """开启异步数据上报(async 按钮 On, GETASYNC#)"""
        self._send_text("GETASYNC")

    def stop_async(self):
        """停止异步数据上报(async 按钮 Off, GETSTOP#)"""
        self._send_text("GETSTOP")

    def cancel(self):
        """取消/清空当前运动指令(CLEANUP#)"""
        self._send_text("CLEANUP")

    def sync_joints(self):
        """同步请求一帧关节角(GETSYNCJOINTS#)"""
        self._send_text("GETSYNCJOINTS")

    def sync_pose(self):
        """同步请求一帧笛卡尔位姿(GETSYNCPOSE#)"""
        self._send_text("GETSYNCPOSE")

    def sync_force(self):
        """同步请求一帧末端力/力矩(GETSYNCFORCE#)"""
        self._send_text("GETSYNCFORCE")

    def sync_ext_torque(self):
        """同步请求一帧关节外部力矩(GETSYNCEXTTORQUE#)"""
        self._send_text("GETSYNCEXTTORQUE")

    # ---------------- V2R 二进制运动命令 ----------------
    @staticmethod
    def _encode_joints(joints) -> bytes:
        """7 个关节角(rad) -> 目标类型字节 + 7 个大端 short(角度×10000)"""
        assert len(joints) == 7, "KUKA IIWA 有且仅有 7 个关节"
        payload = bytearray([GOAL_JOINTS])
        for angle in joints:
            payload += struct.pack(">h", int(angle * 10000.0))
        return bytes(payload)

    def _wrap_command(self, header: bytes, payload: bytes = b"") -> bytes:
        return b"_CM" + header + payload + b"_EE" + END_SYMBOL

    def smart_servo(self, joints):
        """
        发送一帧 SmartServo 异步关节指令(真机实际使用的运动命令)。
        需要以固定频率(约 50~100Hz)持续调用, 中途停止需发 cancel()。
        """
        header = bytes([MSG_ROBOT_MOVE, MOVE_ASYNC, MOTION_SMART_SERVO, 1])
        self._send_binary(self._wrap_command(header, self._encode_joints(joints)))

    def move_joints_ptp(self, target_joints, duration: float = 2.0,
                        rate_hz: float = 50.0):
        """
        从当前关节角梯形/线性插值运动到 target_joints(对应 Planning 窗口 PTP):
        上位机里 PTPMotion 生成轨迹后, 也是逐帧 SmartServo 下发, 这里做同样的事。
        duration: 运动时长(秒); rate_hz: 下发频率。
        """
        start = self.get_joints()
        if start is None:
            raise RuntimeError("读取不到当前关节角, 请先 start_async() 等待数据")
        target = list(target_joints)
        n_steps = max(1, int(duration * rate_hz))
        dt = 1.0 / rate_hz
        for i in range(1, n_steps + 1):
            t = i / n_steps
            # 简单梯形速度规划(加速-匀速-减速), 保证起止速度为 0
            s = self._trapezoid(t)
            joints = [start[j] + (target[j] - start[j]) * s for j in range(7)]
            self.smart_servo(joints)
            time.sleep(dt)

    @staticmethod
    def _trapezoid(t: float, blend: float = 0.3) -> float:
        """归一化时间 t->位移 s 的梯形速度规划(起止速度为0, 总位移归一到 1)"""
        b = blend
        if t < b:
            return t * t / (2.0 * b * (1.0 - b))
        if t > 1.0 - b:
            u = 1.0 - t
            return 1.0 - u * u / (2.0 * b * (1.0 - b))
        return (t - 0.5 * b) / (1.0 - b)

    def gripper_set_mode(self, mode: int = 0):
        """夹爪模式: 0=Basic 1=Pinch 2=Wide 3=Scissor"""
        header = bytes([MSG_GRIPPER_MODE, MOVE_BASIC + mode])
        self._send_binary(self._wrap_command(header))

    def gripper_move(self, position: int, grasp: bool = True):
        """夹爪开合位置(如 grasp=90, release=30)"""
        motion = MOTION_GRASP if grasp else MOTION_RELEASE
        header = bytes([MSG_GRIPPER_MOVE, position & 0xFF, motion])
        self._send_binary(self._wrap_command(header))

    def gripper_grasp(self, position: int = 90):
        self.gripper_set_mode(0)
        self.gripper_move(position, grasp=True)

    def gripper_release(self, position: int = 30):
        self.gripper_set_mode(0)
        self.gripper_move(position, grasp=False)

    # ---------------- R2V 读取接口 ----------------
    def get_joints(self):
        """最新一帧 7 关节当前角(rad), 无数据返回 None"""
        with self._data_lock:
            return list(self._latest.get(DT_JOINT_CUR_POS)) \
                if DT_JOINT_CUR_POS in self._latest else None

    def get_pose(self):
        """最新一帧笛卡尔位姿 [x,y,z(mm), A,B,C(rad)]"""
        with self._data_lock:
            return list(self._latest.get(DT_CART_CUR_POS)) \
                if DT_CART_CUR_POS in self._latest else None

    def get_wrench(self):
        """最新一帧末端力/力矩 [Fx,Fy,Fz(N), Mx,My,Mz(Nm)]"""
        with self._data_lock:
            return list(self._latest.get(DT_WRENCH)) \
                if DT_WRENCH in self._latest else None

    def wait_until(self, dtype: int, timeout: float = 2.0):
        """等待指定类型的一帧数据到达"""
        t0 = time.time()
        while time.time() - t0 < timeout:
            with self._data_lock:
                if dtype in self._latest:
                    return list(self._latest[dtype])
            time.sleep(0.02)
        return None

    # ---------------- R2V 接收与解码 ----------------
    def _recv_loop(self):
        while self._running:
            try:
                chunk = self._sock.recv(4096)
            except OSError:
                break
            if not chunk:
                break
            self._recv_buf += chunk
            self._parse_buffer()

    def _parse_buffer(self):
        """按 0xEF 0xEF 分隔符切帧并逐帧解码"""
        while True:
            idx = self._recv_buf.find(FRAME_SEP)
            if idx < 0:
                # 保留一个字节, 防止分隔符跨 TCP 包被截断
                if len(self._recv_buf) > 1:
                    self._recv_buf = self._recv_buf[-1:]
                return
            frame = self._recv_buf[:idx]
            self._recv_buf = self._recv_buf[idx + len(FRAME_SEP):]
            if frame:
                self._decode_frame(frame)

    def _decode_frame(self, frame: bytes):
        # 消息类型: MSG:xxx(控制权等级)
        if frame[:4] == b"MSG:":
            text = frame.decode("ascii", errors="replace")
            self.rank = RANK_MSGS.get(text, self.rank)
            if self.on_rank:
                self.on_rank(self.rank, text)
            return
        # 异步状态: ASYNC:ON / ASYNC:OFF
        if frame[:6] == b"ASYNC:":
            self.async_on = (frame[6:8] == b"ON")
            if self.on_async:
                self.on_async(self.async_on)
            return
        # 数据帧: _ST + 类型字节 + short 载荷 + _EE
        if len(frame) >= 7 and frame[:3] == b"_ST" and frame[-3:] == b"_EE":
            dtype = frame[3]
            values = self._decode_values(dtype, frame[4:-3])
            if values is not None:
                with self._data_lock:
                    self._latest[dtype] = values
                if self.on_data:
                    self.on_data(dtype, values)

    @staticmethod
    def _decode_values(dtype: int, payload: bytes):
        n = len(payload) // 2
        raws = [struct.unpack(">h", payload[2 * i:2 * i + 2])[0] for i in range(n)]
        if dtype in (DT_JOINT_CUR_POS, DT_JOINT_CMD_POS):
            return [v * 0.0001 for v in raws[:7]]          # rad
        if dtype in (DT_CART_CUR_POS, DT_CART_CMD_POS):
            return [v * 0.1 for v in raws[:3]] + \
                   [v * 0.0001 for v in raws[3:6]]          # mm / rad
        if dtype == DT_WRENCH:
            return [v * 0.01 for v in raws[:6]]            # N / Nm
        if dtype in (DT_EXT_J_TORQUE, DT_RAW_J_TORQUE):
            return [v * 0.01 for v in raws[:7]]            # Nm
        return None


# ----------------------------------------------------------------------------
# 演示
# ----------------------------------------------------------------------------
def fmt(vals, ndig=4):
    return " ".join(f"{v: .{ndig}f}" for v in vals)


def demo_read(robot: KukaIiwa, duration: float):
    """仅读取(R2V): 持续打印机械臂状态"""
    print(f"读取 {duration:.0f} 秒(Ctrl+C 提前结束) ...")
    t0 = time.time()
    try:
        while time.time() - t0 < duration:
            j = robot.get_joints()
            p = robot.get_pose()
            if j:
                print(f"关节(rad): {fmt(j)}")
            if p:
                print(f"位姿 xyz(mm)={fmt(p[:3], 1)}  abc(rad)={fmt(p[3:])}")
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass


def demo_interactive(robot: KukaIiwa):
    """交互式: 对应上位机 Network + Planning 的基本读写操作"""
    help_text = (
        "\n命令:\n"
        "  r          读取并打印当前关节角/位姿\n"
        "  j 0,0,..   输入 7 个关节角(rad)运动到该位置(PTP 式轨迹)\n"
        "  g / o      夹爪 夹紧 / 松开\n"
        "  c          取消当前运动(CLEANUP)\n"
        "  q          退出\n"
    )
    print(help_text)
    while True:
        try:
            line = input(">> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if not line:
            continue
        cmd = line[0].lower()
        if cmd == "q":
            break
        elif cmd == "r":
            j = robot.get_joints()
            p = robot.get_pose()
            print("关节(rad):", fmt(j) if j else "无数据")
            print("位姿:", f"xyz(mm)={fmt(p[:3],1)} abc={fmt(p[3:])}" if p else "无数据")
        elif cmd == "j":
            try:
                vals = [float(x) for x in line[1:].replace(",", " ").split()]
                assert len(vals) == 7
            except (ValueError, AssertionError):
                print("请输入 7 个关节角(rad), 例如: j 0,0.2,0,-1.0,0,1.2,0")
                continue
            print(f"运动到: {vals} (运动中请勿断开)")
            robot.move_joints_ptp(vals, duration=2.0)
            print("到位。")
        elif cmd == "g":
            robot.gripper_grasp(90)
            print("夹紧指令已发送")
        elif cmd == "o":
            robot.gripper_release(30)
            print("松开指令已发送")
        elif cmd == "c":
            robot.cancel()
            print("已发送 CLEANUP")
        else:
            print(help_text)


def main():
    parser = argparse.ArgumentParser(description="KUKA IIWA 最小读取/控制程序")
    parser.add_argument("--ip", default="192.168.1.101", help="控制柜 IP")
    parser.add_argument("--port", type=int, default=30002, help="控制柜端口")
    parser.add_argument("--read", action="store_true", help="仅读取演示(不交互)")
    parser.add_argument("--duration", type=float, default=20.0, help="--read 模式持续秒数")
    args = parser.parse_args()

    robot = KukaIiwa()
    robot.on_rank = lambda rank, text: \
        print(f"[权限] {RANK_NAMES.get(rank, rank)} ({text})")
    robot.on_async = lambda flag: print(f"[异步] 数据上报: {'ON' if flag else 'OFF'}")

    print(f"连接 {args.ip}:{args.port} ...")
    robot.connect(args.ip, args.port)
    print("已连接, 请求控制权(REQUEST#) ...")
    robot.request_controller()
    time.sleep(0.8)
    if robot.rank < 3:
        print("警告: 未获得 Controller 权限, 控制指令可能无效(仍可读取)。")
    print("开启异步数据上报(GETASYNC#) ...")
    robot.start_async()
    robot.wait_until(DT_JOINT_CUR_POS, timeout=2.0)

    try:
        if args.read:
            demo_read(robot, args.duration)
        else:
            demo_interactive(robot)
    finally:
        print("停止异步并断开 ...")
        robot.stop_async()
        time.sleep(0.2)
        robot.disconnect()
        print("已断开。")


if __name__ == "__main__":
    main()
