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
    python kuka_iiwa.py --debug              # 额外打印原始收发字节, 排查通讯问题
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
MAX_BUF = 1 << 20            # 接收缓冲上限(1MB), 防止流损坏时无限增长

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

# MotionType: PTP=0, CartesianPTP=1, Line=2, Circle=3, Spline=4,
# SmartServo=5, SmartServoLin=6, Grasp=7, Release=8.
MOTION_PTP = 0
MOTION_SMART_SERVO = 5
MOTION_GRASP = 7
MOTION_RELEASE = 8

GOAL_JOINTS = 0

RANK_MSGS = {
    "MSG:LINK_SUCESS": 1,             # 连接成功, Observer
    "MSG:NOT_OBSERVER_ANYMORE": 2,    # Requester
    "MSG:WAIT_FOR_CONTROLLER": 2,
    "MSG:GET_HIGHEST_AURTHORITY": 3,  # Controller(可控制), 拼写与机器人端一致
}
RANK_NAMES = {0: "None", 1: "Observer", 2: "Requester", 3: "Controller"}
DATA_NAMES = {0: "关节当前角", 1: "关节指令角", 2: "笛卡尔位姿", 3: "指令位姿",
              4: "力/力矩", 5: "关节外力矩", 6: "关节力矩"}


class KukaIiwa:
    """KUKA IIWA 控制柜 TCP 客户端。一台机械臂一个连接。"""

    def __init__(self, debug: bool = False):
        self._sock = None
        self._recv_buf = b""
        self._send_lock = threading.Lock()
        self._data_lock = threading.Lock()
        self._rank_cond = threading.Condition()
        self._running = False
        self._recv_thread = None
        self.debug = debug
        # 最新一帧数据缓存(R2V)
        self._latest = {}
        self.rank = 0
        self.async_on = False
        # 用户回调
        self.on_data = None     # on_data(data_type:int, values:list[float])
        self.on_rank = None     # on_rank(rank:int, text:str)
        self.on_async = None    # on_async(flag:bool)
        self._checking_count = 0
        self._last_checking_log = 0.0
        # SmartServo needs a continuous command stream. The bridge updates only
        # the latest target; this driver re-sends it at a fixed high rate and
        # never queues obsolete VR waypoints.
        self._servo_state_lock = threading.Lock()
        self._servo_thread = None
        self._servo_running = False
        self._servo_target = None
        self._servo_target_updated_at = 0.0
        self._servo_rate_hz = 100.0
        self._servo_target_timeout_s = 0.25
        self._servo_error = None
        self._servo_sent_count = 0

    # ---------------- 连接管理 ----------------
    def connect(self, ip: str, port: int, timeout: float = 3.0):
        """TCP 连接控制柜(对应 Network 窗口 connect 按钮)"""
        self._sock = socket.create_connection((ip, port), timeout=timeout)
        self._sock.settimeout(None)                       # 接收线程永久等待
        self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # 小指令即时发出
        try:                                              # 发送最多阻塞 3s, 防卡死
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDTIMEO,
                                  struct.pack("ll", 3, 0))
        except OSError:
            pass
        self._running = True
        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._recv_thread.start()

    def disconnect(self):
        """断开(对应 quit 按钮 / 窗口关闭, 发送 quit#)"""
        if self._sock is None:
            return
        self.stop_smart_servo_stream()
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
        if self.debug:
            print(f"[发送-text] {data!r}")
        with self._send_lock:
            self._sock.sendall(data)

    def _send_binary(self, data: bytes, log_debug: bool = True):
        if self.debug and log_debug:
            print(f"[发送-bin ] {data.hex(' ')}")
        with self._send_lock:
            self._sock.sendall(data)

    def request_controller(self, timeout: float = 3.0) -> bool:
        """请求最高控制权(request 按钮, REQUEST#), 并等待升为 Controller。"""
        self._send_text("REQUEST")
        return self.wait_for_rank(3, timeout)

    def wait_for_rank(self, target: int, timeout: float = 3.0) -> bool:
        """等待权限等级达到 target"""
        t0 = time.time()
        with self._rank_cond:
            while self.rank < target and time.time() - t0 < timeout:
                self._rank_cond.wait(timeout - (time.time() - t0))
            return self.rank >= target

    def start_async(self):
        """开启异步数据上报(async 按钮 On, GETASYNC#)"""
        self._send_text("GETASYNC")

    def stop_async(self):
        """停止异步数据上报(async 按钮 Off, GETSTOP#)"""
        self._send_text("GETSTOP")

    def cancel(self):
        """取消/清空当前运动指令(CLEANUP#)"""
        self.clear_smart_servo_target()
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

    @staticmethod
    def _validate_smart_servo_joints(joints):
        values = [float(q) for q in joints]
        if len(values) != 7 or not all(-3.2 < q < 3.2 for q in values):
            raise ValueError("SmartServo expects seven finite joint angles in radians")
        return values

    def _send_smart_servo_frame(self, joints, log_debug: bool = True):
        """Encode and immediately send one SmartServo protocol frame."""
        if self.rank < 3:
            raise RuntimeError(
                f"SmartServo requires Controller authority, current rank={self.rank}"
            )
        values = self._validate_smart_servo_joints(joints)
        header = bytes([MSG_ROBOT_MOVE, MOVE_ASYNC, MOTION_SMART_SERVO, 1])
        self._send_binary(
            self._wrap_command(header, self._encode_joints(values)),
            log_debug=log_debug,
        )

    def smart_servo(self, joints):
        """
        更新 SmartServo 最新关节目标。

        若高频发送线程已经启动，本调用只原子替换最新目标，由发送线程按固定频率
        刷新控制柜；旧目标不会排队。未启动发送线程时保持兼容，立即发送一帧。
        """
        values = self._validate_smart_servo_joints(joints)
        with self._servo_state_lock:
            streaming = self._servo_running
            error = self._servo_error
            if streaming:
                self._servo_target = values
                self._servo_target_updated_at = time.monotonic()
        if error is not None:
            raise RuntimeError("SmartServo sending thread stopped") from error
        if not streaming:
            self._send_smart_servo_frame(values)

    def start_smart_servo_stream(
        self, rate_hz: float = 100.0, target_timeout_s: float = 0.25
    ):
        """Start a latest-target-only SmartServo sender at ``rate_hz``."""
        rate_hz = float(rate_hz)
        target_timeout_s = float(target_timeout_s)
        if not 10.0 <= rate_hz <= 200.0:
            raise ValueError("SmartServo stream rate must be in 10..200 Hz")
        if not 0.05 <= target_timeout_s <= 5.0:
            raise ValueError("SmartServo target timeout must be in 0.05..5.0 s")
        if self._sock is None:
            raise RuntimeError("connect before starting SmartServo stream")
        if self.rank < 3:
            raise RuntimeError("Controller authority is required before SmartServo stream")
        with self._servo_state_lock:
            if self._servo_running:
                raise RuntimeError("SmartServo stream is already running")
            self._servo_rate_hz = rate_hz
            self._servo_target_timeout_s = target_timeout_s
            self._servo_target = None
            self._servo_error = None
            self._servo_sent_count = 0
            self._servo_running = True
        self._servo_thread = threading.Thread(
            target=self._smart_servo_send_loop,
            name="kuka-smart-servo-tx",
            daemon=True,
        )
        self._servo_thread.start()
        if self.debug:
            print(
                f"[SmartServo] latest-target stream {rate_hz:g} Hz, "
                f"timeout {target_timeout_s:g}s"
            )

    def _smart_servo_send_loop(self):
        period = 1.0 / self._servo_rate_hz
        next_tick = time.monotonic()
        while True:
            with self._servo_state_lock:
                if not self._servo_running:
                    return
                target = None if self._servo_target is None else list(self._servo_target)
                target_age = time.monotonic() - self._servo_target_updated_at
                timeout = self._servo_target_timeout_s
            if target is not None and target_age <= timeout:
                try:
                    self._send_smart_servo_frame(target, log_debug=False)
                    self._servo_sent_count += 1
                    if self.debug and self._servo_sent_count % int(self._servo_rate_hz) == 0:
                        print(
                            f"[SmartServo] sent {self._servo_sent_count} frames; "
                            f"target age={target_age:.3f}s"
                        )
                except Exception as exc:  # surfaced on the next target update
                    with self._servo_state_lock:
                        self._servo_error = exc
                        self._servo_running = False
                    return
            next_tick += period
            delay = next_tick - time.monotonic()
            if delay > 0.0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()

    def clear_smart_servo_target(self):
        """Stop re-sending immediately while keeping the sender thread alive."""
        with self._servo_state_lock:
            self._servo_target = None
            self._servo_target_updated_at = 0.0

    def stop_smart_servo_stream(self):
        """Stop and join the high-rate sender thread."""
        with self._servo_state_lock:
            self._servo_running = False
            self._servo_target = None
        thread = self._servo_thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=0.5)
        self._servo_thread = None

    def move_joints_ptp(self, target_joints, duration: float = 1.0,
                        rate_hz: float = 100.0,
                        max_joint_speed_rad_s: float = 1.5):
        """
        从当前关节角梯形插值运动到 target_joints(对应 Planning 窗口 PTP):
        上位机里 PTPMotion 生成轨迹后, 也是逐帧 SmartServo 下发, 这里做同样的事。
        duration: 期望运动时长(秒)，越小越快。
        rate_hz: SmartServo 下发频率，只影响轨迹平滑度，不直接决定速度。
        max_joint_speed_rad_s: 任一关节的峰值速度上限；必要时自动延长 duration。
        """
        if duration <= 0.0:
            raise ValueError("duration 必须大于 0")
        if not 10.0 <= rate_hz <= 200.0:
            raise ValueError("rate_hz 必须在 10~200 Hz")
        if not 0.05 <= max_joint_speed_rad_s <= 2.0:
            raise ValueError("max_joint_speed_rad_s 必须在 0.05~2.0 rad/s")
        # 起点必须能读到当前关节角; 读不到则等待数据到达, 而不是直接抛错退出
        start = self.wait_for_data(DT_JOINT_CUR_POS, timeout=2.0)
        if start is None:
            raise RuntimeError("读取不到当前关节角, 无法确定运动起点(请确认已 GETASYNC 且有数据)")
        target = [float(value) for value in target_joints]
        if len(target) != 7 or not all(-3.2 < value < 3.2 for value in target):
            raise ValueError("目标必须是 7 个有效的 rad 关节角")
        # The trapezoid profile's normalized peak derivative is 1/(1-blend).
        # Extend an over-aggressive request instead of emitting unsafe steps.
        max_distance = max(abs(target[j] - start[j]) for j in range(7))
        minimum_duration = max_distance / (
            max_joint_speed_rad_s * (1.0 - 0.3)
        )
        if duration < minimum_duration:
            print(
                f"请求时长 {duration:.3f}s 超出关节速度上限，"
                f"自动调整为 {minimum_duration:.3f}s"
            )
            duration = minimum_duration
        n_steps = max(1, int(duration * rate_hz))
        period = 1.0 / rate_hz
        t_next = time.time()
        for i in range(1, n_steps + 1):
            s = self._trapezoid(i / n_steps)
            joints = [start[j] + (target[j] - start[j]) * s for j in range(7)]
            self.smart_servo(joints)
            if self.debug and (i == 1 or i == n_steps or i % 25 == 0):
                print(f"  运动帧 {i}/{n_steps}  关节[0]={joints[0]:.4f}")
            t_next += period
            delay = t_next - time.time()
            if delay > 0:
                time.sleep(delay)
            else:
                t_next = time.time()     # 发送跟不上节拍时重置, 不累积漂移

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
        """
        设置夹爪模式: 0=Basic, 1=Pinch, 2=Wide, 3=Scissor。

        协议编码器的命令头固定为 4 字节：
        ``_CM 04 <move_type> 00 00 _EE#``。
        """
        mode = int(mode)
        if mode not in (0, 1, 2, 3):
            raise ValueError("gripper mode must be 0..3")
        if self.rank < 3:
            raise RuntimeError("gripper command requires Controller authority")
        header = bytes([MSG_GRIPPER_MODE, MOVE_BASIC + mode, 0, 0])
        self._send_binary(self._wrap_command(header))

    def gripper_move(self, position: int):
        """
        移动夹爪到位置 ``0..255``；开/合由位置值区分。

        与 ``kuka_iiwa(2).py`` 的实际编码一致，运动类型固定为 Grasp，命令头
        固定为 4 字节：``_CM 03 <position> 07 00 _EE#``。
        """
        position = int(position)
        if not 0 <= position <= 255:
            raise ValueError("gripper position must be in 0..255")
        if self.rank < 3:
            raise RuntimeError("gripper command requires Controller authority")
        header = bytes([MSG_GRIPPER_MOVE, position, MOTION_GRASP, 0])
        self._send_binary(self._wrap_command(header))

    @staticmethod
    def _validate_gripper_action(position: int, mode_settle_s: float):
        position = int(position)
        mode_settle_s = float(mode_settle_s)
        if not 0 <= position <= 255:
            raise ValueError("gripper position must be in 0..255")
        if not 0.0 <= mode_settle_s <= 2.0:
            raise ValueError("gripper mode settle time must be in 0..2 s")
        return position, mode_settle_s

    def gripper_grasp(self, position: int = 100, mode_settle_s: float = 0.15):
        """切换到 Basic 模式后闭合到 ``position``。"""
        position, mode_settle_s = self._validate_gripper_action(
            position, mode_settle_s
        )
        self.gripper_set_mode(0)
        time.sleep(mode_settle_s)
        self.gripper_move(position)

    def gripper_release(self, position: int = 30, mode_settle_s: float = 0.15):
        """切换到 Basic 模式后张开到 ``position``。"""
        position, mode_settle_s = self._validate_gripper_action(
            position, mode_settle_s
        )
        self.gripper_set_mode(0)
        time.sleep(mode_settle_s)
        self.gripper_move(position)

    def set_gripper_binary(
        self,
        command: int,
        open_position: int = 30,
        close_position: int = 100,
        mode_settle_s: float = 0.15,
    ):
        """Execute the teleoperation contract: ``0=open``, ``1=close``."""
        command = int(command)
        if command not in (0, 1):
            raise ValueError("binary gripper command must be 0 or 1")
        position = close_position if command else open_position
        position, mode_settle_s = self._validate_gripper_action(
            position, mode_settle_s
        )
        self.gripper_set_mode(0)
        time.sleep(mode_settle_s)
        self.gripper_move(position)

    # ---------------- R2V 读取接口 ----------------
    def get_joints(self):
        """最新一帧 7 关节当前角(rad), 无数据返回 None"""
        with self._data_lock:
            return list(self._latest[DT_JOINT_CUR_POS]) \
                if DT_JOINT_CUR_POS in self._latest else None

    def get_pose(self):
        """最新一帧笛卡尔位姿 [x,y,z(mm), A,B,C(rad)]"""
        with self._data_lock:
            return list(self._latest[DT_CART_CUR_POS]) \
                if DT_CART_CUR_POS in self._latest else None

    def get_wrench(self):
        """最新一帧末端力/力矩 [Fx,Fy,Fz(N), Mx,My,Mz(Nm)]"""
        with self._data_lock:
            return list(self._latest[DT_WRENCH]) \
                if DT_WRENCH in self._latest else None

    def wait_for_data(self, dtype: int, timeout: float = 2.0):
        """等待指定类型的一帧数据到达, 返回该数据; 超时返回 None"""
        t0 = time.time()
        while time.time() - t0 < timeout:
            with self._data_lock:
                if dtype in self._latest:
                    return list(self._latest[dtype])
            time.sleep(0.01)
        return None

    # ---------------- R2V 接收与解码 ----------------
    def _recv_loop(self):
        while self._running:
            try:
                chunk = self._sock.recv(4096)
            except OSError:
                break
            if not chunk:
                if self.debug:
                    print("[接收] 对端关闭连接")
                break
            # 任何解析异常都不能杀死接收线程, 否则后续数据全部中断
            try:
                self._recv_buf += chunk
                if len(self._recv_buf) > MAX_BUF:     # 流损坏保护
                    self._recv_buf = b""
                self._parse_buffer()
            except Exception as exc:                  # noqa: BLE001
                if self.debug:
                    print(f"[解析异常] {exc!r}, 已忽略")

    def _parse_buffer(self):
        """
        按 0xEF 0xEF 分隔符切帧。流式重组: 没看到分隔符时必须保留整个缓冲区
        (一帧可能跨多个 TCP 包), 绝不能截断, 否则关节数据会被丢弃。
        """
        while True:
            idx = self._recv_buf.find(FRAME_SEP)
            if idx < 0:
                return                               # 保留全部, 等待更多数据
            frame = self._recv_buf[:idx]
            self._recv_buf = self._recv_buf[idx + len(FRAME_SEP):]
            if frame:
                self._decode_frame(frame)

    def _decode_frame(self, frame: bytes):
        # Server-side controller arbitration/liveness heartbeat. It is not a
        # state-data frame and this custom protocol does not document a client
        # reply, so recognize it without guessing an acknowledgement.
        if frame == b"CHECKING":
            self._checking_count += 1
            now = time.monotonic()
            if self.debug and (
                self._checking_count == 1 or now - self._last_checking_log >= 5.0
            ):
                print(f"[接收-heartbeat] CHECKING (count={self._checking_count})")
                self._last_checking_log = now
            return
        # 消息类型: MSG:xxx(控制权等级)
        if frame[:4] == b"MSG:":
            text = frame.decode("ascii", errors="replace")
            new_rank = RANK_MSGS.get(text)
            if self.debug:
                print(f"[接收-msg] {text!r}")
            if new_rank is not None:
                with self._rank_cond:
                    self.rank = new_rank
                    self._rank_cond.notify_all()
                if self.on_rank:
                    self.on_rank(self.rank, text)
            return
        # 异步状态: ASYNC:ON / ASYNC:OFF
        if frame[:6] == b"ASYNC:":
            self.async_on = (frame[6:8] == b"ON")
            if self.debug:
                print(f"[接收-async] {frame[:8]!r}")
            if self.on_async:
                self.on_async(self.async_on)
            return
        # 数据帧: _ST + 类型字节 + short 载荷 + _EE
        if len(frame) >= 7 and frame[:3] == b"_ST" and frame[-3:] == b"_EE":
            dtype = frame[3]
            values = self._decode_values(dtype, frame[4:-3])
            if values:
                with self._data_lock:
                    self._latest[dtype] = values
                if self.debug:
                    print(f"[接收-data] 类型{dtype}({DATA_NAMES.get(dtype, '?')}) "
                          f"{len(values)}值: {[round(v, 4) for v in values]}")
                if self.on_data:
                    self.on_data(dtype, values)

    @staticmethod
    def _decode_values(dtype: int, payload: bytes):
        expected_bytes = {
            DT_JOINT_CUR_POS: 14,
            DT_JOINT_CMD_POS: 14,
            DT_CART_CUR_POS: 12,
            DT_CART_CMD_POS: 12,
            DT_WRENCH: 12,
            DT_EXT_J_TORQUE: 14,
            DT_RAW_J_TORQUE: 14,
        }.get(dtype)
        if expected_bytes is None or len(payload) != expected_bytes:
            return None
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


def demo_interactive(robot: KukaIiwa, move_duration: float = 1.0,
                     servo_rate: float = 100.0,
                     max_joint_speed: float = 1.5):
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
        try:
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
                print(
                    f"运动到: {vals} (期望 {move_duration:g}s, "
                    f"{servo_rate:g}Hz, 限速 {max_joint_speed:g}rad/s)"
                )
                robot.move_joints_ptp(
                    vals,
                    duration=move_duration,
                    rate_hz=servo_rate,
                    max_joint_speed_rad_s=max_joint_speed,
                )
                print("轨迹发送完毕。")
            elif cmd == "g":
                robot.gripper_grasp(58)
                print("夹紧指令已发送")
            elif cmd == "o":
                robot.gripper_release(30)
                print("松开指令已发送")
            elif cmd == "c":
                robot.cancel()
                print("已发送 CLEANUP")
            else:
                print(help_text)
        except Exception as exc:      # 单条命令失败不退出整个程序
            print(f"命令执行失败: {exc!r}")


def main():
    parser = argparse.ArgumentParser(description="KUKA IIWA 最小读取/控制程序")
    parser.add_argument("--ip", default="192.168.1.101", help="控制柜 IP")
    parser.add_argument("--port", type=int, default=30002, help="控制柜端口")
    parser.add_argument("--read", action="store_true", help="仅读取演示(不交互)")
    parser.add_argument("--duration", type=float, default=20.0, help="--read 模式持续秒数")
    parser.add_argument(
        "--move-duration", type=float, default=1.0,
        help="交互 j 指令的期望运动时长，越小越快（默认 1.0s）",
    )
    parser.add_argument(
        "--servo-rate", type=float, default=100.0,
        help="SmartServo 轨迹发送频率 Hz（默认 100）",
    )
    parser.add_argument(
        "--max-joint-speed", type=float, default=1.5,
        help="软件关节峰值速度上限 rad/s（默认 1.5，最大允许 2.0）",
    )
    parser.add_argument("--debug", action="store_true", help="打印原始收发字节")
    args = parser.parse_args()

    robot = KukaIiwa(debug=args.debug)
    robot.on_rank = lambda rank, text: \
        print(f"[权限] {RANK_NAMES.get(rank, rank)} ({text})")
    robot.on_async = lambda flag: print(f"[异步] 数据上报: {'ON' if flag else 'OFF'}")

    print(f"连接 {args.ip}:{args.port} ...")
    robot.connect(args.ip, args.port)
    async_started = False
    controller_acquired = False
    try:
        # Always observe and capture the actual pose before requesting motion
        # authority. In --read mode REQUEST# is never sent.
        if not robot.wait_for_rank(1, timeout=3.0):
            raise RuntimeError("未收到 MSG:LINK_SUCESS，无法确认 Observer 链路")
        print("链路正常，当前为 Observer。")

        print("开启状态上报(GETASYNC#)，先读取实际关节角 ...")
        robot.start_async()
        async_started = True
        joints = robot.wait_for_data(DT_JOINT_CUR_POS, timeout=3.0)
        if joints is None:
            raise RuntimeError("3 秒内未收到关节角；为防止目标跳变，禁止获取控制权")
        print(f"已收到实际关节角: {fmt(joints)}")

        if args.read:
            print("--read: 保持 Observer，不发送 REQUEST# 或任何运动指令。")
            demo_read(robot, args.duration)
        else:
            print("请求控制权(REQUEST#) ...")
            if not robot.request_controller(timeout=3.0):
                raise RuntimeError(
                    f"未获得 Controller，当前权限={RANK_NAMES.get(robot.rank)}"
                )
            controller_acquired = True
            # Drop any target retained by the controller from an earlier
            # session, then seed position control with the measured pose.
            robot.cancel()
            robot.smart_servo(joints)
            print("已获得 Controller；旧目标已 CLEANUP，当前关节角已设为保持目标。")
            demo_interactive(
                robot,
                move_duration=args.move_duration,
                servo_rate=args.servo_rate,
                max_joint_speed=args.max_joint_speed,
            )
    finally:
        print("安全停止并断开 ...")
        try:
            if controller_acquired:
                robot.cancel()
            if async_started:
                robot.stop_async()
                time.sleep(0.2)
        finally:
            robot.disconnect()
        print("已断开。")


if __name__ == "__main__":
    main()
