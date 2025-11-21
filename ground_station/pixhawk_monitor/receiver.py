"""
Pixhawk数据接收器模块

提供MAVLink数据接收、解析和缓冲功能
"""

import time
import threading
from queue import Queue, Empty
from typing import Optional, Callable, Dict
from pymavlink import mavutil
from .utils import FrequencyCalculator, PacketLossDetector


class PixhawkReceiver:
    """Pixhawk MAVLink数据接收器"""

    def __init__(self, port: str, baudrate: int = 921600, timeout: float = 1.0):
        """
        初始化接收器

        参数:
            port: 串口设备 ('/dev/ttyUSB0' 或 'COM3')
            baudrate: 波特率 (默认921600)
            timeout: 连接超时时间（秒）
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout

        # MAVLink连接
        self.connection: Optional[mavutil.mavlink_connection] = None
        self.connected = False

        # 数据队列
        self.highres_imu_queue = Queue(maxsize=1000)
        self.attitude_queue = Queue(maxsize=1000)

        # 接收线程
        self.receive_thread: Optional[threading.Thread] = None
        self.running = False

        # 回调函数
        self.highres_imu_callback: Optional[Callable] = None
        self.attitude_callback: Optional[Callable] = None
        self.heartbeat_callback: Optional[Callable] = None

        # 统计信息
        self.freq_calc_imu = FrequencyCalculator()
        self.freq_calc_att = FrequencyCalculator()
        self.packet_loss_detector = PacketLossDetector(expected_interval_ms=3.33)

        # 数据计数
        self.imu_count = 0
        self.attitude_count = 0
        self.start_time = None

    def connect(self) -> bool:
        """
        连接到Pixhawk

        返回:
            是否连接成功
        """
        print(f"🔌 正在连接 {self.port} @ {self.baudrate}...")

        try:
            self.connection = mavutil.mavlink_connection(
                self.port,
                baud=self.baudrate,
                autoreconnect=True
            )

            # 等待心跳包
            print("⏳ 等待心跳包...")
            heartbeat = self.connection.wait_heartbeat(timeout=self.timeout)

            if heartbeat:
                self.connected = True
                print(f"✅ 已连接到系统 {self.connection.target_system}:{self.connection.target_component}")
                print(f"   类型: {heartbeat.type}, 自动驾驶仪: {heartbeat.autopilot}")
                return True
            else:
                print("❌ 连接超时，未收到心跳包")
                return False

        except Exception as e:
            print(f"❌ 连接失败: {e}")
            return False

    def disconnect(self):
        """断开连接"""
        self.stop_receiving()

        if self.connection:
            try:
                self.connection.close()
                print("🔌 已断开连接")
            except:
                pass

        self.connected = False

    def start_receiving(self):
        """启动数据接收线程"""
        if not self.connected:
            print("❌ 未连接到Pixhawk")
            return

        if self.running:
            print("⚠️  接收线程已在运行")
            return

        self.running = True
        self.start_time = time.time()
        self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.receive_thread.start()
        print("🚀 数据接收线程已启动")

    def stop_receiving(self):
        """停止数据接收线程"""
        if self.running:
            self.running = False
            if self.receive_thread:
                self.receive_thread.join(timeout=2.0)
            print("🛑 数据接收线程已停止")

    def _receive_loop(self):
        """数据接收循环（在独立线程中运行）"""
        print("📡 开始接收数据...")

        while self.running:
            try:
                # 接收MAVLink消息
                msg = self.connection.recv_match(blocking=True, timeout=0.1)

                if msg is None:
                    continue

                msg_type = msg.get_type()

                # 处理不同类型的消息
                if msg_type == 'HIGHRES_IMU':
                    self._handle_highres_imu(msg)

                elif msg_type == 'ATTITUDE_QUATERNION':
                    self._handle_attitude_quaternion(msg)

                elif msg_type == 'HEARTBEAT':
                    if self.heartbeat_callback:
                        self.heartbeat_callback(msg)

            except Exception as e:
                if self.running:  # 只在运行时报错
                    print(f"❌ 接收错误: {e}")
                    time.sleep(0.1)

    def _handle_highres_imu(self, msg):
        """处理HIGHRES_IMU消息"""
        self.imu_count += 1

        # 计算频率
        current_time = time.time()
        freq = self.freq_calc_imu.update(current_time)

        # 提取数据
        data = {
            'timestamp': msg.time_usec / 1e6,  # 转换为秒
            'accel': [msg.xacc, msg.yacc, msg.zacc],  # m/s²
            'gyro': [msg.xgyro, msg.ygyro, msg.zgyro],  # rad/s
            'mag': [msg.xmag, msg.ymag, msg.zmag],  # Gauss
            'pressure': msg.abs_pressure,  # mbar
            'temperature': msg.temperature,  # °C
            'frequency': freq
        }

        # 放入队列
        try:
            self.highres_imu_queue.put_nowait(data)
        except:
            # 队列满，丢弃最旧的数据
            try:
                self.highres_imu_queue.get_nowait()
                self.highres_imu_queue.put_nowait(data)
            except:
                pass

        # 回调函数
        if self.highres_imu_callback:
            self.highres_imu_callback(data)

    def _handle_attitude_quaternion(self, msg):
        """处理ATTITUDE_QUATERNION消息"""
        self.attitude_count += 1

        # 计算频率
        current_time = time.time()
        freq = self.freq_calc_att.update(current_time)

        # 检测丢包
        is_lost, gap_ms = self.packet_loss_detector.check(msg.time_boot_ms)
        if is_lost:
            print(f"⚠️  检测到丢包! 时间间隔: {gap_ms:.1f}ms")

        # 提取数据
        data = {
            'timestamp': msg.time_boot_ms / 1000.0,  # 转换为秒
            'quaternion': [msg.q1, msg.q2, msg.q3, msg.q4],  # w, x, y, z
            'rates': [msg.rollspeed, msg.pitchspeed, msg.yawspeed],  # rad/s
            'frequency': freq
        }

        # 放入队列
        try:
            self.attitude_queue.put_nowait(data)
        except:
            try:
                self.attitude_queue.get_nowait()
                self.attitude_queue.put_nowait(data)
            except:
                pass

        # 回调函数
        if self.attitude_callback:
            self.attitude_callback(data)

    def get_highres_imu(self, timeout: float = 0.1) -> Optional[Dict]:
        """
        从队列获取HIGHRES_IMU数据

        参数:
            timeout: 超时时间（秒）

        返回:
            数据字典或None
        """
        try:
            return self.highres_imu_queue.get(timeout=timeout)
        except Empty:
            return None

    def get_attitude(self, timeout: float = 0.1) -> Optional[Dict]:
        """
        从队列获取姿态数据

        参数:
            timeout: 超时时间（秒）

        返回:
            数据字典或None
        """
        try:
            return self.attitude_queue.get(timeout=timeout)
        except Empty:
            return None

    def get_stats(self) -> Dict:
        """
        获取统计信息

        返回:
            统计信息字典
        """
        elapsed = time.time() - self.start_time if self.start_time else 0
        packet_loss_stats = self.packet_loss_detector.get_stats()

        return {
            'elapsed_time': elapsed,
            'imu_count': self.imu_count,
            'attitude_count': self.attitude_count,
            'imu_frequency': self.imu_count / elapsed if elapsed > 0 else 0,
            'attitude_frequency': self.attitude_count / elapsed if elapsed > 0 else 0,
            **packet_loss_stats
        }

    def print_stats(self):
        """打印统计信息"""
        stats = self.get_stats()

        print("\n" + "="*60)
        print("📊 接收统计")
        print("="*60)
        print(f"运行时间:     {stats['elapsed_time']:.1f} 秒")
        print(f"IMU数据包:    {stats['imu_count']} ({stats['imu_frequency']:.1f} Hz)")
        print(f"姿态数据包:   {stats['attitude_count']} ({stats['attitude_frequency']:.1f} Hz)")
        print(f"总数据包:     {stats['total_packets']}")
        print(f"丢包数:       {stats['lost_packets']}")
        print(f"丢包率:       {stats['loss_rate_percent']:.2f}%")
        print(f"时间戳跳变:   {stats['timestamp_jumps']}")
        print("="*60 + "\n")

    def set_highres_imu_callback(self, callback: Callable):
        """设置HIGHRES_IMU回调函数"""
        self.highres_imu_callback = callback

    def set_attitude_callback(self, callback: Callable):
        """设置姿态数据回调函数"""
        self.attitude_callback = callback

    def set_heartbeat_callback(self, callback: Callable):
        """设置心跳包回调函数"""
        self.heartbeat_callback = callback
