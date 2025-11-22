"""
工具函数模块

提供四元数转换、频率计算、丢包检测等功能
"""

import math
import time
from collections import deque
from typing import Tuple, List


def quat_to_euler(q: List[float]) -> Tuple[float, float, float]:
    """
    四元数转欧拉角 (Hamilton约定)

    参数:
        q: 四元数 [w, x, y, z]

    返回:
        (roll, pitch, yaw) 单位：弧度
    """
    w, x, y, z = q[0], q[1], q[2], q[3]

    # Roll (φ) - 绕X轴旋转
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (θ) - 绕Y轴旋转
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # 使用±90度当|sinp| ≥ 1
    else:
        pitch = math.asin(sinp)

    # Yaw (ψ) - 绕Z轴旋转
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def quat_to_euler_deg(q: List[float]) -> Tuple[float, float, float]:
    """
    四元数转欧拉角 (度数)

    参数:
        q: 四元数 [w, x, y, z]

    返回:
        (roll_deg, pitch_deg, yaw_deg) 单位：度
    """
    roll, pitch, yaw = quat_to_euler(q)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


class FrequencyCalculator:
    """频率计算器"""

    def __init__(self, window_size: int = 100):
        """
        初始化

        参数:
            window_size: 时间窗口大小（样本数）
        """
        self.window_size = window_size
        self.timestamps = deque(maxlen=window_size)

    def update(self, timestamp: float) -> float:
        """
        更新时间戳并计算频率

        参数:
            timestamp: 时间戳（秒）

        返回:
            当前频率 (Hz)
        """
        self.timestamps.append(timestamp)

        if len(self.timestamps) < 2:
            return 0.0

        # 计算时间跨度
        time_span = self.timestamps[-1] - self.timestamps[0]

        if time_span > 0:
            # 频率 = 样本数 / 时间跨度
            frequency = (len(self.timestamps) - 1) / time_span
            return frequency
        else:
            return 0.0

    def reset(self):
        """重置计算器"""
        self.timestamps.clear()


class PacketLossDetector:
    """数据包丢失检测器"""

    def __init__(self, expected_interval_ms: float = 5.0):
        """
        初始化

        参数:
            expected_interval_ms: 预期时间间隔（毫秒）, 默认5.0ms (200Hz)
        """
        self.expected_interval_ms = expected_interval_ms
        self.last_timestamp = None
        self.total_packets = 0
        self.lost_packets = 0
        self.timestamp_jumps = 0

    def check(self, timestamp_ms: float) -> Tuple[bool, float]:
        """
        检查数据包是否丢失

        参数:
            timestamp_ms: 当前时间戳（毫秒）

        返回:
            (is_lost, gap_ms): (是否丢包, 时间间隔)
        """
        self.total_packets += 1

        if self.last_timestamp is None:
            self.last_timestamp = timestamp_ms
            return False, 0.0

        # 检查时间戳回退
        if timestamp_ms < self.last_timestamp:
            self.timestamp_jumps += 1
            print(f"⚠️  WARNING: Timestamp jump detected! {self.last_timestamp} -> {timestamp_ms}")
            self.last_timestamp = timestamp_ms
            return False, 0.0

        # 计算时间间隔
        gap_ms = timestamp_ms - self.last_timestamp
        self.last_timestamp = timestamp_ms

        # 判断是否丢包（间隔超过预期的1.5倍）
        if gap_ms > self.expected_interval_ms * 1.5:
            self.lost_packets += 1
            return True, gap_ms
        else:
            return False, gap_ms

    def get_stats(self) -> dict:
        """
        获取统计信息

        返回:
            包含丢包率、总包数等信息的字典
        """
        loss_rate = (self.lost_packets / self.total_packets * 100) if self.total_packets > 0 else 0.0

        return {
            'total_packets': self.total_packets,
            'lost_packets': self.lost_packets,
            'loss_rate_percent': loss_rate,
            'timestamp_jumps': self.timestamp_jumps
        }

    def reset(self):
        """重置统计"""
        self.last_timestamp = None
        self.total_packets = 0
        self.lost_packets = 0
        self.timestamp_jumps = 0


class MovingAverage:
    """移动平均滤波器"""

    def __init__(self, window_size: int = 10):
        """
        初始化

        参数:
            window_size: 窗口大小
        """
        self.window_size = window_size
        self.values = deque(maxlen=window_size)

    def update(self, value: float) -> float:
        """
        更新数值并返回移动平均值

        参数:
            value: 新数值

        返回:
            移动平均值
        """
        self.values.append(value)
        return sum(self.values) / len(self.values)

    def reset(self):
        """重置滤波器"""
        self.values.clear()


def calculate_frequency(timestamps: List[float]) -> float:
    """
    计算频率

    参数:
        timestamps: 时间戳列表（秒）

    返回:
        频率 (Hz)
    """
    if len(timestamps) < 2:
        return 0.0

    time_span = timestamps[-1] - timestamps[0]
    if time_span > 0:
        return (len(timestamps) - 1) / time_span
    else:
        return 0.0


def format_time(seconds: float) -> str:
    """
    格式化时间

    参数:
        seconds: 秒数

    返回:
        格式化字符串 "HH:MM:SS"
    """
    hours = int(seconds // 3600)
    minutes = int((seconds % 3600) // 60)
    secs = int(seconds % 60)
    return f"{hours:02d}:{minutes:02d}:{secs:02d}"


def normalize_quaternion(q: List[float]) -> List[float]:
    """
    归一化四元数

    参数:
        q: 四元数 [w, x, y, z]

    返回:
        归一化后的四元数
    """
    norm = math.sqrt(sum(x*x for x in q))
    if norm > 0:
        return [x / norm for x in q]
    else:
        return [1.0, 0.0, 0.0, 0.0]  # 默认单位四元数


def is_quaternion_valid(q: List[float], tolerance: float = 0.01) -> bool:
    """
    检查四元数是否有效

    参数:
        q: 四元数 [w, x, y, z]
        tolerance: 允许的误差范围

    返回:
        是否有效
    """
    norm = math.sqrt(sum(x*x for x in q))
    return abs(norm - 1.0) < tolerance
