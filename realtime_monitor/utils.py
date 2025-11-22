"""
工具函数模块

提供四元数转换、频率计算等通用功能
"""

import math
import time
import asyncio
from typing import Tuple, List
from collections import deque


def quat_to_euler(q: List[float]) -> Tuple[float, float, float]:
    """
    四元数转欧拉角（Hamilton约定）

    参数:
        q: 四元数 [w, x, y, z]

    返回:
        (roll, pitch, yaw) 弧度制
    """
    w, x, y, z = q[0], q[1], q[2], q[3]

    # Roll (φ)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))

    # Pitch (θ)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # 使用90度，如果超出范围
    else:
        pitch = math.asin(sinp)

    # Yaw (ψ)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    return roll, pitch, yaw


def quat_to_euler_deg(q: List[float]) -> Tuple[float, float, float]:
    """
    四元数转欧拉角（度数）

    参数:
        q: 四元数 [w, x, y, z]

    返回:
        (roll, pitch, yaw) 角度制
    """
    roll, pitch, yaw = quat_to_euler(q)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


class FrequencyCalculator:
    """异步频率计算器"""

    def __init__(self, window_size: int = 50):
        """
        初始化

        参数:
            window_size: 计算窗口大小（数据点数量）
        """
        self.window_size = window_size
        self.timestamps = deque(maxlen=window_size)

    def update(self, timestamp: float = None) -> float:
        """
        更新频率计算

        参数:
            timestamp: 时间戳（秒），如果为None则使用当前时间

        返回:
            当前频率 (Hz)
        """
        if timestamp is None:
            timestamp = time.time()

        self.timestamps.append(timestamp)

        if len(self.timestamps) < 2:
            return 0.0

        # 计算时间跨度
        time_span = self.timestamps[-1] - self.timestamps[0]

        if time_span > 0:
            # 频率 = 数据点数 / 时间跨度
            return (len(self.timestamps) - 1) / time_span
        else:
            return 0.0

    def get_avg_frequency(self) -> float:
        """获取平均频率"""
        return self.update()

    def reset(self):
        """重置计算器"""
        self.timestamps.clear()


class PacketLossDetector:
    """数据包丢失检测器（协程安全）"""

    def __init__(self, expected_interval_ms: float = 5.0):
        """
        初始化

        参数:
            expected_interval_ms: 预期时间间隔（毫秒）
        """
        self.expected_interval_ms = expected_interval_ms
        self.last_timestamp = None
        self.total_packets = 0
        self.lost_packets = 0
        self.timestamp_jumps = 0

    def check(self, timestamp_ms: float) -> Tuple[bool, float]:
        """
        检测数据包是否丢失

        参数:
            timestamp_ms: 当前数据包时间戳（毫秒）

        返回:
            (is_lost, gap_ms): 是否丢包，时间间隔
        """
        self.total_packets += 1

        if self.last_timestamp is None:
            self.last_timestamp = timestamp_ms
            return False, 0.0

        # 计算时间间隔
        gap_ms = timestamp_ms - self.last_timestamp

        # 检测时间戳跳变（回退）
        if gap_ms < 0:
            self.timestamp_jumps += 1
            self.last_timestamp = timestamp_ms
            return True, gap_ms

        # 检测丢包（间隔超过预期的1.5倍）
        is_lost = gap_ms > self.expected_interval_ms * 1.5

        if is_lost:
            # 估算丢失的包数量
            estimated_lost = int(gap_ms / self.expected_interval_ms) - 1
            self.lost_packets += max(1, estimated_lost)

        self.last_timestamp = timestamp_ms
        return is_lost, gap_ms

    def get_stats(self) -> dict:
        """
        获取统计信息

        返回:
            统计信息字典
        """
        loss_rate = 0.0
        if self.total_packets > 0:
            loss_rate = (self.lost_packets / self.total_packets) * 100

        return {
            'total_packets': self.total_packets,
            'lost_packets': self.lost_packets,
            'loss_rate_percent': loss_rate,
            'timestamp_jumps': self.timestamp_jumps
        }

    def reset(self):
        """重置检测器"""
        self.last_timestamp = None
        self.total_packets = 0
        self.lost_packets = 0
        self.timestamp_jumps = 0


class MovingAverageFilter:
    """移动平均滤波器"""

    def __init__(self, window_size: int = 5):
        """
        初始化

        参数:
            window_size: 窗口大小
        """
        self.window_size = window_size
        self.buffer = deque(maxlen=window_size)

    def filter(self, value: float) -> float:
        """
        滤波

        参数:
            value: 输入值

        返回:
            滤波后的值
        """
        self.buffer.append(value)
        return sum(self.buffer) / len(self.buffer)

    def reset(self):
        """重置滤波器"""
        self.buffer.clear()


async def run_periodic(coro, interval: float):
    """
    定期运行协程

    参数:
        coro: 协程函数
        interval: 间隔时间（秒）

    示例:
        async def task():
            print("Hello")

        await run_periodic(task, 1.0)  # 每秒执行一次
    """
    while True:
        start_time = time.time()
        await coro()
        elapsed = time.time() - start_time

        # 计算需要等待的时间
        wait_time = max(0, interval - elapsed)
        await asyncio.sleep(wait_time)


class AsyncRateLimiter:
    """异步速率限制器"""

    def __init__(self, max_rate: float):
        """
        初始化

        参数:
            max_rate: 最大速率（次/秒）
        """
        self.min_interval = 1.0 / max_rate
        self.last_call = 0.0

    async def acquire(self):
        """获取许可（如果需要，会等待）"""
        now = time.time()
        elapsed = now - self.last_call

        if elapsed < self.min_interval:
            await asyncio.sleep(self.min_interval - elapsed)

        self.last_call = time.time()


def format_frequency(freq: float) -> str:
    """格式化频率显示"""
    return f"{freq:.1f} Hz"


def format_percentage(value: float) -> str:
    """格式化百分比显示"""
    return f"{value:.2f}%"


def format_angle(angle: float, unit: str = 'deg') -> str:
    """
    格式化角度显示

    参数:
        angle: 角度值
        unit: 单位 ('deg' 或 'rad')
    """
    if unit == 'deg':
        return f"{angle:.2f}°"
    else:
        return f"{angle:.4f} rad"
