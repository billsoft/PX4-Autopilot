"""
数据缓冲和处理模块

提供线程/协程安全的数据缓冲区
"""

import asyncio
import time
from collections import deque
from typing import Optional, List, Tuple
from dataclasses import dataclass, field

from .utils import quat_to_euler, quat_to_euler_deg, FrequencyCalculator, PacketLossDetector
from .config import get_config


@dataclass
class AttitudeData:
    """姿态数据类"""
    timestamp: float  # 时间戳（秒）
    quaternion: List[float]  # 四元数 [w, x, y, z]
    euler: Tuple[float, float, float]  # 欧拉角 (roll, pitch, yaw) 弧度
    euler_deg: Tuple[float, float, float]  # 欧拉角 (roll, pitch, yaw) 角度
    rates: List[float]  # 角速度 [roll_rate, pitch_rate, yaw_rate] rad/s
    frequency: float = 0.0  # 当前频率


@dataclass
class DataStatistics:
    """数据统计"""
    total_packets: int = 0
    lost_packets: int = 0
    loss_rate: float = 0.0
    avg_frequency: float = 0.0
    timestamp_jumps: int = 0


class AttitudeBuffer:
    """
    姿态数据缓冲区（协程安全）

    功能：
    - 存储四元数和欧拉角数据
    - 自动四元数转欧拉角
    - 频率计算
    - 丢包检测
    """

    def __init__(self, maxlen: int = 2000):
        """
        初始化缓冲区

        参数:
            maxlen: 最大缓冲长度
        """
        data_cfg = get_config('data')

        self.maxlen = maxlen

        # 时间戳缓冲
        self.timestamps = deque(maxlen=maxlen)

        # 四元数缓冲
        self.quaternions = deque(maxlen=maxlen)  # 每个元素是 [w, x, y, z]

        # 欧拉角缓冲（弧度）
        self.roll_rad = deque(maxlen=maxlen)
        self.pitch_rad = deque(maxlen=maxlen)
        self.yaw_rad = deque(maxlen=maxlen)

        # 欧拉角缓冲（角度）
        self.roll_deg = deque(maxlen=maxlen)
        self.pitch_deg = deque(maxlen=maxlen)
        self.yaw_deg = deque(maxlen=maxlen)

        # 角速度缓冲
        self.roll_rate = deque(maxlen=maxlen)
        self.pitch_rate = deque(maxlen=maxlen)
        self.yaw_rate = deque(maxlen=maxlen)

        # 统计工具
        self.freq_calc = FrequencyCalculator(window_size=data_cfg.freq_window_size)
        self.loss_detector = PacketLossDetector(
            expected_interval_ms=data_cfg.expected_interval_ms
        )

        # 起始时间（用于相对时间计算）
        self.start_time = time.time()

        # 锁（虽然deque是线程安全的，但我们还是加锁保证多字段操作的原子性）
        self._lock = asyncio.Lock()

    async def append(self, data: AttitudeData):
        """
        异步添加数据

        参数:
            data: AttitudeData对象
        """
        async with self._lock:
            # 转换欧拉角
            roll_rad, pitch_rad, yaw_rad = quat_to_euler(data.quaternion)
            roll_deg, pitch_deg, yaw_deg = quat_to_euler_deg(data.quaternion)

            # 添加到缓冲区
            self.timestamps.append(data.timestamp)
            self.quaternions.append(data.quaternion)

            self.roll_rad.append(roll_rad)
            self.pitch_rad.append(pitch_rad)
            self.yaw_rad.append(yaw_rad)

            self.roll_deg.append(roll_deg)
            self.pitch_deg.append(pitch_deg)
            self.yaw_deg.append(yaw_deg)

            self.roll_rate.append(data.rates[0])
            self.pitch_rate.append(data.rates[1])
            self.yaw_rate.append(data.rates[2])

            # 更新统计
            self.freq_calc.update(data.timestamp)

    def append_sync(self, data: AttitudeData):
        """
        同步添加数据（用于非协程环境）

        参数:
            data: AttitudeData对象
        """
        # 转换欧拉角
        roll_rad, pitch_rad, yaw_rad = quat_to_euler(data.quaternion)
        roll_deg, pitch_deg, yaw_deg = quat_to_euler_deg(data.quaternion)

        # 添加到缓冲区
        self.timestamps.append(data.timestamp)
        self.quaternions.append(data.quaternion)

        self.roll_rad.append(roll_rad)
        self.pitch_rad.append(pitch_rad)
        self.yaw_rad.append(yaw_rad)

        self.roll_deg.append(roll_deg)
        self.pitch_deg.append(pitch_deg)
        self.yaw_deg.append(yaw_deg)

        self.roll_rate.append(data.rates[0])
        self.pitch_rate.append(data.rates[1])
        self.yaw_rate.append(data.rates[2])

        # 更新统计
        self.freq_calc.update(data.timestamp)

    def get_data_for_plot(self, window_size: Optional[int] = None) -> dict:
        """
        获取用于绘图的数据

        参数:
            window_size: 获取最近N个数据点，None表示获取全部

        返回:
            包含所有绘图数据的字典
        """
        if window_size is None:
            window_size = len(self.timestamps)

        # 获取最近的数据
        n = min(window_size, len(self.timestamps))

        return {
            'timestamps': list(self.timestamps)[-n:] if n > 0 else [],

            # 四元数
            'quaternions': list(self.quaternions)[-n:] if n > 0 else [],

            # 欧拉角（弧度）
            'roll_rad': list(self.roll_rad)[-n:] if n > 0 else [],
            'pitch_rad': list(self.pitch_rad)[-n:] if n > 0 else [],
            'yaw_rad': list(self.yaw_rad)[-n:] if n > 0 else [],

            # 欧拉角（角度）
            'roll_deg': list(self.roll_deg)[-n:] if n > 0 else [],
            'pitch_deg': list(self.pitch_deg)[-n:] if n > 0 else [],
            'yaw_deg': list(self.yaw_deg)[-n:] if n > 0 else [],

            # 角速度
            'roll_rate': list(self.roll_rate)[-n:] if n > 0 else [],
            'pitch_rate': list(self.pitch_rate)[-n:] if n > 0 else [],
            'yaw_rate': list(self.yaw_rate)[-n:] if n > 0 else [],
        }

    def get_latest(self) -> Optional[AttitudeData]:
        """
        获取最新的数据

        返回:
            最新的AttitudeData对象，如果没有数据则返回None
        """
        if len(self.timestamps) == 0:
            return None

        roll_rad, pitch_rad, yaw_rad = self.roll_rad[-1], self.pitch_rad[-1], self.yaw_rad[-1]
        roll_deg, pitch_deg, yaw_deg = self.roll_deg[-1], self.pitch_deg[-1], self.yaw_deg[-1]

        return AttitudeData(
            timestamp=self.timestamps[-1],
            quaternion=list(self.quaternions[-1]),
            euler=(roll_rad, pitch_rad, yaw_rad),
            euler_deg=(roll_deg, pitch_deg, yaw_deg),
            rates=[self.roll_rate[-1], self.pitch_rate[-1], self.yaw_rate[-1]],
            frequency=self.freq_calc.get_avg_frequency()
        )

    def get_statistics(self) -> DataStatistics:
        """
        获取统计信息

        返回:
            DataStatistics对象
        """
        loss_stats = self.loss_detector.get_stats()

        return DataStatistics(
            total_packets=loss_stats['total_packets'],
            lost_packets=loss_stats['lost_packets'],
            loss_rate=loss_stats['loss_rate_percent'],
            avg_frequency=self.freq_calc.get_avg_frequency(),
            timestamp_jumps=loss_stats['timestamp_jumps']
        )

    def check_packet_loss(self, timestamp_ms: float) -> Tuple[bool, float]:
        """
        检测数据包丢失

        参数:
            timestamp_ms: 时间戳（毫秒）

        返回:
            (is_lost, gap_ms)
        """
        return self.loss_detector.check(timestamp_ms)

    def clear(self):
        """清空缓冲区"""
        self.timestamps.clear()
        self.quaternions.clear()

        self.roll_rad.clear()
        self.pitch_rad.clear()
        self.yaw_rad.clear()

        self.roll_deg.clear()
        self.pitch_deg.clear()
        self.yaw_deg.clear()

        self.roll_rate.clear()
        self.pitch_rate.clear()
        self.yaw_rate.clear()

        self.freq_calc.reset()
        self.loss_detector.reset()

    def __len__(self) -> int:
        """返回缓冲区当前长度"""
        return len(self.timestamps)


class DataQueue:
    """
    异步数据队列

    封装asyncio.Queue，提供统计功能
    """

    def __init__(self, maxsize: int = 500):
        """
        初始化队列

        参数:
            maxsize: 最大队列长度
        """
        self.queue = asyncio.Queue(maxsize=maxsize)
        self.put_count = 0
        self.get_count = 0
        self.dropped_count = 0  # 队列满时丢弃的数据

    async def put(self, item):
        """放入数据（如果队列满会等待）"""
        await self.queue.put(item)
        self.put_count += 1

    async def put_nowait_or_drop(self, item):
        """
        尝试放入数据，如果队列满则丢弃

        返回:
            是否成功放入
        """
        try:
            self.queue.put_nowait(item)
            self.put_count += 1
            return True
        except asyncio.QueueFull:
            self.dropped_count += 1
            return False

    async def get(self):
        """获取数据"""
        item = await self.queue.get()
        self.get_count += 1
        return item

    def qsize(self) -> int:
        """返回当前队列长度"""
        return self.queue.qsize()

    def get_stats(self) -> dict:
        """获取统计信息"""
        return {
            'put_count': self.put_count,
            'get_count': self.get_count,
            'dropped_count': self.dropped_count,
            'current_size': self.qsize()
        }
