"""
异步串口接收器模块

使用asyncio协程实现高性能数据接收
"""

import asyncio
import logging
from typing import Optional, Callable
from pymavlink import mavutil

from .data_buffer import AttitudeData, DataQueue
from .config import get_config


# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class AsyncMAVLinkReceiver:
    """
    异步MAVLink数据接收器

    特性:
    - 使用asyncio协程进行异步I/O
    - 自动重连
    - 数据队列缓冲
    - 统计信息
    """

    def __init__(self, port: str = None, baudrate: int = None):
        """
        初始化接收器

        参数:
            port: 串口设备
            baudrate: 波特率
        """
        # 加载配置
        serial_cfg = get_config('serial')
        data_cfg = get_config('data')

        self.port = port or serial_cfg.port
        self.baudrate = baudrate or serial_cfg.baudrate
        self.timeout = serial_cfg.timeout
        self.auto_reconnect = serial_cfg.auto_reconnect
        self.reconnect_interval = serial_cfg.reconnect_interval

        # MAVLink连接
        self.connection: Optional[mavutil.mavlink_connection] = None
        self.connected = False

        # 异步队列
        self.attitude_queue = DataQueue(maxsize=data_cfg.queue_maxsize)

        # 协程任务
        self._receive_task: Optional[asyncio.Task] = None
        self._running = False

        # 回调函数
        self.on_attitude_callback: Optional[Callable] = None
        self.on_heartbeat_callback: Optional[Callable] = None

        # 统计
        self.received_count = 0
        self.error_count = 0

    async def connect(self) -> bool:
        """
        异步连接到Pixhawk

        返回:
            是否连接成功
        """
        logger.info(f"🔌 正在连接 {self.port} @ {self.baudrate}...")

        try:
            # 注意：pymavlink不是原生异步的，所以我们使用run_in_executor
            loop = asyncio.get_event_loop()
            self.connection = await loop.run_in_executor(
                None,
                mavutil.mavlink_connection,
                self.port,
                self.baudrate
            )

            # 等待心跳包
            logger.info("⏳ 等待心跳包...")
            heartbeat = await loop.run_in_executor(
                None,
                self.connection.wait_heartbeat,
                self.timeout
            )

            if heartbeat:
                self.connected = True
                logger.info(
                    f"✅ 已连接到系统 "
                    f"{self.connection.target_system}:{self.connection.target_component}"
                )
                logger.info(f"   类型: {heartbeat.type}, 自动驾驶仪: {heartbeat.autopilot}")
                return True
            else:
                logger.error("❌ 连接超时，未收到心跳包")
                return False

        except Exception as e:
            logger.error(f"❌ 连接失败: {e}")
            return False

    async def disconnect(self):
        """异步断开连接"""
        await self.stop()

        if self.connection:
            try:
                # 在执行器中关闭连接
                loop = asyncio.get_event_loop()
                await loop.run_in_executor(None, self.connection.close)
                logger.info("🔌 已断开连接")
            except Exception as e:
                logger.warning(f"断开连接时出错: {e}")

        self.connected = False

    async def start(self):
        """启动异步接收"""
        if not self.connected:
            logger.error("❌ 未连接到Pixhawk")
            return

        if self._running:
            logger.warning("⚠️  接收器已在运行")
            return

        self._running = True
        self._receive_task = asyncio.create_task(self._receive_loop())
        logger.info("🚀 数据接收已启动")

    async def stop(self):
        """停止异步接收"""
        if not self._running:
            return

        self._running = False

        if self._receive_task:
            self._receive_task.cancel()
            try:
                await self._receive_task
            except asyncio.CancelledError:
                pass

        logger.info("🛑 数据接收已停止")

    async def _receive_loop(self):
        """
        异步接收循环

        关键点:
        - 使用run_in_executor避免阻塞事件循环
        - 定期让出控制权（asyncio.sleep(0)）
        - 异常处理和重连
        """
        logger.info("📡 开始接收数据...")

        loop = asyncio.get_event_loop()

        try:
            while self._running:
                try:
                    # 在执行器中接收消息（避免阻塞）
                    msg = await loop.run_in_executor(
                        None,
                        self.connection.recv_match,
                        None,  # blocking
                        True,  # blocking
                        0.1    # timeout
                    )

                    if msg is None:
                        # 超时，让出控制权
                        await asyncio.sleep(0)
                        continue

                    # 处理消息
                    msg_type = msg.get_type()

                    if msg_type == 'ATTITUDE_QUATERNION':
                        await self._handle_attitude_quaternion(msg)

                    elif msg_type == 'HEARTBEAT':
                        if self.on_heartbeat_callback:
                            await self._call_callback(self.on_heartbeat_callback, msg)

                    # 让出控制权给其他协程
                    await asyncio.sleep(0)

                except asyncio.CancelledError:
                    raise

                except Exception as e:
                    self.error_count += 1
                    logger.error(f"❌ 接收错误: {e}")

                    if self.auto_reconnect:
                        logger.info(f"⏳ {self.reconnect_interval}秒后尝试重连...")
                        await asyncio.sleep(self.reconnect_interval)
                        await self.connect()
                    else:
                        raise

        except asyncio.CancelledError:
            logger.info("接收循环已取消")
            raise

    async def _handle_attitude_quaternion(self, msg):
        """
        处理ATTITUDE_QUATERNION消息

        参数:
            msg: MAVLink消息对象
        """
        self.received_count += 1

        # 提取数据
        data = AttitudeData(
            timestamp=msg.time_boot_ms / 1000.0,  # 转换为秒
            quaternion=[msg.q1, msg.q2, msg.q3, msg.q4],  # w, x, y, z
            euler=(0, 0, 0),  # 将在buffer中自动计算
            euler_deg=(0, 0, 0),
            rates=[msg.rollspeed, msg.pitchspeed, msg.yawspeed],
            frequency=0.0  # 将在buffer中计算
        )

        # 放入队列（如果队列满会丢弃）
        success = await self.attitude_queue.put_nowait_or_drop(data)

        if not success:
            logger.warning("⚠️  队列已满，数据被丢弃")

        # 回调函数
        if self.on_attitude_callback:
            await self._call_callback(self.on_attitude_callback, data)

    async def _call_callback(self, callback, *args):
        """安全调用回调函数"""
        try:
            if asyncio.iscoroutinefunction(callback):
                await callback(*args)
            else:
                callback(*args)
        except Exception as e:
            logger.error(f"回调函数错误: {e}")

    async def get_attitude_data(self) -> Optional[AttitudeData]:
        """
        从队列获取姿态数据

        返回:
            AttitudeData对象或None
        """
        return await self.attitude_queue.get()

    def set_attitude_callback(self, callback: Callable):
        """
        设置姿态数据回调函数

        参数:
            callback: 回调函数（可以是同步或异步）
        """
        self.on_attitude_callback = callback

    def set_heartbeat_callback(self, callback: Callable):
        """设置心跳回调函数"""
        self.on_heartbeat_callback = callback

    def get_statistics(self) -> dict:
        """获取统计信息"""
        queue_stats = self.attitude_queue.get_stats()

        return {
            'connected': self.connected,
            'received_count': self.received_count,
            'error_count': self.error_count,
            'queue_size': queue_stats['current_size'],
            'queue_dropped': queue_stats['dropped_count']
        }


async def test_receiver():
    """测试接收器（示例代码）"""
    # 创建接收器
    receiver = AsyncMAVLinkReceiver(port='/dev/ttyUSB0', baudrate=921600)

    # 定义回调函数
    async def on_attitude(data: AttitudeData):
        logger.info(f"收到姿态数据: timestamp={data.timestamp:.3f}s, "
                   f"quaternion={data.quaternion}")

    # 设置回调
    receiver.set_attitude_callback(on_attitude)

    # 连接
    if await receiver.connect():
        # 启动接收
        await receiver.start()

        # 运行10秒
        await asyncio.sleep(10)

        # 停止
        await receiver.stop()
        await receiver.disconnect()

        # 打印统计
        stats = receiver.get_statistics()
        logger.info(f"统计信息: {stats}")


if __name__ == '__main__':
    # 运行测试
    asyncio.run(test_receiver())
