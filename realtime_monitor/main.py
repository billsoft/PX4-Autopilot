"""
主程序 - 异步实时姿态监控

整合所有模块，提供完整的监控功能
"""

import asyncio
import argparse
import logging
import signal
import sys

from .async_receiver import AsyncMAVLinkReceiver
from .async_plotter import AsyncOscilloscopePlotter, SimpleAnglePlotter
from .data_buffer import AttitudeBuffer
from .config import get_config, update_config


# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class RealtimeMonitor:
    """
    实时监控主程序

    协调接收器、缓冲区和绘图器
    """

    def __init__(self, port: str, baudrate: int, plot_mode: str = 'full'):
        """
        初始化监控系统

        参数:
            port: 串口设备
            baudrate: 波特率
            plot_mode: 绘图模式 ('full' 或 'simple')
        """
        self.port = port
        self.baudrate = baudrate
        self.plot_mode = plot_mode

        # 创建组件
        self.buffer = AttitudeBuffer(maxlen=2000)
        self.receiver = AsyncMAVLinkReceiver(port, baudrate)

        if plot_mode == 'full':
            self.plotter = AsyncOscilloscopePlotter(self.buffer)
        else:
            self.plotter = SimpleAnglePlotter(self.buffer)

        # 任务列表
        self.tasks = []

        # 运行标志
        self.running = False

    async def start(self):
        """启动监控系统"""
        logger.info("=" * 60)
        logger.info("🚀 Pixhawk 实时姿态监控系统 (异步协程版)")
        logger.info("=" * 60)
        logger.info(f"串口: {self.port}")
        logger.info(f"波特率: {self.baudrate}")
        logger.info(f"绘图模式: {self.plot_mode}")
        logger.info("=" * 60)

        # 连接接收器
        logger.info("⏳ 正在连接Pixhawk...")
        if not await self.receiver.connect():
            logger.error("❌ 连接失败，退出")
            return False

        logger.info("✅ 连接成功！")

        # 设置回调：将接收到的数据放入缓冲区
        async def on_attitude_data(data):
            self.buffer.append_sync(data)  # 使用同步方法避免协程嵌套

        self.receiver.set_attitude_callback(on_attitude_data)

        # 启动接收器
        await self.receiver.start()

        # 等待一些数据
        logger.info("⏳ 等待数据...")
        await asyncio.sleep(1.0)

        # 启动绘图器
        await self.plotter.start()

        # 启动统计信息打印任务
        stats_task = asyncio.create_task(self._print_stats_periodically())
        self.tasks.append(stats_task)

        self.running = True
        logger.info("✅ 监控系统已启动！")
        logger.info("按 Ctrl+C 停止...")

        return True

    async def stop(self):
        """停止监控系统"""
        if not self.running:
            return

        logger.info("\n⏸️  正在停止监控系统...")

        self.running = False

        # 取消所有任务
        for task in self.tasks:
            task.cancel()

        # 等待任务完成
        await asyncio.gather(*self.tasks, return_exceptions=True)

        # 停止组件
        await self.plotter.stop()
        await self.receiver.stop()
        await self.receiver.disconnect()

        # 打印最终统计
        logger.info("\n" + "=" * 60)
        logger.info("📊 最终统计")
        logger.info("=" * 60)

        stats = self.buffer.get_statistics()
        logger.info(f"总数据包:   {stats.total_packets}")
        logger.info(f"丢包数量:   {stats.lost_packets}")
        logger.info(f"丢包率:     {stats.loss_rate:.2f}%")
        logger.info(f"平均频率:   {stats.avg_frequency:.1f} Hz")

        recv_stats = self.receiver.get_statistics()
        logger.info(f"接收错误:   {recv_stats['error_count']}")
        logger.info(f"队列丢弃:   {recv_stats['queue_dropped']}")

        logger.info("=" * 60)
        logger.info("✅ 监控系统已停止")

    async def _print_stats_periodically(self):
        """定期打印统计信息"""
        try:
            while self.running:
                await asyncio.sleep(5.0)  # 每5秒打印一次

                stats = self.buffer.get_statistics()
                latest = self.buffer.get_latest()

                if latest:
                    logger.info(
                        f"📊 频率: {stats.avg_frequency:.1f} Hz | "
                        f"丢包: {stats.loss_rate:.2f}% | "
                        f"Roll: {latest.euler_deg[0]:.2f}° | "
                        f"Pitch: {latest.euler_deg[1]:.2f}° | "
                        f"Yaw: {latest.euler_deg[2]:.2f}°"
                    )

        except asyncio.CancelledError:
            pass

    async def run_forever(self):
        """运行直到中断"""
        if not await self.start():
            return

        try:
            # 等待中断信号
            while self.running:
                await asyncio.sleep(0.1)
        except asyncio.CancelledError:
            pass
        finally:
            await self.stop()


# ========== 主函数 ==========

async def main():
    """异步主函数"""
    # 解析命令行参数
    parser = argparse.ArgumentParser(
        description='Pixhawk 实时姿态监控系统 (异步协程版)'
    )
    parser.add_argument(
        '--port', '-p',
        type=str,
        default=None,
        help='串口设备 (例如: /dev/ttyUSB0 或 COM3)'
    )
    parser.add_argument(
        '--baud', '-b',
        type=int,
        default=None,
        help='波特率 (默认: 921600)'
    )
    parser.add_argument(
        '--mode', '-m',
        type=str,
        choices=['full', 'simple'],
        default='full',
        help='绘图模式: full(完整) 或 simple(简化)'
    )
    parser.add_argument(
        '--window', '-w',
        type=int,
        default=1000,
        help='显示窗口大小(数据点数量)'
    )

    args = parser.parse_args()

    # 更新配置
    if args.port:
        update_config('serial', port=args.port)
    if args.baud:
        update_config('serial', baudrate=args.baud)
    if args.window:
        update_config('plot', window_size=args.window)

    # 获取最终配置
    serial_cfg = get_config('serial')

    # 创建监控系统
    monitor = RealtimeMonitor(
        port=serial_cfg.port,
        baudrate=serial_cfg.baudrate,
        plot_mode=args.mode
    )

    # 设置信号处理
    loop = asyncio.get_event_loop()

    def signal_handler():
        logger.info("\n⚠️  收到中断信号...")
        asyncio.create_task(monitor.stop())

    # Windows和Linux的信号处理不同
    if sys.platform == 'win32':
        # Windows不支持SIGINT在asyncio中
        pass
    else:
        loop.add_signal_handler(signal.SIGINT, signal_handler)
        loop.add_signal_handler(signal.SIGTERM, signal_handler)

    # 运行监控系统
    try:
        await monitor.run_forever()
    except KeyboardInterrupt:
        logger.info("\n⚠️  用户中断...")
        await monitor.stop()


def run():
    """同步入口函数"""
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("\n👋 再见！")
    except Exception as e:
        logger.error(f"❌ 发生错误: {e}", exc_info=True)
        sys.exit(1)


if __name__ == '__main__':
    run()
