#!/usr/bin/env python3
"""
纯数据接收示例

仅接收和打印数据，不进行绘图
适合嵌入式设备或数据记录
"""

import asyncio
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from realtime_monitor import (
    AsyncMAVLinkReceiver,
    AttitudeBuffer,
    AttitudeData
)


async def main():
    """主函数"""
    PORT = 'COM3'  # 修改为您的串口
    BAUDRATE = 921600

    # 创建组件
    buffer = AttitudeBuffer(maxlen=1000)
    receiver = AsyncMAVLinkReceiver(port=PORT, baudrate=BAUDRATE)

    # 定义回调函数：打印接收到的数据
    async def on_attitude_data(data: AttitudeData):
        """数据回调"""
        # 每10个数据打印一次
        if buffer.received_count % 10 == 0:
            stats = buffer.get_statistics()
            print(
                f"\r⏰ {data.timestamp:.2f}s | "
                f"频率: {stats.avg_frequency:.1f} Hz | "
                f"Roll: {data.euler_deg[0]:7.2f}° | "
                f"Pitch: {data.euler_deg[1]:7.2f}° | "
                f"Yaw: {data.euler_deg[2]:7.2f}° | "
                f"丢包: {stats.loss_rate:.2f}%",
                end='', flush=True
            )

        # 添加到缓冲区
        buffer.append_sync(data)

    # 设置回调
    receiver.set_attitude_callback(on_attitude_data)

    # 连接
    print(f"🔌 正在连接 {PORT} @ {BAUDRATE}...")
    if not await receiver.connect():
        print("❌ 连接失败")
        return

    print("✅ 连接成功！")
    print("📡 开始接收数据...\n")

    # 启动接收
    await receiver.start()

    # 统计任务
    async def print_stats():
        """定期打印统计信息"""
        while True:
            await asyncio.sleep(10.0)
            stats = buffer.get_statistics()
            print(f"\n\n📊 统计 (最近10秒):")
            print(f"   总数据包: {stats.total_packets}")
            print(f"   丢包数量: {stats.lost_packets}")
            print(f"   丢包率:   {stats.loss_rate:.2f}%")
            print(f"   平均频率: {stats.avg_frequency:.1f} Hz\n")

    stats_task = asyncio.create_task(print_stats())

    try:
        print("按 Ctrl+C 停止...\n")
        # 运行直到中断
        await asyncio.Event().wait()
    except KeyboardInterrupt:
        print("\n\n⏸️  正在停止...")
    finally:
        stats_task.cancel()
        await receiver.stop()
        await receiver.disconnect()

        # 打印最终统计
        final_stats = buffer.get_statistics()
        print("\n" + "=" * 60)
        print("📊 最终统计")
        print("=" * 60)
        print(f"总数据包:   {final_stats.total_packets}")
        print(f"丢包数量:   {final_stats.lost_packets}")
        print(f"丢包率:     {final_stats.loss_rate:.2f}%")
        print(f"平均频率:   {final_stats.avg_frequency:.1f} Hz")
        print("=" * 60)


if __name__ == '__main__':
    asyncio.run(main())
