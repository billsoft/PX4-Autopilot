#!/usr/bin/env python3
"""
自定义绘图示例

演示如何创建自定义的绘图布局
"""

import asyncio
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from realtime_monitor import (
    AsyncMAVLinkReceiver,
    AttitudeBuffer,
    SimpleAnglePlotter
)


async def main():
    """主函数"""
    PORT = 'COM3'
    BAUDRATE = 921600

    print("=" * 60)
    print("自定义绘图示例 - 仅显示欧拉角")
    print("=" * 60)

    # 创建组件
    buffer = AttitudeBuffer(maxlen=2000)
    receiver = AsyncMAVLinkReceiver(port=PORT, baudrate=BAUDRATE)
    plotter = SimpleAnglePlotter(buffer)  # 简化绘图器

    # 数据回调
    async def on_data(data):
        buffer.append_sync(data)

    receiver.set_attitude_callback(on_data)

    # 连接
    if not await receiver.connect():
        print("❌ 连接失败")
        return

    # 启动接收
    await receiver.start()

    # 等待数据
    await asyncio.sleep(1.0)

    # 启动绘图
    await plotter.start()

    print("✅ 监控已启动")
    print("📊 绘图窗口已打开 - 仅显示欧拉角")
    print("按 Ctrl+C 停止...\n")

    try:
        # 运行直到中断
        await asyncio.Event().wait()
    except KeyboardInterrupt:
        print("\n⏸️  正在停止...")
    finally:
        await plotter.stop()
        await receiver.stop()
        await receiver.disconnect()

        stats = buffer.get_statistics()
        print(f"\n📊 最终统计: 平均频率 {stats.avg_frequency:.1f} Hz")


if __name__ == '__main__':
    asyncio.run(main())
