#!/usr/bin/env python3
"""
基础监控示例

演示如何使用RealtimeMonitor进行完整的姿态监控
"""

import asyncio
import sys
from pathlib import Path

# 添加父目录到路径
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from realtime_monitor.main import RealtimeMonitor
from realtime_monitor.config import update_config


async def main():
    """主函数"""
    # 配置串口（根据您的设备修改）
    PORT = 'COM3'  # Windows: COMx, Linux: /dev/ttyUSBx
    BAUDRATE = 921600

    # 更新配置
    update_config('serial', port=PORT, baudrate=BAUDRATE)
    update_config('plot', window_size=1000, update_interval=0.05)

    print("=" * 60)
    print("Pixhawk 基础监控示例")
    print("=" * 60)
    print(f"串口: {PORT}")
    print(f"波特率: {BAUDRATE}")
    print("=" * 60)
    print("\n按 Ctrl+C 停止...\n")

    # 创建监控系统
    monitor = RealtimeMonitor(
        port=PORT,
        baudrate=BAUDRATE,
        plot_mode='full'  # 完整模式：4个子图
    )

    # 运行监控
    try:
        await monitor.run_forever()
    except KeyboardInterrupt:
        print("\n用户中断...")
    finally:
        await monitor.stop()


if __name__ == '__main__':
    asyncio.run(main())
