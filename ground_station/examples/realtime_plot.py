#!/usr/bin/env python3
"""
实时数据可视化示例

实时绘制Pixhawk的IMU、磁力计和姿态数据
"""

import sys
import time
import argparse
from pathlib import Path

# 添加模块路径
sys.path.insert(0, str(Path(__file__).parent.parent))

from pixhawk_monitor import PixhawkReceiver, RealtimePlotter


def main():
    parser = argparse.ArgumentParser(description='Pixhawk 实时数据可视化')
    parser.add_argument('--port', type=str, required=True,
                       help='串口设备 (例如: /dev/ttyUSB0 或 COM3)')
    parser.add_argument('--baud', type=int, default=921600,
                       help='波特率 (默认: 921600)')
    parser.add_argument('--window', type=int, default=500,
                       help='显示的数据点数量 (默认: 500)')
    parser.add_argument('--update-interval', type=int, default=50,
                       help='绘图更新间隔(毫秒) (默认: 50)')
    args = parser.parse_args()

    print("="*70)
    print("Pixhawk 实时数据可视化工具")
    print("="*70)
    print(f"串口: {args.port}")
    print(f"波特率: {args.baud}")
    print(f"数据窗口: {args.window} 个点")
    print(f"更新间隔: {args.update_interval} ms")
    print("="*70 + "\n")

    # 创建接收器
    receiver = PixhawkReceiver(args.port, args.baud)

    # 连接到Pixhawk
    print("🔌 正在连接Pixhawk...")
    if not receiver.connect():
        print("❌ 连接失败，退出")
        return

    # 启动接收线程
    print("📡 启动数据接收...")
    receiver.start_receiving()

    # 等待接收一些数据
    print("⏳ 等待数据...")
    time.sleep(2.0)

    # 创建绘图器
    print("📊 启动实时绘图...\n")
    plotter = RealtimePlotter(
        window_size=args.window,
        update_interval=args.update_interval
    )

    try:
        # 启动绘图（这会阻塞直到窗口关闭）
        plotter.start(receiver)

    except KeyboardInterrupt:
        print("\n⏹️  用户中断")

    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()

    finally:
        print("\n🛑 正在停止...")

        # 停止绘图
        plotter.stop()

        # 停止接收
        receiver.stop_receiving()

        # 打印统计信息
        receiver.print_stats()

        # 断开连接
        receiver.disconnect()

        print("✅ 程序已退出\n")


if __name__ == '__main__':
    main()
