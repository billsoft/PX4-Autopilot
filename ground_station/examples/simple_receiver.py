#!/usr/bin/env python3
"""
简单数据接收示例

演示如何接收和打印Pixhawk的IMU和姿态数据
"""

import sys
import time
import argparse
from pathlib import Path

# 添加模块路径
sys.path.insert(0, str(Path(__file__).parent.parent))

from pixhawk_monitor import PixhawkReceiver, quat_to_euler_deg


def main():
    parser = argparse.ArgumentParser(description='Pixhawk 简单数据接收器')
    parser.add_argument('--port', type=str, required=True,
                       help='串口设备 (例如: /dev/ttyUSB0 或 COM3)')
    parser.add_argument('--baud', type=int, default=921600,
                       help='波特率 (默认: 921600)')
    args = parser.parse_args()

    # 创建接收器
    receiver = PixhawkReceiver(args.port, args.baud)

    # 连接到Pixhawk
    if not receiver.connect():
        print("连接失败，退出")
        return

    # 启动接收线程
    receiver.start_receiving()

    print("\n" + "="*70)
    print("开始接收数据 (Ctrl+C 退出)")
    print("="*70 + "\n")

    try:
        last_print_time = time.time()
        print_interval = 0.5  # 每0.5秒打印一次

        while True:
            # 获取IMU数据
            imu_data = receiver.get_highres_imu(timeout=0.01)
            if imu_data:
                # 每0.5秒打印一次IMU数据
                current_time = time.time()
                if current_time - last_print_time >= print_interval:
                    print(f"\n⏰ 时间: {imu_data['timestamp']:.3f}s | 频率: {imu_data['frequency']:.1f} Hz")
                    print(f"📊 加速度 (m/s²): X={imu_data['accel'][0]:7.3f}  Y={imu_data['accel'][1]:7.3f}  Z={imu_data['accel'][2]:7.3f}")
                    print(f"🔄 陀螺仪 (rad/s): X={imu_data['gyro'][0]:7.3f}  Y={imu_data['gyro'][1]:7.3f}  Z={imu_data['gyro'][2]:7.3f}")
                    print(f"🧭 磁力计 (Gauss): X={imu_data['mag'][0]:7.3f}  Y={imu_data['mag'][1]:7.3f}  Z={imu_data['mag'][2]:7.3f}")
                    print(f"🌡️  温度: {imu_data['temperature']:.1f}°C | 气压: {imu_data['pressure']:.1f} mbar")

            # 获取姿态数据
            att_data = receiver.get_attitude(timeout=0.01)
            if att_data and current_time - last_print_time >= print_interval:
                # 转换为欧拉角
                roll, pitch, yaw = quat_to_euler_deg(att_data['quaternion'])

                print(f"\n✈️  姿态 (度):     Roll={roll:7.2f}°  Pitch={pitch:7.2f}°  Yaw={yaw:7.2f}°")
                print(f"🔄 角速度 (rad/s): Roll={att_data['rates'][0]:7.3f}  Pitch={att_data['rates'][1]:7.3f}  Yaw={att_data['rates'][2]:7.3f}")
                print(f"🎯 四元数: [{att_data['quaternion'][0]:.4f}, {att_data['quaternion'][1]:.4f}, {att_data['quaternion'][2]:.4f}, {att_data['quaternion'][3]:.4f}]")

                last_print_time = current_time

            time.sleep(0.001)  # 短暂休眠

    except KeyboardInterrupt:
        print("\n\n⏹️  接收中断，正在停止...")

    finally:
        # 停止接收并打印统计
        receiver.stop_receiving()
        receiver.print_stats()
        receiver.disconnect()

        print("\n✅ 程序已退出\n")


if __name__ == '__main__':
    main()
