#!/usr/bin/env python3
"""
综合诊断脚本 - Nucleo-H743ZI-FC USB CDC ACM + MAVLink 验证
"""
import serial
import time
import sys

def send_command(ser, cmd, wait_time=0.5, description=""):
    """发送 NSH 命令并返回输出"""
    print(f"\n{'='*60}")
    print(f"测试: {description}")
    print(f"命令: {cmd}")
    print(f"{'='*60}")

    ser.reset_input_buffer()
    ser.write(f"{cmd}\n".encode())
    time.sleep(wait_time)

    output = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
    print(output)
    return output

def main():
    if len(sys.argv) < 2:
        print("用法: python comprehensive_diagnosis.py <串口>")
        print("示例: python comprehensive_diagnosis.py COM5")
        sys.exit(1)

    port = sys.argv[1]

    print(f"连接到 {port}...")
    try:
        ser = serial.Serial(port, 115200, timeout=2)
    except Exception as e:
        print(f"错误: 无法打开串口 {port}: {e}")
        sys.exit(1)

    time.sleep(0.5)

    print("""
╔════════════════════════════════════════════════════════════╗
║  Nucleo-H743ZI-FC 综合诊断脚本                             ║
║  测试项目：                                                ║
║  1. 设备节点检查                                           ║
║  2. USB CDC ACM 状态                                       ║
║  3. sensor_stub 状态                                       ║
║  4. dual_imu_fusion 状态                                   ║
║  5. uORB 话题发布状态                                      ║
║  6. MAVLink 状态                                           ║
║  7. 工作队列状态                                           ║
╚════════════════════════════════════════════════════════════╝
    """)

    results = {}

    # 1. 设备节点检查
    output = send_command(ser, "ls /dev", 0.3, "检查设备节点")
    results['ttyS0'] = 'ttyS0' in output
    results['ttyACM0'] = 'ttyACM0' in output

    # 2. USB CDC ACM 连接状态
    output = send_command(ser, "sercon", 0.5, "连接 USB CDC ACM")
    # sercon 成功通常无输出或返回提示

    # 3. sensor_stub 状态
    output = send_command(ser, "sensor_stub status", 0.3, "检查 sensor_stub")
    results['sensor_stub_running'] = 'Running: yes' in output or 'not running' not in output

    # 4. dual_imu_fusion 状态
    output = send_command(ser, "dual_imu_fusion status", 0.3, "检查 dual_imu_fusion")
    results['fusion_running'] = 'Running: yes' in output or 'not running' not in output

    # 5. uORB 状态
    output = send_command(ser, "uorb status", 1.0, "检查 uORB 话题")
    results['sensor_accel_exists'] = 'sensor_accel' in output
    results['vehicle_attitude_exists'] = 'vehicle_attitude' in output

    # 提取发布次数
    for line in output.split('\n'):
        if 'sensor_accel' in line:
            parts = line.split()
            if len(parts) >= 4:
                results['sensor_accel_published'] = int(parts[3]) > 0
                print(f"  → sensor_accel 发布次数: {parts[3]}")

        if 'vehicle_attitude' in line:
            parts = line.split()
            if len(parts) >= 4:
                results['vehicle_attitude_published'] = int(parts[3]) > 0
                print(f"  → vehicle_attitude 发布次数: {parts[3]}")

    # 6. MAVLink 状态
    output = send_command(ser, "mavlink status", 1.0, "检查 MAVLink")
    results['mavlink_running'] = 'mode:' in output
    results['mavlink_usb_cdc'] = 'type: USB CDC' in output or '/dev/ttyACM0' in output
    results['mavlink_heartbeat'] = 'GCS heartbeat valid: YES' in output

    # 7. 工作队列状态
    output = send_command(ser, "work_queue status", 0.5, "检查工作队列")
    results['wq_hp_default'] = 'hp_default' in output

    # 8. 查看系统日志中的错误
    output = send_command(ser, "dmesg | grep -i error", 1.5, "检查系统错误日志")
    error_lines = [line for line in output.split('\n') if 'ERROR' in line.upper()]
    results['errors_found'] = len(error_lines)

    # 9. 查看 USB 相关日志
    output = send_command(ser, "dmesg | grep -i usb", 1.0, "检查 USB 初始化日志")
    results['usb_initialized'] = 'usb' in output.lower() or 'USB' in output

    # 汇总结果
    print(f"\n\n{'='*60}")
    print("诊断结果汇总")
    print(f"{'='*60}\n")

    def print_result(name, status, good_msg, bad_msg):
        symbol = "✅" if status else "❌"
        msg = good_msg if status else bad_msg
        print(f"{symbol} {name}: {msg}")

    print("【设备节点】")
    print_result("ttyS0 (控制台)", results.get('ttyS0', False),
                 "存在", "缺失")
    print_result("ttyACM0 (USB CDC)", results.get('ttyACM0', False),
                 "存在", "缺失（USB 初始化失败？）")

    print("\n【模块状态】")
    print_result("sensor_stub", results.get('sensor_stub_running', False),
                 "运行中", "未运行")
    print_result("dual_imu_fusion", results.get('fusion_running', False),
                 "运行中", "未运行")

    print("\n【数据发布】")
    print_result("sensor_accel 话题", results.get('sensor_accel_exists', False),
                 "已创建", "不存在")
    print_result("sensor_accel 发布", results.get('sensor_accel_published', False),
                 "正在发布", "从未发布（sensor_stub 可能未工作）")
    print_result("vehicle_attitude 话题", results.get('vehicle_attitude_exists', False),
                 "已创建", "不存在")
    print_result("vehicle_attitude 发布", results.get('vehicle_attitude_published', False),
                 "正在发布", "从未发布（融合模块未输出）")

    print("\n【MAVLink】")
    print_result("MAVLink 运行", results.get('mavlink_running', False),
                 "正常运行", "未运行")
    print_result("USB CDC ACM 传输", results.get('mavlink_usb_cdc', False),
                 "使用 /dev/ttyACM0", "未使用 USB CDC")
    print_result("GCS 心跳", results.get('mavlink_heartbeat', False),
                 "已接收（QGC 已连接）", "未接收（QGC 未连接）")

    print("\n【系统】")
    print_result("USB 初始化", results.get('usb_initialized', False),
                 "已初始化", "未发现初始化日志")
    print_result("工作队列", results.get('wq_hp_default', False),
                 "hp_default 可用", "hp_default 不可用")

    if results.get('errors_found', 0) > 0:
        print(f"⚠️  系统错误: 发现 {results['errors_found']} 条错误日志")
    else:
        print(f"✅ 系统错误: 无错误")

    # 关键问题诊断
    print(f"\n{'='*60}")
    print("关键问题诊断")
    print(f"{'='*60}\n")

    if not results.get('ttyACM0', False):
        print("🔴 问题 1: /dev/ttyACM0 不存在")
        print("   原因: USB CDC ACM 初始化失败")
        print("   检查:")
        print("   - defconfig 中是否启用 CONFIG_USBDEV 和 CONFIG_CDCACM")
        print("   - init.cpp 中是否调用 stm32_usbinitialize()")
        print("   - board.h 中是否定义 GPIO_OTGFS_DM/DP")

    if not results.get('sensor_accel_published', False):
        print("\n🔴 问题 2: sensor_accel 从未发布")
        print("   原因: sensor_stub 的 Run() 方法未被调用")
        print("   可能:")
        print("   - ScheduleOnInterval() 失败")
        print("   - hp_default 工作队列未就绪")
        print("   建议: 改用 lp_default 工作队列")

    if not results.get('vehicle_attitude_published', False):
        print("\n🔴 问题 3: vehicle_attitude 从未发布")
        print("   原因: dual_imu_fusion 未输出")
        print("   可能:")
        print("   - sensor_accel/gyro 未发布（上游问题）")
        print("   - fusion 订阅失败")
        print("   - fusion 算法条件不满足")

    # Windows PC 端检查提示
    print(f"\n{'='*60}")
    print("Windows PC 端检查")
    print(f"{'='*60}\n")

    if results.get('ttyACM0', False) and results.get('mavlink_usb_cdc', False):
        print("✅ 设备端 USB CDC ACM 正常")
        print("\n请在 Windows 上检查:")
        print("1. 设备管理器 → 端口 (COM 和 LPT)")
        print("   应显示: PX4 Nucleo-H743ZI (COMx)")
        print("\n2. 如果未显示:")
        print("   - 确认第二根 USB 线已连接到 CN13（板子下方 USB 口）")
        print("   - 设备管理器 → 通用串行总线控制器")
        print("     查找 'PX4 Nucleo' 或 '未知设备'")
        print("\n3. 测试 MAVLink 接收:")
        print("   python Tools/flash/mavlink_recv_test.py COMx")
        print("   (将 COMx 替换为实际端口号)")
    else:
        print("❌ 设备端 USB CDC ACM 异常，无法在 PC 端测试")

    ser.close()

    # 返回码
    critical_issues = 0
    if not results.get('ttyACM0', False):
        critical_issues += 1
    if not results.get('sensor_accel_published', False):
        critical_issues += 1
    if not results.get('vehicle_attitude_published', False):
        critical_issues += 1

    print(f"\n{'='*60}")
    print(f"诊断完成。发现 {critical_issues} 个关键问题。")
    print(f"{'='*60}\n")

    sys.exit(critical_issues)

if __name__ == "__main__":
    main()
