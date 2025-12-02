#!/usr/bin/env python3
"""
Nucleo-H743ZI-FC 即时测试脚本
快速验证板子状态并生成报告

用法: python Tools/test_board_now.py
"""

import serial
import time
import re
from datetime import datetime

# 配置
PORT = 'COM5'
BAUDRATE = 115200
TIMEOUT = 2.0

class QuickTester:
    def __init__(self):
        self.ser = serial.Serial(PORT, BAUDRATE, timeout=TIMEOUT)
        time.sleep(0.5)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.results = []

    def send_cmd(self, cmd, wait=1.0):
        """发送命令并读取输出"""
        self.ser.write(f"{cmd}\n".encode('utf-8'))
        time.sleep(wait)
        output = ""
        while self.ser.in_waiting > 0:
            output += self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
            time.sleep(0.1)
        return output

    def test(self, name, cmd, check_func, wait=1.0):
        """执行单个测试"""
        print(f"[TEST] {name}...", end=' ')
        output = self.send_cmd(cmd, wait)
        passed, msg = check_func(output)
        status = "✅ PASS" if passed else "❌ FAIL"
        print(status)
        if not passed:
            print(f"  └─ {msg}")
        self.results.append({
            'name': name,
            'passed': passed,
            'output': output,
            'msg': msg
        })
        return passed, output

    def run_all_tests(self):
        """运行所有测试"""
        print("\n" + "="*60)
        print("  Nucleo-H743ZI-FC 自动化测试")
        print("  端口: COM5 @ 115200")
        print("="*60 + "\n")

        # 清屏并发送回车
        self.send_cmd("", 0.5)

        # 测试1: 检查错误
        self.test(
            "1. 启动日志检查",
            "dmesg | grep -i error",
            lambda out: (
                'ERROR' not in out.upper() or out.count('ERROR') == 0,
                "检测到启动错误" if 'ERROR' in out.upper() else "无错误"
            ),
            wait=1.5
        )

        # 测试2: dual_imu_fusion进程
        self.test(
            "2. dual_imu_fusion进程",
            "ps | grep dual_imu",
            lambda out: (
                'dual_imu_fusion' in out,
                "进程运行中" if 'dual_imu_fusion' in out else "进程未运行"
            )
        )

        # 测试3: I2C设备
        self.test(
            "3. I2C1 BMM150检测",
            "i2cdetect -b 1",
            lambda out: (
                '10' in out or '0x10' in out,
                "检测到0x10" if '10' in out else "未检测到BMM150"
            ),
            wait=2.0
        )

        # 测试4: ICM42688P状态
        passed, out = self.test(
            "4. ICM42688P驱动状态",
            "icm42688p status",
            lambda out: (
                out.count('Running: yes') >= 2,
                f"{out.count('Running: yes')}个IMU运行" if out.count('Running: yes') > 0 else "IMU未运行"
            ),
            wait=1.5
        )

        # 测试5: BMM150状态
        self.test(
            "5. BMM150驱动状态",
            "bmm150 status",
            lambda out: (
                'Running: yes' in out,
                "运行中" if 'Running: yes' in out else "未运行"
            )
        )

        # 测试6: 加速度计数据
        print(f"[TEST] 6. 加速度计数据...", end=' ')
        self.ser.write(b"listener sensor_accel 0\n")
        time.sleep(2.5)
        self.ser.write(b"\x03")  # Ctrl+C
        time.sleep(0.5)
        output = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')

        z_values = re.findall(r'z:\s*([-\d.]+)', output)
        if z_values:
            z_avg = sum(float(z) for z in z_values) / len(z_values)
            passed = -10.2 <= z_avg <= -9.6
            msg = f"z={z_avg:.2f} m/s²"
            print("✅ PASS" if passed else "❌ FAIL")
            if not passed:
                print(f"  └─ {msg} (期望 -9.8±0.4)")
        else:
            passed = False
            msg = "无数据"
            print("❌ FAIL")
            print(f"  └─ {msg}")

        self.results.append({
            'name': '6. 加速度计数据',
            'passed': passed,
            'output': output[:300],
            'msg': msg
        })

        # 测试7: 姿态数据
        print(f"[TEST] 7. 姿态数据更新...", end=' ')
        self.ser.write(b"listener vehicle_attitude\n")
        time.sleep(2.5)
        self.ser.write(b"\x03")
        time.sleep(0.5)
        output = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')

        passed = 'roll:' in output and 'pitch:' in output
        timestamps = re.findall(r'timestamp:\s*(\d+)', output)
        if len(timestamps) >= 2:
            dt = (int(timestamps[-1]) - int(timestamps[0])) / 1e6 / (len(timestamps) - 1)
            freq = 1.0 / dt if dt > 0 else 0
            msg = f"{freq:.1f} Hz"
        else:
            msg = "检测到数据" if passed else "无数据"

        print("✅ PASS" if passed else "❌ FAIL")
        if not passed:
            print(f"  └─ {msg}")

        self.results.append({
            'name': '7. 姿态数据更新',
            'passed': passed,
            'output': output[:300],
            'msg': msg
        })

        # 测试8: MAVLink流
        self.test(
            "8. MAVLink流配置",
            "mavlink status streams",
            lambda out: (
                'ATTITUDE_QUATERNION' in out and 'HIGHRES_IMU' not in out,
                "配置正确" if 'ATTITUDE_QUATERNION' in out else "缺少ATTITUDE_QUATERNION"
            ),
            wait=1.5
        )

    def print_summary(self):
        """打印测试总结"""
        passed = sum(1 for r in self.results if r['passed'])
        total = len(self.results)

        print("\n" + "="*60)
        print("  测试总结")
        print("="*60)
        print(f"通过: {passed}/{total} ({passed/total*100:.1f}%)")
        print(f"失败: {total-passed}/{total}")

        if passed == total:
            print("\n✅ 所有测试通过！系统运行正常。")
        else:
            print("\n❌ 检测到以下问题:\n")
            for r in self.results:
                if not r['passed']:
                    print(f"  • {r['name']}: {r['msg']}")

        # 保存报告
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        report_file = f"test_results_{timestamp}.md"

        with open(report_file, 'w', encoding='utf-8') as f:
            f.write(f"# 测试报告\n\n")
            f.write(f"**时间**: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"**端口**: {PORT}\n")
            f.write(f"**结果**: {passed}/{total} 通过\n\n")

            for r in self.results:
                status = "✅ PASS" if r['passed'] else "❌ FAIL"
                f.write(f"## {r['name']} - {status}\n\n")
                f.write(f"**信息**: {r['msg']}\n\n")
                if not r['passed']:
                    f.write(f"**输出**:\n```\n{r['output'][:500]}\n```\n\n")

        print(f"\n报告已保存: {report_file}")

        return passed == total

    def close(self):
        """关闭串口"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("\n串口已关闭")

def main():
    try:
        print("正在连接到板子...")
        tester = QuickTester()

        # 运行测试
        tester.run_all_tests()

        # 打印总结
        all_passed = tester.print_summary()

        # 关闭连接
        tester.close()

        return 0 if all_passed else 1

    except serial.SerialException as e:
        print(f"\n❌ 串口错误: {e}")
        print("请检查:")
        print("  1. COM5 是否正确")
        print("  2. 板子是否已连接")
        print("  3. 串口是否被其他程序占用")
        return 1
    except Exception as e:
        print(f"\n❌ 测试异常: {e}")
        import traceback
        traceback.print_exc()
        return 1

if __name__ == '__main__':
    exit(main())
