import serial
import time
import sys

port = "COM5"
baud = 115200

try:
    ser = serial.Serial(port, baud, timeout=1)
except Exception as e:
    print(f"无法打开串口 {port}: {e}")
    sys.exit(1)

def send_cmd(cmd, wait_time=1.0):
    print(f"\n--- 发送命令: {cmd} ---")
    ser.write(f"{cmd}\n".encode())
    time.sleep(0.1) # Give time for transmission

    response = b""
    start_time = time.time()
    while time.time() - start_time < wait_time:
        if ser.in_waiting > 0:
            chunk = ser.read(ser.in_waiting)
            response += chunk
            # Reset timeout if we got data, to catch long outputs
            if time.time() - start_time > wait_time - 0.5:
                 start_time = time.time() - (wait_time - 0.5)
        else:
            time.sleep(0.1)

    try:
        print(response.decode('utf-8', errors='replace'))
    except:
        print(response)

# 唤醒 NSH
print("正在连接 NSH...")
ser.write(b"\n\n\n")
time.sleep(1)
if ser.in_waiting:
    print(ser.read(ser.in_waiting).decode('utf-8', errors='replace'), end='')

# 基本系统信息
send_cmd("ls /", 2)
send_cmd("ls /etc", 2)
send_cmd("ls /etc/init.d", 2)
send_cmd("free", 2)
send_cmd("ps", 2)

# 检查 dmesg（等待系统启动完成）
send_cmd("dmesg", 3)

# 检查 uORB 与驱动状态
send_cmd("uorb status", 2)
send_cmd("uorb top -1", 3) # Run once
send_cmd("icm42688p status", 2)
send_cmd("bmm150 status", 2)

# 检查 MAVLink 状态
send_cmd("mavlink status", 2)

send_cmd("board_status_leds start", 1.5)
send_cmd("board_status_leds status", 2)
send_cmd("board_status_leds test 5", 2)

ser.close()
