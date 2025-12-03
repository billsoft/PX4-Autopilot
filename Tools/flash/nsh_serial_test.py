import sys
import time

def main():
    try:
        import serial
    except Exception as e:
        print("pyserial not available:", e)
        sys.exit(2)

    port = sys.argv[1] if len(sys.argv) > 1 else 'COM5'
    baud = 115200
    try:
        ser = serial.Serial(port, baud, timeout=1)
    except Exception as e:
        print("open serial failed:", e)
        sys.exit(3)

    def send(cmd):
        ser.write((cmd+'\r\n').encode('ascii'))
        time.sleep(0.2)

    send('help')
    send('sercon')
    time.sleep(1.0)
    # retry starting modules after init thread
    for _ in range(3):
        send('sensor_stub start')
        send('dual_imu_fusion start')
        time.sleep(0.5)
    send('sensors start')
    send('uorb status')
    send('ls /dev')
    send('dual_imu_fusion status')
    send('board_status_leds status')
    send('listener sensor_accel 0')
    send('listener sensor_accel 1')
    send('listener sensor_gyro 0')
    send('listener sensor_gyro 1')
    send('listener sensor_mag 0')
    # dataman required by mavlink
    send('dataman start')
    # attempt to start MAVLink on CDC ACM
    send('mavlink start -d /dev/ttyACM0')
    time.sleep(0.5)
    send('mavlink stream -d /dev/ttyACM0 -s ATTITUDE_QUATERNION -r 120')
    send('mavlink status')
    send('uorb top')
    send('listener vehicle_attitude')
    start = time.time()
    buf = []
    while time.time()-start < 3.0:
        line = ser.readline().decode('ascii', errors='ignore')
        if line:
            buf.append(line)
    ser.close()
    out = ''.join(buf)
    print('--- OUTPUT START ---')
    print(out)
    print('--- OUTPUT END ---')
    va_present = ('vehicle_attitude' in out)
    print('vehicle_attitude topic present:', va_present)
    sys.exit(0 if va_present else 1)

if __name__ == '__main__':
    main()
