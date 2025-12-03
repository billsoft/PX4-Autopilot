import sys
import time
import struct

try:
    import serial
except Exception as e:
    print("pyserial not available:", e)
    sys.exit(2)

def parse_v2(buf, i):
    if buf[i] != 0xFD:
        return None
    if i + 10 >= len(buf):
        return None
    length = buf[i+1]
    incompat = buf[i+2]
    compat = buf[i+3]
    seq = buf[i+4]
    sysid = buf[i+5]
    compid = buf[i+6]
    msgid = buf[i+7] | (buf[i+8] << 8) | (buf[i+9] << 16)
    total = 10 + length + 2  # header + payload + crc (no sig)
    if i + total > len(buf):
        return None
    return (msgid, total)

def parse_v1(buf, i):
    if buf[i] != 0xFE:
        return None
    if i + 6 >= len(buf):
        return None
    length = buf[i+1]
    msgid = buf[i+5]
    total = 6 + length + 2
    if i + total > len(buf):
        return None
    return (msgid, total)

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else None
    if not port:
        print("Usage: python mavlink_recv_test.py <COMx|/dev/ttyACM0>")
        sys.exit(1)
    try:
        ser = serial.Serial(port, 115200, timeout=0.1)
    except Exception as e:
        print("open serial failed:", e)
        sys.exit(3)
    print("opened", port)
    start = time.time()
    count_att_q = 0
    count_heartbeat = 0
    buf = bytearray()
    while time.time() - start < 5.0:
        data = ser.read(1024)
        if data:
            buf.extend(data)
            i = 0
            while i < len(buf):
                res = parse_v2(buf, i)
                if not res:
                    res = parse_v1(buf, i)
                if not res:
                    i += 1
                    continue
                msgid, total = res
                if msgid == 31:
                    count_att_q += 1
                if msgid == 0:
                    count_heartbeat += 1
                i += total
            if i > 0:
                del buf[:i]
    ser.close()
    dt = time.time() - start
    print("Duration:", dt)
    print("ATTITUDE_QUATERNION frames:", count_att_q)
    print("HEARTBEAT frames:", count_heartbeat)
    rate = count_att_q / dt if dt > 0 else 0
    print("Estimated QUAT rate (Hz):", round(rate, 1))
    sys.exit(0 if count_heartbeat > 0 else 1)

if __name__ == '__main__':
    main()
