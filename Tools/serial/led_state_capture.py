import argparse
import time
import serial

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--port', required=True, help='COM port, e.g., COM5')
    p.add_argument('--baud', type=int, default=115200)
    p.add_argument('--seconds', type=int, default=60)
    p.add_argument('--outfile', default='logs/led_state_capture.txt')
    p.add_argument('--send', default=None, help='Optional text to send after opening (e.g., "mavlink status\n")')
    args = p.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.2)
    t0 = time.time()
    lines = []
    # small delay to settle
    time.sleep(0.2)
    if args.send:
        try:
            ser.write(args.send.encode('utf-8'))
            ser.flush()
        except Exception:
            pass
    try:
        while time.time() - t0 < args.seconds:
            data = ser.read(1024)
            if data:
                ts = time.strftime('%Y-%m-%d %H:%M:%S')
                lines.append(f'[{ts}] {data.decode(errors="ignore")}')
            time.sleep(0.05)
    finally:
        ser.close()
    # ensure logs directory
    import os
    os.makedirs('logs', exist_ok=True)
    with open(args.outfile, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines))
    print(f'Saved {len(lines)} lines to {args.outfile}')

if __name__ == '__main__':
    main()

