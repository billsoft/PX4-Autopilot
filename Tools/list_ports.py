import serial.tools.list_ports
ports = serial.tools.list_ports.comports()
for port, desc, hwid in ports:
    print(f"{port}: {desc} [{hwid}]")
