import serial, time, sys

s = serial.Serial('/dev/ttyACM0', 115200, timeout=2)
time.sleep(0.5)

cmds = sys.argv[1:] if len(sys.argv) > 1 else ['?:']
for cmd in cmds:
    s.write((cmd + '\n').encode())
    time.sleep(0.3)
    try:
        resp = s.read(s.in_waiting)
        print(f"{cmd} -> {resp.decode().strip()}")
    except OSError:
        print(f"{cmd} -> (port closed - reboot triggered)")
        break

s.close()
