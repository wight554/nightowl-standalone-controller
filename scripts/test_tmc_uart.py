import serial
import time
import sys
import glob

def find_port():
    ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/cu.usbmodem*')
    if not ports:
        print("Error: Could not find a serial port.")
        sys.exit(1)
    return ports[0]

def send_cmd(s, cmd):
    print(f"Send: {cmd}")
    s.write((cmd + '\n').encode())
    time.sleep(0.3)
    resp = s.read(s.in_waiting).decode().strip()
    print(f"Recv: {resp}")
    return resp

def main():
    port = find_port()
    if len(sys.argv) > 1:
        port = sys.argv[1]
        
    print(f"Connecting to {port}...")
    try:
        s = serial.Serial(port, 115200, timeout=1)
        time.sleep(0.5)
    except Exception as e:
        print(f"Failed to open port: {e}")
        sys.exit(1)

    print("\n--- Verifying TMC UART Read/Write ---")
    
    # 1. Read the current CHOPCONF register (Register 0x6C / 108) on Lane 1
    # Note: Klipper/TMC defaults CHOPCONF to something like 0x10410150
    print("\n1. Reading initial CHOPCONF (reg 108) on Lane 1:")
    send_cmd(s, "TR:1:108")
    
    # 2. Write a new CHOPCONF value
    # Let's set it to 0x10410153 (same but TOFF=3)
    # We'll write this and then read it back to verify.
    print("\n2. Writing new CHOPCONF value (0x10410153) to Lane 1:")
    send_cmd(s, "TW:1:108:272695635") # 272695635 is 0x10410153 in decimal
    
    # 3. Verify the write
    print("\n3. Reading back CHOPCONF to verify write:")
    send_cmd(s, "TR:1:108")

    # 4. Write back original value (assuming it was 0x10410150)
    # This is just an example of writing back a value.
    print("\n4. Resetting CHOPCONF back to 0x10410150:")
    send_cmd(s, "TW:1:108:272695632") # 272695632 is 0x10410150 in decimal
    
    print("\n5. Reading back CHOPCONF to verify reset:")
    send_cmd(s, "TR:1:108")

    s.close()
    print("\nDone.")

if __name__ == '__main__':
    main()
