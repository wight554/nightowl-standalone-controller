#!/bin/bash
set -e

REPO="$(cd "$(dirname "$0")/.." && pwd)"
PORT=/dev/ttyACM0
MOUNT=/mnt/rp2

echo "=== Pulling latest ==="
cd $REPO
git pull

echo "=== Building ==="
mkdir -p build
cd build
cmake -G Ninja ../firmware -DPICO_SDK_PATH=~/dev/pico-sdk 2>&1 | tail -3
ninja

echo "=== Triggering BOOTSEL via USB ==="
if [ -e $PORT ]; then
    python3 - << 'PYEOF'
import serial, time
s = serial.Serial('/dev/ttyACM0', 115200, timeout=2)
time.sleep(0.5)
s.write(b'BOOT:\n')
time.sleep(0.3)
try:
    print(s.read(s.in_waiting).decode().strip())
except OSError:
    print("(port closed - reboot triggered)")
s.close()
PYEOF
else
    echo "Port not found — assuming already in BOOTSEL"
fi

echo "=== Waiting for RPI-RP2 ==="
for i in $(seq 1 15); do
    sleep 1
    if lsblk | grep -q sda; then
        echo "Found after ${i}s"
        break
    fi
    echo "Waiting... ${i}s"
done

echo "=== Mounting and flashing ==="
sudo mkdir -p $MOUNT
sudo mount /dev/sda1 $MOUNT
sudo cp $REPO/build/nightowl_controller.uf2 $MOUNT/
sudo umount $MOUNT

echo "=== Waiting for USB serial ==="
for i in $(seq 1 15); do
    sleep 1
    if [ -e $PORT ]; then
        echo "Serial up after ${i}s"
        break
    fi
    echo "Waiting... ${i}s"
done

echo "=== Verifying ==="
python3 - << 'PYEOF'
import serial, time
s = serial.Serial('/dev/ttyACM0', 115200, timeout=2)
time.sleep(0.5)
s.write(b'VR:\n')
time.sleep(0.3)
print(s.read(s.in_waiting).decode().strip())
s.close()
PYEOF

echo "=== Done ==="
