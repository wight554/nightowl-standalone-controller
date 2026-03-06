
# Build and Flash Guide

This firmware uses the Raspberry Pi Pico SDK.

---

# Requirements

Install:

- cmake
- ninja
- gcc-arm-none-eabi
- pico-sdk
- picotool

---

# Build

Clean build:

cd ~/dev/nightowl-standalone-controller
rm -rf build
mkdir build
cd build

export PICO_SDK_PATH=~/dev/pico-sdk

cmake -G Ninja ../firmware
ninja

Result:

build/nightowl_controller.elf

---

# Flash

Using picotool:

sudo ~/dev/picotool/build/picotool load build/nightowl_controller.elf -f
sudo ~/dev/picotool/build/picotool reboot

---

# Full rebuild

rm -rf build
mkdir build
cd build
cmake -G Ninja ../firmware
ninja

---

# Troubleshooting

If cmake fails:

verify PICO_SDK_PATH

If flashing fails:

put Pico in BOOTSEL mode
