# NightOwl Standalone Controller

NightOwl is a standalone dual-lane filament controller for ERB v2.0 (RP2040).
It can run without a host plugin and handles lane switching, buffer-driven feed,
and TMC2209 register/config tuning over USB serial.

## What Is In This Repo

- `firmware/`: RP2040 firmware (C, Pico SDK)
- `scripts/`: build/config/flash/tuning helpers
- `config.ini`: user tuning source for motor and TMC defaults
- `config.ini.example`: template for new setups

## Quick Start

1. Copy and edit config:

```bash
cp config.ini.example config.ini
```

2. Generate compile-time tuning header from `config.ini`:

```bash
python3 scripts/gen_motor_config.py
```

3. Build firmware:

```bash
cmake -S firmware -B build_local -G Ninja -DPICO_SDK_PATH=/path/to/pico-sdk
cmake --build build_local
```

4. Flash firmware (auto-detect serial, trigger BOOTSEL when possible):

```bash
bash scripts/flash_nightowl.sh
```

## Configuration Model

`config.ini` is the source of compile-time motor/TMC defaults.
`scripts/gen_motor_config.py` generates `firmware/include/tune.h`.

Mandatory keys:

- `microsteps`
- `rotation_distance`
- `run_current`

Typical workflow:

```bash
python3 scripts/gen_motor_config.py
cmake --build build_local
```

Runtime changes (serial protocol `SET:/GET:/TW:/TR:`) can be saved to flash via `SV:`.

## Tuning Scripts

- `scripts/klipper_tune.py`: apply/read Klipper-style parameters
- `scripts/tmc_chopconf.py`: direct CHOPCONF read/write utility
- `scripts/nightowl_test.py`: send arbitrary serial commands quickly

All scripts support `--port`; if omitted they auto-detect from available serial devices.

Examples:

```bash
python3 scripts/klipper_tune.py --lane 1 read
python3 scripts/klipper_tune.py --lane 2 apply config.ini
python3 scripts/tmc_chopconf.py --lane 1 read
python3 scripts/nightowl_test.py "VR:" "?:"
```

## Build Notes

- Build output directory used in this repo is `build_local/`.
- Flash script uses `picotool` if available in PATH, otherwise checks local build outputs.
- For detailed flash/build troubleshooting, see `BUILD_FLASH.md`.

## Hardware and Operation Docs

- `HARDWARE.md`: board wiring and hardware assumptions
- `MANUAL.md`: runtime behavior and operator guidance
- `WORKFLOW.md`: Git branching and release discipline

---

# Development

Branches:

main      stable firmware  
dev       integration branch  
feature/* development branches  
fix/*     bug fixes  

Never develop directly on **main**.

---

# Safety

Always test firmware changes at low speed.

Verify sensor polarity before enabling automatic swap.
