
# Hardware Overview

The controller uses an RP2040 board.

Main functions:

- dual filament drive
- filament sensors
- buffer monitoring
- Y splitter detection
- OLED display
- rotary encoder control

---

# Pin Mapping

Lane 1 IN  : GPIO24
Lane 1 OUT : GPIO25

Lane 2 IN  : GPIO22
Lane 2 OUT : GPIO12

Y splitter : GPIO2

Buffer LOW : GPIO6
Buffer HIGH: GPIO7

Motion sensor : GPIO5

Runout output : GPIO18

---

# Motor Drivers

Motor 1

EN   : GPIO8
DIR  : GPIO9
STEP : GPIO10

Motor 2

EN   : GPIO14
DIR  : GPIO15
STEP : GPIO16

---

# Display

OLED controller:
SH1106

Interface:
I2C

Pins:

SDA : GPIO26
SCL : GPIO27

Address:
0x3C

---

# Encoder

A : GPIO28
B : GPIO4

Buttons

Back    : GPIO3
Confirm : GPIO29

---

# Power

Stepper drivers require external supply.

Logic runs from RP2040 3.3V.

Never power motors from Pico 5V rail.
