# MIMO

**A tiny electronic companion that brings personality and playful motion to your keychain.**

MIMO pairs an expressive animated face with a gravity-driven liquid simulation. Tilt it, shake it, or let it rest: the display reacts to movement, falls asleep when idle, and wakes with motion.

## Prototype v0.1.0

- **Buddy mode:** springy eyes, blinking, curious glances, and motion-driven expressions.
- **Liquid mode:** 480 simulated particles that respond to tilt and shaking.
- **Physical control:** a two-position switch selects the animation mode.
- **Sleep and wake:** deep sleep after 10 seconds of inactivity; MPU6500 motion interrupt wakes the ESP32-C3.
- **Local animations:** no network connection required.

This version targets the working monochrome prototype. A custom PCB and color display are future development goals.

## Hardware

| Component / signal | Connection |
| --- | --- |
| Controller | ESP32-C3 SuperMini |
| Display | SH1106 128 x 64 OLED, I2C |
| Motion sensor | MPU6500, I2C address 0x68 or 0x69 |
| Shared I2C SDA | GPIO8 |
| Shared I2C SCL | GPIO9 |
| MPU6500 INT | GPIO3 |
| Mode switch | GPIO10 to GND; internal pull-up enabled |
| Switch LOW / HIGH | Liquid / buddy |
| Portable power | Li-Po with suitable charging and regulated supply |

Use a common ground and 3.3 V-compatible signal levels. Battery charging and supply circuitry are external to this firmware.

## Build and flash

1. Install the ESP32 board package by Espressif Systems in Arduino IDE.
2. Install **U8g2** by oliver through Library Manager.
3. Open [`firmware/mimo/mimo.ino`](firmware/mimo/mimo.ino).
4. Select your ESP32-C3 SuperMini board and its serial port, then upload.
5. Open Serial Monitor at **115200 baud** to check sensor initialization and mode selection.

The MPU6500 is configured directly over I2C; no separate sensor library is needed. Display orientation and motion thresholds can be adjusted in the sketch.

## Versioning

The first prototype is **v0.1.0**. The original local filename, `mimo_v8_face.ino`, was an iteration label, not a product release number. The imported sketch changes only the project version labels; animation and hardware behavior are preserved.

Hardware operation was reported working by the author. This repository import has not been independently compiled or tested on a device.