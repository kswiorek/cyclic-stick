# Cyclic Stick

This project is about building a cyclic stick with adjustable trim using servos.

## Overview

- Uses servos for trim adjustment
- Reads buttons and ministick from a Kosmosima grip
- ESP32 acts as the base and can forward grip data over serial
- Plans for Bluetooth joystick support

## Kosmosima Grip

The Kosmosima grip is connected directly to an Arduino or ESP32, so you don't need to use the original base or extend the cable. This makes wiring easier and keeps all the grip buttons available.

## Communication

- The grip uses a single-wire UART protocol at 500,000 bps (two stop bits).
- The base sends a polling message, and the grip replies with data (button states and ministick values).
- A 330 Ohm resistor on TX helps avoid signal conflicts.

## Status

- The ESP32 code currently just forwards grip data to serial.
- Ministick values are decoded with some bit manipulation.
- A Python script is used for testing.

## Next Steps

- Add Bluetooth joystick support to the ESP32
- Look into controlling the grip LEDs
- Add wiring diagrams and more documentation

---
See the code for details.
