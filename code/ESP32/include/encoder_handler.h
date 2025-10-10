#ifndef ENCODER_HANDLER_H
#define ENCODER_HANDLER_H

#include <Arduino.h>
#include <Wire.h>
#include "config.h"

// Initialize I2C buses for encoders
void initEncoders();

// Read angle from MT6701 encoder
uint16_t readMT6701Angle(TwoWire &wire);

// Map encoder value to gamepad range (-2048 to 2047)
int mapEncoderToRange(uint16_t value, uint16_t min, uint16_t max);

// Map ministick value to gamepad range (-2048 to 2047)
int mapMinistickToRange(uint16_t value, uint16_t min, uint16_t max);

// External references to I2C buses
extern TwoWire Wire2;

#endif // ENCODER_HANDLER_H
