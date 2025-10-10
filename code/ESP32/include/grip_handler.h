#ifndef GRIP_HANDLER_H
#define GRIP_HANDLER_H

#include <Arduino.h>
#include "config.h"

struct GripData {
  uint8_t buttons[BUTTON_COUNT];
  uint16_t ministick_x;
  uint16_t ministick_y;
};

// Initialize grip serial communication
void initGripSerial();

// Read grip data from serial
bool readGripData(GripData &data);

// Decoding functions
uint16_t decodeMSBY(uint8_t byte1);
uint16_t decodeMSBX(uint8_t byte1);
uint8_t decodeLSBY(uint8_t byte0);
uint8_t decodeLSBX(uint8_t byte0);

#endif // GRIP_HANDLER_H
