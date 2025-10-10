#include "encoder_handler.h"

TwoWire Wire2 = TwoWire(1); // Create a second I2C instance for the new sensor

void initEncoders() {
  Wire.begin(I2C1_SDA_PIN, I2C1_SCL_PIN);      // First sensor
  Wire2.begin(I2C2_SDA_PIN, I2C2_SCL_PIN);     // Second sensor
}

uint16_t readMT6701Angle(TwoWire &wire) {
  wire.beginTransmission(MT6701_ADDR);
  wire.write(MT6701_ANGLE_REG); // Angle register high byte
  wire.endTransmission(false);
  wire.requestFrom(MT6701_ADDR, 2);
  if (wire.available() == 2) {
    uint8_t high = wire.read();
    uint8_t low = wire.read();
    return ((high << 6) | (low >> 2)) & ENCODER_MAX_VALUE; // 14-bit angle
  }
  return ENCODER_ERROR_VALUE; // Error value
}

int mapEncoderToRange(uint16_t value, uint16_t min, uint16_t max) {
  if (max == min) return 0;
  long mapped = ((long)(value - min) * AXIS_RANGE) / (max - min) + AXIS_MIN;
  if (mapped < AXIS_MIN) mapped = AXIS_MIN;
  if (mapped > AXIS_MAX) mapped = AXIS_MAX;
  return (int)mapped;
}

int mapMinistickToRange(uint16_t value, uint16_t min, uint16_t max) {
  if (max == min) return 0;
  long mapped = ((long)(value - min) * AXIS_RANGE) / (max - min) + AXIS_MIN;
  if (mapped < AXIS_MIN) mapped = AXIS_MIN;
  if (mapped > AXIS_MAX) mapped = AXIS_MAX;
  return (int)mapped;
}
