#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <Arduino.h>
#include "config.h"

// Calibration data structure
struct CalibrationData {
  int servo1_min;
  int servo1_max;
  uint16_t encoder1_min;
  uint16_t encoder1_max;
  int servo2_min;
  int servo2_max;
  uint16_t encoder2_min;
  uint16_t encoder2_max;
  bool isCalibrated = false;
};

// External reference to calibration data
extern CalibrationData calibration;

// Initialize EEPROM and load calibration
void initCalibration();

// Save calibration data to EEPROM
void saveCalibrationToEEPROM();

// Load calibration data from EEPROM
void loadCalibrationFromEEPROM();

// Print calibration data to serial
void printCalibrationData();

// Perform full calibration sequence
void performCalibration();

#endif // CALIBRATION_H
