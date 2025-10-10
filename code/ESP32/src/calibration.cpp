#include "calibration.h"
#include "encoder_handler.h"
#include "servo_control.h"
#include <EEPROM.h>

CalibrationData calibration;

void initCalibration() {
  loadCalibrationFromEEPROM();
  printCalibrationData();
}

void saveCalibrationToEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.put(EEPROM_CALIBRATION_ADDR, calibration);
  EEPROM.commit();
  EEPROM.end();
  Serial.println("Calibration data saved to EEPROM.");
}

void loadCalibrationFromEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(EEPROM_CALIBRATION_ADDR, calibration);
  EEPROM.end();
  if (calibration.isCalibrated) {
    Serial.println("Calibration data loaded from EEPROM.");
  } else {
    Serial.println("No valid calibration data in EEPROM.");
  }
}

void printCalibrationData() {
  Serial.println("=== Calibration Data ===");
  Serial.print("Servo1 min: "); Serial.print(calibration.servo1_min);
  Serial.print(" | Servo1 max: "); Serial.print(calibration.servo1_max);
  Serial.print(" | Encoder1 min: "); Serial.print(calibration.encoder1_min);
  Serial.print(" | Encoder1 max: "); Serial.println(calibration.encoder1_max);

  Serial.print("Servo2 min: "); Serial.print(calibration.servo2_min);
  Serial.print(" | Servo2 max: "); Serial.print(calibration.servo2_max);
  Serial.print(" | Encoder2 min: "); Serial.print(calibration.encoder2_min);
  Serial.print(" | Encoder2 max: "); Serial.println(calibration.encoder2_max);

  Serial.print("Calibrated: "); Serial.println(calibration.isCalibrated ? "Yes" : "No");
  Serial.println("=======================");
}

void performCalibration() {
  Serial.println("=== Starting Calibration ===");

  // Suspend servo task
  suspendServoTask();

  // === SERVO 1 (X-axis) ===
  Serial.println("Calibrating Servo 1 (X-axis)...");

  // Move to minimum position
  Serial.println("Moving to minimum...");
#if FLIP_SERVO1_DIRECTION
  writeServo1(SERVO_MAX_POSITION);
#else
  writeServo1(SERVO_MIN_POSITION);
#endif
  delay(CALIBRATION_DELAY_MS);
  uint16_t extreme_encoder1_min = readMT6701Angle(Wire);

  // SERVO 1 min backoff
#if FLIP_SERVO1_DIRECTION
  for (int pos = SERVO_MAX_POSITION; pos >= SERVO_MIN_POSITION; pos--) {
#else
  for (int pos = SERVO_MIN_POSITION; pos <= SERVO_MAX_POSITION; pos++) {
#endif
    writeServo1(pos);
    delay(CALIBRATION_STEP_DELAY_MS);
    uint16_t current_encoder1 = readMT6701Angle(Wire);

    Serial.print("[Servo1 min backoff] Servo: ");
    Serial.print(pos);
    Serial.print(" | Encoder: ");
    Serial.println(current_encoder1);

    if (abs((int)current_encoder1 - (int)extreme_encoder1_min) > CALIBRATION_BACKOFF_DISTANCE) {
      calibration.servo1_min = pos;
      calibration.encoder1_min = current_encoder1;
      calibration.encoder1_min += CALIBRATION_ENCODER1_MIN_MARGIN;
      Serial.print("Servo1 min: ");
      Serial.print(calibration.servo1_min);
      Serial.print(" | Encoder1 min: ");
      Serial.println(calibration.encoder1_min);
      break;
    }
  }
  
  // Move to maximum position
  Serial.println("Moving to maximum...");
#if FLIP_SERVO1_DIRECTION
  writeServo1(SERVO_MIN_POSITION);
#else
  writeServo1(SERVO_MAX_POSITION);
#endif
  delay(CALIBRATION_DELAY_MS);
  uint16_t extreme_encoder1_max = readMT6701Angle(Wire);

  // SERVO 1 max backoff
#if FLIP_SERVO1_DIRECTION
  for (int pos = SERVO_MIN_POSITION; pos <= SERVO_MAX_POSITION; pos++) {
#else
  for (int pos = SERVO_MAX_POSITION; pos >= SERVO_MIN_POSITION; pos--) {
#endif
    writeServo1(pos);
    delay(CALIBRATION_STEP_DELAY_MS);
    uint16_t current_encoder1 = readMT6701Angle(Wire);

    Serial.print("[Servo1 max backoff] Servo: ");
    Serial.print(pos);
    Serial.print(" | Encoder: ");
    Serial.println(current_encoder1);

    if (abs((int)current_encoder1 - (int)extreme_encoder1_max) > CALIBRATION_BACKOFF_DISTANCE) {
      calibration.servo1_max = pos;
      calibration.encoder1_max = current_encoder1;
      calibration.encoder1_max -= CALIBRATION_ENCODER1_MAX_MARGIN;
      Serial.print("Servo1 max: ");
      Serial.print(calibration.servo1_max);
      Serial.print(" | Encoder1 max: ");
      Serial.println(calibration.encoder1_max);
      break;
    }
  }

  // === SERVO 2 (Y-axis) ===
  Serial.println("Calibrating Servo 2 (Y-axis)...");

  // Move to minimum position
  Serial.println("Moving to minimum...");
#if FLIP_SERVO2_DIRECTION
  writeServo2(SERVO_MAX_POSITION);
#else
  writeServo2(SERVO_MIN_POSITION);
#endif
  delay(CALIBRATION_DELAY_MS);
  uint16_t extreme_encoder2_min = readMT6701Angle(Wire2);

  // SERVO 2 min backoff
#if FLIP_SERVO2_DIRECTION
  for (int pos = SERVO_MAX_POSITION; pos >= SERVO_MIN_POSITION; pos--) {
#else
  for (int pos = SERVO_MIN_POSITION; pos <= SERVO_MAX_POSITION; pos++) {
#endif
    writeServo2(pos);
    delay(CALIBRATION_STEP_DELAY_MS);
    uint16_t current_encoder2 = readMT6701Angle(Wire2);

    Serial.print("[Servo2 min backoff] Servo: ");
    Serial.print(pos);
    Serial.print(" | Encoder: ");
    Serial.println(current_encoder2);

    if (abs((int)current_encoder2 - (int)extreme_encoder2_min) > CALIBRATION_BACKOFF_DISTANCE) {
      calibration.servo2_min = pos;
      calibration.encoder2_min = current_encoder2;
      calibration.encoder2_min += CALIBRATION_ENCODER2_MIN_MARGIN;
      Serial.print("Servo2 min: ");
      Serial.print(calibration.servo2_min);
      Serial.print(" | Encoder2 min: ");
      Serial.println(calibration.encoder2_min);
      break;
    }
  }

  // Move to maximum position
  Serial.println("Moving to maximum...");
#if FLIP_SERVO2_DIRECTION
  writeServo2(SERVO_MIN_POSITION);
#else
  writeServo2(SERVO_MAX_POSITION);
#endif
  delay(CALIBRATION_DELAY_MS);
  uint16_t extreme_encoder2_max = readMT6701Angle(Wire2);

  // SERVO 2 max backoff
#if FLIP_SERVO2_DIRECTION
  for (int pos = SERVO_MIN_POSITION; pos <= SERVO_MAX_POSITION; pos++) {
#else
  for (int pos = SERVO_MAX_POSITION; pos >= SERVO_MIN_POSITION; pos--) {
#endif
    writeServo2(pos);
    delay(CALIBRATION_STEP_DELAY_MS);
    uint16_t current_encoder2 = readMT6701Angle(Wire2);

    Serial.print("[Servo2 max backoff] Servo: ");
    Serial.print(pos);
    Serial.print(" | Encoder: ");
    Serial.println(current_encoder2);

    if (abs((int)current_encoder2 - (int)extreme_encoder2_max) > CALIBRATION_BACKOFF_DISTANCE) {
      calibration.servo2_max = pos;
      calibration.encoder2_max = current_encoder2;
      calibration.encoder2_max -= CALIBRATION_ENCODER2_MAX_MARGIN;
      Serial.print("Servo2 max: ");
      Serial.print(calibration.servo2_max);
      Serial.print(" | Encoder2 max: ");
      Serial.println(calibration.encoder2_max);
      break;
    }
  }

  // === CENTER SERVOS ===
  Serial.println("Centering servos...");
  int servo1_center = (calibration.servo1_min + calibration.servo1_max) / 2;
  int servo2_center = (calibration.servo2_min + calibration.servo2_max) / 2;

  setServoTargets(servo1_center, servo2_center);
  writeServo1(servo1_center);
  writeServo2(servo2_center);

  delay(CALIBRATION_DELAY_MS);

  // Verify encoder positions
  uint16_t encoder1_center = readMT6701Angle(Wire);
  uint16_t encoder2_center = readMT6701Angle(Wire2);
  uint16_t encoder1_expected = (calibration.encoder1_min + calibration.encoder1_max) / 2;
  uint16_t encoder2_expected = (calibration.encoder2_min + calibration.encoder2_max) / 2;

  Serial.println("=== Calibration Complete ===");
  Serial.print("Servo1 center: ");
  Serial.print(servo1_center);
  Serial.print(" | Encoder1: ");
  Serial.print(encoder1_center);
  Serial.print(" (expected ~");
  Serial.print(encoder1_expected);
  Serial.println(")");

  Serial.print("Servo2 center: ");
  Serial.print(servo2_center);
  Serial.print(" | Encoder2: ");
  Serial.print(encoder2_center);
  Serial.print(" (expected ~");
  Serial.print(encoder2_expected);
  Serial.println(")");

  calibration.isCalibrated = true;

  // --- Apply proportional servo margins based on encoder margins ---
  int encoder1_range = calibration.encoder1_max - calibration.encoder1_min;
  int servo1_range   = calibration.servo1_max   - calibration.servo1_min;
  int encoder2_range = calibration.encoder2_max - calibration.encoder2_min;
  int servo2_range   = calibration.servo2_max   - calibration.servo2_min;

  int servo1_min_margin = (int)((float)(CALIBRATION_ENCODER1_MIN_MARGIN) / encoder1_range * servo1_range);
  int servo1_max_margin = (int)((float)(CALIBRATION_ENCODER1_MAX_MARGIN) / encoder1_range * servo1_range);
  int servo2_min_margin = (int)((float)(CALIBRATION_ENCODER2_MIN_MARGIN) / encoder2_range * servo2_range);
  int servo2_max_margin = (int)((float)(CALIBRATION_ENCODER2_MAX_MARGIN) / encoder2_range * servo2_range);

  calibration.servo1_min += servo1_min_margin;
  calibration.servo1_max -= servo1_max_margin;
  calibration.servo2_min += servo2_min_margin;
  calibration.servo2_max -= servo2_max_margin;

  // Save calibration to EEPROM
  saveCalibrationToEEPROM();

  // Resume servo task
  resumeServoTask();
}
