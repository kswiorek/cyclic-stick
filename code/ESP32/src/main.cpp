#include <Arduino.h>
#include <BleGamepad.h>
#include "config.h"
#include "grip_handler.h"
#include "encoder_handler.h"
#include "calibration.h"

#include <EEPROM.h>
#include "servo_control.h"

BleGamepad bleGamepad(GAMEPAD_NAME, GAMEPAD_MANUFACTURER, GAMEPAD_BATTERY_LEVEL);
int calibrationWaitCounter = 0;
bool manualServoEnabled = true;
bool followStickEnabled = true;

void setup() {
  Serial.begin(DEBUG_SERIAL_BAUD);
  Serial.println("Grip Serial Reader Started");
  
  // Initialize all subsystems
  initGripSerial();
  initEncoders();
  initServos();
  initCalibration();
  loadManualServoEnabledFromEEPROM();
  loadFollowStickEnabledFromEEPROM();

  // Start servo control task on core 0
  startServoTask();

  // BLE Gamepad setup
  BleGamepadConfiguration bleGamepadConfig;
  bleGamepadConfig.setAutoReport(false);
  bleGamepadConfig.setControllerType(CONTROLLER_TYPE_JOYSTICK);
  bleGamepadConfig.setButtonCount(BUTTON_COUNT);
  bleGamepadConfig.setHatSwitchCount(0);
  bleGamepadConfig.setAxesMin(AXIS_MIN);
  bleGamepadConfig.setAxesMax(AXIS_MAX);
  bleGamepadConfig.setWhichAxes(true, true, false, false, false, false, true, true); // Enable only X, Y, Slider1, Slider2
  bleGamepad.begin(&bleGamepadConfig);
  Serial.print("BLE Gamepad initialized. Manual servo enabled: ");
  Serial.println(manualServoEnabled ? "true" : "false");
}



void loop() {
  GripData grip;
  if (readGripData(grip)) {
    Serial.print("Buttons: ");
    for (size_t i = 0; i < BUTTON_COUNT; i++) {
      Serial.print(grip.buttons[i]);
      Serial.print(" ");
    }
    Serial.print(" | MINISTICK_X: ");
    Serial.print(grip.ministick_x);
    Serial.print(" MINISTICK_Y: ");
    Serial.print(grip.ministick_y);

    // --- Read and print encoder angles ---
    uint16_t mt6701_angle_1 = readMT6701Angle(Wire);
    uint16_t mt6701_angle_2 = readMT6701Angle(Wire2);

    int mapped_x = mapEncoderToRange(mt6701_angle_1, calibration.encoder1_min, calibration.encoder1_max);
    int mapped_y = mapEncoderToRange(mt6701_angle_2, calibration.encoder2_min, calibration.encoder2_max);

    Serial.print(" | X: ");
    Serial.print(mapped_x);
    Serial.print(" Y: ");
    Serial.print(mapped_y);
    Serial.print(" | RAW_ENCODER1: ");
    Serial.print(mt6701_angle_1);
    Serial.print(" RAW_ENCODER2: ");
    Serial.println(mt6701_angle_2);

    // Wait a few iterations before calibration can be started
    if (calibrationWaitCounter < CALIBRATION_WAIT_THRESHOLD) {
      calibrationWaitCounter++;
    } else {
      // Check for calibration trigger (buttons 1 AND 2 pressed)
      if (grip.buttons[BUTTON_CALIBRATE_1] && grip.buttons[BUTTON_CALIBRATE_2]) {
        performCalibration();
        calibrationWaitCounter = 0; // Reset after calibration
      }
    }


    static bool lastManualServoEnabled = true;
    static bool lastFollowStickEnabled = true;
    // Disable manual servo movement on buttons 1 and 10
    if (grip.buttons[BUTTON_MODIFIER] && grip.buttons[BUTTON_MANUAL_DISABLE]) {
      manualServoEnabled = false;
    }
    // Enable manual servo movement on buttons 1 and 11
    if (grip.buttons[BUTTON_MODIFIER] && grip.buttons[BUTTON_MANUAL_ENABLE]) {
      manualServoEnabled = true;
    }
    // Disable follow stick mode on buttons 1 and 12
    if (grip.buttons[BUTTON_MODIFIER] && grip.buttons[BUTTON_FOLLOW_DISABLE]) {
      followStickEnabled = false;
    }
    // Enable follow stick mode on buttons 1 and 13
    if (grip.buttons[BUTTON_MODIFIER] && grip.buttons[BUTTON_FOLLOW_ENABLE]) {
      followStickEnabled = true;
    }

    if (manualServoEnabled != lastManualServoEnabled) {
      saveManualServoEnabledToEEPROM();
      lastManualServoEnabled = manualServoEnabled;
    }
    if (followStickEnabled != lastFollowStickEnabled) {
      saveFollowStickEnabledToEEPROM();
      lastFollowStickEnabled = followStickEnabled;
    }

    if (manualServoEnabled) {
      setServoMovementFlags(
        grip.buttons[BUTTON_SERVO1_INC],
        grip.buttons[BUTTON_SERVO1_DEC],
        grip.buttons[BUTTON_SERVO2_INC],
        grip.buttons[BUTTON_SERVO2_DEC]
      );
    } else {
      setServoMovementFlags(false, false, false, false);
    }

    if(grip.buttons[BUTTON_FOLLOW_STICK] && grip.buttons[BUTTON_MODIFIER]) {
      resetServosToCenter();
    }

    // Set follow stick mode
    setFollowStickMode(grip.buttons[BUTTON_FOLLOW_STICK] && calibration.isCalibrated && !grip.buttons[BUTTON_MODIFIER] && followStickEnabled);

    // Set main X and Y axes (mapped_x, mapped_y)
    // Flip axes for Bluetooth output only
    int ble_x = FLIP_X_DIRECTION ? 1-mapped_x : mapped_x;
    int ble_y = FLIP_Y_DIRECTION ? 1-mapped_y : mapped_y;
    bleGamepad.setX(ble_x);
    bleGamepad.setY(ble_y);

    // Map ministick values
    int mapped_ministick_x = mapMinistickToRange(grip.ministick_x, MINISTICK_X_MAX, MINISTICK_X_MIN);
    int mapped_ministick_y = mapMinistickToRange(grip.ministick_y, MINISTICK_Y_MAX, MINISTICK_Y_MIN);

    // Set ministick axes
    bleGamepad.setSlider(mapped_ministick_x);
    bleGamepad.setSlider2(mapped_ministick_y);

    // Set button states
    for (size_t i = 0; i < BUTTON_COUNT; i++) {
      if (grip.buttons[i]) {
        bleGamepad.press(i + 1); // Button numbering starts at 1
      } else {
        bleGamepad.release(i + 1);
      }
    }

    // Send the report via Bluetooth
    bleGamepad.sendReport();
  }
  delay(MAIN_LOOP_DELAY_MS);
}
