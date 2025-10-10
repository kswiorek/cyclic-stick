#include <Arduino.h>
#include <driver/uart.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <EEPROM.h>
#include <BleGamepad.h>

const uint8_t pollSignal[9] = {0xA5, 0x0B, 0x11, 0x98, 0x00, 0x00, 0x00, 0xE5, 0x20};
const uint8_t TX_PIN = 22;
const uint8_t RX_PIN = 23;

HardwareSerial gripSerial(2);

// Invert mask for 40 buttons
const uint8_t invertMask[40] = {1, 0, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1};
const uint8_t buttonIndices[] = {8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 24, 25, 26, 27, 28, 29, 30, 31, 36, 37, 39};
const size_t buttonCount = sizeof(buttonIndices) / sizeof(buttonIndices[0]);

#define MT6701_ADDR 0x06 // Default I2C address for MT6701
#define CALIBRATION_BACKOFF_DISTANCE 10
#define CALIBRATION_ENCODER1_MIN_MARGIN -100
#define CALIBRATION_ENCODER1_MAX_MARGIN -60
#define CALIBRATION_ENCODER2_MIN_MARGIN 0
#define CALIBRATION_ENCODER2_MAX_MARGIN 0

TwoWire Wire2 = TwoWire(1); // Create a second I2C instance for the new sensor

uint16_t readMT6701Angle(TwoWire &wire) {
  wire.beginTransmission(MT6701_ADDR);
  wire.write(0x03); // Angle register high byte
  wire.endTransmission(false);
  wire.requestFrom(MT6701_ADDR, 2);
  if (wire.available() == 2) {
    uint8_t high = wire.read();
    uint8_t low = wire.read();
    return ((high << 6) | (low >> 2)) & 0x3FFF; // 14-bit angle
  }
  return 0xFFFF; // Error value
}

const int SERVO1_PIN = 25;
const int SERVO2_PIN = 27;
Servo servo1;
Servo servo2;
volatile int servo1_target = 90;
volatile int servo2_target = 90;
SemaphoreHandle_t servoMutex;

// Servo movement flags
volatile bool servo1_dec = false;
volatile bool servo1_inc = false;
volatile bool servo2_dec = false;
volatile bool servo2_inc = false;

volatile bool followStickMode = false;

TaskHandle_t servoTaskHandle = NULL; // Add this global

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

CalibrationData calibration;

BleGamepad bleGamepad("Cyclic Stick", "Custom", 100);

// --- Servo control task running on core 0 ---
void servoTask(void *pvParameters) {
  unsigned long lastManualMove = 0;
  for (;;) {
    if (xSemaphoreTake(servoMutex, portMAX_DELAY) == pdTRUE) {
      if (followStickMode && calibration.isCalibrated) {
        // Follow stick mode: update servos every 1 ms
        uint16_t encoder1 = readMT6701Angle(Wire);
        uint16_t encoder2 = readMT6701Angle(Wire2);

        int servo1_range = calibration.servo1_max - calibration.servo1_min;
        int servo2_range = calibration.servo2_max - calibration.servo2_min;

        int servo1_pos = calibration.servo1_min;
        int servo2_pos = calibration.servo2_min;

        if (servo1_range > 0) {
          servo1_pos = calibration.servo1_min + ((long)(encoder1 - calibration.encoder1_min) * servo1_range) / (calibration.encoder1_max - calibration.encoder1_min);
          servo1_pos = constrain(servo1_pos, calibration.servo1_min, calibration.servo1_max);
        }
        if (servo2_range > 0) {
          servo2_pos = calibration.servo2_min + ((long)(encoder2 - calibration.encoder2_min) * servo2_range) / (calibration.encoder2_max - calibration.encoder2_min);
          servo2_pos = constrain(servo2_pos, calibration.servo2_min, calibration.servo2_max);
        }

        servo1.write(servo1_pos);
        servo2.write(servo2_pos);
        servo1_target = servo1_pos;
        servo2_target = servo2_pos;
      } else {
        // Manual movement: update every 20 ms
        unsigned long now = millis();
        if (now - lastManualMove >= 20) {
          lastManualMove = now;
          if (servo1_dec && servo1_target > 0) servo1_target--;
          if (servo1_inc && servo1_target < 180) servo1_target++;
          if (servo2_dec && servo2_target > 0) servo2_target--;
          if (servo2_inc && servo2_target < 180) servo2_target++;
          servo1.write(servo1_target);
          servo2.write(servo2_target);
        }
      }
      xSemaphoreGive(servoMutex);
    }
    vTaskDelay(1 / portTICK_PERIOD_MS); // Loop every 1 ms
  }
}

#define EEPROM_SIZE 64
#define EEPROM_CALIBRATION_ADDR 0

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

void setup() {
  Serial.begin(115200);
  Serial.println("Grip Serial Reader Started");
  gripSerial.begin(500000, SERIAL_8N1, RX_PIN, TX_PIN);
  delay(1000);

  Wire.begin(16, 17);      // First sensor on pins 16 (SDA), 17 (SCL)
  Wire2.begin(4, 32);      // Second sensor on pins 4 (SDA), 32 (SCL)

  // Servo setup
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo1.write(servo1_target);
  servo2.write(servo2_target);

  servoMutex = xSemaphoreCreateMutex();

  // Start servo control task on core 0
  xTaskCreatePinnedToCore(servoTask, "ServoTask", 2048, NULL, 1, &servoTaskHandle, 0); // Save handle

  // Load calibration from EEPROM
  loadCalibrationFromEEPROM();
  printCalibrationData();

  // BLE Gamepad setup
  BleGamepadConfiguration bleGamepadConfig;
  bleGamepadConfig.setAutoReport(false);
  bleGamepadConfig.setControllerType(CONTROLLER_TYPE_JOYSTICK);
  bleGamepadConfig.setButtonCount(buttonCount);
  bleGamepadConfig.setHatSwitchCount(0);
  bleGamepad.begin(&bleGamepadConfig);
  Serial.println("BLE Gamepad initialized");
}

// Decoding functions ported from Python
uint16_t decodeMSBY(uint8_t byte1) {
    uint8_t ret = byte1 ^ 0x89;
    ret = (ret & 0x80) | ((ret & 0x0F) << 3);
    return ret << 4;
}

uint16_t decodeMSBX(uint8_t byte1) {
    uint8_t ret = byte1 ^ 0x8E;
    ret = (ret & 0x80) | ((ret & 0x0F) << 3);
    return ret << 4;
}

uint8_t decodeLSBY(uint8_t byte0) {
    uint8_t ret = (byte0 ^ 0xD8);
    ret = (ret & 0xFE) >> 1;
    return ret;
}

uint8_t decodeLSBX(uint8_t byte0) {
    uint8_t ret = (byte0 ^ 0xC8);
    ret = (ret & 0xFE) >> 1;
    return ret;
}

struct GripData {
  uint8_t buttons[buttonCount];
  uint16_t ministick_x;
  uint16_t ministick_y;
};

bool readGripData(GripData &data) {
  gripSerial.write(pollSignal, sizeof(pollSignal));
  gripSerial.flush();

  uart_set_pin(2, UART_PIN_NO_CHANGE, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  pinMode(TX_PIN, INPUT);

  uint8_t rxBuffer[64];
  size_t rxCount = 0;

  unsigned long startTime = millis();
  while (millis() - startTime < 3) {
    while (gripSerial.available() && rxCount < sizeof(rxBuffer)) {
      rxBuffer[rxCount++] = gripSerial.read();
    }
  }

  uart_set_pin(2, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

  if (rxCount > 37) {
    // Decode all 40 buttons
    uint8_t buttonStates[40] = {0};
    for (int byteIdx = 33; byteIdx < 38 && byteIdx < rxCount; byteIdx++) {
      for (int bitIdx = 0; bitIdx < 8; bitIdx++) {
        int buttonIdx = (byteIdx - 33) * 8 + bitIdx;
        if (buttonIdx < 40) {
          uint8_t bitVal = (rxBuffer[byteIdx] >> (7 - bitIdx)) & 1;
          buttonStates[buttonIdx] = bitVal ^ invertMask[buttonIdx];
        }
      }
    }
    // Fill only usable buttons
    for (size_t i = 0; i < buttonCount; i++) {
      data.buttons[i] = buttonStates[buttonIndices[i]];
    }

    // Decode ministick
    uint16_t lsb_x = decodeLSBX(rxBuffer[21]);
    uint16_t msb_x = decodeMSBX(rxBuffer[22]);
    data.ministick_x = msb_x | lsb_x;

    uint16_t lsb_y = decodeLSBY(rxBuffer[23]);
    uint16_t msb_y = decodeMSBY(rxBuffer[24]);
    data.ministick_y = msb_y | lsb_y;

    return true;
  }
  return false;
}

int mapEncoderToRange(uint16_t value, uint16_t min, uint16_t max) {
  if (max == min) return 0;
  long mapped = ((long)(value - min) * 4095) / (max - min) - 2048;
  if (mapped < -2048) mapped = -2048;
  if (mapped > 2047) mapped = 2047;
  return (int)mapped;
}

// --- Calibration function ---
void performCalibration() {
  Serial.println("=== Starting Calibration ===");

  // Suspend only the servo task
  if (servoTaskHandle) vTaskSuspend(servoTaskHandle);

  // === SERVO 1 (X-axis) ===
  Serial.println("Calibrating Servo 1 (X-axis)...");

  // Move to minimum position
  Serial.println("Moving to minimum...");
  servo1.write(0);
  delay(2000);
  uint16_t extreme_encoder1_min = readMT6701Angle(Wire);

  // SERVO 1 min backoff
  for (int pos = 0; pos <= 180; pos++) {
    servo1.write(pos);
    delay(50);
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
  servo1.write(180);
  delay(2000);
  uint16_t extreme_encoder1_max = readMT6701Angle(Wire);

  // SERVO 1 max backoff
  for (int pos = 180; pos >= 0; pos--) {
    servo1.write(pos);
    delay(50);
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
  servo2.write(0);
  delay(2000);
  uint16_t extreme_encoder2_min = readMT6701Angle(Wire2);

  // SERVO 2 min backoff
  for (int pos = 0; pos <= 180; pos++) {
    servo2.write(pos);
    delay(50);
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
  servo2.write(180);
  delay(2000);
  uint16_t extreme_encoder2_max = readMT6701Angle(Wire2);

  // SERVO 2 max backoff
  for (int pos = 180; pos >= 0; pos--) {
    servo2.write(pos);
    delay(50);
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

  servo1.write(servo1_center);
  servo2.write(servo2_center);
  servo1_target = servo1_center;
  servo2_target = servo2_center;

  delay(2000);

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

  // Resume the servo task
  if (servoTaskHandle) vTaskResume(servoTaskHandle);
}

void followStickWithServos() {
  // Read encoder positions
  uint16_t encoder1 = readMT6701Angle(Wire);
  uint16_t encoder2 = readMT6701Angle(Wire2);

  // Map encoder values to servo range
  int servo1_range = calibration.servo1_max - calibration.servo1_min;
  int servo2_range = calibration.servo2_max - calibration.servo2_min;

  int servo1_pos = calibration.servo1_min;
  int servo2_pos = calibration.servo2_min;

  if (servo1_range > 0) {
    servo1_pos = calibration.servo1_min + ((long)(encoder1 - calibration.encoder1_min) * servo1_range) / (calibration.encoder1_max - calibration.encoder1_min);
    servo1_pos = constrain(servo1_pos, calibration.servo1_min, calibration.servo1_max);
  }
  if (servo2_range > 0) {
    servo2_pos = calibration.servo2_min + ((long)(encoder2 - calibration.encoder2_min) * servo2_range) / (calibration.encoder2_max - calibration.encoder2_min);
    servo2_pos = constrain(servo2_pos, calibration.servo2_min, calibration.servo2_max);
  }

  // Set servo targets
  servo1_target = servo1_pos;
  servo2_target = servo2_pos;
}

int calibrationWaitCounter = 0;
const int calibrationWaitThreshold = 100; // ~0.5 seconds at 5ms per loop

void loop() {
  GripData grip;
  if (readGripData(grip)) {
    Serial.print("Buttons: ");
    for (size_t i = 0; i < buttonCount; i++) {
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
    // -------------------------------------

    // Wait a few iterations before calibration can be started
    if (calibrationWaitCounter < calibrationWaitThreshold) {
      calibrationWaitCounter++;
    } else {
      // Check for calibration trigger (buttons 1 AND 22 pressed)
      if (grip.buttons[1] && grip.buttons[0]) {
        performCalibration();
        calibrationWaitCounter = 0; // Reset after calibration
      }
    }

    // Set flags for servo movement
    servo1_inc = (grip.buttons[16]);
    servo1_dec = (grip.buttons[17]);
    servo2_inc = (grip.buttons[18]);
    servo2_dec = (grip.buttons[19]);

    followStickMode = grip.buttons[22] && calibration.isCalibrated;
  }
  delay(5);
}