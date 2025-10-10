#include "servo_control.h"
#include "encoder_handler.h"
#include "calibration.h"
#include <EEPROM.h>
#include "config.h"

// Defines
#define EEPROM_MANUAL_SERVO_ADDR (EEPROM_CALIBRATION_ADDR + sizeof(CalibrationData))
#define EEPROM_FOLLOW_STICK_ADDR (EEPROM_MANUAL_SERVO_ADDR + sizeof(bool))

// Externs
extern bool manualServoEnabled;
extern bool followStickEnabled;

// Servo objects and state
Servo servo1;
Servo servo2;
volatile int servo1_target = SERVO_INITIAL_POSITION;
volatile int servo2_target = SERVO_INITIAL_POSITION;
SemaphoreHandle_t servoMutex;

// Servo movement flags
volatile bool servo1_dec = false;
volatile bool servo1_inc = false;
volatile bool servo2_dec = false;
volatile bool servo2_inc = false;

volatile bool followStickMode = false;
TaskHandle_t servoTaskHandle = NULL;

// EEPROM persistence for manual servo enable
void saveManualServoEnabledToEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.put(EEPROM_MANUAL_SERVO_ADDR, manualServoEnabled);
  EEPROM.commit();
  EEPROM.end();
}

void loadManualServoEnabledFromEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(EEPROM_MANUAL_SERVO_ADDR, manualServoEnabled);
  EEPROM.end();
}

void saveFollowStickEnabledToEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.put(EEPROM_FOLLOW_STICK_ADDR, followStickEnabled);
  EEPROM.commit();
  EEPROM.end();
}

void loadFollowStickEnabledFromEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(EEPROM_FOLLOW_STICK_ADDR, followStickEnabled);
  EEPROM.end();
}

// Servo initialization
void initServos() {
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo1.write(servo1_target);
  servo2.write(servo2_target);
  
  servoMutex = xSemaphoreCreateMutex();
}

// Servo task management
void startServoTask() {
  xTaskCreatePinnedToCore(servoTask, "ServoTask", SERVO_TASK_STACK_SIZE, NULL, SERVO_TASK_PRIORITY, &servoTaskHandle, SERVO_TASK_CORE);
}

void suspendServoTask() {
  if (servoTaskHandle) vTaskSuspend(servoTaskHandle);
}

void resumeServoTask() {
  if (servoTaskHandle) vTaskResume(servoTaskHandle);
}

// Servo movement and mode
void setServoMovementFlags(bool s1_inc, bool s1_dec, bool s2_inc, bool s2_dec) {
  servo1_inc = s1_inc;
  servo1_dec = s1_dec;
  servo2_inc = s2_inc;
  servo2_dec = s2_dec;
}

void setFollowStickMode(bool enabled) {
  followStickMode = enabled;
}

void setServoTargets(int servo1_pos, int servo2_pos) {
  servo1_target = servo1_pos;
  servo2_target = servo2_pos;
}

void writeServo1(int position) {
  servo1.write(position);
}

void writeServo2(int position) {
  servo2.write(position);
}

// Reset servos to center position
void resetServosToCenter() {
  int servo1_center = (calibration.servo1_min + calibration.servo1_max) / 2;
  int servo2_center = (calibration.servo2_min + calibration.servo2_max) / 2;
  setServoTargets(servo1_center, servo2_center);
  writeServo1(servo1_center);
  writeServo2(servo2_center);
}

// Servo control task
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
        if (now - lastManualMove >= MANUAL_SERVO_UPDATE_MS) {
          lastManualMove = now;
          if (servo1_dec && servo1_target > SERVO_MIN_POSITION) servo1_target--;
          if (servo1_inc && servo1_target < SERVO_MAX_POSITION) servo1_target++;
          if (servo2_dec && servo2_target > SERVO_MIN_POSITION) servo2_target--;
          if (servo2_inc && servo2_target < SERVO_MAX_POSITION) servo2_target++;
          servo1.write(servo1_target);
          servo2.write(servo2_target);
        }
      }
      xSemaphoreGive(servoMutex);
    }
    vTaskDelay(SERVO_TASK_DELAY_MS / portTICK_PERIOD_MS); // Loop every 1 ms
  }
}
