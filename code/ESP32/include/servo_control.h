#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

// Includes
#include <Arduino.h>
#include <ESP32Servo.h>
#include <EEPROM.h>
#include "config.h"

// Defines (if any needed)

// Manual servo enable persistence
void saveManualServoEnabledToEEPROM();
void loadManualServoEnabledFromEEPROM();

// Follow stick mode persistence
void saveFollowStickEnabledToEEPROM();
void loadFollowStickEnabledFromEEPROM();

// Servo initialization
void initServos();

// Servo task management
void startServoTask();
void suspendServoTask();
void resumeServoTask();

// Servo movement and mode
void setServoMovementFlags(bool s1_inc, bool s1_dec, bool s2_inc, bool s2_dec);
void setFollowStickMode(bool enabled);
void setServoTargets(int servo1_pos, int servo2_pos);
void writeServo1(int position);
void writeServo2(int position);

// Reset servos to center position
void resetServosToCenter();

// Servo control task
void servoTask(void *pvParameters);

#endif // SERVO_CONTROL_H
