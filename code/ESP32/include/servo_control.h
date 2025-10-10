#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <Arduino.h>
#include <ESP32Servo.h>
#include "config.h"

// Initialize servos
void initServos();

// Start servo control task
void startServoTask();

// Suspend servo task (for calibration)
void suspendServoTask();

// Resume servo task (after calibration)
void resumeServoTask();

// Set servo movement flags
void setServoMovementFlags(bool s1_inc, bool s1_dec, bool s2_inc, bool s2_dec);

// Set follow stick mode
void setFollowStickMode(bool enabled);

// Set servo targets
void setServoTargets(int servo1_pos, int servo2_pos);

// Direct servo write functions (for calibration)
void writeServo1(int position);
void writeServo2(int position);

// Servo task function
void servoTask(void *pvParameters);

#endif // SERVO_CONTROL_H
