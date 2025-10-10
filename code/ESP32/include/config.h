#ifndef CONFIG_H
#define CONFIG_H

// ==================== PIN DEFINITIONS ====================
#define TX_PIN 22
#define RX_PIN 23
#define SERVO1_PIN 25
#define SERVO2_PIN 27

// I2C Pins for first sensor
#define I2C1_SDA_PIN 16
#define I2C1_SCL_PIN 17

// I2C Pins for second sensor
#define I2C2_SDA_PIN 4
#define I2C2_SCL_PIN 32

// ==================== SERIAL CONFIGURATION ====================
#define GRIP_SERIAL_BAUD 500000
#define DEBUG_SERIAL_BAUD 115200

// ==================== SERVO CONFIGURATION ====================
#define FLIP_X_DIRECTION 0  // Set to 1 to flip direction
#define FLIP_Y_DIRECTION 1  // Set to 1 to flip direction
#define SERVO_INITIAL_POSITION 90
#define SERVO_MIN_POSITION 0
#define SERVO_MAX_POSITION 180

// ==================== ENCODER CONFIGURATION ====================
#define MT6701_ADDR 0x06 // Default I2C address for MT6701
#define MT6701_ANGLE_REG 0x03 // Angle register high byte
#define ENCODER_BITS 14
#define ENCODER_MAX_VALUE 0x3FFF // 14-bit angle
#define ENCODER_ERROR_VALUE 0xFFFF

// ==================== CALIBRATION PARAMETERS ====================
#define CALIBRATION_BACKOFF_DISTANCE 10
#define CALIBRATION_ENCODER1_MIN_MARGIN -100
#define CALIBRATION_ENCODER1_MAX_MARGIN -60
#define CALIBRATION_ENCODER2_MIN_MARGIN 0
#define CALIBRATION_ENCODER2_MAX_MARGIN 0
#define CALIBRATION_DELAY_MS 2000
#define CALIBRATION_STEP_DELAY_MS 50
#define CALIBRATION_WAIT_THRESHOLD 100 // ~0.5 seconds at 5ms per loop

// ==================== EEPROM CONFIGURATION ====================
#define EEPROM_SIZE 64
#define EEPROM_CALIBRATION_ADDR 0

// ==================== BUTTON CONFIGURATION ====================
// Invert mask for 40 buttons
const uint8_t INVERT_MASK[40] = {1, 0, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1};
const uint8_t BUTTON_INDICES[] = {8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 24, 25, 26, 27, 28, 29, 30, 31, 36, 37, 39};
const size_t BUTTON_COUNT = sizeof(BUTTON_INDICES) / sizeof(BUTTON_INDICES[0]);


// Button indices for special functions
#define BUTTON_MODIFIER 0
#define BUTTON_MANUAL_DISABLE 10
#define BUTTON_MANUAL_ENABLE 11
#define BUTTON_FOLLOW_DISABLE 9
#define BUTTON_FOLLOW_ENABLE 8
#define BUTTON_CALIBRATE_1 0
#define BUTTON_CALIBRATE_2 1
#define BUTTON_SERVO1_INC 16
#define BUTTON_SERVO1_DEC 17
#define BUTTON_SERVO2_INC 18
#define BUTTON_SERVO2_DEC 19
#define BUTTON_FOLLOW_STICK 22

// ==================== MINISTICK CONFIGURATION ====================
#define MINISTICK_X_MIN 1267
#define MINISTICK_X_MAX 2951
#define MINISTICK_Y_MIN 1174
#define MINISTICK_Y_MAX 2973

// ==================== GAMEPAD CONFIGURATION ====================
#define GAMEPAD_NAME "Cyclic Stick"
#define GAMEPAD_MANUFACTURER "Custom"
#define GAMEPAD_BATTERY_LEVEL 100
#define AXIS_MIN -2048
#define AXIS_MAX 2047
#define AXIS_RANGE 4095

// ==================== TIMING CONFIGURATION ====================
#define MAIN_LOOP_DELAY_MS 5
#define SERVO_TASK_DELAY_MS 1
#define MANUAL_SERVO_UPDATE_MS 20
#define GRIP_READ_TIMEOUT_MS 3

// ==================== POLL SIGNAL ====================
const uint8_t POLL_SIGNAL[9] = {0xA5, 0x0B, 0x11, 0x98, 0x00, 0x00, 0x00, 0xE5, 0x20};

// ==================== GRIP DATA INDICES ====================
#define GRIP_RX_BUFFER_SIZE 64
#define GRIP_BUTTON_START_BYTE 33
#define GRIP_BUTTON_END_BYTE 38
#define GRIP_MINISTICK_X_LSB_INDEX 21
#define GRIP_MINISTICK_X_MSB_INDEX 22
#define GRIP_MINISTICK_Y_LSB_INDEX 23
#define GRIP_MINISTICK_Y_MSB_INDEX 24
#define GRIP_MIN_RESPONSE_SIZE 37

// ==================== TASK CONFIGURATION ====================
#define SERVO_TASK_STACK_SIZE 2048
#define SERVO_TASK_PRIORITY 1
#define SERVO_TASK_CORE 0

#endif // CONFIG_H
