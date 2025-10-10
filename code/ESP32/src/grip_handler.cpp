#include "grip_handler.h"
#include <driver/uart.h>

HardwareSerial gripSerial(2);

void initGripSerial() {
  gripSerial.begin(GRIP_SERIAL_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
}

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

bool readGripData(GripData &data) {
  gripSerial.write(POLL_SIGNAL, sizeof(POLL_SIGNAL));
  gripSerial.flush();

  uart_set_pin(2, UART_PIN_NO_CHANGE, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  pinMode(TX_PIN, INPUT);

  uint8_t rxBuffer[GRIP_RX_BUFFER_SIZE];
  size_t rxCount = 0;

  unsigned long startTime = millis();
  while (millis() - startTime < GRIP_READ_TIMEOUT_MS) {
    while (gripSerial.available() && rxCount < sizeof(rxBuffer)) {
      rxBuffer[rxCount++] = gripSerial.read();
    }
  }

  uart_set_pin(2, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

  if (rxCount > GRIP_MIN_RESPONSE_SIZE) {
    // Decode all 40 buttons
    uint8_t buttonStates[40] = {0};
    for (int byteIdx = GRIP_BUTTON_START_BYTE; byteIdx < GRIP_BUTTON_END_BYTE && byteIdx < rxCount; byteIdx++) {
      for (int bitIdx = 0; bitIdx < 8; bitIdx++) {
        int buttonIdx = (byteIdx - GRIP_BUTTON_START_BYTE) * 8 + bitIdx;
        if (buttonIdx < 40) {
          uint8_t bitVal = (rxBuffer[byteIdx] >> (7 - bitIdx)) & 1;
          buttonStates[buttonIdx] = bitVal ^ INVERT_MASK[buttonIdx];
        }
      }
    }
    // Fill only usable buttons
    for (size_t i = 0; i < BUTTON_COUNT; i++) {
      data.buttons[i] = buttonStates[BUTTON_INDICES[i]];
    }

    // Decode ministick
    uint16_t lsb_x = decodeLSBX(rxBuffer[GRIP_MINISTICK_X_LSB_INDEX]);
    uint16_t msb_x = decodeMSBX(rxBuffer[GRIP_MINISTICK_X_MSB_INDEX]);
    data.ministick_x = msb_x | lsb_x;

    uint16_t lsb_y = decodeLSBY(rxBuffer[GRIP_MINISTICK_Y_LSB_INDEX]);
    uint16_t msb_y = decodeMSBY(rxBuffer[GRIP_MINISTICK_Y_MSB_INDEX]);
    data.ministick_y = msb_y | lsb_y;

    return true;
  }
  return false;
}
