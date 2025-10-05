#include <Arduino.h>
#include <driver/uart.h>

const uint8_t pollSignal[9] = {0xA5, 0x0B, 0x11, 0x98, 0x00, 0x00, 0x00, 0xE5, 0x20};
const uint8_t TX_PIN = 22;
const uint8_t RX_PIN = 23;

HardwareSerial gripSerial(2);

void setup() {
  Serial.begin(115200);
  Serial.println("VKB Base Emulator Started");
  gripSerial.begin(500000, SERIAL_8N1, RX_PIN, TX_PIN);
  delay(1000);
}

void loop() {
  gripSerial.write(pollSignal, sizeof(pollSignal));
  gripSerial.flush();
  
  // Detach TX from UART2 and set to input
  uart_set_pin(2, UART_PIN_NO_CHANGE, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  pinMode(TX_PIN, INPUT);
  
  uint8_t rxBuffer[64];
  size_t rxCount = 0;
  
  unsigned long startTime = millis();
  while (millis() - startTime < 10) {
    while (gripSerial.available() && rxCount < sizeof(rxBuffer)) {
      rxBuffer[rxCount++] = gripSerial.read();
    }
  }
  
  if (rxCount > 0) {
    // Serial.printf("Received %d bytes: ", rxCount);
    for (size_t i = 0; i < rxCount; i++) {
      Serial.printf("%02X ", rxBuffer[i]);
    }
    Serial.println();
  }
  
  // Reattach TX to UART2
  uart_set_pin(2, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  
  delay(5);
}