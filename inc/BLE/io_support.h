#ifndef IO_SUPPORT_H
#define IO_SUPPORT_H

#define LOW           0x00
#define HIGH          0x01

#define INPUT         0x00
#define OUTPUT        0x01
#define INPUT_PULLUP  0x02

void __ble_assert(const char *file, uint16_t line);
void delay(uint16_t delay);
uint8_t digitalRead(uint8_t pin);
uint8_t digitalWrite(uint8_t pin, uint8_t value);
void pinMode(uint8_t pin, uint8_t mode);

#endif