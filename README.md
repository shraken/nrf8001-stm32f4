# Nordic nrf8001 BLE example for STM32F4

An example showing the ble-sdk-arduino library for the Nordic nrf8001 device family ported to the ST STM32F407/discovery kit.  A UART RX interrupt buffers the characters entered beforing sending a 20 byte notification packet.

## Requirements

Install the GNU ARM Embedded Toolchain:
https://launchpad.net/~terry.guo/+archive/ubuntu/gcc-arm-embedded

Clone the ST-Link linux line programmer and build from source, modify
the Makefile with the build location.
https://github.com/texane/stlink

Clone the STM32F4-workarea repo and specify the path in the Makefile.
https://github.com/g4lvanix/STM32F4-workarea

## Building

Type make

## Flashing

Type sudo make burn
