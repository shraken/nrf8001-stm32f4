/**
 * @description SPI support library for stm32f4 boards
 */

#ifndef SPI_H
#define SPI_H

#include <stm32f4xx.h>
#include "globals.h"

/**
 * CONSTANTS
 */

#define NRF8001_SPI		SPI1

#define SPI_COMMON_PORT GPIOA
#define MOSI_PIN        GPIO_Pin_7
#define MISO_PIN        GPIO_Pin_6
#define SCLK_PIN        GPIO_Pin_5

#define RESET_GPIO_PORT GPIOB
#define RESET_PIN		GPIO_Pin_1

#define RDYN_GPIO_PORT 	GPIOA
#define RDYN_PIN		GPIO_Pin_4

#define REQN_GPIO_PORT	GPIOB
#define REQN_PIN 		GPIO_Pin_0

#define RST_LOW         GPIO_ResetBits(RESET_GPIO_PORT, RESET_PIN)
#define RST_HIGH        GPIO_SetBits(RESET_GPIO_PORT, RESET_PIN)

#define REQN_LOW     	GPIO_ResetBits(REQN_GPIO_PORT, REQN_PIN)
#define REQN_HIGH    	GPIO_SetBits(REQN_GPIO_PORT, REQN_PIN)

/**
 * PROTOTYPES
 */

error_type init_spi1(SPI_TypeDef* SPIx, unsigned int prescaler);
error_type send_byte_SPI(SPI_TypeDef* SPIx, uint8_t byte);
error_type recv_byte_SPI(SPI_TypeDef* SPIx, uint8_t *byte);
error_type send_multibyte_SPI(SPI_TypeDef* SPIx, uint8_t *data, uint16_t length);
error_type recv_multibyte_SPI(SPI_TypeDef* SPIx, uint8_t *data, uint16_t length);
error_type transmit_SPI(SPI_TypeDef* SPIx, uint8_t *txbuf, uint8_t *rxbuf, uint16_t length);

#endif
