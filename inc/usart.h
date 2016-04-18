/**
 * @filename usart.h
 * @description USART support library for stm32f4 boards
 * @author Nicholas Shrake, <shraken@gmail.com>
 */

#ifndef RTC_USART_H
#define RTC_USART_H

/**
 * CONSTANTS
 */

/**
 * PROTOTYPES
 */

void init_usart2(uint32_t baudrate);
void puts_usart(USART_TypeDef* USARTx, volatile char *s);

#endif
