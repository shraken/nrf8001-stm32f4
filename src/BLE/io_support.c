/**
 * @description IO support library for stm32f4 and nrf8001.  Defines
 	middleware functions which forward Arduino SDK GPIO, SPI, and time
 	delay to equivalent STM32F4 routines.
 */

#include <debug.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <stm32f4xx.h>
#include <stm32f4xx_usart.h>
#include <millis.h>
#include <io_support.h>
#include <spi.h>

void __ble_assert(const char *file, uint16_t line)
{
    log_info("ERROR: %s, line %d\r\n", file, line);

	/*
  	  Serial.print("ERROR ");
  	  Serial.print(file);
  	  Serial.print(": ");
  	  Serial.print(line);
  	  Serial.print("\n");
  	  while(1);
	*/
}

void delay(uint16_t delay) 
{
    uint32_t old_time = millis();

    while ((millis() - old_time) < delay) {}
}

uint8_t digitalRead(uint8_t pin)
{
    uint8_t value;

    if (pin == RDYN_PIN) {
        if (GPIO_ReadInputData(RDYN_GPIO_PORT) & pin)
            value = 1;
        else
            value = 0;
 	  }

    //log_info("digitalRead(%d) = %d\r\n", pin, value);
    return value;
}

uint8_t digitalWrite(uint8_t pin, uint8_t value)
{
 	  // only do for REQN pin
 	  if ((pin == REQN_PIN) || (pin == RESET_PIN)) {
 	  	  if (value)
            GPIO_SetBits(REQN_GPIO_PORT, pin);
 	  	  else
 	  	  	  GPIO_ResetBits(REQN_GPIO_PORT, pin);
 	  }

    //log_info("digitalWrite(%d) = %d\r\n", pin, value);
 	  return 0;
}

void pinMode(uint8_t pin, uint8_t mode)
{
 	  GPIO_InitTypeDef gpio_init;

 	  gpio_init.GPIO_Pin = pin;
      gpio_init.GPIO_Speed = GPIO_Speed_50MHz;

 	  switch (mode) {
 	      case OUTPUT:
 	          gpio_init.GPIO_Mode = GPIO_Mode_OUT;
 	          gpio_init.GPIO_OType = GPIO_OType_PP;
 	          gpio_init.GPIO_PuPd = GPIO_PuPd_UP;
 	      	  break;

 	       case INPUT:
 	       case INPUT_PULLUP:
    		  gpio_init.GPIO_Mode = GPIO_Mode_IN;
    		  gpio_init.GPIO_PuPd = GPIO_PuPd_UP;
 	          break; 
 	  	   
 	  	   default:
 	  	       break;
 	  }

 	  if (pin == RDYN_PIN) {
 	      GPIO_Init(RDYN_GPIO_PORT, &gpio_init);
 	  } else if (pin == REQN_PIN) {
 	  	  GPIO_Init(REQN_GPIO_PORT, &gpio_init);
 	  }
}