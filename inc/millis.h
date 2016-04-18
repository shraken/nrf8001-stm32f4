/**
 ******************************************************************************
 * @file	millis.h
 * @author	Hampus Sandberg
 * @version	0.1
 * @date	2013-05-09
 * @brief	Manage a millis counter
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MILLIS_H_
#define MILLIS_H_

/* Includes ------------------------------------------------------------------*/
#if defined(STM32F40_41xxx)
#include "stm32f4xx.h"
#elif defined(STM32F10X_MD) || defined(STM32F10X_MD_VL)
#include "stm32f10x.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <stm32f4xx.h>
#include <stm32f4xx_usart.h>
#include <misc.h>

/* Defines -------------------------------------------------------------------*/
/* Typedefs ------------------------------------------------------------------*/
/* Function prototypes -------------------------------------------------------*/
void millis_init(void);
uint32_t millis(void);
void millisDelay(uint32_t);

#endif /* MILLIS_H_ */
