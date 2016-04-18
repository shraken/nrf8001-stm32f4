/**
 * @description SPI support library for stm32f4 boards derived
 *				from g4lvanix STM32F4-workarea and examples.
 */

#include <stm32f4xx.h>
#include <stm32f4xx_usart.h>
#include <misc.h>
#include "spi.h"

error_type init_spi1(SPI_TypeDef* SPIx, unsigned int prescaler)
{
    GPIO_InitTypeDef gpio_init;
    SPI_InitTypeDef spi_init;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    /*
        PA5 = SPI1 SCLK (all outputs)
        PA6 = SP11 MISO
        PA7 = SP1 MOSI
    */

    gpio_init.GPIO_Pin = MOSI_PIN | MISO_PIN | SCLK_PIN;
    gpio_init.GPIO_Mode = GPIO_Mode_AF;
    gpio_init.GPIO_OType = GPIO_OType_PP;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(SPI_COMMON_PORT, &gpio_init);
    
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

    /*
        PA0 = RST
        PA4 = SPI1 RDYN (input)
        PB0 = SPI1 REQN (output)
    */

    gpio_init.GPIO_Pin = RESET_PIN;
    gpio_init.GPIO_Mode = GPIO_Mode_OUT;
    gpio_init.GPIO_OType = GPIO_OType_PP;
    gpio_init.GPIO_PuPd = GPIO_PuPd_UP;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(RESET_GPIO_PORT, &gpio_init);

    gpio_init.GPIO_Pin = REQN_PIN;
    gpio_init.GPIO_Mode = GPIO_Mode_OUT;
    gpio_init.GPIO_OType = GPIO_OType_PP;
    gpio_init.GPIO_PuPd = GPIO_PuPd_UP;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(REQN_GPIO_PORT, &gpio_init);

    /*
    GPIO_InitStructure.GPIO_Pin = INDIC_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(INDIC_PORT, &GPIO_InitStructure);
	*/

    gpio_init.GPIO_Pin = RDYN_PIN;
    gpio_init.GPIO_Mode = GPIO_Mode_IN;
    gpio_init.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(RDYN_GPIO_PORT, &gpio_init);

    // initial pin state high for MOSI and REQN pins
    GPIO_SetBits(REQN_GPIO_PORT, REQN_PIN);

    GPIO_ResetBits(SPI_COMMON_PORT, MISO_PIN);
    GPIO_ResetBits(SPI_COMMON_PORT, MOSI_PIN);
    GPIO_ResetBits(SPI_COMMON_PORT, SCLK_PIN);

    /*
        SPI specific
    */
    spi_init.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    spi_init.SPI_Mode = SPI_Mode_Master;
    spi_init.SPI_DataSize = SPI_DataSize_8b;
    spi_init.SPI_CPOL = SPI_CPOL_Low;
    spi_init.SPI_CPHA = SPI_CPHA_1Edge;
    spi_init.SPI_NSS = SPI_NSS_Soft | SPI_NSSInternalSoft_Set;
    spi_init.SPI_BaudRatePrescaler = prescaler;
    spi_init.SPI_FirstBit = SPI_FirstBit_LSB;
    SPI_Init(SPIx, &spi_init); 
    SPI_Cmd(SPIx, ENABLE);

    // DMA for SPI1 RX/TX
    //init_spi_dma(SPIx);

    return E_SUCCESS;
}

error_type send_byte_SPI(SPI_TypeDef* SPIx, uint8_t data)
{
	unsigned char temp;

	SPIx->DR = data;
	while( !(SPIx->SR & SPI_I2S_FLAG_TXE) );
	while( !(SPIx->SR & SPI_I2S_FLAG_RXNE) );
	while( SPIx->SR & SPI_I2S_FLAG_BSY );
	temp = SPIx->DR;

    return E_SUCCESS;
}

error_type recv_byte_SPI(SPI_TypeDef* SPIx, uint8_t *data)
{
	SPIx->DR = 0xFF;
	while( !(SPIx->SR & SPI_I2S_FLAG_TXE) );
	while( !(SPIx->SR & SPI_I2S_FLAG_RXNE) );
	while( SPIx->SR & SPI_I2S_FLAG_BSY );
	*data = SPIx->DR;

    return E_SUCCESS;
}

error_type send_multibyte_SPI(SPI_TypeDef* SPIx, uint8_t *data, uint16_t length)
{
	while (length--)
	{
		send_byte_SPI(SPIx, *data);
		data++;
	}

    return E_SUCCESS;
}

error_type recv_multibyte_SPI(SPI_TypeDef* SPIx, uint8_t *data, uint16_t length)
{
	while (length--)
	{
		recv_byte_SPI(SPIx, data);
		data++;
	}

    return E_SUCCESS;
}

error_type transmit_SPI(SPI_TypeDef* SPIx, uint8_t *txbuf, uint8_t *rxbuf, uint16_t length)
{
	while (length--)
	{
		SPIx->DR = *txbuf; // write data to be transmitted to the SPI data register
		while( !(SPIx->SR & SPI_I2S_FLAG_TXE) ); // wait until transmit complete
		while( !(SPIx->SR & SPI_I2S_FLAG_RXNE) ); // wait until receive complete
		while( SPIx->SR & SPI_I2S_FLAG_BSY ); // wait until SPI is not busy anymore
		*rxbuf = SPIx->DR; // return received data from SPI data register
		txbuf++;
		rxbuf++;	 
	}

    return E_SUCCESS;  
}
