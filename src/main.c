#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stm32f4xx.h>
#include <stm32f4xx_usart.h>
#include <usart.h>
#include <debug.h>
#include <spi.h>
#include <millis.h>
#include <globals.h>

int main(void) {
    int lastUpdate;

    init_usart2(115200); // initialize the USART peripheral
    millis_init();
    log_info("millis_init passed\r\n");
    lastUpdate = millis();

    // bring up the GPIO pins and SPI HW interface
    if (init_spi1(NRF8001_SPI, SPI_BaudRatePrescaler_32) != E_SUCCESS) {
        log_err("GPIO and SPI HW bringup failed");
        return -1;
    }

    log_info("init_spi1 passed\r\n");

    while(1) {
        printf("millis_now = %d, millis_old = %d\r\n", millis(), lastUpdate);
        lastUpdate = millis();

        millisDelay(1000);
    }
}