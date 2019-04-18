#include "spi_encoder.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_spi.h"

void SPIEncoder::trigger() {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
    asm("NOP; NOP; NOP; NOP; NOP; NOP; NOP; NOP; NOP; NOP; NOP; NOP; NOP;");
    // wait tdelay, 80 ns ma732 (14.4 cycles at 180 MHz)
    SPI3->DR = 0;
}

int32_t SPIEncoder::get_value() {
    // wait until SPI_FLAG_RXNE, ma 732 max frequency 25 Mbps, 640 ns (115 cycles at 180 MHz)
    while(!(SPI3->SR & SPI_SR_RXNE)); // RXNE: 1 -> data available
    int16_t data = SPI3->DR;
    // wait tdelay, 25 ns ma732
    asm("NOP; NOP; NOP; NOP;");
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    return data;
}