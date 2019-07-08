#include "aksim2_encoder.h"

void Aksim2Encoder::trigger() {
    // two 16 bit reads
    regs_.DR = 0;
    while((regs_.SR & SPI_SR_TXE) != SPI_SR_TXE); //1 buffer empty, should be available quickly
    regs_.DR = 0;
}

int32_t Aksim2Encoder::get_value() {
    // wait until SPI_FLAG_RXNE, at 1.4 Mbps this will be 22.9 us after trigger
    while(!(regs_.SR & SPI_SR_RXNE)); // RXNE: 1 -> data available
    int16_t data = regs_.DR;
    while(!(regs_.SR & SPI_SR_RXNE)); // RXNE: 1 -> data available
    data = regs_.DR;
    return data;
}

