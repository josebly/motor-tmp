#include "aksim2_encoder.h"

extern  uint16_t spi1_rx[4];
void Aksim2Encoder::trigger() {
   // if (!(DMA2_Stream2->CR & DMA_SxCR_EN)) { // EN == 0 if not transferring
    if (!(DMA2_Stream2->CR & DMA_SxCR_EN) | (DMA2->LISR & DMA_LISR_TCIF2)) {
        if (++count > 1) {  // TODO time based sampling
            count = 0;
            DMA2->LIFCR = DMA_LISR_TCIF3 | DMA_LISR_HTIF3 | DMA_LISR_TCIF2 | DMA_LISR_HTIF2;
            DMA2_Stream2->NDTR = 4;
            DMA2_Stream3->NDTR = 4;
            DMA2_Stream2->CR |= DMA_SxCR_EN;
            DMA2_Stream3->CR |= DMA_SxCR_EN;
        }
    }
    // regs_.DR = 0;
    // while((regs_.SR & SPI_SR_TXE) != SPI_SR_TXE); //1 buffer empty, should be available quickly
    // regs_.DR = 0;
    // while(!(regs_.SR & SPI_SR_RXNE)); // RXNE: 1 -> data available
    // int16_t data = regs_.DR;
    // while((regs_.SR & SPI_SR_TXE) != SPI_SR_TXE); //1 buffer empty, should be available quickly
    // regs_.DR = 0;
    // while(!(regs_.SR & SPI_SR_RXNE)); // RXNE: 1 -> data available
    // data = regs_.DR;
    // while((regs_.SR & SPI_SR_TXE) != SPI_SR_TXE); //1 buffer empty, should be available quickly
    // regs_.DR = 0;
}

int32_t Aksim2Encoder::get_value() {
    // wait until SPI_FLAG_RXNE, at 1.4 Mbps this will be 22.9 us after trigger
//     while(!(regs_.SR & SPI_SR_RXNE)); // RXNE: 1 -> data available
//     int16_t data = regs_.DR;
//     while(!(regs_.SR & SPI_SR_RXNE)); // RXNE: 1 -> data available
//     data = regs_.DR;
//     return data;
    return spi1_rx[2];
 }

