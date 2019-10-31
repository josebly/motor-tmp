#include "aksim2_encoder.h"

void Aksim2Encoder::init() {
    regs_.CR2 |= SPI_CR2_RXDMAEN;

    // init dma: dma2 stream2 ch3 spi_rx, dma2 stream3 ch3 spi_tx
    // no fifo, half word
    DMA2_Stream2->CR &= ~DMA_SxCR_EN;
    DMA2_Stream2->CR = (3 << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_0 | DMA_SxCR_MINC;
    DMA2_Stream2->PAR = reinterpret_cast<uint32_t>(&regs_.DR);
    DMA2_Stream2->M0AR = reinterpret_cast<uint32_t>(rx_data_);

    DMA2_Stream3->CR &= ~DMA_SxCR_EN;
    DMA2_Stream3->CR = (3 << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_0 | DMA_SxCR_MINC | DMA_SxCR_DIR_0;
    DMA2_Stream3->PAR = reinterpret_cast<uint32_t>(&regs_.DR);
    DMA2_Stream3->M0AR = reinterpret_cast<uint32_t>(tx_data_);

    regs_.CR2 |= SPI_CR2_TXDMAEN;
}

void Aksim2Encoder::trigger() {
    if (!init_) {
        init();
    }
    if (init_++ < 10) {
        return;
    }
    // EN == 0 if not transferring, TCIF also means transfer complete
    // Stream 2 is RX, Stream 3 is TX
    if (!(DMA2_Stream2->CR & DMA_SxCR_EN) || (DMA2->LISR & DMA_LISR_TCIF2)) {
        if (++count > 1) {  // A delay. TODO time based delay
            count = 0;
            DMA2->LIFCR = DMA_LISR_TCIF3 | DMA_LISR_HTIF3 | DMA_LISR_TCIF2 | DMA_LISR_HTIF2;
            DMA2_Stream2->NDTR = 4;
            DMA2_Stream3->NDTR = 4;
            DMA2_Stream2->CR |= DMA_SxCR_EN;
            DMA2_Stream3->CR |= DMA_SxCR_EN;

            // current aksim2 encoders are 18 bit with multiturn, data is in bits 32:49
            raw_value_ = (static_cast<uint32_t>(rx_data_[2] & 0x3FFF) << 4) | (rx_data_[3] >> 12);
            
            int32_t diff = raw_value_ - last_raw_value_;
            last_raw_value_ = raw_value_;

            if (diff > (262144 >> 1)) {
                diff += -262144;
            } else if (diff < -(262144 >> 1)) {
                diff += 262144;
            }
            value_ += diff;
        }
    }
}

int32_t Aksim2Encoder::get_value() {
    return value_;
 }

