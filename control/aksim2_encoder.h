#include "spi_encoder.h"

// max clock currently working is 180 MHz/128 = 1.4 MHz
class Aksim2Encoder : public SPIEncoder {
 public:
    Aksim2Encoder(SPI_TypeDef &regs, GPIO &gpio_cs) : SPIEncoder(regs, gpio_cs) {} 
    virtual int32_t get_value();
    virtual void trigger();
 private:
    int count = 0;
};