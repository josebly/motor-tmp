#ifndef SPI_ENCODER
#define SPI_ENCODER

#include "encoder.h"

#ifndef SPI1
struct SPI_TypeDef;
#endif

class GPIO;

class SPIEncoder : public Encoder {
 public:
    SPIEncoder(SPI_TypeDef &regs, GPIO &gpio_cs) : Encoder(0), regs_(regs), gpio_cs_(gpio_cs) {} 
    //void init() {}
    virtual int32_t get_value();
    virtual void trigger();
 protected:
    SPI_TypeDef &regs_;
    GPIO &gpio_cs_;
};

#endif
