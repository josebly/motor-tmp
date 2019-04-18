#ifndef SPI_ENCODER
#define SPI_ENCODER

#include "encoder.h"

#ifndef SPI1
struct SPI_TypeDef;
#endif

class SPIEncoder : public Encoder {
 public:
    SPIEncoder(SPI_TypeDef &regs) : regs_(regs) {} 
    //void init() {}
    virtual int32_t get_value();
    virtual void trigger();
 private:
    SPI_TypeDef &regs_;
};

#endif
