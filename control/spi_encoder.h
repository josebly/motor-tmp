#ifndef SPI_ENCODER
#define SPI_ENCODER

#include "encoder.h"

class SPIEncoder : public Encoder {
 public:
    //void init() {}
    virtual int32_t get_value();
    virtual void trigger();
 private:
    
};

#endif
