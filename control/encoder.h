#ifndef ENCODER_H
#define ENCODER_H

#include <cstdint>
#include "stm32f446xx.h"

class Encoder {
 public:
    void init(volatile int32_t *counter_reg) { counter_reg_ = counter_reg; }
    virtual int32_t get_value() { return *counter_reg_; }
    virtual void trigger() {}
 private:
    volatile int32_t *counter_reg_;
};

#endif
