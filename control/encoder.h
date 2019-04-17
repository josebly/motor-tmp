#ifndef ENCODER_H
#define ENCODER_H

#include <cstdint>
#include "stm32f446xx.h"

class Encoder {
 public:
    void init(volatile int32_t *counter_reg) { counter_reg_ = counter_reg; }
    int32_t get_value() const { return *counter_reg_; }
 private:
    volatile int32_t *counter_reg_;
};

#endif
