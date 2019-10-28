#ifndef QEP_ENCODER_H
#define QEP_ENCODER_H

#include <cstdint>
#include "encoder.h"
#include "stm32f446xx.h"

#define container_of(ptr, type, member) ({                      \
        const typeof( ((type *)0)->member ) *__mptr = (ptr);    \
        (type *)( (char *)__mptr - offsetof(type,member) );})

class QEPEncoder : public Encoder {
 public:
    QEPEncoder(volatile int32_t *counter_reg, volatile int32_t *index_reg) : Encoder() { 
       counter_reg_ = counter_reg;
       index_reg_ = index_reg;
    }
    virtual int32_t get_value() { return *counter_reg_; }
    virtual void trigger() {}
    virtual int32_t get_index_pos() { return *index_reg_; }
    virtual bool index_received() { return container_of(reinterpret_cast<volatile uint32_t *>(counter_reg_), TIM_TypeDef, CNT)->SR & TIM_SR_CC3IF; }
 private:
    volatile int32_t *counter_reg_, *index_reg_;
};

#endif
