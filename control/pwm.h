#ifndef PWM_H
#define PWM_H

#include <stm32f446xx.h>

class PWM {
 public:
    PWM(TIM_TypeDef &regs) : regs_(regs) {} 
    void set_voltage(float v_abc[3]);
    void set_vbus(float vbus);
 private:
    TIM_TypeDef &regs_;
    float v_to_pwm_ = 899/12;
};

#endif
