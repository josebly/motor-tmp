#ifndef PWM_H
#define PWM_H

#include <cstdint>

#include "stm32f446xx.h"

class GPIO;

class PWM {
 public:
    PWM(uint16_t period, uint32_t &pwm_a, uint32_t &pwm_b, 
         uint32_t &pwm_c, TIM_TypeDef &regs, GPIO &enable) : 
         period_(period), half_period_(period/2),
         pwm_a_(pwm_a), pwm_b_(pwm_b), pwm_c_(pwm_c),
         regs_(regs), enable_(enable) {
      set_vbus(12);
   } 
   void set_voltage(float v_abc[3]);
   void set_vbus(float vbus);
   void open_mode();
   void brake_mode();
   void voltage_mode();
 private:
   const uint16_t period_, half_period_;
   uint32_t &pwm_a_, &pwm_b_, &pwm_c_;
   TIM_TypeDef &regs_;
   GPIO &enable_;
   float v_to_pwm_;
};

#endif
