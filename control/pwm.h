
#include <stm32f446xx.h>

class PWM {
 public:
    PWM(TIM_TypeDef &regs) : regs_(regs) {} 
    void set_voltage(float v_abc[3]);
 private:
    TIM_TypeDef &regs_;
};