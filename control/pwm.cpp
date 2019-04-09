
#include "pwm.h"

void PWM::set_voltage(float v_abc[3]) {
    regs_.CCR3 = v_abc[0] * v_to_pwm_ + 450;
    regs_.CCR2 = v_abc[1] * v_to_pwm_ + 450;
    regs_.CCR1 = v_abc[2] * v_to_pwm_ + 450;
}

void PWM::set_vbus(float vbus) {
    v_to_pwm_ = 899/vbus;
}