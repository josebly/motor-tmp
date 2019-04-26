
#include "pwm.h"

void PWM::set_voltage(float v_abc[3]) {
    pwm_a_ = v_abc[0] * v_to_pwm_ + half_period_;
    pwm_b_ = v_abc[1] * v_to_pwm_ + half_period_;
    pwm_c_ = v_abc[2] * v_to_pwm_ + half_period_;
}

void PWM::set_vbus(float vbus) {
    v_to_pwm_ = period_/vbus;
}