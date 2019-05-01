
#include "pwm.h"
#include "stm32f446xx.h"

void PWM::set_voltage(float v_abc[3]) {
    pwm_a_ = v_abc[0] * v_to_pwm_ + half_period_;
    pwm_b_ = v_abc[1] * v_to_pwm_ + half_period_;
    pwm_c_ = v_abc[2] * v_to_pwm_ + half_period_;
}

void PWM::set_vbus(float vbus) {
    v_to_pwm_ = period_/vbus;
}

void PWM::open_mode() {
    // need to set gpio disable
}

void PWM::brake_mode() {
    // gpio enable
    //MOE = 0; // with OSSI=1 to force low
    regs_.BDTR |= TIM_BDTR_OSSI;
    regs_.BDTR &= ~TIM_BDTR_MOE;
}

void PWM::voltage_mode() {
    // gpio enable
    regs_.BDTR |= TIM_BDTR_MOE;
}

void PWM::set_frequency_hz(uint32_t frequency_hz) {
    regs_.ARR = 180e6/2/frequency_hz; // todo not enabled at startup
    period_ = 180e6/2/frequency_hz;
    half_period_ = period_/2; 
}