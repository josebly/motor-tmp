#include "stm32f446xx.h"

static struct {
    Encoder motor_encoder = {reinterpret_cast<volatile int32_t *>(&TIM2->CNT)};
    PWM motor_pwm = {*TIM8};
    FastLoop fast_loop = {motor_pwm, motor_encoder};
    MainLoop main_loop;
    PIDController controller;
} config_items;