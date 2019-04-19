#include "../control/spi_encoder.h"
#include "stm32f446xx.h"

static struct {
    SPIEncoder motor_encoder = {*SPI3};
    PWM motor_pwm = {*TIM8};
    FastLoop fast_loop = {motor_pwm, motor_encoder};
    LED led = {const_cast<uint16_t*>(reinterpret_cast<volatile uint16_t *>(&TIM3->CCR1)), 
               const_cast<uint16_t*>(reinterpret_cast<volatile uint16_t *>(&TIM3->CCR2)),
               const_cast<uint16_t*>(reinterpret_cast<volatile uint16_t *>(&TIM3->CCR4))};
    PIDController controller;
    MainLoop main_loop = {controller, led};
} config_items;