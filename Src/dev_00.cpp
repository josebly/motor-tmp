#include "../control/spi_encoder.h"
#include "../communication/usb_communication.h"
#include "stm32f446xx.h"

extern const volatile Param initial_param;

static struct {
    Encoder motor_encoder = {reinterpret_cast<volatile int32_t *>(&TIM2->CNT)};
    //SPIEncoder motor_encoder = {*SPI3};
    GPIO enable;
    PWM motor_pwm = {initial_param.fast_loop_param.pwm_frequency, *const_cast<uint32_t*>(&TIM8->CCR3), 
                          *const_cast<uint32_t*>(&TIM8->CCR2), 
                          *const_cast<uint32_t*>(&TIM8->CCR1),
                          *TIM8, enable};
    FastLoop fast_loop = {motor_pwm, motor_encoder};
    LED led = {const_cast<uint16_t*>(reinterpret_cast<volatile uint16_t *>(&TIM3->CCR1)), 
               const_cast<uint16_t*>(reinterpret_cast<volatile uint16_t *>(&TIM3->CCR2)),
               const_cast<uint16_t*>(reinterpret_cast<volatile uint16_t *>(&TIM3->CCR4))};
    PIDController controller;
    USBCommunication communication;
    MainLoop main_loop = {controller, communication, led};
} config_items;
