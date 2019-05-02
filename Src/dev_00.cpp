#include "../control/spi_encoder.h"
#include "../communication/usb_communication.h"
#include "stm32f446xx.h"

static struct {
    SPIEncoder motor_encoder = {*SPI3};
    GPIO enable = {*GPIOC, 14, GPIO::OUTPUT};
    PWM motor_pwm = {899, *const_cast<uint32_t*>(&TIM8->CCR3), 
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
