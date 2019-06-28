#include "config_items.h"
#include "../../control/spi_encoder.h"
#include "../../communication/usb_communication.h"
#include "stm32f446xx.h"

extern const volatile Param initial_param;

static struct {
    GPIO motor_encoder_cs = {*GPIOA, 15, GPIO::OUTPUT};
    GPIO enable = {*GPIOC, 14, GPIO::OUTPUT};
    PWM motor_pwm = {initial_param.fast_loop_param.pwm_frequency, *const_cast<uint32_t*>(&TIM8->CCR3), 
                          *const_cast<uint32_t*>(&TIM8->CCR2), 
                          *const_cast<uint32_t*>(&TIM8->CCR1),
                          *TIM8, enable};
    SPIEncoder motor_encoder = {*SPI1, motor_encoder_cs};
    FastLoop fast_loop = {motor_pwm, motor_encoder};
    LED led = {const_cast<uint16_t*>(reinterpret_cast<volatile uint16_t *>(&TIM3->CCR1)), 
               const_cast<uint16_t*>(reinterpret_cast<volatile uint16_t *>(&TIM3->CCR2)),
               const_cast<uint16_t*>(reinterpret_cast<volatile uint16_t *>(&TIM3->CCR4)), true};
    PIDController controller;
    USBCommunication communication;
    MainLoop main_loop = {controller, communication, led};
} config_struct;

struct ConfigItems config_items = {
  config_struct.fast_loop,
  config_struct.main_loop,
};
