#include "../control/qep_encoder.h"
#include "../control/aksim2_encoder.h"
#include "../communication/usb_communication.h"
#include "stm32f446xx.h"

extern const volatile Param initial_param;

static struct {
    Aksim2Encoder output_encoder = {*SPI1};
    QEPEncoder motor_encoder = {reinterpret_cast<volatile int32_t *>(&TIM2->CNT), reinterpret_cast<volatile int32_t *>(&TIM2->CCR3)};
    GPIO enable = {*GPIOC, 14, GPIO::OUTPUT};
    PWM motor_pwm = {initial_param.fast_loop_param.pwm_frequency, *const_cast<uint32_t*>(&TIM8->CCR3), 
                          *const_cast<uint32_t*>(&TIM8->CCR2), 
                          *const_cast<uint32_t*>(&TIM8->CCR1),
                          *TIM8, enable};
    FastLoop fast_loop = {motor_pwm, motor_encoder};
    LED led = {const_cast<uint16_t*>(reinterpret_cast<volatile uint16_t *>(&TIM3->CCR1)), 
               const_cast<uint16_t*>(reinterpret_cast<volatile uint16_t *>(&TIM3->CCR2)),
               const_cast<uint16_t*>(reinterpret_cast<volatile uint16_t *>(&TIM3->CCR4)), 1};
    PIDDeadbandController controller = {1.0/10000};
    USBCommunication communication;
    MainLoop main_loop = {controller, communication, led, output_encoder};
} config_items;
