#include "../control/spi_encoder.h"
#include "stm32f446xx.h"

static struct {
    SPIEncoder motor_encoder = {*SPI3};
    PWM motor_pwm = {*TIM8};
} config_items;