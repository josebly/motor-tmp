
#include "pin_config.h"
#include "stm32f446xx.h"

uint16_t *const red_reg = (uint16_t *) &TIM3->CCR1;
uint16_t *const green_reg = (uint16_t *) &TIM3->CCR2;
uint16_t *const blue_reg = (uint16_t *) &TIM3->CCR4;