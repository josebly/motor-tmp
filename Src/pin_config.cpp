
#include "pin_config.h"
#include "stm32f446xx.h"

uint16_t *const red_reg = (uint16_t *) &TIM3->CCR1;
uint16_t *const green_reg = (uint16_t *) &TIM3->CCR2;
uint16_t *const blue_reg = (uint16_t *) &TIM3->CCR4;

volatile uint32_t *const drv_en_reg = &GPIOC->ODR;
uint32_t const drv_en_pin = GPIO_ODR_OD14;

uint8_t const adc_ia_channel = 15;
uint8_t const adc_ib_channel = 14;
uint8_t const adc_ic_channel = 13;