
#include "pin_config.h"
#include "../parameters/otp.h"
#include "stm32f446xx.h"


static PinConfig default_pin_config = {
    .red_reg = (uint16_t *) &TIM3->CCR1,
    .green_reg = (uint16_t *) &TIM3->CCR2,
    .blue_reg = (uint16_t *) &TIM3->CCR4,
    .drv_en_reg = &GPIOC->ODR,
    .drv_en_pin = GPIO_ODR_OD14 | GPIO_ODR_OD15,
    .adc_ia_channel = 3,
    .adc_ib_channel = 14,   // note adc12 only
    .adc_ic_channel = 15,   // note adc12 only
    .adc_vbus_channel = 5,
    .pwm_a_reg = (uint16_t *) &TIM8->CCR3,
    .pwm_b_reg = (uint16_t *) &TIM8->CCR2,
    .pwm_c_reg = (uint16_t *) &TIM8->CCR1,
    .crystal_frequency_MHz = 8,
};

Config::Config() {
    pin_config_ = &default_pin_config;
    if (get_board_id()->manufacturer == BoardID::FabulabSL) {
        pin_config_->crystal_frequency_MHz = 24;
    }
}

static Config config;

const PinConfig * const get_pin_config() {
    return config.get_pin_config();
}