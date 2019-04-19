
#include "pin_config.h"
#include "../parameters/otp.h"
#include "stm32f446xx.h"
                            
static PinConfig default_pin_config = {
    .drv_en_reg = &GPIOC->ODR,
    .drv_en_pin = GPIO_ODR_OD14,
    .adc_ia_channel = 3,
    .adc_ib_channel = 14,   // note adc12 only
    .adc_ic_channel = 15,   // note adc12 only
    .adc_vbus_channel = 5,
    .crystal_frequency_MHz = 24,
};

CConfig::CConfig() {
    pin_config_ = &default_pin_config;
    // TODO maybe create objects here
    // motor_encoder_ = new Encoder(reinterpret_cast<volatile int32_t *>(&TIM5->CNT))
}

static CConfig config;

const PinConfig * const get_pin_config() {
    return config.get_pin_config();
}

void config_init() {
    config.init();
}

void CConfig::init() {
    if (get_board_id()->manufacturer == BoardID::ST) {
        pin_config_->crystal_frequency_MHz = 8;
    }
}