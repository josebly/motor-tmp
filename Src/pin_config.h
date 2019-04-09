
#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t *const red_reg;
    uint16_t *const green_reg;
    uint16_t *const blue_reg;
    volatile uint32_t *const drv_en_reg;
    uint32_t const drv_en_pin;
    uint8_t const adc_ia_channel;
    uint8_t const adc_ib_channel;
    uint8_t const adc_ic_channel;
    uint8_t const adc_vbus_channel;
    uint16_t *const pwm_a_reg;
    uint16_t *const pwm_b_reg;
    uint16_t *const pwm_c_reg;
    uint8_t crystal_frequency_MHz;
    volatile int32_t *const motor_encoder_reg;
} PinConfig;

const PinConfig * const get_pin_config();

#ifdef __cplusplus
}

class Config {
 public:
    Config();
    const PinConfig * const get_pin_config() const { return pin_config_; }
 private:
    PinConfig * pin_config_;
};

#endif

#endif