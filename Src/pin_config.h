
#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DRV_EN_GPIO_INIT     GPIOA->MODER |= GPIO_MODER_MODE9_0; \
                             GPIOA->MODER &= ~GPIO_MODER_MODE9_1; \
                             GPIOA->BSRR |= GPIO_BSRR_BS9;

typedef struct {
    volatile uint32_t *const drv_en_reg;
    uint32_t const drv_en_pin;
    uint8_t const adc_ia_channel;
    uint8_t const adc_ib_channel;
    uint8_t const adc_ic_channel;
    uint8_t const adc_vbus_channel;
    uint8_t crystal_frequency_MHz;
    volatile int32_t *const motor_encoder_reg;
} PinConfig;

const PinConfig * const get_pin_config();
void config_init();

#ifdef __cplusplus
}

class CConfig {
 public:
    CConfig();
    void init();
    const PinConfig * const get_pin_config() const { return pin_config_; }
 private:
    PinConfig * pin_config_;
};

#endif

#endif