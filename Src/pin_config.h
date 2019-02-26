
#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern uint16_t *const red_reg;
extern uint16_t *const green_reg;
extern uint16_t *const blue_reg;

extern volatile uint32_t *const drv_en_reg;
extern uint32_t const drv_en_pin;

extern uint8_t const adc_ia_channel;
extern uint8_t const adc_ib_channel;
extern uint8_t const adc_ic_channel;

#ifdef __cplusplus
}
#endif

#endif