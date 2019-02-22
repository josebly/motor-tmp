

#ifndef LED_H
#define LED_H

#include <cstdint>
// A tricolor led for status, pwm compare registers required
class LED {
 public:
    LED(uint16_t *const red_reg, uint16_t *const green_reg, uint16_t *const blue_reg)
        : red_reg_(red_reg), green_reg_(green_reg), blue_reg_(blue_reg) {}
    enum Mode {OFF, ON, BLINKING, PULSING};
    enum Color {RED, ORANGE, YELLOW, GREEN, BLUE, PURPLE, WHITE};
    void set_mode(Mode mode) {}
    void set_color(Color color) {}
    void set_rate(float frequency) {}
    void update() {
        *red_reg_ = i++;
        *green_reg_ = 0xFFFF-i;
        *blue_reg_ = 2*i;
    }
 private:
    uint16_t i = 0;
    uint16_t *const red_reg_;
    uint16_t *const green_reg_;
    uint16_t *const blue_reg_;
};

#endif