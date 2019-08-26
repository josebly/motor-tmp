

#ifndef LED_H
#define LED_H

#include <cstdint>
// A tricolor led for status, pwm compare registers required
class LED {
 public:
    LED(uint16_t *const red_reg, uint16_t *const green_reg, uint16_t *const blue_reg, bool invert=false, float brightness=1.0)
        : red_reg_(red_reg), green_reg_(green_reg), blue_reg_(blue_reg) {
            invert_ = invert ? 1 : 0;
            brightness_ = brightness;
        }
    enum Mode {OFF, ON, BLINKING, PULSING};
    enum Color {RED, ORANGE, YELLOW, GREEN, BLUE, PURPLE, WHITE};
    void set_mode(Mode mode) {}
    void set_color(Color color) {}
    void set_rate(float frequency) {}
    void update() {
        // TODO figure out where to put 10000
        if (i++ >= 10000) {
            i = 0;
        }
        float t = i*(1.f/10000.f);
        float intensity = t < 0.5 ? 2*t : 2-2*t;
        uint16_t intensity_period = 65535*brightness_*fabsf(invert_ - intensity*intensity*intensity);
        *red_reg_ = intensity_period;
        *green_reg_ = intensity_period;
        *blue_reg_ = intensity_period;
    }
 private:
    uint16_t i = 0;
    float invert_, brightness_;
    uint16_t *const red_reg_;
    uint16_t *const green_reg_;
    uint16_t *const blue_reg_;
};

#endif