#ifndef CONFIG_H
#define CONFIG_H

#include "../control/encoder.h"
#include "../control/pwm.h"

struct Config {
    Config();
    Encoder &motor_encoder;
    PWM &motor_pwm;
};

extern const Config config;
#endif
