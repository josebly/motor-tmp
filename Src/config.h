#ifndef CONFIG_H
#define CONFIG_H

#include "../control/encoder.h"
#include "../control/pwm.h"
#include "../control/fast_loop.h"
#include "../control/main_loop.h"
#include "../control/control_fun.h"

struct Config {
    Config();
    Encoder &motor_encoder;
    PWM &motor_pwm;
    FastLoop &fast_loop;
    MainLoop &main_loop;
    PIDController &controller;
};

extern const Config config;
#endif
