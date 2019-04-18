#ifndef CONFIG_H
#define CONFIG_H

#include "../control/encoder.h"

struct Config {
    Config();
    Encoder &motor_encoder;
};

extern const Config config;
#endif
