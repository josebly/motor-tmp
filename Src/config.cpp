#include "config.h"

#include "dev_00.cpp"

const Config config;

Config::Config() :
    motor_encoder(config_items.motor_encoder),
    motor_pwm(motor_pwm)
    {}