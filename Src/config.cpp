#include "config.h"

#include "dev_00.cpp"

const Config config;

Config::Config() :
    motor_encoder(config_items.motor_encoder),
    motor_pwm(config_items.motor_pwm),
    fast_loop(config_items.fast_loop),
    main_loop(config_items.main_loop),
    controller(config_items.controller)
    {}