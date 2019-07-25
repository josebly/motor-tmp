#include "config.h"
#include "../communication/led.h"
#include "../communication/communication.h"
#include "../control/control_fun.h"
#include "../control/encoder.h"
#include "../control/pwm.h"
#include "../control/gpio.h"

#include "dev_00_aksim2.cpp"

const Config config;

Config::Config() :
    fast_loop(config_items.fast_loop),
    main_loop(config_items.main_loop)
    {}