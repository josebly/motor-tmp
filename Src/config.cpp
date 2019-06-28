#include "config.h"
#include "../communication/led.h"
#include "../communication/communication.h"
#include "../control/control_fun.h"
#include "../control/encoder.h"
#include "../control/pwm.h"
#include "../control/gpio.h"

const Config config;
extern ConfigItems config_items;

Config::Config() :
    fast_loop(config_items.fast_loop),
    main_loop(config_items.main_loop)
    {}