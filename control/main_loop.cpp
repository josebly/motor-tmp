
#include "main_loop.h"
#include "../peripheral/led.h"

#include "../Src/pin_config.h"

MainLoop::MainLoop() {
    led_ = new LED(red_reg, green_reg, blue_reg);
}

void MainLoop::update() {
    led_->update();
}