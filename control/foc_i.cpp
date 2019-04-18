#include "fast_loop.h"
#include "control_fun.h"
#include "foc_i.h"
#include "pwm.h"
#include "main_loop.h"
#include "encoder.h"
#include "spi_encoder.h"
#include "../Src/param.h"
#include "../Src/pin_config.h"

static PWM pwm_ = {*TIM8};
static SPIEncoder motor_encoder_;
static FastLoop fast_loop_(pwm_, motor_encoder_);
static PIDController controller_;
static MainLoop main_loop_;

void system_init() {
    main_loop_.init();
    motor_encoder_.init(get_pin_config()->motor_encoder_reg);
}

void fast_loop_update() {
    fast_loop_.update();
}

void fast_loop_maintenance() {
    fast_loop_.maintenance();
}

void fast_loop_set_id_des(float id) {
    fast_loop_.set_id_des(id);
}
void fast_loop_set_iq_des(float iq) {
    fast_loop_.set_iq_des(iq);
}

void fast_loop_phase_lock_mode(float id) {
    fast_loop_.phase_lock_mode(id);
}

void fast_loop_current_mode() {
    fast_loop_.current_mode();
}

void fast_loop_set_param(const FastLoopParam *const fast_loop_param) {
    fast_loop_.set_param(*fast_loop_param);
}

void fast_loop_get_status(FastLoopStatus * const fast_loop_status) {
    fast_loop_.get_status(fast_loop_status);
}

void fast_loop_voltage_mode() {
    fast_loop_.voltage_mode();
}

void fast_loop_zero_current_sensors() {
    fast_loop_.zero_current_sensors();
}


void main_loop_update() {
    main_loop_.update();
}

void main_loop_set_param(MainLoopParam * const main_loop_param) {
    main_loop_.set_param(*main_loop_param);
}

void main_loop_get_status(MainLoopStatus * const main_loop_status) {
    main_loop_.get_status(main_loop_status);
}

// void fast_loop_set_command(FOCCommand *foc_command) {
//     fast_loop_.set_command(*foc_command);
// }

// void foc_get_status(FOCStatus *foc_status) {
//     foc_.get_status(foc_status);
// }

// void foc_set_param(FOCParam *foc_param) {
//     foc_.set_param(*foc_param);
// }

void controller_set_param(PIDParam *pid_param) {
    controller_.set_param(*pid_param);
}

float controller_step(float desired, float measured) {
    return controller_.step(desired, measured);
}