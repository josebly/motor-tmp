#include "fast_loop.h"
#include "control_fun.h"
#include "foc_i.h"
#include "pwm.h"

static PWM pwm_ = {*TIM8};
static FastLoop fast_loop_(pwm_);
static PIDController controller_;

void fast_loop_update() {
    fast_loop_.update();
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