#include "control_fun.h"
//#include "hal_fun.h"

float fsat(float a, float sat) {
    if (a > sat) {
        return sat;
    } else if (a < -sat) {
        return -sat;
    } else {
        return a;
    }
}

void PIController::set_param(const PIParam &pi_param) {
    ki_ = pi_param.ki;
    kp_ = pi_param.kp;
    ki_limit_ = pi_param.ki_limit;
    command_max_ = pi_param.command_max;
}

float PIController::step(float desired, float measured) {
    float error = desired - measured;
    ki_sum_ += ki_ * error;
    ki_sum_ = fsat(ki_sum_, ki_limit_);
    return fsat(kp_*error + ki_sum_, command_max_);
}