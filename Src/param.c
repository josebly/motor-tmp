
#include "param.h"

static const struct Param initial_param = {
    .fast_loop_param.pwm_frequency = 100000,
    .fast_loop_param.foc_param.pi_d.kp=1,
    .fast_loop_param.foc_param.pi_d.ki=.1,
    .fast_loop_param.foc_param.pi_d.ki_limit=4,
    .fast_loop_param.foc_param.pi_d.command_max=5,
    .fast_loop_param.foc_param.pi_q.kp=1,
    .fast_loop_param.foc_param.pi_q.ki=.1,
    .fast_loop_param.foc_param.pi_q.ki_limit=4,
    .fast_loop_param.foc_param.pi_q.command_max=5,
};

static struct Param working_param;

struct Param *param() {
    return &working_param;
}

void init_param_from_flash() {
    working_param = initial_param;
}

void save_param_to_flash() {
    // TODO
}

