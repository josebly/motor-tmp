#include "../Src/param.h"

// Can be written by external methods, e.g. bootloader
const volatile Param __attribute__ ((section ("flash_param"))) initial_param = {
    .fast_loop_param.pwm_frequency = 100000,
    .fast_loop_param.foc_param.pi_d.kp=1,
    .fast_loop_param.foc_param.pi_d.ki=.1,
    .fast_loop_param.foc_param.pi_d.ki_limit=2.5,
    .fast_loop_param.foc_param.pi_d.command_max=3,
    .fast_loop_param.foc_param.pi_q.kp=1,
    .fast_loop_param.foc_param.pi_q.ki=.1,
    .fast_loop_param.foc_param.pi_q.ki_limit=2.5,
    .fast_loop_param.foc_param.pi_q.command_max=3,
    .main_loop_param.update_frequency = 10000,
    .fast_loop_param.adc1_offset = 1990,
    .fast_loop_param.adc2_offset = 1995,
    .fast_loop_param.adc3_offset = 1980,
    .fast_loop_param.adc1_gain = 3.3/4096/(.007*40),  // V/count * A/Vr / Vo/Vr
    .fast_loop_param.adc2_gain = 3.3/4096/(.007*40),
    .fast_loop_param.adc3_gain = 3.3/4096/(.007*40), 
    .fast_loop_param.motor_encoder.dir = -1,
    .fast_loop_param.motor_encoder.cpr = 1024,
    .fast_loop_param.motor_encoder.use_index_electrical_offset_pos = 1,
    .fast_loop_param.motor_encoder.index_electrical_offset_pos = 402,
    .fast_loop_param.cogging.table = {
#include "../cogprocessed.csv"
    }

};