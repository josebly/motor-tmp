#include "../Src/param.h"
#include "math.h"

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
    .fast_loop_param.foc_param.num_poles = 7,
    .main_loop_param.gear_ratio = 50,
    .main_loop_param.kt = .012*sqrt(.5),
    .startup_param.startup_mode = CURRENT,
    .main_loop_param.update_frequency = 10000,
    .fast_loop_param.adc1_offset = 1990,
    .fast_loop_param.adc2_offset = 1995,
    .fast_loop_param.adc3_offset = 1980,
    .fast_loop_param.adc1_gain = 3.0/4096/(.2),  // V/count * A/Vr / Vo/Vr
    .fast_loop_param.adc2_gain = 3.0/4096/(.2),
    .fast_loop_param.adc3_gain = 3.0/4096/(.2), 
    .fast_loop_param.motor_encoder.dir = 1,
    .fast_loop_param.phase_mode = 1,
    .fast_loop_param.motor_encoder.cpr = 33000,
    .fast_loop_param.motor_encoder.use_index_electrical_offset_pos = 0,
    .fast_loop_param.motor_encoder.index_electrical_offset_pos = 402,
    .main_loop_param.torque_bias = 0.025,
    .main_loop_param.torque_gain = -25,
    .fast_loop_param.vbus_gain = 3.0/4096*(13.3+1.78)/1.78,
    .main_loop_param.controller_param.kp = 0,
    .main_loop_param.controller_param.kd = 0,
    .main_loop_param.controller_param.command_max = 3,
    .fast_loop_param.cogging.table = {
#include "../cogprocessed.csv"
    },
    .startup_param.do_phase_lock = 1,
    .startup_param.phase_lock_current = 2,
    .startup_param.phase_lock_duration = 2,
    .name = "J1",
    .main_loop_param.output_encoder.cpr = 262144,
};