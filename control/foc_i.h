#ifndef FOC_I_H
#define FOC_I_H

#include "../messages.h"

#ifdef __cplusplus
extern "C" {
#endif

void fast_loop_update();
void fast_loop_set_id_des(float id);
void fast_loop_set_iq_des(float iq);
void fast_loop_phase_lock_mode(float id);
void fast_loop_current_mode();
void fast_loop_set_param(const FastLoopParam *const fast_loop_param);

void controller_set_param(PIDParam *pid_param);
float controller_step(float desired, float measured);

#ifdef __cplusplus
}
#endif

#endif
