#ifndef FOC_I_H
#define FOC_I_H

#include "../messages.h"

#ifdef __cplusplus
extern "C" {
#endif

void fast_loop_update();

void controller_set_param(PIDParam *pid_param);
float controller_step(float desired, float measured);

#ifdef __cplusplus
}
#endif

#endif
