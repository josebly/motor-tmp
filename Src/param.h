
#ifndef PARAM_H
#define PARAM_H

#include "../messages.h"

#ifdef __cplusplus
extern "C" {
#endif

struct Param {
    FastLoopParam fast_loop_param;
};

struct Param *param();
void init_param_from_flash();
void save_param_to_flash();

#ifdef __cplusplus
}
#endif

#endif