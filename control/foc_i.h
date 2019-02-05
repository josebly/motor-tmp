#ifndef FOC_I_H
#define FOC_I_H

#include "../messages.h"

#ifdef __cplusplus
extern "C" {
#endif

void foc_update();
void foc_set_command(FOCCommand *foc_command);
void foc_get_status(FOCStatus *foc_status);
void foc_set_param(FOCParam *foc_param);


#ifdef __cplusplus
}
#endif

#endif
