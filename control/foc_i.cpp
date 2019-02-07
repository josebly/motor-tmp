#include "foc.h"
#include "control_fun.h"
#include "foc_i.h"

static FOC foc_;
static PIDController controller_;

void foc_update() {
    foc_.update();
}

void foc_set_command(FOCCommand *foc_command) {
    foc_.set_command(*foc_command);
}

void foc_get_status(FOCStatus *foc_status) {
    foc_.get_status(foc_status);
}

void foc_set_param(FOCParam *foc_param) {
    foc_.set_param(*foc_param);
}

void controller_set_param(PIDParam *pid_param) {
    controller_.set_param(*pid_param);
}

float controller_step(float desired, float measured) {
    return controller_.step(desired, measured);
}