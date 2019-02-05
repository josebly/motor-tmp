#include "foc.h"

#include "foc_i.h"

static FOC foc_;

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