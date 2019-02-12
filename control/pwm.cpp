
#include "pwm.h"

static float VtoPWM = 899/12;

void PWM::set_voltage(float v_abc[3]) {
    regs_.CCR1 = (v_abc[0] + 6)*VtoPWM;
    regs_.CCR2 = (v_abc[1] + 6)*VtoPWM;
    regs_.CCR3 = (v_abc[2] + 6)*VtoPWM;
}