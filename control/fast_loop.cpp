
#include "fast_loop.h"
#include "foc.h"
#include <cmath>
#include "pwm.h"

FastLoop::FastLoop(PWM &pwm) : pwm_(pwm) {
    foc_ = new FOC;
}

FastLoop::~FastLoop() {
    delete foc_;
}

extern "C" {
int32_t motor_electrical_zero_pos = 0;
extern uint32_t t_start;
}

uint32_t t_diff3, t_diff4, t_diff5;
void FastLoop::update() {
    // get ADC
    // get encoder
    adc1 = ADC1->JDR1;
    adc2 = ADC2->JDR1;
    adc3 = ADC3->JDR1;
    motor_enc = TIM2->CNT;
    t_diff3 = TIM5->CNT - t_start;
    motor_velocity = (motor_enc-last_motor_enc)*(2*(float) M_PI/1024*100000);
    // motor_velocity_filtered = motor_velocity_filter.update(motor_velocity);
    motor_velocity_filtered = (1-alpha)*motor_velocity_filtered + alpha*motor_velocity;
    last_motor_enc = motor_enc;

    ITM->PORT[0].u32 = adc1;

    // output adc on dac for reference 
 //   hdac.Instance->DHR12R1 = adc1;

    // update FOC
    FOCCommand foc_command;
    foc_command.measured.i_a = adc1_gain*(adc1-adc1_offset);
    foc_command.measured.i_b = adc1_gain*(adc2-adc1_offset);
    foc_command.measured.i_c = adc1_gain*(adc3-adc1_offset);
    foc_command.measured.motor_encoder = motor_encoder_dir*(motor_enc - motor_electrical_zero_pos)*(2*(float) M_PI/1024);
    foc_command.desired.i_q = iq_des;
    foc_command.desired.i_d = id_des;
    
    foc_->set_command(foc_command);
    t_diff4 = TIM5->CNT - t_start;
    foc_->update();
    t_diff5 = TIM5->CNT - t_start;
    FOCStatus foc_status;
    foc_->get_status(&foc_status);

    pwm_.set_voltage(&foc_status.command.v_a);
    // set pwm
}

void FastLoop::set_param(const FastLoopParam &fast_loop_param) {
    foc_->set_param(fast_loop_param.foc_param);
}

void FastLoop::phase_lock_mode(float id) {
    motor_encoder_dir = 0;
    id_des = id;
}

void FastLoop::current_mode() {
    motor_encoder_dir = -1;
    id_des = 0;
}