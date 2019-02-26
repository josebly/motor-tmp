
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

// called at fixed frequency in an interrupt
void FastLoop::update() {
    // get ADC
    // get encoder
    adc1 = ADC1->JDR1;
    adc2 = ADC2->JDR1;
    adc3 = ADC3->JDR1;
    motor_enc = TIM2->CNT;
    motor_velocity = (motor_enc-last_motor_enc)*(2*(float) M_PI/1024*100000);
    // motor_velocity_filtered = motor_velocity_filter.update(motor_velocity);
    motor_velocity_filtered = (1-alpha)*motor_velocity_filtered + alpha*motor_velocity;
    last_motor_enc = motor_enc;

    ITM->PORT[0].u32 = adc1;

    // output adc on dac for reference 
 //   hdac.Instance->DHR12R1 = adc1;

    // cogging compensation, interpolate in the table
    motor_mechanical_position_ = (motor_enc - motor_index_pos_); 
    float i_pos = motor_mechanical_position_*COGGING_TABLE_SIZE*inv_motor_encoder_cpr_;
    uint16_t i = (uint16_t) i_pos & (COGGING_TABLE_SIZE - 1);
    float ifrac = i_pos - i;
    // Note (i+1) & (COGGING_TABLE_SIZE-1) allows wrap around, requires COGGING_TABLE_SIZE is multiple of 2
    float iq_ff = param_.cogging.gain * (param_.cogging.table[i] + ifrac * (param_.cogging.table[(i+1) & (COGGING_TABLE_SIZE-1)] - param_.cogging.table[i]));

    // update FOC
    FOCCommand foc_command;
    foc_command.measured.i_a = adc1_gain*(adc1-adc1_offset);
    foc_command.measured.i_b = adc1_gain*(adc2-adc1_offset);
    foc_command.measured.i_c = adc1_gain*(adc3-adc1_offset);
    foc_command.measured.motor_encoder = motor_encoder_dir*(motor_enc - motor_electrical_zero_pos_)*(2*(float) M_PI/1024);
    foc_command.desired.i_q = iq_des + iq_ff;
    foc_command.desired.i_d = id_des;
    
    foc_->set_command(foc_command);
    foc_->update();
    FOCStatus foc_status;
    foc_->get_status(&foc_status);

    pwm_.set_voltage(&foc_status.command.v_a);
    // set pwm
}

// called at a slow frequency in a non interrupt
void FastLoop::maintenance() {
    if (TIM2->SR & TIM_SR_CC3IF) {
        // qep index received
        // TODO cleared by reading CCR3?
        motor_index_pos_ = TIM2->CCR3;
        if (param_.motor_encoder.use_index_electrical_offset_pos) {
          // motor_index_electrical_offset_pos is the value of an electrical zero minus the index position
          // motor_electrical_zero_pos is the offset to the initial encoder value
          motor_electrical_zero_pos_ = param_.motor_encoder.index_electrical_offset_pos + motor_index_pos_;
        }
      }
}

void FastLoop::set_param(const FastLoopParam &fast_loop_param) {
    foc_->set_param(fast_loop_param.foc_param);
    param_ = fast_loop_param;
    inv_motor_encoder_cpr_ = param_.motor_encoder.cpr != 0 ? 1.f/param_.motor_encoder.cpr : 0;
}

void FastLoop::phase_lock_mode(float id) {
    motor_encoder_dir = 0;
    id_des = id;
    motor_electrical_zero_pos_ = TIM2->CNT;
}

void FastLoop::current_mode() {
    motor_encoder_dir = -1;
    id_des = 0;
}

void FastLoop::get_status(FastLoopStatus *fast_loop_status) {
    foc_->get_status(&(fast_loop_status->foc_status));
    fast_loop_status->motor_mechanical_position = motor_mechanical_position_;
}