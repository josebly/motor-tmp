
#include "main_loop.h"
#include "../peripheral/led.h"
#include "control_fun.h"

#include "../Src/pin_config.h"
#include <cmath>
#include "stm32f4xx_hal.h"
#include "../communication/usb_communication.h"
#include "foc_i.h"

void MainLoop::init() {
    const PinConfig * const p = get_pin_config();
    led_ = new LED(p->red_reg, p->green_reg, p->blue_reg);
    controller_ = new PIDController;
    communication_ = new USBCommunication;
    communication_->init();
}

extern SPI_HandleTypeDef hspi2;
void MainLoop::update() {
    	static uint64_t count = 0;
  count++;
  ReceiveData receive_data;
  int count_received = communication_->receive_data(&receive_data);
  float dt = 0.0001;
  p_dot = -w*w*q;
  p += p_dot*dt;
  q_dot = p;
  q += q_dot*dt;

  // static float amp_sign = 1;
  // if (t % i_period == 0) {
  //   amp_sign *= -1;
  //   iq_des = iq_bias + amp_sign*iq_amp;
  // }


  pos_desired = a*q;
  if (count % 4 == 0) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi2, jl_torque_tx, jl_torque_rx, 9, 10);
    c1 = *((uint32_t *) &jl_torque_rx[1]);
    c2 = *((uint32_t *) &jl_torque_rx[5]);
    // hack for bad noise
    if (c1 != 0 && c2 != 0) {
      float tmp_torque = param_.torque_gain*((float) ((int32_t) (c1-c2)) / (c1+c2)) + param_.torque_bias;
      if (fabsf(tmp_torque) < 10) {
        // more hack
        torque = tmp_torque;
      }
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
  }
  
  fast_loop_get_status(&fast_loop_status);
  float motor_torque = param_.kt*.02 * fast_loop_status.foc_status.measured.i_q;
  float torque_out = torque + motor_torque;

  // virtual wall control
  if (fast_loop_status.motor_position.position/param_.gear_ratio > wall_position) {
    torque_desired = -kwall*(fast_loop_status.motor_position.position/param_.gear_ratio - wall_position);
    if (torque_desired < -wall_max_torque) {
      torque_desired = -wall_max_torque;
    }
  } else {
    torque_desired = 0;
  }

  switch (mode_) {
    case MainLoopParam::CURRENT:
      iq_des = torque_desired;
      break;
    case MainLoopParam::MOTOR_TORQUE:
      iq_des = torque_desired/(param_.gear_ratio*param_.kt);
      break;
    case MainLoopParam::JOINT_TORQUE:
    {
      float torque_des = controller_->step(torque_desired, torque_out);
      iq_des = torque_des/(param_.gear_ratio*param_.kt);
    }
      break;
    default:
      iq_des = 0;
      break;
  }
 // float torque_des = controller_->step(pos_desired, fast_loop_status.motor_position.position);
  
  fast_loop_set_iq_des(iq_des);
  SendData send_data;
  communication_->send_data(send_data);
    led_->update();
}

void MainLoop::set_param(MainLoopParam &param) {
    controller_->set_param(param.controller_param);
    param_ = param;
    mode_ = param.mode;
}

void MainLoop::get_status(MainLoopStatus * const main_loop_status) const {
  main_loop_status->torque = torque;
}