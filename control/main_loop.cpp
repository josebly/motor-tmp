
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
  count_++;
  int count_received = communication_->receive_data(&receive_data_);
  if (count_received) {
    if (mode_ != static_cast<MainControlMode>(receive_data_.mode_desired)) {
      //stuff
    }
  }
  
  fast_loop_get_status(&fast_loop_status_);

  float iq_des = controller_->step(receive_data_.position_desired, fast_loop_status_.motor_position.position) + \
              receive_data_.current_desired;

  fast_loop_set_iq_des(iq_des);
  SendData send_data;
  send_data.iq = fast_loop_status_.foc_status.measured.i_q;
  send_data.timestamp_received = receive_data_.timestamp;
  send_data.timestamp = fast_loop_status_.timestamp;
  send_data.motor_mechanical_position = fast_loop_status_.motor_mechanical_position;
  send_data.motor_position = fast_loop_status_.motor_position.position;
  communication_->send_data(send_data);
  led_->update();
}

void MainLoop::set_param(MainLoopParam &param) {
    controller_->set_param(param.controller_param);
    param_ = param;
    mode_ = param.mode;
}

void MainLoop::get_status(MainLoopStatus * const main_loop_status) const {
  //main_loop_status->torque = torque;
}
