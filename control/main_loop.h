
#ifndef MAIN_LOOP_H
#define MAIN_LOOP_H

#include "../messages.h"
class LED;
class PIDController;
#include <cmath>

class MainLoop {
 public:
    void init();
    void update();
    void set_param(MainLoopParam &);
    void get_status(MainLoopStatus * const main_loop_status) const;
 private:
    MainLoopParam param_;
    LED *led_;
    PIDController *controller_;

    uint32_t c1, c2;

float torque;
float torque_desired = 0;

float w = 2*M_PI*.01;
float p_dot = 0;
float q = 1;
float p = 0;
float q_dot = 0;
float a = 100*M_PI;

float kwall = 0;
float wall_position = 0;
float wall_max_torque = .3;

float iq_bias = 2;
float iq_amp = 0;
float iq_des = 0;
int32_t i_period = 1000;
MainLoopParam::MainControlMode mode_;

FastLoopStatus fast_loop_status;


uint8_t jl_torque_tx[9] = {0x40};
uint8_t jl_torque_rx[9] = {0};


float pos_desired = 0;


inline uint16_t minu16(uint16_t a, uint16_t b) {
  if (a > b) {
    return b;
  } else {
    return a;
  }
}
};

#endif
