#ifndef MOTOR_CONTROL_FUN_H
#define MOTOR_CONTROL_FUN_H

#include "../messages.h"
#include <cmath>

class Hysteresis {
 public:
    float step(float);
    void set_hysteresis(float);
    void set_value(float value) { value_ = value; }
 private:
    float value_ = 0;
    float hysteresis_ = 0;
};

class FirstOrderLowPassFilter {
public:
    FirstOrderLowPassFilter(float frequency_hz, float dt) {
        alpha_ = 2*M_PI*dt*frequency_hz/(2*M_PI*dt*frequency_hz + 1);
    }
    void init(float value) {
        value_ = value;
        last_value_ = value;
    }
    float update(float value) {
        value_ = alpha_*value + (1-alpha_)*last_value_;
        last_value_ = value_;
        return get_value();
    }
    float get_value() const { return value_; }
private:
    float value_ = 0, last_value_ = 0;
    float alpha_;
};

class PIController {
public:
    ~PIController() {}
    float step(float desired, float measured);
    void set_param(const PIParam &pi_param);
private:
    float kp_ = 0, ki_ = 0, ki_sum_ = 0, ki_limit_ = 0, command_max_ = 0;

};

class RateLimiter {
 public:
    void set_limit(float limit) { limit_ = limit; }
    float step(float value) {
        float out_value;
        if (value > (last_value_ + limit_)) {
            out_value = last_value_ + limit_;
        } else if (value < (last_value_ - limit_)) {
            out_value = last_value_ - limit_;
        } else {
            out_value = value;
        }
        last_value_ = out_value;
        return out_value;
    }
 private:
    float limit_ = 1;
    float last_value_ = 0;
};

class PIDController {
public:
    virtual ~PIDController() {}
    virtual float step(float desired, float velocity_desired, float measured);
    void set_param(const PIDParam &param);
private:
    float kp_ = 0, kd_ = 0, ki_ = 0, ki_sum_ = 0, ki_limit_ = 0, command_max_ = 0;
    float measured_last_ = 0;
    float last_desired_ = 0;
    Hysteresis hysteresis_;
    RateLimiter rate_limit_;
    FirstOrderLowPassFilter error_dot_filter_ = {200, 1.0/10000};
};

class PIDDeadbandController : public PIDController {
public:
    virtual ~PIDDeadbandController() {}
    virtual float step(float desired, float velocity_desired, float deadband, float measured);
};




#endif //MOTOR_CONTROL_FUN_H
