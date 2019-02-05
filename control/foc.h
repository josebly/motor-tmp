#ifndef MOTOR_FOC_H
#define MOTOR_FOC_H

#include <../periodic_communication.h>
#include "../messages.h"

// namespace hal {
//     class PWM;
//     class ADC;
// }

class PIController;

//class Encoder;

class FOC : public PeriodicCommunication<FOCCommand, FOCParam, FOCStatus> {
public:
    FOC(); //hal::PWM *pwm, const hal::ADC &adc, const Encoder &);
    ~FOC();
    //TODO:  maybe template these functions
    void update();
    virtual void set_param(FOCParam &param);

private:
 //   hal::PWM *pwm_;
 //   const hal::ADC &adc_;
 //   const Encoder &encoder_;
    uint16_t num_poles_ = 1;
    PIController *pi_id_, *pi_iq_;
};


#endif //MOTOR_FOC_H
