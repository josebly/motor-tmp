#ifndef MOTOR_FOC_H
#define MOTOR_FOC_H

#include "../messages.h"

class PIController;

class FOC {
public:
    FOC();
    ~FOC();

    void update();
    void set_command(const FOCCommand &command) { command_ = command; }
    void set_param(const FOCParam &param);
    void get_status(FOCStatus *status) const { *status = status_; }

private:
    uint16_t num_poles_ = 7;
    PIController *pi_id_, *pi_iq_;
    FOCCommand command_;
    FOCStatus status_;
 //   FOCParam param_;
};


#endif //MOTOR_FOC_H
