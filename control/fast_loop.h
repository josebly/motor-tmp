
#include <cstdint>
#include "../messages.h"

class FOC;
class PWM;

class FastLoop {
 public:
    FastLoop(PWM &pwm);
    ~FastLoop();
    void update();
    void maintenance();
    void set_id_des(float id) { id_des = id; }
    void set_iq_des(float iq) { iq_des = iq; }
    void phase_lock_mode(float id);
    void current_mode();
    void set_param(const FastLoopParam &fast_loop_param); 
    void get_status(FastLoopStatus *fast_loop_status);
 private:
    FastLoopParam param_;
    FOC *foc_;
    PWM &pwm_;

    int32_t motor_enc;
    int32_t last_motor_enc=0;
    float motor_velocity=0;
    float motor_velocity_filtered=0;
    float alpha=0.001;
    float motor_encoder_dir = -1;
    float motor_mechanical_position_ = 0;

    float adc1_offset = 1980;
    float adc1_gain = 3.3/4096/(.007*20);  // V/count * A/Vr / Vo/Vr

    float iq_des = 0;
    float id_des = 0;
    uint16_t adc1, adc2, adc3;

    int32_t motor_index_pos_;
    int32_t motor_electrical_zero_pos_;
    float inv_motor_encoder_cpr_;
};