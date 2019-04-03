
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
    void voltage_mode();
    void set_param(const FastLoopParam &fast_loop_param); 
    void get_status(FastLoopStatus *fast_loop_status);
    void zero_current_sensors();
 private:
    FastLoopParam param_;
    FOC *foc_;
    PWM &pwm_;
    enum {CURRENT_MODE, PHASE_LOCK_MODE, VOLTAGE_MODE} mode_ = CURRENT_MODE;

    int32_t motor_enc;
    int32_t last_motor_enc=0;
    float motor_position_ = 0;
    float motor_velocity=0;
    float motor_velocity_filtered=0;
    float alpha=0.001;
    float phase_mode_ = 1;    // 1: standard or 2: two wires switched
    float motor_mechanical_position_ = 0;

    float iq_des = 0;
    float id_des = 0;
    float iq_des_gain_ = 1;
    uint16_t adc1, adc2, adc3;
    FOCCommand foc_command_ = {};

    int32_t motor_index_pos_;
    int32_t motor_electrical_zero_pos_;
    float inv_motor_encoder_cpr_;
    int32_t frequency_hz_ = 100000;
    float ia_bias_ = 0;
    float ib_bias_ = 0;
    float ic_bias_ = 0;
    float alpha_zero_ = 0.001;
    float v_bus_ = 12;
};