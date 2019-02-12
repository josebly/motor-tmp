
#include <cstdint>

class FOC;
class PWM;

class FastLoop {
 public:
    FastLoop(PWM &pwm);
    ~FastLoop();
    void update();
 private:
    FOC *foc_;
    PWM &pwm_;

    int32_t motor_enc;
    int32_t last_motor_enc=0;
    float motor_velocity=0;
    float motor_velocity_filtered=0;
    float alpha=0.001;

    float adc1_offset = 1980;
    float adc1_gain = 3.3/4096/(.007*20);  // V/count * A/Vr / Vo/Vr

    float motor_encoder_dir = -1;
    float iq_des = 0;
    uint16_t adc1, adc2, adc3;
};