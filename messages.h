
#ifndef MOTOR_MESSAGES_H
#define MOTOR_MESSAGES_H

#include <stdint.h>

#define MAX_DATA_LENGTH 120 // bytes
typedef int32_t mcu_time;

typedef struct {
    float kp;
    float ki;
    float ki_limit;
    float command_max;
} PIParam;

typedef struct {
    float kp;
    float ki;
    float ki_limit;
    float kd;
    float command_max;
} PIDParam;

typedef struct {
    PIParam pi_d;
    PIParam pi_q;
    float current_filter_frequency_hz;
    float num_poles;
} FOCParam;

#define COGGING_TABLE_SIZE 1024  // must be multiple of 2
typedef struct {
    int32_t pwm_frequency;
    float adc1_offset, adc2_offset, adc3_offset;
    float adc1_gain, adc2_gain, adc3_gain;
    FOCParam foc_param;
    uint8_t phase_mode;
    struct {
        float index_electrical_offset_pos;
        uint8_t use_index_electrical_offset_pos;
        uint32_t cpr;
        float dir;
    } motor_encoder;
    struct {
        float table[COGGING_TABLE_SIZE];
        float gain;
    } cogging;
    float vbus_gain;
} FastLoopParam;

enum MainControlMode {OPEN, BRAKE, NORMAL_CONTROL};
typedef struct {
    int32_t update_frequency;
    PIDParam controller_param;
    float torque_gain, torque_bias;
    float kt;
    float gear_ratio;
} MainLoopParam;

typedef struct {
    uint8_t do_phase_lock;          // 1: yes, 0: no
    float phase_lock_current;       // current in A
    float phase_lock_duration;      // duration in seconds
    enum MainControlMode startup_mode;
} StartupParam;

typedef struct {
    FastLoopParam fast_loop_param;
    MainLoopParam main_loop_param;
    StartupParam startup_param;
    char name[64];
} Param;

typedef struct {
    struct { float i_d, i_q; } desired;
    struct { float i_a, i_b, i_c, motor_encoder; } measured;
} FOCCommand;

typedef struct {
    struct {
        float i_d, i_q;
    } desired;
    struct {
        float position;
        float i_d, i_q;
    } measured;
    struct { float v_a, v_b, v_c, v_d, v_q; } command;
} FOCStatus;



typedef struct {
    float position, velocity;
    float v_abc[3];
    FOCStatus foc_status;
} MotorStatus;

typedef struct {
    mcu_time timestamp;
    FOCStatus foc_status;
    struct {
        int32_t raw;
        float position;
        float velocity;
    } motor_position;
    float motor_mechanical_position;
    FOCCommand foc_command;
} FastLoopStatus;

typedef struct {
    float torque;
} MainLoopStatus;

typedef struct {
    uint16_t type;       ///< \sa CommandType
    uint8_t data[MAX_DATA_LENGTH - 2];
} MotorCommand;

enum MessageType {
    MOTOR_COMMAND = 1,
};

typedef struct {
    uint16_t length;
    uint16_t type;      ///< \sa MessageType
    uint8_t data[MAX_DATA_LENGTH];
} Message;


// Internal messages
typedef struct {
    int reserved;
} SystemUpdateCommand;

typedef struct {
    int reserved;
} SystemUpdateParam;

typedef struct {
    int reserved;
} SystemUpdateStatus;

typedef struct {
    int reserved;
} MainControlCommand;

typedef struct {
    int reserved;
} MainControlParam;

typedef struct {
    MotorStatus motor_status;
} MainControlStatus;

typedef struct {
    FOCCommand command;
} FOCControlCommand;

typedef struct {
    FOCParam param;
} FOCControlParam;

typedef struct {
    FOCStatus status;
} FOCControlStatus;

typedef struct {
    int reserved;
} SimulatorCommand;

typedef struct {
    int reserved;
} SimulatorParam;

typedef struct {
    MotorStatus motor_status;
} SimulatorStatus;

typedef struct {
    mcu_time timestamp;
    mcu_time timestamp_received;
    float motor_position;
    float iq;
    float motor_raw_position;
} SendData;

typedef struct {
    mcu_time timestamp;
    uint8_t mode_desired;
    float current_desired;
    float position_desired;
} ReceiveData;

#endif