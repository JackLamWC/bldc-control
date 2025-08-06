#ifndef FOC_H
#define FOC_H

#define FOC_MAX_CURRENT 20.0f
#define FOC_BATTERY_VOLTAGE 12.0f

#define FOC_CURRENT_SENSING_VOLTAGE_OFFSET 0.5f
#define FOC_CURRENT_SENSING_VOLTAGE_MAX 3.3f
#define FOC_CURRENT_SENSING_R_SENSE 0.005
#define FOC_CURRENT_SENSING_GAIN 12.22f
#define FOC_CURRENT_GET_CURRENT_F(CSH_V) ((CSH_V - FOC_CURRENT_SENSING_VOLTAGE_OFFSET) / (FOC_CURRENT_SENSING_R_SENSE * FOC_CURRENT_SENSING_GAIN))

#include "ch.h"

// Function pointer types for hardware abstraction
typedef void (*foc_set_pwm_func_t)(uint16_t duty_a, uint16_t duty_b, uint16_t duty_c);

// PID Controller structure
typedef struct {
    float kp;           // Proportional gain
    float ki;           // Integral gain
    float kd;           // Derivative gain
    float prev_error;   // Previous error for derivative calculation
    float integral;     // Accumulated integral
    float output_min;   // Minimum output limit
    float output_max;   // Maximum output limit
    float dt;           // Sample time
} pid_controller_t;

// PID function declarations
void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float dt, float output_min, float output_max);
float pid_update(pid_controller_t *pid, float setpoint, float process_variable);
void pid_reset(pid_controller_t *pid);

// FOC function declarations
void foc_init(uint16_t frequency_hz, foc_set_pwm_func_t set_pwm_func);
void foc_clarke_transform(float ia, float ib, float ic, float *alpha, float *beta);
void foc_park_transform(float alpha, float beta, float theta, float *d, float *q);
void foc_get_q_pid_output(float q, float q_ref, float *q_pid_output);
void foc_get_d_pid_output(float d, float d_ref, float *d_pid_output);
void foc_inverse_park_transform(float d, float q, float theta, float *v_alpha, float *v_beta);
void foc_inverse_clarke_transform(float alpha, float beta, float *va, float *vb, float *vc);
void foc_update(float theta, float q_ref, float ia_volts, float ib_volts, float ic_volts);
#endif