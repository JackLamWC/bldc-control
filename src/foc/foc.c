#include "foc.h"
#include <math.h>
#include <stdint.h>

#define SQRT_3_BY_2 0.86602540378443864676372317075294

// Global PID controllers for d and q axis
static pid_controller_t d_pid;
static pid_controller_t q_pid;

// Global function pointers for hardware abstraction
foc_get_currents_func_t foc_get_currents = NULL;
foc_set_pwm_func_t foc_set_pwm = NULL;

// PID Controller Functions
void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float dt, float output_min, float output_max) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
    pid->output_min = output_min;
    pid->output_max = output_max;
    pid->dt = dt;
}

float pid_update(pid_controller_t *pid, float setpoint, float process_variable) {
    float error = setpoint - process_variable;
    
    // Proportional term
    float proportional = pid->kp * error;
    
    // Integral term
    pid->integral += error * pid->dt;
    float integral = pid->ki * pid->integral;
    
    // Derivative term
    float derivative = pid->kd * (error - pid->prev_error) / pid->dt;
    pid->prev_error = error;
    
    // Calculate output
    float output = proportional + integral + derivative;
    
    // Apply output limits
    if (output > pid->output_max) {
        output = pid->output_max;
        // Anti-windup: prevent integral from growing when output is saturated
        if (pid->ki != 0.0f) {
            pid->integral = (output - proportional - derivative) / pid->ki;
        }
    } else if (output < pid->output_min) {
        output = pid->output_min;
        // Anti-windup: prevent integral from growing when output is saturated
        if (pid->ki != 0.0f) {
            pid->integral = (output - proportional - derivative) / pid->ki;
        }
    }
    
    return output;
}

void pid_reset(pid_controller_t *pid) {
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
}

void foc_init(float dt, foc_get_currents_func_t get_currents_func, foc_set_pwm_func_t set_pwm_func) {
    // Initialize PID controllers
    pid_init(&d_pid, 1.0f, 10.0f, 0.0f, dt, -FOC_BATTERY_VOLTAGE, FOC_BATTERY_VOLTAGE);
    pid_init(&q_pid, 1.0f, 10.0f, 0.0f, dt, -FOC_BATTERY_VOLTAGE, FOC_BATTERY_VOLTAGE);
    
    // Set hardware function pointers
    foc_get_currents = get_currents_func;
    foc_set_pwm = set_pwm_func;
}

void foc_clarke_transform(float ia, float ib, float ic, float *alpha, float *beta) {
    *alpha = ia + -0.5 * ib + -0.5 * ic;
    *beta = SQRT_3_BY_2 * (ib - ic);
}

void foc_park_transform(float alpha, float beta, float theta, float *d, float *q) {
    *d = alpha * cosf(theta) + beta * sinf(theta);
    *q = -alpha * sinf(theta) + beta * cosf(theta);
}

void foc_get_q_pid_output(float q, float q_ref, float *q_pid_output) {
    *q_pid_output = pid_update(&q_pid, q_ref, q);
}

void foc_get_d_pid_output(float d, float d_ref, float *d_pid_output) {
    *d_pid_output = pid_update(&d_pid, d_ref, d);
}

void foc_inverse_park_transform(float d, float q, float theta, float *v_alpha, float *v_beta) {
    *v_alpha = d * cosf(theta) - q * sinf(theta);
    *v_beta = d * sinf(theta) + q * cosf(theta);
}

void foc_inverse_clarke_transform(float alpha, float beta, float *va, float *vb, float *vc) {
    *va = alpha;
    *vb = -0.5 * alpha + SQRT_3_BY_2 * beta;
    *vc = -0.5 * alpha - SQRT_3_BY_2 * beta;
}

// Space Vector PWM implementation for better voltage utilization
void foc_svpwm_modulation(float va, float vb, float vc, float *duty_a, float *duty_b, float *duty_c) {
    // Find the maximum and minimum voltages
    float v_max = va;
    if (vb > v_max) v_max = vb;
    if (vc > v_max) v_max = vc;
    
    float v_min = va;
    if (vb < v_min) v_min = vb;
    if (vc < v_min) v_min = vc;
    
    // Calculate common mode offset for SVPWM
    float v_offset = -(v_max + v_min) / 2.0f;
    
    // Apply offset to center the voltages
    va += v_offset;
    vb += v_offset;
    vc += v_offset;
    
    // Normalize to battery voltage and convert to duty cycle
    *duty_a = (va / FOC_BATTERY_VOLTAGE + 1.0f) * 0.5f;
    *duty_b = (vb / FOC_BATTERY_VOLTAGE + 1.0f) * 0.5f;
    *duty_c = (vc / FOC_BATTERY_VOLTAGE + 1.0f) * 0.5f;
    
    // Clamp duty cycles to valid range [0, 1]
    if (*duty_a < 0.0f) *duty_a = 0.0f;
    if (*duty_a > 1.0f) *duty_a = 1.0f;
    if (*duty_b < 0.0f) *duty_b = 0.0f;
    if (*duty_b > 1.0f) *duty_b = 1.0f;
    if (*duty_c < 0.0f) *duty_c = 0.0f;
    if (*duty_c > 1.0f) *duty_c = 1.0f;
}

// q_ref should be normalized to -1 to 1
void foc_update(float theta, float q_ref) {
    uint16_t ia, ib, ic;
    float alpha, beta;
    float v_alpha, v_beta;
    float d, q;
    float d_pid_output, q_pid_output;
    float va, vb, vc;
    
    // For d-axis reference (usually 0 for surface mounted PMSM)
    float d_ref = 0.0f;

    // Check if function pointers are set
    if (foc_get_currents == NULL || foc_set_pwm == NULL) {
        return; // Hardware functions not initialized
    }

    // Get current readings from hardware
    foc_get_currents(&ia, &ib, &ic);

    // Convert ADC readings to actual current values
    float ia_f = FOC_CURRENT_GET_CURRENT_F(ia);
    float ib_f = FOC_CURRENT_GET_CURRENT_F(ib);
    float ic_f = FOC_CURRENT_GET_CURRENT_F(ic);

    // Clarke transform to get alpha and beta
    foc_clarke_transform(ia_f, ib_f, ic_f, &alpha, &beta);

    // Park transform to get d and q
    foc_park_transform(alpha, beta, theta, &d, &q);

    // Get d PID output (flux control)
    foc_get_d_pid_output(d, d_ref, &d_pid_output);

    q_ref = q_ref * FOC_MAX_CURRENT;
    foc_get_q_pid_output(q, q_ref, &q_pid_output);

    // Inverse Park transform to get alpha and beta voltages
    foc_inverse_park_transform(d_pid_output, q_pid_output, theta, &v_alpha, &v_beta);

    // Inverse Clarke transform to get va, vb, vc
    foc_inverse_clarke_transform(v_alpha, v_beta, &va, &vb, &vc);
    
    // Direct mapping of voltages to PWM range (0-10000)
    // Normalize voltages from [-FOC_BATTERY_VOLTAGE, +FOC_BATTERY_VOLTAGE] to [0, 10000]
    uint16_t duty_a_pwm = (uint16_t)((va / FOC_BATTERY_VOLTAGE + 1.0f) * 5000.0f);
    uint16_t duty_b_pwm = (uint16_t)((vb / FOC_BATTERY_VOLTAGE + 1.0f) * 5000.0f);
    uint16_t duty_c_pwm = (uint16_t)((vc / FOC_BATTERY_VOLTAGE + 1.0f) * 5000.0f);
    
    // Clamp to valid PWM range [0, 10000]
    if (duty_a_pwm > 10000) duty_a_pwm = 10000;
    if (duty_b_pwm > 10000) duty_b_pwm = 10000;
    if (duty_c_pwm > 10000) duty_c_pwm = 10000;
    
    // Apply duty cycles using the function pointer
    foc_set_pwm(duty_a_pwm, duty_b_pwm, duty_c_pwm);
}