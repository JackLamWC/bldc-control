#include "foc.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdint.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// Mathematical constants for transforms (optimized for speed)
#define SQRT_3_BY_2 0.866025403784f  // sqrt(3)/2 as float
#define TWO_BY_3    0.666666666667f  // 2/3

// Fast sin/cos lookup table (256 entries for 0-2π)
#define SINCOS_TABLE_SIZE 256
#define SINCOS_SCALE (SINCOS_TABLE_SIZE / (2.0f * M_PI))

static const float sin_table[SINCOS_TABLE_SIZE] = {
    0.000000f, 0.024541f, 0.049068f, 0.073565f, 0.098017f, 0.122411f, 0.146730f, 0.170962f,
    0.195090f, 0.219101f, 0.242980f, 0.266713f, 0.290285f, 0.313682f, 0.336890f, 0.359895f,
    0.382683f, 0.405241f, 0.427555f, 0.449611f, 0.471397f, 0.492898f, 0.514103f, 0.534998f,
    0.555570f, 0.575808f, 0.595699f, 0.615232f, 0.634393f, 0.653173f, 0.671559f, 0.689541f,
    0.707107f, 0.724247f, 0.740951f, 0.757209f, 0.773010f, 0.788346f, 0.803208f, 0.817585f,
    0.831470f, 0.844854f, 0.857729f, 0.870087f, 0.881921f, 0.893224f, 0.903989f, 0.914210f,
    0.923880f, 0.932993f, 0.941544f, 0.949528f, 0.956940f, 0.963776f, 0.970031f, 0.975702f,
    0.980785f, 0.985278f, 0.989177f, 0.992480f, 0.995185f, 0.997290f, 0.998795f, 0.999699f,
    1.000000f, 0.999699f, 0.998795f, 0.997290f, 0.995185f, 0.992480f, 0.989177f, 0.985278f,
    0.980785f, 0.975702f, 0.970031f, 0.963776f, 0.956940f, 0.949528f, 0.941544f, 0.932993f,
    0.923880f, 0.914210f, 0.903989f, 0.893224f, 0.881921f, 0.870087f, 0.857729f, 0.844854f,
    0.831470f, 0.817585f, 0.803208f, 0.788346f, 0.773010f, 0.757209f, 0.740951f, 0.724247f,
    0.707107f, 0.689541f, 0.671559f, 0.653173f, 0.634393f, 0.615232f, 0.595699f, 0.575808f,
    0.555570f, 0.534998f, 0.514103f, 0.492898f, 0.471397f, 0.449611f, 0.427555f, 0.405241f,
    0.382683f, 0.359895f, 0.336890f, 0.313682f, 0.290285f, 0.266713f, 0.242980f, 0.219101f,
    0.195090f, 0.170962f, 0.146730f, 0.122411f, 0.098017f, 0.073565f, 0.049068f, 0.024541f,
    0.000000f, -0.024541f, -0.049068f, -0.073565f, -0.098017f, -0.122411f, -0.146730f, -0.170962f,
    -0.195090f, -0.219101f, -0.242980f, -0.266713f, -0.290285f, -0.313682f, -0.336890f, -0.359895f,
    -0.382683f, -0.405241f, -0.427555f, -0.449611f, -0.471397f, -0.492898f, -0.514103f, -0.534998f,
    -0.555570f, -0.575808f, -0.595699f, -0.615232f, -0.634393f, -0.653173f, -0.671559f, -0.689541f,
    -0.707107f, -0.724247f, -0.740951f, -0.757209f, -0.773010f, -0.788346f, -0.803208f, -0.817585f,
    -0.831470f, -0.844854f, -0.857729f, -0.870087f, -0.881921f, -0.893224f, -0.903989f, -0.914210f,
    -0.923880f, -0.932993f, -0.941544f, -0.949528f, -0.956940f, -0.963776f, -0.970031f, -0.975702f,
    -0.980785f, -0.985278f, -0.989177f, -0.992480f, -0.995185f, -0.997290f, -0.998795f, -0.999699f,
    -1.000000f, -0.999699f, -0.998795f, -0.997290f, -0.995185f, -0.992480f, -0.989177f, -0.985278f,
    -0.980785f, -0.975702f, -0.970031f, -0.963776f, -0.956940f, -0.949528f, -0.941544f, -0.932993f,
    -0.923880f, -0.914210f, -0.903989f, -0.893224f, -0.881921f, -0.870087f, -0.857729f, -0.844854f,
    -0.831470f, -0.817585f, -0.803208f, -0.788346f, -0.773010f, -0.757209f, -0.740951f, -0.724247f,
    -0.707107f, -0.689541f, -0.671559f, -0.653173f, -0.634393f, -0.615232f, -0.595699f, -0.575808f,
    -0.555570f, -0.534998f, -0.514103f, -0.492898f, -0.471397f, -0.449611f, -0.427555f, -0.405241f,
    -0.382683f, -0.359895f, -0.336890f, -0.313682f, -0.290285f, -0.266713f, -0.242980f, -0.219101f,
    -0.195090f, -0.170962f, -0.146730f, -0.122411f, -0.098017f, -0.073565f, -0.049068f, -0.024541f
};

// Fast sin/cos lookup functions
static inline float fast_sin(float angle) {
    // Normalize angle to [0, 2π)
    while (angle < 0.0f) angle += 2.0f * M_PI;
    while (angle >= 2.0f * M_PI) angle -= 2.0f * M_PI;
    
    int index = (int)(angle * SINCOS_SCALE) & (SINCOS_TABLE_SIZE - 1);
    return sin_table[index];
}

static inline float fast_cos(float angle) {
    // cos(x) = sin(x + π/2)
    return fast_sin(angle + M_PI * 0.5f);
}

// Global PID controllers for d and q axis
static pid_controller_t d_pid;
static pid_controller_t q_pid;

// Global function pointer for hardware abstraction
foc_set_pwm_func_t foc_set_pwm = NULL;

// Debug variables for d-axis reference override
static bool d_ref_override_enabled = false;
static float d_ref_override_value = 0.0f;

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

// New function for angular PID control that accepts pre-calculated error
float pid_update_with_error(pid_controller_t *pid, float error) {
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

void foc_init(uint16_t frequency_hz, foc_set_pwm_func_t set_pwm_func) {
    // Calculate dt from frequency
    float dt = 1.0f / (float)frequency_hz;
    
    // Initialize PID controllers
    // Physical phase voltage range is approximately ±Vdc/2 with the current PWM mapping
    pid_init(&d_pid, 0.3f, 0.0f, 0.00f, dt, -FOC_BATTERY_VOLTAGE * 0.5f, FOC_BATTERY_VOLTAGE * 0.5f);
    pid_init(&q_pid, 1.0f, 0.0f, 0.00f, dt, -FOC_BATTERY_VOLTAGE * 0.5f, FOC_BATTERY_VOLTAGE * 0.5f);
    
    // Set hardware function pointer
    foc_set_pwm = set_pwm_func;
}

void foc_clarke_transform(float ia, float ib, float ic, float *alpha, float *beta) {
    *alpha = TWO_BY_3 * (ia - 0.5f * ib - 0.5f * ic);
    *beta = TWO_BY_3 * (SQRT_3_BY_2 * (ib - ic));
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
void foc_update(float theta, float q_ref, float ia_volts, float ib_volts, float ic_volts) {
    float alpha, beta;
    float v_alpha, v_beta;
    float d, q;
    float d_pid_output, q_pid_output;
    float va, vb, vc;
    
    // For d-axis reference (usually 0 for surface mounted PMSM)
    float d_ref = d_ref_override_enabled ? d_ref_override_value : 0.0f;

    // Check if function pointer is set
    if (foc_set_pwm == NULL) {
        return; // Hardware function not initialized
    }

    // Convert voltage readings to actual current values
    float ia_f = FOC_CURRENT_GET_CURRENT_F(ia_volts);
    float ib_f = FOC_CURRENT_GET_CURRENT_F(ib_volts);
    float ic_f = FOC_CURRENT_GET_CURRENT_F(ic_volts);

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
    // duty = (0.5 + v_phase / Vdc) * 10000
    int32_t duty_a_raw = (int32_t)((6.0f + va / FOC_BATTERY_VOLTAGE) * 10000.0f);
    int32_t duty_b_raw = (int32_t)((6.0f + vb / FOC_BATTERY_VOLTAGE) * 10000.0f);
    int32_t duty_c_raw = (int32_t)((6.0f + vc / FOC_BATTERY_VOLTAGE) * 10000.0f);

    // Clamp to valid PWM range [0, 10000]
    uint16_t duty_a_pwm = (duty_a_raw < 0) ? 0 : ((duty_a_raw > 10000) ? 10000 : (uint16_t)duty_a_raw);
    uint16_t duty_b_pwm = (duty_b_raw < 0) ? 0 : ((duty_b_raw > 10000) ? 10000 : (uint16_t)duty_b_raw);
    uint16_t duty_c_pwm = (duty_c_raw < 0) ? 0 : ((duty_c_raw > 10000) ? 10000 : (uint16_t)duty_c_raw);
    
    // Apply duty cycles using the function pointer
    foc_set_pwm(duty_a_pwm, duty_b_pwm, duty_c_pwm);
}

// Optimized FOC function with all transforms inlined and lookup tables
void  foc_update_optimized(float theta, float q_ref, float ia_volts, float ib_volts, float ic_volts) {
    // Check if function pointer is set
    if (foc_set_pwm == NULL) {
        return; // Hardware function not initialized
    }

    // Timing measurements using DWT cycle counter
    extern volatile uint32_t *DWT_CYCCNT;
    uint32_t start_cycles = *((volatile uint32_t *)0xE0001004); // DWT->CYCCNT
    uint32_t checkpoint;

    // === STEP 1: Current Conversion (optimized) ===
    float ia_f = FOC_CURRENT_GET_CURRENT_F(ia_volts);
    float ib_f = FOC_CURRENT_GET_CURRENT_F(ib_volts);
    float ic_f = FOC_CURRENT_GET_CURRENT_F(ic_volts);

    checkpoint = *((volatile uint32_t *)0xE0001004);
    foc_timing_current_conversion = checkpoint - start_cycles;

    // === STEP 2: Clarke Transform (inlined) ===
    float alpha = TWO_BY_3 * (ia_f - 0.5f * ib_f - 0.5f * ic_f);
    float beta = TWO_BY_3 * (SQRT_3_BY_2 * (ib_f - ic_f));

    checkpoint = *((volatile uint32_t *)0xE0001004);
    foc_timing_clarke_transform = checkpoint - start_cycles - foc_timing_current_conversion;

    // === STEP 3: Park Transform (inlined with hardware FPU) ===
    float cos_theta = fast_cos(theta);
    float sin_theta = fast_sin(theta);
    float d = alpha * cos_theta + beta * sin_theta;
    float q = -alpha * sin_theta + beta * cos_theta;

    checkpoint = *((volatile uint32_t *)0xE0001004);
    foc_timing_park_transform = checkpoint - start_cycles - foc_timing_current_conversion - foc_timing_clarke_transform;

    // === STEP 4: PID Controllers ===
    float d_ref = d_ref_override_enabled ? d_ref_override_value : 0.0f;
    float d_pid_output = pid_update(&d_pid, d_ref, d);
    
    float q_ref_scaled = q_ref * FOC_MAX_CURRENT;
    float q_pid_output = pid_update(&q_pid, q_ref_scaled, q);

    checkpoint = *((volatile uint32_t *)0xE0001004);
    foc_timing_pid_controllers = checkpoint - start_cycles - foc_timing_current_conversion - foc_timing_clarke_transform - foc_timing_park_transform;

    // === STEP 5: Inverse Park Transform (inlined, reuse sin/cos) ===
    float v_alpha = d_pid_output * cos_theta - q_pid_output * sin_theta;
    float v_beta = d_pid_output * sin_theta + q_pid_output * cos_theta;

    checkpoint = *((volatile uint32_t *)0xE0001004);
    foc_timing_inverse_park = checkpoint - start_cycles - foc_timing_current_conversion - foc_timing_clarke_transform - foc_timing_park_transform - foc_timing_pid_controllers;

    // === STEP 6: Inverse Clarke Transform (inlined) ===
    float va = v_alpha;
    float vb = -0.5f * v_alpha + SQRT_3_BY_2 * v_beta;
    float vc = -0.5f * v_alpha - SQRT_3_BY_2 * v_beta;

    checkpoint = *((volatile uint32_t *)0xE0001004);
    foc_timing_inverse_clarke = checkpoint - start_cycles - foc_timing_current_conversion - foc_timing_clarke_transform - foc_timing_park_transform - foc_timing_pid_controllers - foc_timing_inverse_park;
    
    // === STEP 7: PWM Conversion (optimized) ===
    const float pwm_scale = 10000.0f / FOC_BATTERY_VOLTAGE;
    const float pwm_offset = 5000.0f;
    
    // Fast float-to-int conversion with optimized clamping
    int32_t duty_a_raw = (int32_t)(va * pwm_scale + pwm_offset);
    int32_t duty_b_raw = (int32_t)(vb * pwm_scale + pwm_offset);
    int32_t duty_c_raw = (int32_t)(vc * pwm_scale + pwm_offset);
    
    // Branchless clamping for better performance
    uint16_t duty_a_pwm = (duty_a_raw < 0) ? 0 : ((duty_a_raw > 10000) ? 10000 : (uint16_t)duty_a_raw);
    uint16_t duty_b_pwm = (duty_b_raw < 0) ? 0 : ((duty_b_raw > 10000) ? 10000 : (uint16_t)duty_b_raw);
    uint16_t duty_c_pwm = (duty_c_raw < 0) ? 0 : ((duty_c_raw > 10000) ? 10000 : (uint16_t)duty_c_raw);
    
    // Apply duty cycles
    foc_set_pwm(duty_a_pwm, duty_b_pwm, duty_c_pwm);

    checkpoint = *((volatile uint32_t *)0xE0001004);
    foc_timing_pwm_conversion = checkpoint - start_cycles - foc_timing_current_conversion - foc_timing_clarke_transform - foc_timing_park_transform - foc_timing_pid_controllers - foc_timing_inverse_park - foc_timing_inverse_clarke;
}

// Debug/testing functions for d-axis reference override
void foc_set_d_ref_override(float d_ref_override) {
    d_ref_override_enabled = true;
    d_ref_override_value = d_ref_override;
}

void foc_clear_d_ref_override(void) {
    d_ref_override_enabled = false;
    d_ref_override_value = 0.0f;
}

// PID parameter access functions
void foc_set_d_pid_params(float kp, float ki, float kd) {
    d_pid.kp = kp;
    d_pid.ki = ki;
    d_pid.kd = kd;
}

void foc_set_q_pid_params(float kp, float ki, float kd) {
    q_pid.kp = kp;
    q_pid.ki = ki;
    q_pid.kd = kd;
}

void foc_get_d_pid_params(float *kp, float *ki, float *kd) {
    *kp = d_pid.kp;
    *ki = d_pid.ki;
    *kd = d_pid.kd;
}

void foc_get_q_pid_params(float *kp, float *ki, float *kd) {
    *kp = q_pid.kp;
    *ki = q_pid.ki;
    *kd = q_pid.kd;
}

void foc_reset_pid_controllers(void) {
    pid_reset(&d_pid);
    pid_reset(&q_pid);
}