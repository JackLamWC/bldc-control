#include "ch.h"
#include "hal.h"
#include "log.h"
#include "shell.h"
#include <stdlib.h>
#include <string.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include "chprintf.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define LINE_DRV_ENABLE             PAL_LINE(GPIOC, GPIOC_PIN8)
#define LINE_DRV_N_FAULT            PAL_LINE(GPIOC, GPIOC_PIN10) 
#define LINE_DRV_N_OCTW             PAL_LINE(GPIOC, GPIOC_PIN11)
#define LINE_DRV_M_OC               PAL_LINE(GPIOC, GPIOC_PIN12)
#define LINE_DRV_OC_ADJ             PAL_LINE(GPIOD, GPIOD_PIN2)
#define LINE_DRV_M_PWM              PAL_LINE(GPIOC, GPIOD_PIN9)
#define LINE_DRV_DEBUG              PAL_LINE(GPIOC, GPIOC_PIN5)
#define LINE_DRV_DEBUG2             PAL_LINE(GPIOC, GPIOC_PIN6)

// Motor configuration
#define MOTOR_POLE_PAIRS            7
// Motor configuration - encoder offset is adjustable at runtime
static float motor_offset_radians = -0.6963f;  // Default encoder offset
static int motor_direction = 1;               // Motor direction: +1 or -1

static uint32_t last_isr_cycles = 0;
static uint32_t debug_isr_duration_cycles = 0;


/*===========================================================================*/
/* ADC Data Structure and Mailbox                                           */
/*===========================================================================*/
typedef struct {
  uint32_t phase_current[3];
  uint32_t position_current;
  systime_t timestamp;
} adc_data_t;

// Mailbox and event system removed - FOC now runs directly in ISR

/*===========================================================================*/
/* Motor Control PWM                                                         */
/*===========================================================================*/
#define PWM_CLOCK_FREQ 84000000
#define PWM_FREQ 21000

#define LINE_PWM_PHASE_AH PAL_LINE(GPIOA, GPIOA_ARD_D7)
#define LINE_PWM_PHASE_AL PAL_LINE(GPIOA, GPIOA_ARD_D11)
#define LINE_PWM_PHASE_BH PAL_LINE(GPIOA, GPIOA_ARD_D8)
#define LINE_PWM_PHASE_BL PAL_LINE(GPIOB, GPIOB_ARD_A3)
#define LINE_PWM_PHASE_CH PAL_LINE(GPIOA, GPIOA_ARD_D2)
#define LINE_PWM_PHASE_CL PAL_LINE(GPIOB, GPIOB_PIN1)


static void pwm_init(void) {
  palSetLineMode(LINE_PWM_PHASE_AH, PAL_MODE_ALTERNATE(1));
  palSetLineMode(LINE_PWM_PHASE_BH, PAL_MODE_ALTERNATE(1));
  palSetLineMode(LINE_PWM_PHASE_CH, PAL_MODE_ALTERNATE(1));

  palSetLineMode(LINE_PWM_PHASE_AL, PAL_MODE_ALTERNATE(1));
  palSetLineMode(LINE_PWM_PHASE_BL, PAL_MODE_ALTERNATE(1));
  palSetLineMode(LINE_PWM_PHASE_CL, PAL_MODE_ALTERNATE(1));


  palClearLine(LINE_PWM_PHASE_AH);
  palClearLine(LINE_PWM_PHASE_BH);
  palClearLine(LINE_PWM_PHASE_CH);

  palClearLine(LINE_PWM_PHASE_AL);
  palClearLine(LINE_PWM_PHASE_BL);
  palClearLine(LINE_PWM_PHASE_CL);

  // Enable TIM1 clock
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

  // Set to center aligned mode
  TIM1->CR1 &= ~TIM_CR1_CMS_Msk;
  TIM1->CR1 |= TIM_CR1_CMS_1 | TIM_CR1_CMS_0;

  // Set the compare mode to PWM to channel 1
  TIM1->CCMR1 &= ~TIM_CCMR1_OC1M_Msk;
  TIM1->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE;

  // Set the compare mode to PWM to channel 2
  TIM1->CCMR1 &= ~TIM_CCMR1_OC2M_Msk;
  TIM1->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE;

  // Set the compare mode to PWM to channel 3
  TIM1->CCMR2 &= ~TIM_CCMR2_OC3M_Msk;
  TIM1->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3PE;

  // Set the compare mode to PWM to channel 4
  TIM1->CCMR2 &= ~TIM_CCMR2_OC4M_Msk;
  TIM1->CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4PE;

  // Set the prescaler to 0 for 84MHz clock
  TIM1->PSC = 0;

  // Set the period to 84MHz / (21 * 2)kHz = 4000 for center aligned mode
  TIM1->ARR = PWM_CLOCK_FREQ / (PWM_FREQ * 2);

  // Initialize the duty cycle to 0
  TIM1->CCR1 = 0;
  TIM1->CCR2 = 0;
  TIM1->CCR3 = 0;
  TIM1->CCR4 = 1; 

  // Enable the Output
  TIM1->CCER |=  TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;

  // Enable the complementary output
  TIM1->CCER |= TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE;

  // Set the polarity to active high
  TIM1->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC3P);

  // Set the complementary polarity to active high
  TIM1->CCER &= ~(TIM_CCER_CC1NP | TIM_CCER_CC2NP | TIM_CCER_CC3NP);

  // Configure the TIM1 as master
  TIM1->CR2 &= ~TIM_CR2_MMS_Msk;
  TIM1->CR2 |= TIM_CR2_MMS_2 | TIM_CR2_MMS_1 | TIM_CR2_MMS_0;

  // Enable the main output
  TIM1->BDTR |= TIM_BDTR_MOE;

  // Enable the TIM1 (master timer)
  TIM1->CR1 |= TIM_CR1_CEN;
}

static void pwm_stop(void) {
  // Disable the TIM1
  TIM1->CR1 &= ~TIM_CR1_CEN;
}

static void pwm_set_duty_cycle(uint8_t channel, uint16_t duty_cycle) {
  uint16_t duty_cycle_value = (duty_cycle * TIM1->ARR) / 10000.0f;
  if (channel == 0) {
    TIM1->CCR1 = duty_cycle_value;
  } else if (channel == 1) {
    TIM1->CCR2 = duty_cycle_value;
  } else if (channel == 2) {
    TIM1->CCR3 = duty_cycle_value;
  }
}



/*===========================================================================*/
/* FOC                                                                     */
/*===========================================================================*/
#include "foc.h"

static uint16_t next_duty_a_pwm = 0;
static uint16_t next_duty_b_pwm = 0;
static uint16_t next_duty_c_pwm = 0;

static void foc_set_pwm(uint16_t duty_a, uint16_t duty_b, uint16_t duty_c) {
  next_duty_a_pwm = duty_a * TIM1->ARR / 10000.0f;
  next_duty_b_pwm = duty_b * TIM1->ARR / 10000.0f;
  next_duty_c_pwm = duty_c * TIM1->ARR / 10000.0f;

  TIM1->CCR1 = next_duty_a_pwm;
  TIM1->CCR2 = next_duty_b_pwm;
  TIM1->CCR3 = next_duty_c_pwm;
}



/*===========================================================================*/
/* Angle Control                                                             */
/*===========================================================================*/

// AS5600 encoder configuration
#define AS5600_RESOLUTION 4096
#define AS5600_TO_RADIANS (2.0f * M_PI / AS5600_RESOLUTION)

// Angle control configuration (PID now runs in ADC ISR)

// Angle control state structure
typedef struct {
  pid_controller_t angle_pid;     // Reuse FOC PID structure
  float target_angle;             // Target angle in radians (accumulated, can be > 2π)
  float current_angle;            // Current angle in radians (accumulated, can be > 2π)
  float angle_error;              // For monitoring
  float torque_output;            // Torque output (current in Amperes, proportional to torque)
  bool enabled;                   // Control enabled flag
  
  // Revolution tracking for accumulated angle
  float prev_raw_angle;           // Previous raw encoder angle [-π, +π]
  int32_t revolution_count;       // Number of complete revolutions
  bool first_reading;             // Flag for first encoder reading
} angle_control_t;

// Global angle control instance
static angle_control_t angle_control = {0};

// Update encoder position from ADC data (called from ADC processing)
static void angle_control_update_position(uint16_t position_adc) {
  // Convert ADC reading to mechanical angle in radians
  float raw_angle = position_adc * AS5600_TO_RADIANS;
  
  // Wrap raw angle to [-π, +π] range
  while (raw_angle > M_PI) {
    raw_angle -= 2.0f * M_PI;
  }
  while (raw_angle <= -M_PI) {
    raw_angle += 2.0f * M_PI;
  }
  
  // Handle first reading - initialize without revolution counting
  if (angle_control.first_reading) {
    angle_control.prev_raw_angle = raw_angle;
    angle_control.current_angle = raw_angle;  // Start at current position
    angle_control.revolution_count = 0;
    angle_control.first_reading = false;
    return;
  }
  
  // Detect revolution crossings by checking for large angle jumps
  float angle_diff = raw_angle - angle_control.prev_raw_angle;
  
  // Handle wrap-around detection (crossing ±π boundary)
  if (angle_diff > M_PI) {
    // Crossed from +π to -π (negative revolution)
    angle_control.revolution_count--;
  } else if (angle_diff < -M_PI) {
    // Crossed from -π to +π (positive revolution)
    angle_control.revolution_count++;
  }
  
  // Calculate accumulated angle: revolutions + current position within revolution
  angle_control.current_angle = (float)angle_control.revolution_count * 2.0f * M_PI + raw_angle;
  
  // Store current raw angle for next iteration
  angle_control.prev_raw_angle = raw_angle;
}

// Initialize angle control system
static void angle_control_init(float kp, float ki, float kd, float max_torque) {
  // PID now runs in ADC ISR at PWM frequency (21kHz), so dt = 1/21000
  float dt_isr = 1.0f / (float)PWM_FREQ;
  
  // Initialize PID controller using FOC functions (output is torque via current in Amperes)
  pid_init(&angle_control.angle_pid, kp, ki, kd, dt_isr, -max_torque, max_torque);
  
  angle_control.target_angle = 0.0f;
  angle_control.current_angle = 0.0f;
  angle_control.angle_error = 0.0f;
  angle_control.torque_output = 0.0f;
  angle_control.enabled = false;
  
  // Initialize revolution tracking
  angle_control.prev_raw_angle = 0.0f;
  angle_control.revolution_count = 0;
  angle_control.first_reading = true;
}

// Set target angle
static void angle_control_set_target(float target_angle_rad) {
  angle_control.target_angle = target_angle_rad;
}

// Enable/disable angle control
static void angle_control_enable(bool enable) {
  if (enable && !angle_control.enabled) {
    // Reset PID when enabling
    pid_reset(&angle_control.angle_pid);
    // Current angle will be updated by ADC processing
  }
  angle_control.enabled = enable;
}

// Get current angle
static float angle_control_get_current_angle(void) {
  return angle_control.current_angle;
}

// Get angle error  
static float angle_control_get_error(void) {
  return angle_control.angle_error;
}

// Get torque output
static float angle_control_get_output(void) {
  return angle_control.torque_output;
}




/*===========================================================================*/
/* Current Sensing                                                           */
/*===========================================================================*/

#define LINE_ADC_PHASE_A  PAL_LINE(GPIOA, GPIOA_ARD_A0)
#define LINE_ADC_PHASE_B  PAL_LINE(GPIOA, GPIOA_ARD_A1)
#define LINE_ADC_PHASE_C  PAL_LINE(GPIOA, GPIOA_ARD_A2)
#define LINE_ADC_POSITION PAL_LINE(GPIOC, GPIOC_ARD_A4)

#define ADC_NUM_CHANNELS 4

#define ADC_CHANNEL_PHASE_A 0
#define ADC_CHANNEL_PHASE_B 1
#define ADC_CHANNEL_PHASE_C 4
#define ADC_CHANNEL_POSITION 11

uint32_t phase_current[3] = {0, 0, 0};
uint32_t position_current = 0;
uint32_t debug_counter = 0;        // ADC interrupt counter

// Current voltage readings (updated in ISR, displayed in shell)
static float ia_volts_last = 0.0f;
static float ib_volts_last = 0.0f;
static float ic_volts_last = 0.0f;

// Per-channel current sense offsets (in volts), default to nominal 1.65V
static float current_offset_a_volts = FOC_CURRENT_SENSING_VOLTAGE_OFFSET;
static float current_offset_b_volts = FOC_CURRENT_SENSING_VOLTAGE_OFFSET;
static float current_offset_c_volts = FOC_CURRENT_SENSING_VOLTAGE_OFFSET;

// Last computed phase currents in Amps (from ISR)
static float ia_amps_last = 0.0f;
static float ib_amps_last = 0.0f;
static float ic_amps_last = 0.0f;

// Per-channel polarity (+1 or -1). If your amplifier inverts, set to -1
static int current_sign_a = -1;
static int current_sign_b = +1;
static int current_sign_c = +1;

// Mapping from measured channels [A,B,C] to true order passed to FOC.
// Each entry is an index 0..2 selecting from [A,B,C]. Default is identity {0,1,2}.
static uint8_t current_map[3] = {2, 0, 1};

// Global variables for reference overrides
static bool q_ref_override_enabled = false;
static float q_ref_override_value = 0.0f;
static bool d_ref_override_enabled = false;
static float d_ref_override_value = 0.0f;

// Alignment mode to apply constant current in phase A (Method 1)
static bool align_mode_enabled = false;           // When true, bypass FOC and output fixed duties
static uint16_t align_duty_a = 6500;             // 0..10000 scale, default A high
static uint16_t align_duty_b = 3500;             // 0..10000 scale, default B low
static uint16_t align_duty_c = 5000;             // 0..10000 scale, neutral C

// ISR profiling variables
static uint32_t timing_adc_read = 0;
static uint32_t timing_conversions = 0;
static uint32_t timing_angle_update = 0;
static uint32_t timing_foc_computation = 0;

// FOC detailed profiling variables (accessible from foc.c)
uint32_t foc_timing_current_conversion = 0;
uint32_t foc_timing_clarke_transform = 0;
uint32_t foc_timing_park_transform = 0;
uint32_t foc_timing_pid_controllers = 0;
uint32_t foc_timing_inverse_park = 0;
uint32_t foc_timing_inverse_clarke = 0;
uint32_t foc_timing_pwm_conversion = 0;

// Single ADC data structure for debugging/shell commands  
static adc_data_t current_adc_data;

CH_IRQ_HANDLER(STM32_ADC_HANDLER) {
  if (ADC1->SR & ADC_SR_JEOC) {
    uint32_t start_cycles = DWT->CYCCNT;
    uint32_t checkpoint;
    ADC1->SR &= ~ADC_SR_JEOC; // Clear interrupt flag

    if(palReadLine(LINE_DRV_DEBUG2)) {
      palClearLine(LINE_DRV_DEBUG2);
    } else {
      palSetLine(LINE_DRV_DEBUG2);
    }
    
    // === SECTION 1: ADC Reading ===
    uint16_t phase_a_current = ADC1->JDR1 & ADC_JDR1_JDATA_Msk;
    uint16_t phase_b_current = ADC1->JDR2 & ADC_JDR2_JDATA_Msk;
    uint16_t phase_c_current = ADC1->JDR3 & ADC_JDR3_JDATA_Msk;
    uint16_t position = ADC1->JDR4 & ADC_JDR4_JDATA_Msk;
    
    // Update current_adc_data for shell commands
    current_adc_data.phase_current[0] = phase_a_current;
    current_adc_data.phase_current[1] = phase_b_current;
    current_adc_data.phase_current[2] = phase_c_current;
    current_adc_data.position_current = position;
    current_adc_data.timestamp = chVTGetSystemTimeX();
    
    checkpoint = DWT->CYCCNT;
    timing_adc_read = checkpoint - start_cycles;
    
    // === SECTION 2: Angle Control Update & PID ===
    // Update mechanical angle position
    angle_control_update_position(position);
    
    // Update angle PID directly in ISR for minimal latency (if enabled)
    if (angle_control.enabled) {
        // Calculate error for accumulated angles (no wrap-around, preserve multi-revolution errors)
        // Apply motor direction correction for proper feedback polarity
        float raw_error = angle_control.target_angle - angle_control.current_angle;
        angle_control.angle_error = raw_error * motor_direction;
        
        // Update PID controller using accumulated angle error
        angle_control.torque_output = pid_update_with_error(&angle_control.angle_pid, 
                                                           angle_control.angle_error);
    } else {
        // When disabled, output zero torque
        angle_control.torque_output = 0.0f;
    }
    
    checkpoint = DWT->CYCCNT;
    timing_angle_update = checkpoint - start_cycles - timing_adc_read;
    
    // === SECTION 3: ADC to Float Conversions ===
    float ia_volts = (phase_a_current * 3.3f) / 4095.0f;
    float ib_volts = (phase_b_current * 3.3f) / 4095.0f;
    float ic_volts = (phase_c_current * 3.3f) / 4095.0f;
    
    // Compute instantaneous phase currents (Amps) using per-channel offsets
    ia_amps_last = current_sign_a * ((ia_volts - current_offset_a_volts) * FOC_CURRENT_SENSING_INV_GAIN);
    ib_amps_last = current_sign_b * ((ib_volts - current_offset_b_volts) * FOC_CURRENT_SENSING_INV_GAIN);
    ic_amps_last = current_sign_c * ((ic_volts - current_offset_c_volts) * FOC_CURRENT_SENSING_INV_GAIN);
    
    // Store raw voltage readings for shell commands
    ia_volts_last = ia_volts;
    ib_volts_last = ib_volts;
    ic_volts_last = ic_volts;
    
    // Convert AS5600 position (0-4095) to mechanical radians, wrap to [-π, +π]
    float mechanical_angle = position * AS5600_TO_RADIANS;
    while (mechanical_angle > M_PI) {
        mechanical_angle -= 2.0f * M_PI;
    }
    while (mechanical_angle <= -M_PI) {
        mechanical_angle += 2.0f * M_PI;
    }
    
    // Convert mechanical angle to electrical angle for FOC
    float electrical_angle = (mechanical_angle + motor_offset_radians) * MOTOR_POLE_PAIRS;
    
    // Efficient wrap to [-π, +π] range (optimized for ISR performance)
    // For 7 pole pairs, electrical_angle will be in range [-7π, +7π], so we only need a few iterations
    while (electrical_angle > M_PI) {
        electrical_angle -= 2.0f * M_PI;
    }
    while (electrical_angle <= -M_PI) {
        electrical_angle += 2.0f * M_PI;
    }
    
    // Get torque reference normalized to [-1, 1]
    float q_ref_normalized;
    if (q_ref_override_enabled) {
        q_ref_normalized = q_ref_override_value;
    } else {
        q_ref_normalized = angle_control.torque_output / FOC_MAX_CURRENT;
    }
    
    checkpoint = DWT->CYCCNT;
    timing_conversions = checkpoint - start_cycles - timing_adc_read - timing_angle_update;
    
    // === SECTION 4: FOC Computation or Manual Alignment ===
    if (align_mode_enabled) {
      // Apply fixed duty to align rotor with alpha-axis (phase A bias)
      foc_set_pwm(align_duty_a, align_duty_b, align_duty_c);
    } else {
      // Adjust the voltages so that downstream conversion using a single 1.65V offset
      // yields the desired per-channel offset-compensated currents.
      // Vadj = Vmeas - (offset_ch - 1.65)
      // Build adjusted, sign-corrected, offset-compensated voltages in A,B,C order
      float v_adj[3];
      v_adj[0] = (current_sign_a > 0 ? 1.0f : -1.0f) * (ia_volts - current_offset_a_volts) + FOC_CURRENT_SENSING_VOLTAGE_OFFSET;
      v_adj[1] = (current_sign_b > 0 ? 1.0f : -1.0f) * (ib_volts - current_offset_b_volts) + FOC_CURRENT_SENSING_VOLTAGE_OFFSET;
      v_adj[2] = (current_sign_c > 0 ? 1.0f : -1.0f) * (ic_volts - current_offset_c_volts) + FOC_CURRENT_SENSING_VOLTAGE_OFFSET;

      // Remap measured channels into true order expected by FOC
      float ia_volts_adj = v_adj[current_map[0]];
      float ib_volts_adj = v_adj[current_map[1]];
      float ic_volts_adj = v_adj[current_map[2]];

      foc_update_optimized(electrical_angle, q_ref_normalized, ia_volts_adj, ib_volts_adj, ic_volts_adj);
    }
    
    checkpoint = DWT->CYCCNT;
    timing_foc_computation = checkpoint - start_cycles - timing_adc_read - timing_angle_update - timing_conversions;
    
    // Measure total ISR duration
    debug_isr_duration_cycles = DWT->CYCCNT - start_cycles;
    debug_counter++;
  }
}



static void adc_init(void) {
  // Configure the GPIOs for ADC
  palSetLineMode(LINE_ADC_PHASE_A,  PAL_MODE_INPUT_ANALOG);
  palSetLineMode(LINE_ADC_PHASE_B,  PAL_MODE_INPUT_ANALOG);
  palSetLineMode(LINE_ADC_PHASE_C,  PAL_MODE_INPUT_ANALOG);
  palSetLineMode(LINE_ADC_POSITION, PAL_MODE_INPUT_ANALOG);

  // Enable ADC1 clock
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

  // Enable scan mode since we are using multiple channels
  ADC1->CR1 |= ADC_CR1_SCAN;

  
  // Enable ADC Injected Convesion Interrupts
  ADC1->CR1 |= ADC_CR1_JEOCIE;


  // Select TIM4 (PWM Slave) as trigger source on rising edge
  ADC1->CR2 &= ~(ADC_CR2_JEXTSEL_Msk | ADC_CR2_JEXTEN_Msk);
  ADC1->CR2 |= ADC_CR2_JEXTSEL_0;
  ADC1->CR2 |= ADC_CR2_JEXTEN_0;

  // Enable ADC Conversion
  ADC1->CR2 |= ADC_CR2_ADON;

  // Configure the sampling cycles
  // 100 -> 84 CYCLE

  ADC1->SMPR1 &= ~(ADC_SMPR1_SMP11_Msk);
  ADC1->SMPR1 |= ADC_SMPR1_SMP11_2 | ADC_SMPR1_SMP11_1 | ADC_SMPR1_SMP11_0;

  ADC1->SMPR2 &= ~(ADC_SMPR2_SMP0_Msk | ADC_SMPR2_SMP1_Msk | ADC_SMPR2_SMP4_Msk);
  ADC1->SMPR2 |= ADC_SMPR2_SMP0_2 | ADC_SMPR2_SMP1_2 | ADC_SMPR2_SMP4_2 | ADC_SMPR2_SMP0_1 | ADC_SMPR2_SMP1_1 | ADC_SMPR2_SMP4_1 | ADC_SMPR2_SMP0_0 | ADC_SMPR2_SMP1_0 | ADC_SMPR2_SMP4_0;

  // Configure the sequence length to 4 channels
  ADC1->JSQR &= ~ADC_JSQR_JL_Msk;
  ADC1->JSQR |= ADC_JSQR_JL_1 | ADC_JSQR_JL_0;

  // Configure the sequence of injected channels
  ADC1->JSQR &= ~(ADC_JSQR_JSQ1_Msk | ADC_JSQR_JSQ2_Msk | ADC_JSQR_JSQ3_Msk | ADC_JSQR_JSQ4_Msk);
  ADC1->JSQR |= (ADC_CHANNEL_PHASE_A << ADC_JSQR_JSQ1_Pos) |
                (ADC_CHANNEL_PHASE_B << ADC_JSQR_JSQ2_Pos) |
                (ADC_CHANNEL_PHASE_C << ADC_JSQR_JSQ3_Pos) |
                (ADC_CHANNEL_POSITION << ADC_JSQR_JSQ4_Pos);

  NVIC_EnableIRQ(ADC_IRQn);
  NVIC_SetPriority(ADC_IRQn, 2);
}



/*===========================================================================*/
/* Worker Thread for Mailbox + Event Processing                              */
/*===========================================================================*/
// Worker thread removed - FOC now runs directly in ADC ISR for ultra-low latency

/*===========================================================================*/
/* Shell                                                                     */
/*===========================================================================*/
#define SHELL_WA_SIZE THD_WORKING_AREA_SIZE(2048)

static void cmd_test_adc_manual(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)argc;
  (void)argv;
  
  uint32_t old_counter = debug_counter;
  
  chprintf(chp, "Testing manual ADC trigger...\n");
  chprintf(chp, "Counter before: %lu\n", debug_counter);
  
  // Trigger ADC manually (bypass TIM2 trigger)
  ADC1->CR2 |= ADC_CR2_JSWSTART;  // Manual software start
  
  chThdSleepMilliseconds(10);
  
  chprintf(chp, "Counter after: %lu\n", debug_counter);
  
  if (debug_counter > old_counter) {
    chprintf(chp, "✅ ADC interrupt working!\n");
  } else {
    chprintf(chp, "❌ ADC interrupt not firing\n");
    chprintf(chp, "ADC1->SR = 0x%08lx\n", ADC1->SR);
    chprintf(chp, "ADC1->CR1 = 0x%08lx\n", ADC1->CR1);
    chprintf(chp, "ADC1->CR2 = 0x%08lx\n", ADC1->CR2);
    chprintf(chp, "ADC1->JSQR = 0x%08lx\n", ADC1->JSQR);
  }
}

static void cmd_check_pwm_trigger(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)argc;
  (void)argv;
  
  chprintf(chp, "PWM/Timer Status:\n");
  chprintf(chp, "TIM1->CR1 = 0x%08lx (should have CEN=1)\n", TIM1->CR1);
  chprintf(chp, "TIM4->CR1 = 0x%08lx (should have CEN=1)\n", TIM4->CR1);
  chprintf(chp, "TIM1->CNT = %lu\n", TIM1->CNT);
  chprintf(chp, "TIM4->CNT = %lu (should be counting)\n", TIM4->CNT);
  
  chThdSleepMilliseconds(100);
  chprintf(chp, "TIM4->CNT after 100ms = %lu (should be different)\n", TIM4->CNT);
  
  chprintf(chp, "\nADC Trigger Configuration:\n");
  chprintf(chp, "ADC1->CR2 = 0x%08lx\n", ADC1->CR2);
  chprintf(chp, "Expected JEXTSEL = 0x%lx (TIM4_TRGO)\n", (ADC1->CR2 & ADC_CR2_JEXTSEL_Msk) >> ADC_CR2_JEXTSEL_Pos);
  chprintf(chp, "Expected JEXTEN = 0x%lx (Rising edge)\n", (ADC1->CR2 & ADC_CR2_JEXTEN_Msk) >> ADC_CR2_JEXTEN_Pos);
}

// Start/stop alignment mode and configure duty split
// Usage:
//  align_start [A B C]    -> enable alignment; optional A/B/C in 0..10000
//  align_stop             -> disable alignment and resume FOC
//  align_status           -> show current alignment settings and raw position
static void cmd_align_start(BaseSequentialStream *chp, int argc, char *argv[]) {
  if (argc == 3) {
    int a = atoi(argv[0]);
    int b = atoi(argv[1]);
    int c = atoi(argv[2]);
    if (a < 0) a = 0; if (a > 10000) a = 10000;
    if (b < 0) b = 0; if (b > 10000) b = 10000;
    if (c < 0) c = 0; if (c > 10000) c = 10000;
    align_duty_a = (uint16_t)a;
    align_duty_b = (uint16_t)b;
    align_duty_c = (uint16_t)c;
  }
  align_mode_enabled = true;
  chprintf(chp, "Alignment ENABLED. Duties: A=%u B=%u C=%u (0..10000)\n", align_duty_a, align_duty_b, align_duty_c);
  chprintf(chp, "Tip: Increase A-B spread slowly while monitoring current.\n");
}

static void cmd_align_stop(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)argc; (void)argv;
  align_mode_enabled = false;
  chprintf(chp, "Alignment DISABLED. Resuming FOC.\n");
}

static void cmd_align_status(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)argc; (void)argv;
  chprintf(chp, "Align mode: %s\n", align_mode_enabled ? "ENABLED" : "disabled");
  chprintf(chp, "Duties (0..10000): A=%u B=%u C=%u\n", align_duty_a, align_duty_b, align_duty_c);
  chprintf(chp, "Latest ADC position (AS5600 raw): %lu\n", current_adc_data.position_current);
  float mech_deg = (current_adc_data.position_current * 360.0f) / 4096.0f;
  chprintf(chp, "Mechanical angle ≈ %.2f deg (from last ISR)\n", mech_deg);
  chprintf(chp, "Set offset now with: encoder_offset %.2f\n", -mech_deg);
}

static void cmd_adc_registers(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)argc;
  (void)argv;
  
  chprintf(chp, "ADC Register Dump:\n");
  chprintf(chp, "ADC1->SR   = 0x%08lx\n", ADC1->SR);
  chprintf(chp, "ADC1->CR1  = 0x%08lx\n", ADC1->CR1);
  chprintf(chp, "ADC1->CR2  = 0x%08lx\n", ADC1->CR2);
  chprintf(chp, "ADC1->JSQR = 0x%08lx\n", ADC1->JSQR);
  chprintf(chp, "ADC1->JDR1 = %lu\n", ADC1->JDR1);
  chprintf(chp, "ADC1->JDR2 = %lu\n", ADC1->JDR2);
  chprintf(chp, "ADC1->JDR3 = %lu\n", ADC1->JDR3);
  chprintf(chp, "ADC1->JDR4 = %lu\n", ADC1->JDR4);
  
  chprintf(chp, "\nNVIC Status:\n");
  chprintf(chp, "ADC_IRQn enabled: %s\n", (NVIC->ISER[ADC_IRQn >> 5] & (1 << (ADC_IRQn & 0x1F))) ? "YES" : "NO");
  chprintf(chp, "ADC_IRQn priority: %lu\n", NVIC_GetPriority(ADC_IRQn));
}


static void cmd_get_adc(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)argc;
  (void)argv;

  chprintf(chp, "Current ADC readings (from last interrupt):\n");
  chprintf(chp, "Phase A: %lu (%.3f V)\n", current_adc_data.phase_current[0], ia_volts_last);
  chprintf(chp, "Phase B: %lu (%.3f V)\n", current_adc_data.phase_current[1], ib_volts_last);
  chprintf(chp, "Phase C: %lu (%.3f V)\n", current_adc_data.phase_current[2], ic_volts_last);
  chprintf(chp, "Position: %lu\n", current_adc_data.position_current);
  chprintf(chp, "Timestamp: %lu\n", (uint32_t)current_adc_data.timestamp);
}

static void cmd_adc_status(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)argc;
  (void)argv;
  
  chprintf(chp, "ADC + FOC Status (now in ISR):\n");
  chprintf(chp, "\nLatest ADC Data:\n");
  chprintf(chp, "Phase A: %lu, Phase B: %lu, Phase C: %lu\n", 
           current_adc_data.phase_current[0],
           current_adc_data.phase_current[1], 
           current_adc_data.phase_current[2]);
  chprintf(chp, "Position: %lu\n", 
           current_adc_data.position_current);
  chprintf(chp, "Timestamp: %lu\n", (uint32_t)current_adc_data.timestamp);
  chprintf(chp, "\nVoltage Readings:\n");
  chprintf(chp, "ia_volts: %.3f V, ib_volts: %.3f V, ic_volts: %.3f V\n", 
           ia_volts_last, ib_volts_last, ic_volts_last);
  chprintf(chp, "Current Offsets: OA=%.4f V, OB=%.4f V, OC=%.4f V\n", 
           current_offset_a_volts, current_offset_b_volts, current_offset_c_volts);
  chprintf(chp, "Currents (A): Ia=%.3f, Ib=%.3f, Ic=%.3f, Sum=%.3f\n",
           ia_amps_last, ib_amps_last, ic_amps_last, ia_amps_last + ib_amps_last + ic_amps_last);
  chprintf(chp, "Current Signs: SA=%d, SB=%d, SC=%d\n", current_sign_a, current_sign_b, current_sign_c);
  chprintf(chp, "Current Map: [%u,%u,%u] (0=A,1=B,2=C)\n", current_map[0], current_map[1], current_map[2]);
  chprintf(chp, "Debug counter: %lu\n", debug_counter);
  chprintf(chp, "Last ISR duration = %lu cycles (including FOC)\n", debug_isr_duration_cycles);
  chprintf(chp, "Next duty A: %lu, Next duty B: %lu, Next duty C: %lu\n", 
           next_duty_a_pwm,
           next_duty_b_pwm,
           next_duty_c_pwm);
}

// Show or set per-channel current offsets
static void cmd_current_offsets(BaseSequentialStream *chp, int argc, char *argv[]) {
  if (argc == 0) {
    chprintf(chp, "Offsets (V): A=%.4f, B=%.4f, C=%.4f\n",
             current_offset_a_volts, current_offset_b_volts, current_offset_c_volts);
    chprintf(chp, "Usage: current_offsets <A_volts> <B_volts> <C_volts>\n");
    return;
  }
  if (argc == 3) {
    current_offset_a_volts = strtof(argv[0], NULL);
    current_offset_b_volts = strtof(argv[1], NULL);
    current_offset_c_volts = strtof(argv[2], NULL);
    chprintf(chp, "Offsets updated. A=%.4f, B=%.4f, C=%.4f\n",
             current_offset_a_volts, current_offset_b_volts, current_offset_c_volts);
  } else {
    chprintf(chp, "Usage: current_offsets <A_volts> <B_volts> <C_volts>\n");
  }
}

// Set one offset by phase letter
static void cmd_current_offset_set(BaseSequentialStream *chp, int argc, char *argv[]) {
  if (argc != 2) {
    chprintf(chp, "Usage: current_offset_set <A|B|C> <volts>\n");
    return;
  }
  float v = strtof(argv[1], NULL);
  if (argv[0][0] == 'A' || argv[0][0] == 'a') {
    current_offset_a_volts = v;
  } else if (argv[0][0] == 'B' || argv[0][0] == 'b') {
    current_offset_b_volts = v;
  } else if (argv[0][0] == 'C' || argv[0][0] == 'c') {
    current_offset_c_volts = v;
  } else {
    chprintf(chp, "Error: phase must be A, B, or C\n");
    return;
  }
  chprintf(chp, "Offset updated: %c=%.4f V\n", argv[0][0], v);
}

// Auto-calibrate offsets at idle by averaging N samples
static void cmd_current_offset_auto(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)argc; (void)argv;
  const int samples = 64;
  float sum_a = 0.0f, sum_b = 0.0f, sum_c = 0.0f;
  chprintf(chp, "Calibrating current offsets... ensure no PWM (align_stop) and angle_enable 0\n");
  for (int i = 0; i < samples; ++i) {
    chThdSleepMilliseconds(2);
    sum_a += ia_volts_last;
    sum_b += ib_volts_last;
    sum_c += ic_volts_last;
  }
  current_offset_a_volts = sum_a / samples;
  current_offset_b_volts = sum_b / samples;
  current_offset_c_volts = sum_c / samples;
  chprintf(chp, "Offsets calibrated. A=%.4f V, B=%.4f V, C=%.4f V\n",
           current_offset_a_volts, current_offset_b_volts, current_offset_c_volts);
}

// Print last computed phase currents (Amps)
static void cmd_current_amps(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)argc; (void)argv;
  chprintf(chp, "Ia=%.3f A, Ib=%.3f A, Ic=%.3f A, Sum=%.3f A\n",
           ia_amps_last, ib_amps_last, ic_amps_last,
           ia_amps_last + ib_amps_last + ic_amps_last);
}

// Show or set per-channel current signs (+1 or -1)
static void cmd_current_signs(BaseSequentialStream *chp, int argc, char *argv[]) {
  if (argc == 0) {
    chprintf(chp, "Current signs: A=%d, B=%d, C=%d\n", current_sign_a, current_sign_b, current_sign_c);
    chprintf(chp, "Usage: current_signs <A_sign> <B_sign> <C_sign>   (each +1 or -1)\n");
    return;
  }
  if (argc == 3) {
    int sa = atoi(argv[0]);
    int sb = atoi(argv[1]);
    int sc = atoi(argv[2]);
    if (!((sa == 1) || (sa == -1)) || !((sb == 1) || (sb == -1)) || !((sc == 1) || (sc == -1))) {
      chprintf(chp, "Error: signs must be +1 or -1\n");
      return;
    }
    current_sign_a = sa;
    current_sign_b = sb;
    current_sign_c = sc;
    chprintf(chp, "Current signs updated: A=%d, B=%d, C=%d\n", current_sign_a, current_sign_b, current_sign_c);
  } else {
    chprintf(chp, "Usage: current_signs <A_sign> <B_sign> <C_sign>   (each +1 or -1)\n");
  }
}

// Show or set current channel mapping: three indices in 0..2 selecting from measured [A,B,C]
// Example: current_map 1 2 0 maps true (Ia,Ib,Ic) = measured (Ib,Ic,Ia)
static void cmd_current_map(BaseSequentialStream *chp, int argc, char *argv[]) {
  if (argc == 0) {
    chprintf(chp, "Current map: [%u,%u,%u]   (0=A,1=B,2=C)\n", current_map[0], current_map[1], current_map[2]);
    chprintf(chp, "Usage: current_map <iA> <iB> <iC>   each in 0..2\n");
    return;
  }
  if (argc == 3) {
    int ia = atoi(argv[0]);
    int ib = atoi(argv[1]);
    int ic = atoi(argv[2]);
    if (ia < 0 || ia > 2 || ib < 0 || ib > 2 || ic < 0 || ic > 2) {
      chprintf(chp, "Error: indices must be 0,1,2\n");
      return;
    }
    current_map[0] = (uint8_t)ia;
    current_map[1] = (uint8_t)ib;
    current_map[2] = (uint8_t)ic;
    chprintf(chp, "Current map updated: [%u,%u,%u]\n", current_map[0], current_map[1], current_map[2]);
  } else {
    chprintf(chp, "Usage: current_map <iA> <iB> <iC>\n");
  }
}

// Shell command to show ISR timing breakdown
static void cmd_timing_profile(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)argc;
  (void)argv;
  
  float freq_mhz = 84.0f; // System clock in MHz
  
  chprintf(chp, "ISR Timing Profile (last interrupt):\n");
  chprintf(chp, "===========================================\n");
  chprintf(chp, "Section 1 - ADC Read & Data Update:\n");
  chprintf(chp, "  Cycles: %lu (%.2f μs, %.1f%%)\n", 
           timing_adc_read, 
           timing_adc_read / freq_mhz,
           (timing_adc_read * 100.0f) / debug_isr_duration_cycles);
  
  chprintf(chp, "Section 2 - Angle Update & PID:\n");
  chprintf(chp, "  Cycles: %lu (%.2f μs, %.1f%%)\n", 
           timing_angle_update, 
           timing_angle_update / freq_mhz,
           (timing_angle_update * 100.0f) / debug_isr_duration_cycles);
  
  chprintf(chp, "Section 3 - Float Conversions:\n");
  chprintf(chp, "  Cycles: %lu (%.2f μs, %.1f%%)\n", 
           timing_conversions, 
           timing_conversions / freq_mhz,
           (timing_conversions * 100.0f) / debug_isr_duration_cycles);
  
  chprintf(chp, "Section 4 - FOC Computation:\n");
  chprintf(chp, "  Cycles: %lu (%.2f μs, %.1f%%)\n", 
           timing_foc_computation, 
           timing_foc_computation / freq_mhz,
           (timing_foc_computation * 100.0f) / debug_isr_duration_cycles);
  
  chprintf(chp, "===========================================\n");
  chprintf(chp, "TOTAL ISR Duration: %lu cycles (%.2f μs)\n", 
           debug_isr_duration_cycles,
           debug_isr_duration_cycles / freq_mhz);
  chprintf(chp, "Available budget:   4000 cycles (47.6 μs)\n");
  chprintf(chp, "CPU Usage: %.1f%%\n", (debug_isr_duration_cycles * 100.0f) / 4000.0f);
}

// Shell command to show detailed FOC timing breakdown
static void cmd_foc_timing(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)argc;
  (void)argv;
  
  float freq_mhz = 84.0f; // System clock in MHz
  
  chprintf(chp, "FOC Detailed Timing Profile (last interrupt):\n");
  chprintf(chp, "===========================================\n");
  chprintf(chp, "Step 1 - Current Conversion:\n");
  chprintf(chp, "  Cycles: %lu (%.2f μs, %.1f%%)\n", 
           foc_timing_current_conversion, 
           foc_timing_current_conversion / freq_mhz,
           (foc_timing_current_conversion * 100.0f) / timing_foc_computation);
  
  chprintf(chp, "Step 2 - Clarke Transform:\n");
  chprintf(chp, "  Cycles: %lu (%.2f μs, %.1f%%)\n", 
           foc_timing_clarke_transform, 
           foc_timing_clarke_transform / freq_mhz,
           (foc_timing_clarke_transform * 100.0f) / timing_foc_computation);
  
  chprintf(chp, "Step 3 - Park Transform (sin/cos):\n");
  chprintf(chp, "  Cycles: %lu (%.2f μs, %.1f%%)\n", 
           foc_timing_park_transform, 
           foc_timing_park_transform / freq_mhz,
           (foc_timing_park_transform * 100.0f) / timing_foc_computation);
  
  chprintf(chp, "Step 4 - PID Controllers:\n");
  chprintf(chp, "  Cycles: %lu (%.2f μs, %.1f%%)\n", 
           foc_timing_pid_controllers, 
           foc_timing_pid_controllers / freq_mhz,
           (foc_timing_pid_controllers * 100.0f) / timing_foc_computation);
  
  chprintf(chp, "Step 5 - Inverse Park (sin/cos):\n");
  chprintf(chp, "  Cycles: %lu (%.2f μs, %.1f%%)\n", 
           foc_timing_inverse_park, 
           foc_timing_inverse_park / freq_mhz,
           (foc_timing_inverse_park * 100.0f) / timing_foc_computation);
  
  chprintf(chp, "Step 6 - Inverse Clarke:\n");
  chprintf(chp, "  Cycles: %lu (%.2f μs, %.1f%%)\n", 
           foc_timing_inverse_clarke, 
           foc_timing_inverse_clarke / freq_mhz,
           (foc_timing_inverse_clarke * 100.0f) / timing_foc_computation);
  
  chprintf(chp, "Step 7 - PWM Conversion & Output:\n");
  chprintf(chp, "  Cycles: %lu (%.2f μs, %.1f%%)\n", 
           foc_timing_pwm_conversion, 
           foc_timing_pwm_conversion / freq_mhz,
           (foc_timing_pwm_conversion * 100.0f) / timing_foc_computation);
  
  chprintf(chp, "===========================================\n");
  chprintf(chp, "TOTAL FOC: %lu cycles (%.2f μs)\n", 
           timing_foc_computation,
           timing_foc_computation / freq_mhz);
  chprintf(chp, "Expected sin/cos calls: 4x (Park + Inverse Park)\n");
}

// Shell command to set angle target
static void cmd_angle_set(BaseSequentialStream *chp, int argc, char *argv[]) {
  if (argc != 1) {
    chprintf(chp, "Usage: angle_set <degrees>\n");
    chprintf(chp, "Examples:\n");
    chprintf(chp, "  angle_set 90     - Set to 90 degrees\n");
    chprintf(chp, "  angle_set -45    - Set to -45 degrees\n");
    chprintf(chp, "  angle_set 180    - Set to 180 degrees\n");
    chprintf(chp, "  angle_set 360    - Set to 360 degrees (1 full revolution)\n");
    chprintf(chp, "Note: Can exceed ±180° for multi-revolution moves\n");
    return;
  }
  
  float target_degrees = atof(argv[0]);
  float target_radians = target_degrees * M_PI / 180.0f;
  
  angle_control_set_target(target_radians);
  chprintf(chp, "Angle target set to %.2f degrees (%.3f rad)\n", target_degrees, target_radians);
  
  // Show helpful info
  float revolutions = target_radians / (2.0f * M_PI);
  chprintf(chp, "Equivalent: %.2f revolutions\n", revolutions);
}

// Shell command to enable/disable angle control
static void cmd_angle_enable(BaseSequentialStream *chp, int argc, char *argv[]) {
  if (argc != 1) {
    chprintf(chp, "Usage: angle_enable <0|1>\n");
    chprintf(chp, "0 = disable, 1 = enable\n");
    return;
  }
  
  bool enable = (atoi(argv[0]) != 0);
  angle_control_enable(enable);
  chprintf(chp, "Angle control %s\n", enable ? "enabled" : "disabled");
}

// Shell command to get angle status
static void cmd_angle_status(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)argc;
  (void)argv;
  
  float current_deg = angle_control_get_current_angle() * 180.0f / M_PI;
  float target_deg = angle_control.target_angle * 180.0f / M_PI;
  float error_deg = angle_control_get_error() * 180.0f / M_PI;
  float output = angle_control_get_output();
  
  chprintf(chp, "Angle Control Status:\n");
  chprintf(chp, "  Enabled: %s\n", angle_control.enabled ? "YES" : "NO");
  chprintf(chp, "  Current: %.2f deg (%.3f rad) [range: -π to +π]\n", current_deg, angle_control_get_current_angle());
  chprintf(chp, "  Target:  %.2f deg (%.3f rad) [range: -π to +π]\n", target_deg, angle_control.target_angle);
  chprintf(chp, "  Error:   %.2f deg (%.3f rad) [range: -π to +π]\n", error_deg, angle_control_get_error());
  chprintf(chp, "  Torque Output: %.3f A (current ∝ torque)\n", output);
  chprintf(chp, "  Raw ADC: %lu (0-4095)\n", current_adc_data.position_current);
}

// Shell command to set angle target in revolutions
static void cmd_angle_set_revolutions(BaseSequentialStream *chp, int argc, char *argv[]) {
  if (argc != 1) {
    chprintf(chp, "Usage: angle_revs <revolutions>\n");
    chprintf(chp, "Example: angle_revs 2.5  (2.5 full revolutions)\n");
    chprintf(chp, "Example: angle_revs -1.0 (1 revolution backwards)\n");
    return;
  }
  
  float revolutions = atof(argv[0]);
  float target_radians = revolutions * 2.0f * M_PI;
  
  angle_control_set_target(target_radians);
  chprintf(chp, "Angle target set to %.2f revolutions (%.3f rad)\n", revolutions, target_radians);
}

// Shell command to zero the current position (reset revolution counter)
static void cmd_angle_zero(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)argc;
  (void)argv;
  
  // Reset revolution tracking to treat current position as zero
  angle_control.revolution_count = 0;
  angle_control.current_angle = 0.0f;
  angle_control.first_reading = true;  // Force re-initialization on next reading
  
  // Reset PID to clear any accumulated error
  pid_reset(&angle_control.angle_pid);
  
  chprintf(chp, "Position zeroed. Current position is now 0.0 revolutions.\n");
  chprintf(chp, "Note: PID controller was reset to clear accumulated error.\n");
}

// Shell command for quick angle presets
static void cmd_angle_preset(BaseSequentialStream *chp, int argc, char *argv[]) {
  if (argc != 1) {
    chprintf(chp, "Usage: angle_preset <preset>\n");
    chprintf(chp, "Available presets:\n");
    chprintf(chp, "  0, 90, 180, 270     - Cardinal angles (degrees)\n");
    chprintf(chp, "  45, 135, 225, 315   - Diagonal angles (degrees)\n");
    chprintf(chp, "  cw, ccw             - 1 revolution clockwise/counter-clockwise\n");
    chprintf(chp, "  home                - Return to 0 degrees\n");
    return;
  }
  
  float target_degrees = 0.0f;
  const char* preset = argv[0];
  
  // Parse preset
  if (strcmp(preset, "0") == 0 || strcmp(preset, "home") == 0) {
    target_degrees = 0.0f;
  } else if (strcmp(preset, "45") == 0) {
    target_degrees = 45.0f;
  } else if (strcmp(preset, "90") == 0) {
    target_degrees = 90.0f;
  } else if (strcmp(preset, "135") == 0) {
    target_degrees = 135.0f;
  } else if (strcmp(preset, "180") == 0) {
    target_degrees = 180.0f;
  } else if (strcmp(preset, "225") == 0) {
    target_degrees = 225.0f;
  } else if (strcmp(preset, "270") == 0) {
    target_degrees = 270.0f;
  } else if (strcmp(preset, "315") == 0) {
    target_degrees = 315.0f;
  } else if (strcmp(preset, "cw") == 0) {
    target_degrees = 360.0f;  // 1 full revolution clockwise
  } else if (strcmp(preset, "ccw") == 0) {
    target_degrees = -360.0f; // 1 full revolution counter-clockwise
  } else {
    chprintf(chp, "Error: Unknown preset '%s'\n", preset);
    return;
  }
  
  float target_radians = target_degrees * M_PI / 180.0f;
  angle_control_set_target(target_radians);
  chprintf(chp, "Preset '%s': Angle target set to %.2f degrees (%.3f rad)\n", 
           preset, target_degrees, target_radians);
}

// Shell command for relative angle moves
static void cmd_angle_move(BaseSequentialStream *chp, int argc, char *argv[]) {
  if (argc != 1) {
    chprintf(chp, "Usage: angle_move <degrees>\n");
    chprintf(chp, "Examples:\n");
    chprintf(chp, "  angle_move 90    - Move +90 degrees from current target\n");
    chprintf(chp, "  angle_move -45   - Move -45 degrees from current target\n");
    chprintf(chp, "  angle_move 360   - Move +1 full revolution\n");
    chprintf(chp, "Note: This is relative to current TARGET, not current position\n");
    return;
  }
  
  float move_degrees = atof(argv[0]);
  float move_radians = move_degrees * M_PI / 180.0f;
  
  float new_target = angle_control.target_angle + move_radians;
  angle_control_set_target(new_target);
  
  float new_target_degrees = new_target * 180.0f / M_PI;
  chprintf(chp, "Relative move: %.2f degrees\n", move_degrees);
  chprintf(chp, "New target: %.2f degrees (%.3f rad)\n", new_target_degrees, new_target);
}

// Shell command to get detailed angle status with revolutions
static void cmd_angle_status_detailed(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)argc;
  (void)argv;
  
  float current_rev = angle_control_get_current_angle() / (2.0f * M_PI);
  float target_rev = angle_control.target_angle / (2.0f * M_PI);
  float error_rev = angle_control_get_error() / (2.0f * M_PI);
  float output = angle_control_get_output();
  
  chprintf(chp, "Angle Control Status (Detailed):\n");
  chprintf(chp, "  Enabled: %s\n", angle_control.enabled ? "YES" : "NO");
  chprintf(chp, "  Current Position: %.3f revolutions (%.3f rad)\n", current_rev, angle_control_get_current_angle());
  chprintf(chp, "  Target Position:  %.3f revolutions (%.3f rad)\n", target_rev, angle_control.target_angle);
  chprintf(chp, "  Error:           %.3f revolutions (%.3f rad)\n", error_rev, angle_control_get_error());
  chprintf(chp, "  Revolution Count: %ld complete revolutions\n", (long)angle_control.revolution_count);
  chprintf(chp, "  Raw Encoder:     %.2f deg (%.3f rad within revolution)\n", 
           angle_control.prev_raw_angle * 180.0f / M_PI, angle_control.prev_raw_angle);
  chprintf(chp, "  Torque Output:   %.3f A (current ∝ torque)\n", output);
  chprintf(chp, "  Raw ADC:         %lu (0-4095)\n", current_adc_data.position_current);
}

// Shell command to get/set encoder offset
static void cmd_encoder_offset(BaseSequentialStream *chp, int argc, char *argv[]) {
  if (argc == 0) {
    // Get current offset - display in degrees
    float offset_degrees = motor_offset_radians * 180.0f / M_PI;
    chprintf(chp, "Encoder Offset: %.2f degrees (%.4f radians)\n", offset_degrees, motor_offset_radians);
    chprintf(chp, "Usage: encoder_offset <degrees> - to set new offset\n");
    chprintf(chp, "Example: encoder_offset 102.0\n");
    return;
  }
  
  if (argc == 1) {
    // Set new offset - input in degrees, store in radians
    float offset_degrees = atof(argv[0]);
    motor_offset_radians = offset_degrees * M_PI / 180.0f;
    
    // Normalize offset to [-π, +π] range
    while (motor_offset_radians > M_PI) {
      motor_offset_radians -= 2.0f * M_PI;
    }
    while (motor_offset_radians <= -M_PI) {
      motor_offset_radians += 2.0f * M_PI;
    }
    
    float normalized_degrees = motor_offset_radians * 180.0f / M_PI;
    chprintf(chp, "Encoder offset set to %.2f degrees (%.4f radians)\n", normalized_degrees, motor_offset_radians);
    chprintf(chp, "Note: Offset normalized to [-180, +180] degree range\n");
    return;
  }
  
  chprintf(chp, "Usage: encoder_offset [degrees]\n");
  chprintf(chp, "  encoder_offset          - show current offset\n");
  chprintf(chp, "  encoder_offset <deg>    - set offset in degrees\n");
  chprintf(chp, "Example: encoder_offset 102.0\n");
}

// Shell command to set d-axis PID parameters
static void cmd_d_pid_set(BaseSequentialStream *chp, int argc, char *argv[]) {
  if (argc != 3) {
    chprintf(chp, "Usage: d_pid_set <kp> <ki> <kd>\n");
    chprintf(chp, "Example: d_pid_set 0.5 0.1 0.01\n");
    return;
  }
  
  float kp = strtof(argv[0], NULL);
  float ki = strtof(argv[1], NULL);
  float kd = strtof(argv[2], NULL);
  
  foc_set_d_pid_params(kp, ki, kd);
  chprintf(chp, "D-axis PID set: Kp=%.3f, Ki=%.3f, Kd=%.3f\n", kp, ki, kd);
}

// Shell command to set q-axis PID parameters
static void cmd_q_pid_set(BaseSequentialStream *chp, int argc, char *argv[]) {
  if (argc != 3) {
    chprintf(chp, "Usage: q_pid_set <kp> <ki> <kd>\n");
    chprintf(chp, "Example: q_pid_set 0.5 0.1 0.01\n");
    return;
  }
  
  float kp = strtof(argv[0], NULL);
  float ki = strtof(argv[1], NULL);
  float kd = strtof(argv[2], NULL);
  
  foc_set_q_pid_params(kp, ki, kd);
  chprintf(chp, "Q-axis PID set: Kp=%.3f, Ki=%.3f, Kd=%.3f\n", kp, ki, kd);
}

// Shell command to get current PID parameters
static void cmd_pid_get(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)argc;
  (void)argv;
  
  float d_kp, d_ki, d_kd;
  float q_kp, q_ki, q_kd;
  
  foc_get_d_pid_params(&d_kp, &d_ki, &d_kd);
  foc_get_q_pid_params(&q_kp, &q_ki, &q_kd);
  
  chprintf(chp, "Current PID Parameters:\n");
  chprintf(chp, "D-axis: Kp=%.3f, Ki=%.3f, Kd=%.3f\n", d_kp, d_ki, d_kd);
  chprintf(chp, "Q-axis: Kp=%.3f, Ki=%.3f, Kd=%.3f\n", q_kp, q_ki, q_kd);
}

// Shell command to reset PID controllers (clear integral windup)
static void cmd_pid_reset(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)argc;
  (void)argv;
  
  foc_reset_pid_controllers();
  chprintf(chp, "PID controllers reset (integral cleared)\n");
}

// Shell command to set d-axis reference override
static void cmd_d_ref_set(BaseSequentialStream *chp, int argc, char *argv[]) {
  if (argc == 0) {
    chprintf(chp, "Usage: d_ref_set <value> | d_ref_set off\n");
    chprintf(chp, "Examples:\n");
    chprintf(chp, "  d_ref_set 0.1     - Set d_ref to 0.1A\n");
    chprintf(chp, "  d_ref_set -0.05   - Set d_ref to -0.05A (field weakening)\n");
    chprintf(chp, "  d_ref_set off     - Disable override (use 0.0)\n");
    chprintf(chp, "WARNING: Non-zero d_ref can cause vibration and losses!\n");
    return;
  }
  
  if (strcmp(argv[0], "off") == 0) {
    d_ref_override_enabled = false;
    foc_clear_d_ref_override();
    chprintf(chp, "D-axis reference override disabled (using 0.0A)\n");
  } else {
    float d_ref = strtof(argv[0], NULL);
    d_ref_override_enabled = true;
    d_ref_override_value = d_ref;
    foc_set_d_ref_override(d_ref);
    chprintf(chp, "D-axis reference override set to %.3fA\n", d_ref);
    chprintf(chp, "WARNING: This may cause vibration and increased losses!\n");
  }
}

// Shell command to set q-axis reference override
static void cmd_q_ref_set(BaseSequentialStream *chp, int argc, char *argv[]) {
  if (argc == 0) {
    chprintf(chp, "Usage: q_ref_set <value> | q_ref_set off\n");
    chprintf(chp, "Examples:\n");
    chprintf(chp, "  q_ref_set 0.1     - Set q_ref to 0.1 (normalized, 0.1*27A = 2.7A)\n");
    chprintf(chp, "  q_ref_set -0.05   - Set q_ref to -0.05 (reverse torque)\n");
    chprintf(chp, "  q_ref_set off     - Disable override (use angle controller)\n");
    chprintf(chp, "Range: -1.0 to +1.0 (normalized to ±27A max current)\n");
    return;
  }
  
  if (strcmp(argv[0], "off") == 0) {
    q_ref_override_enabled = false;
    chprintf(chp, "Q-axis reference override disabled (using angle controller)\n");
  } else {
    float q_ref = strtof(argv[0], NULL);
    
    // Clamp to safe range
    if (q_ref > 1.0f) q_ref = 1.0f;
    if (q_ref < -1.0f) q_ref = -1.0f;
    
    q_ref_override_enabled = true;
    q_ref_override_value = q_ref;
    chprintf(chp, "Q-axis reference override set to %.3f (%.1fA)\n", q_ref, q_ref * FOC_MAX_CURRENT);
    chprintf(chp, "Note: This bypasses the angle controller!\n");
  }
}

// Shell command to get current reference values
static void cmd_ref_get(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)argc;
  (void)argv;
  
  chprintf(chp, "Current Reference Values:\n");
  
  if (d_ref_override_enabled) {
    chprintf(chp, "D-axis: %.3fA (OVERRIDE ACTIVE)\n", d_ref_override_value);
  } else {
    chprintf(chp, "D-axis: 0.000A (normal operation)\n");
  }
  
  if (q_ref_override_enabled) {
    chprintf(chp, "Q-axis: %.3f (%.1fA) (OVERRIDE ACTIVE)\n", 
             q_ref_override_value, q_ref_override_value * FOC_MAX_CURRENT);
  } else {
    float current_q_ref = angle_control.torque_output / FOC_MAX_CURRENT;
    chprintf(chp, "Q-axis: %.3f (%.1fA) (from angle controller)\n", 
             current_q_ref, angle_control.torque_output);
  }
}

// Shell command to get/set motor direction
static void cmd_motor_direction(BaseSequentialStream *chp, int argc, char *argv[]) {
  if (argc == 0) {
    chprintf(chp, "Current motor direction: %d\n", motor_direction);
    chprintf(chp, "Usage: motor_direction [1 | -1]\n");
    chprintf(chp, "  +1: Normal direction\n");
    chprintf(chp, "  -1: Inverted direction\n");
    return;
  }
  
  int new_direction = atoi(argv[0]);
  if (new_direction == 1 || new_direction == -1) {
    motor_direction = new_direction;
    chprintf(chp, "Motor direction set to: %d\n", motor_direction);
    chprintf(chp, "Note: This affects PID feedback polarity\n");
  } else {
    chprintf(chp, "Error: Direction must be +1 or -1\n");
  }
}

static const ShellCommand shell_commands[] = {
  {"adc", cmd_get_adc},
  {"status", cmd_adc_status},
  {"timing", cmd_timing_profile},         // ISR timing breakdown
  {"foc_timing", cmd_foc_timing},         // Detailed FOC timing
  {"test_adc", cmd_test_adc_manual},      // Manual ADC trigger test
  {"test_pwm", cmd_check_pwm_trigger},    // PWM timer test  
  {"registers", cmd_adc_registers},       // Register dump
  {"angle_set", cmd_angle_set},           // Set angle target in degrees
  {"angle_revs", cmd_angle_set_revolutions}, // Set angle target in revolutions
  {"angle_zero", cmd_angle_zero},         // Zero current position
  {"angle_enable", cmd_angle_enable},     // Enable/disable angle control
  {"angle_status", cmd_angle_status},     // Get angle status (basic)
  {"angle_info", cmd_angle_status_detailed}, // Get detailed angle status with revolutions
  {"angle_preset", cmd_angle_preset},     // Quick angle presets (0, 90, 180, etc.)
  {"angle_move", cmd_angle_move},         // Relative angle moves
  {"encoder_offset", cmd_encoder_offset}, // Get/set encoder offset in degrees
  {"d_pid_set", cmd_d_pid_set},           // Set d-axis PID parameters
  {"q_pid_set", cmd_q_pid_set},           // Set q-axis PID parameters
  {"pid_get", cmd_pid_get},               // Get current PID parameters
  {"pid_reset", cmd_pid_reset},           // Reset PID controllers
  {"d_ref_set", cmd_d_ref_set},           // Set d-axis reference override
  {"q_ref_set", cmd_q_ref_set},           // Set q-axis reference override
  {"ref_get", cmd_ref_get},               // Get current reference values
  {"motor_direction", cmd_motor_direction}, // Get/set motor direction
  {"current_offsets", cmd_current_offsets}, // Show or set per-channel current offsets
  {"current_offset_set", cmd_current_offset_set}, // Set one offset: phase A|B|C value_volts
  {"current_offset_auto", cmd_current_offset_auto}, // Auto-calibrate offsets at idle
  {"current_amps", cmd_current_amps}, // Print last Ia/Ib/Ic in Amps and sum
  {"current_signs", cmd_current_signs}, // Show/set current signs
  {"current_map", cmd_current_map}, // Show/set channel mapping
  {"align_start", cmd_align_start},       // Enable constant phase-A alignment
  {"align_stop", cmd_align_stop},         // Disable alignment
  {"align_status", cmd_align_status},     // Show alignment info
  {NULL, NULL},
};

char shell_history[SHELL_MAX_HIST_BUFF];
char *shell_completions[SHELL_MAX_COMPLETIONS];
const SerialConfig sd2_config = {
  .speed = 115200,
  .cr1 = 0,
  .cr2 = 0,
  .cr3 = 0,
};

static const ShellConfig shell_cfg = {
  (BaseSequentialStream *)&SD2,
  shell_commands,
  shell_history,
  sizeof(shell_history),
  shell_completions,
};

/*
 * Application entry point.
 */
int main(void) {
  halInit();
  chSysInit();

  sdStart(&SD2, &sd2_config);
  shellInit();

  // Mailbox/event system removed - FOC now runs directly in ADC ISR

  adc_init();
  pwm_init();

  pwm_set_duty_cycle(1, 5000);
  pwm_set_duty_cycle(2, 5000);
  pwm_set_duty_cycle(3, 5000);

  // Initialize angle control system (PID runs in ADC ISR for minimal latency)
  // PID gains: kp=0.01, ki=0.001, kd=0.0001, max_torque=1.0A
  angle_control_init(1.0f, 0.00f, 0.0000f, 10.0f);
  angle_control_set_target(M_PI);
  angle_control_enable(true);

  foc_init(PWM_FREQ, foc_set_pwm);  // 21 kHz sample rate

  palSetLineMode(LINE_DRV_N_FAULT, PAL_MODE_INPUT_PULLDOWN);
  palSetLineMode(LINE_DRV_N_OCTW, PAL_MODE_INPUT_PULLDOWN);
  palSetLineMode(LINE_DRV_ENABLE, PAL_MODE_OUTPUT_PUSHPULL);
  palSetLineMode(LINE_DRV_M_OC, PAL_MODE_OUTPUT_PUSHPULL);
  palSetLineMode(LINE_DRV_OC_ADJ, PAL_MODE_OUTPUT_PUSHPULL);
  palSetLineMode(LINE_DRV_M_PWM, PAL_MODE_OUTPUT_PUSHPULL);
  palSetLineMode(LINE_DRV_DEBUG, PAL_MODE_OUTPUT_PUSHPULL);
  palClearLine(LINE_DRV_DEBUG);
  palSetLineMode(LINE_DRV_DEBUG2, PAL_MODE_OUTPUT_PUSHPULL);

  while (true) {
    thread_t *shell_thd = chThdCreateFromHeap(NULL, SHELL_WA_SIZE, "shell", NORMALPRIO + 1, shellThread, (void *)&shell_cfg);
    chThdWait(shell_thd);
    chThdSleepMilliseconds(1);
  }
}