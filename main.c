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
  TIM1->CR1 |= TIM_CR1_CMS_1;

  // Set the compare mode to PWM to channel 1
  TIM1->CCMR1 &= ~TIM_CCMR1_OC1M_Msk;
  TIM1->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE;

  // Set the compare mode to PWM to channel 2
  TIM1->CCMR1 &= ~TIM_CCMR1_OC2M_Msk;
  TIM1->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE;

  // Set the compare mode to PWM to channel 3
  TIM1->CCMR2 &= ~TIM_CCMR2_OC3M_Msk;
  TIM1->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3PE;

  // Set the prescaler to 0 for 84MHz clock
  TIM1->PSC = 0;

  // Set the period to 84MHz / (21 * 2)kHz = 4000 for center aligned mode
  TIM1->ARR = PWM_CLOCK_FREQ / (PWM_FREQ * 2);

  // Initialize the duty cycle to 0
  TIM1->CCR1 = 0;
  TIM1->CCR2 = 0;
  TIM1->CCR3 = 0;
  TIM1->CCR4 = 0;

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
  TIM1->CR2 |= TIM_CR2_MMS_1;
}

static void pwm_stop(void) {
  // Disable the TIM1
  TIM1->CR1 &= ~TIM_CR1_CEN;
}

static void pwm_set_duty_cycle(uint8_t channel, uint16_t duty_cycle) {
  uint16_t duty_cycle_value = (duty_cycle * TIM1->ARR) / 100.0f;
  if (channel == 0) {
    TIM1->CCR1 = duty_cycle_value;
  } else if (channel == 1) {
    TIM1->CCR2 = duty_cycle_value;
  } else if (channel == 2) {
    TIM1->CCR3 = duty_cycle_value;
  }
}

static void pwm_slave_init(void) {
  // Enable TIM4 clock
  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

  // Configure TIM4 to use external clock mode
  TIM4->PSC = 0;                    
  TIM4->ARR = 1;                    // Count to 2 (0,1) then reset

  // Configure TIM4 as external clock mode
  TIM4->SMCR &= ~TIM_SMCR_SMS_Msk;
  TIM4->SMCR |= TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0;  // External clock (111)

  // Select TIM1 as trigger source
  TIM4->SMCR &= ~TIM_SMCR_TS_Msk;

  // Enable the update event for ADC triggering
  TIM4->CR2 &= ~TIM_CR2_MMS_Msk;
  TIM4->CR2 |= TIM_CR2_MMS_1;      // Update event as TRGO

  // Enable TIM4 update interrupt
  TIM4->DIER |= TIM_DIER_UIE;      // Enable Update Interrupt Enable
  
  // Enable TIM4 counter
  TIM4->CR1 |= TIM_CR1_CEN;
  
  // Configure TIM4 interrupt in NVIC
  NVIC_EnableIRQ(TIM4_IRQn);
  NVIC_SetPriority(TIM4_IRQn, 3);  // Lower priority than ADC

  // Enable the main output
  TIM1->BDTR |= TIM_BDTR_MOE;

  // Enable the TIM1 (master timer)
  TIM1->CR1 |= TIM_CR1_CEN;
}

/*===========================================================================*/
/* FOC                                                                     */
/*===========================================================================*/
#include "foc.h"

static uint16_t next_duty_a_pwm = 0;
static uint16_t next_duty_b_pwm = 0;
static uint16_t next_duty_c_pwm = 0;

static void foc_set_pwm(uint16_t duty_a, uint16_t duty_b, uint16_t duty_c) {
  uint16_t duty_a_pwm = (duty_a * TIM1->ARR) / 10000.0f;
  uint16_t duty_b_pwm = (duty_b * TIM1->ARR) / 10000.0f;
  uint16_t duty_c_pwm = (duty_c * TIM1->ARR) / 10000.0f;

  next_duty_a_pwm = duty_a_pwm;
  next_duty_b_pwm = duty_b_pwm;
  next_duty_c_pwm = duty_c_pwm;
}



/*===========================================================================*/
/* Angle Control                                                             */
/*===========================================================================*/

// AS5600 encoder configuration
#define AS5600_RESOLUTION 4096
#define AS5600_TO_RADIANS (2.0f * M_PI / AS5600_RESOLUTION)

// Angle control configuration
#define ANGLE_CONTROL_FREQUENCY_HZ 50.0f
#define ANGLE_CONTROL_DT (1.0f / ANGLE_CONTROL_FREQUENCY_HZ)
#define ANGLE_CONTROL_WA_SIZE THD_WORKING_AREA_SIZE(512)

// Angle control state structure
typedef struct {
  pid_controller_t angle_pid;     // Reuse FOC PID structure
  float target_angle;             // Target angle in radians
  float current_angle;            // Current angle in radians
  float angle_error;              // For monitoring
  float torque_output;            // Torque output (current in Amperes, proportional to torque)
  bool enabled;                   // Control enabled flag
  systime_t last_update_time;     // For timing
} angle_control_t;

// Global angle control instance
static angle_control_t angle_control = {0};

// Update encoder position from ADC data (called from ADC processing)
static void angle_control_update_position(uint16_t position_adc) {
  // Convert ADC reading to angle in radians
  angle_control.current_angle = position_adc * AS5600_TO_RADIANS;
}

// Initialize angle control system
static void angle_control_init(float kp, float ki, float kd, float max_torque) {
  // Initialize PID controller using FOC functions (output is torque via current in Amperes)
  pid_init(&angle_control.angle_pid, kp, ki, kd, ANGLE_CONTROL_DT, -max_torque, max_torque);
  
  angle_control.target_angle = 0.0f;
  angle_control.current_angle = 0.0f;
  angle_control.angle_error = 0.0f;
  angle_control.torque_output = 0.0f;
  angle_control.enabled = false;
  angle_control.last_update_time = chVTGetSystemTimeX();
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

// Angle control thread
static THD_WORKING_AREA(angle_control_wa, ANGLE_CONTROL_WA_SIZE);
static THD_FUNCTION(angle_control_thread, arg) {
  (void)arg;
  
  systime_t time = chVTGetSystemTimeX();
  
  while (true) {
    // Wait for next control period (50 Hz = 20ms)
    time = chThdSleepUntilWindowed(time, time + TIME_MS2I(20));
    
    if (angle_control.enabled) {
      // Current angle is updated by ADC processing, just calculate error
      // Calculate error (handle wrap-around for angle)
      angle_control.angle_error = angle_control.target_angle - angle_control.current_angle;
      
      // Handle angle wrap-around (-π to π)
      while (angle_control.angle_error > M_PI) {
        angle_control.angle_error -= 2.0f * M_PI;
      }
      while (angle_control.angle_error < -M_PI) {
        angle_control.angle_error += 2.0f * M_PI;
      }
      
      // Update PID controller
      angle_control.torque_output = pid_update(&angle_control.angle_pid, 
                                              angle_control.target_angle, 
                                              angle_control.current_angle);
      
      // Send torque command to FOC (via current in Amperes)
      // Note: Torque is proportional to current (Torque = Kt × Current)
      // The FOC will normalize this: q_ref_normalized = torque_output / FOC_MAX_CURRENT
      // foc_update(angle_control.current_angle, angle_control.torque_output / FOC_MAX_CURRENT);
      
    } else {
      // When disabled, output zero torque
      angle_control.torque_output = 0.0f;
    }
  }
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
    
    // === SECTION 2: Angle Control Update ===
    angle_control_update_position(position);
    
    checkpoint = DWT->CYCCNT;
    timing_angle_update = checkpoint - start_cycles - timing_adc_read;
    
    // === SECTION 3: ADC to Float Conversions ===
    float ia_volts = (phase_a_current * 3.3f) / 4095.0f;
    float ib_volts = (phase_b_current * 3.3f) / 4095.0f;
    float ic_volts = (phase_c_current * 3.3f) / 4095.0f;
    
    // Convert AS5600 position (0-4095) to radians (0-2π) for FOC
    float position_radians = position * AS5600_TO_RADIANS;
    
    // Get torque reference from angle control (thread-safe read)
    float q_ref_normalized = angle_control.torque_output;
    
    checkpoint = DWT->CYCCNT;
    timing_conversions = checkpoint - start_cycles - timing_adc_read - timing_angle_update;
    
    // === SECTION 4: FOC Computation ===
    foc_update_optimized(position_radians, q_ref_normalized, ia_volts, ib_volts, ic_volts);
    
    checkpoint = DWT->CYCCNT;
    timing_foc_computation = checkpoint - start_cycles - timing_adc_read - timing_angle_update - timing_conversions;
    
    // Measure total ISR duration
    debug_isr_duration_cycles = DWT->CYCCNT - start_cycles;
    debug_counter++;
  }
}

// TIM4 Update Interrupt Handler
CH_FAST_IRQ_HANDLER(STM32_TIM4_HANDLER) {
  if (TIM4->SR & TIM_SR_UIF) {       
    TIM4->SR &= ~TIM_SR_UIF;
    last_isr_cycles = DWT->CYCCNT;
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
  ADC1->CR2 |= ADC_CR2_JEXTSEL_3 | ADC_CR2_JEXTSEL_0;
  ADC1->CR2 |= ADC_CR2_JEXTEN_0;

  // Enable ADC Conversion
  ADC1->CR2 |= ADC_CR2_ADON;

  // Configure the sampling cycles
  // 100 -> 84 CYCLE

  ADC1->SMPR1 &= ~(ADC_SMPR1_SMP11_Msk);
  // ADC1->SMPR1 |= ADC_SMPR1_SMP11_2;

  ADC1->SMPR2 &= ~(ADC_SMPR2_SMP0_Msk | ADC_SMPR2_SMP1_Msk | ADC_SMPR2_SMP4_Msk);
  // ADC1->SMPR2 |= ADC_SMPR2_SMP0_2 | ADC_SMPR2_SMP1_2 | ADC_SMPR2_SMP4_2;

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
/* Worker Thread for Mailbox + Event Processing                             */
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
  chprintf(chp, "Phase A: %lu\n", current_adc_data.phase_current[0]);
  chprintf(chp, "Phase B: %lu\n", current_adc_data.phase_current[1]);
  chprintf(chp, "Phase C: %lu\n", current_adc_data.phase_current[2]);
  chprintf(chp, "Position: %lu\n", current_adc_data.position_current);
  chprintf(chp, "Timestamp: %lu\n", (uint32_t)current_adc_data.timestamp);
}

static void cmd_adc_status(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)argc;
  (void)argv;
  
  chprintf(chp, "ADC + FOC Status (now in ISR):\n");
  // chprintf(chp, "\nLatest ADC Data:\n");
  // chprintf(chp, "Phase A: %lu, Phase B: %lu, Phase C: %lu\n", 
  //          current_adc_data.phase_current[0],
  //          current_adc_data.phase_current[1], 
  //          current_adc_data.phase_current[2]);
  // chprintf(chp, "Position: %lu\n", 
  //          current_adc_data.position_current,
  //          current_adc_data.position_current);
  chprintf(chp, "Timestamp: %lu\n", (uint32_t)current_adc_data.timestamp);
  chprintf(chp, "Debug counter: %lu\n", debug_counter);
  chprintf(chp, "Last ISR duration = %lu cycles (including FOC)\n", debug_isr_duration_cycles);
  chprintf(chp, "Next duty A: %lu, Next duty B: %lu, Next duty C: %lu\n", 
           next_duty_a_pwm,
           next_duty_b_pwm,
           next_duty_c_pwm);
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
  
  chprintf(chp, "Section 2 - Angle Control Update:\n");
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
    chprintf(chp, "Example: angle_set 90.0\n");
    return;
  }
  
  float target_degrees = atof(argv[0]);
  float target_radians = target_degrees * M_PI / 180.0f;
  
  angle_control_set_target(target_radians);
  chprintf(chp, "Angle target set to %.2f degrees (%.3f rad)\n", target_degrees, target_radians);
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
  chprintf(chp, "  Current: %.2f deg (%.3f rad)\n", current_deg, angle_control_get_current_angle());
  chprintf(chp, "  Target:  %.2f deg (%.3f rad)\n", target_deg, angle_control.target_angle);
  chprintf(chp, "  Error:   %.2f deg (%.3f rad)\n", error_deg, angle_control_get_error());
  chprintf(chp, "  Torque Output: %.3f A (current ∝ torque)\n", output);
  chprintf(chp, "  Raw ADC: %lu\n", current_adc_data.position_current);
}

static const ShellCommand shell_commands[] = {
  {"adc", cmd_get_adc},
  {"status", cmd_adc_status},
  {"timing", cmd_timing_profile},         // ISR timing breakdown
  {"foc_timing", cmd_foc_timing},         // Detailed FOC timing
  {"test_adc", cmd_test_adc_manual},      // Manual ADC trigger test
  {"test_pwm", cmd_check_pwm_trigger},    // PWM timer test  
  {"registers", cmd_adc_registers},       // Register dump
  {"angle_set", cmd_angle_set},           // Set angle target
  {"angle_enable", cmd_angle_enable},     // Enable/disable angle control
  {"angle_status", cmd_angle_status},     // Get angle status
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
  pwm_slave_init();
  pwm_set_duty_cycle(0, 10);
  pwm_set_duty_cycle(1, 10);
  pwm_set_duty_cycle(2, 10);

  // Initialize angle control system 
  // PID gains: kp=0.01, ki=0.001, kd=0.0001, max_torque=1.0A
  angle_control_init(0.01f, 0.001f, 0.0001f, 1.0f);

  foc_init(PWM_FREQ, foc_set_pwm);  // 21 kHz sample rate
  
  // Start angle control thread
  chThdCreateStatic(angle_control_wa, sizeof(angle_control_wa), NORMALPRIO + 1, angle_control_thread, NULL);

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