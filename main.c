/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "ch.h"
#include "hal.h"
#include "log.h"
#include "shell.h"
#include <stdlib.h>
#include <math.h>
#include "chprintf.h"

#define MTR_PHASE_NUM 3
#define MTR_PHASE_A 0
#define MTR_PHASE_B 1
#define MTR_PHASE_C 2

#define LINE_DRV_ENABLE             PAL_LINE(GPIOC, GPIOC_PIN8)
#define LINE_DRV_N_FAULT            PAL_LINE(GPIOC, GPIOC_PIN10) 
#define LINE_DRV_N_OCTW             PAL_LINE(GPIOC, GPIOC_PIN11)
#define LINE_DRV_M_OC               PAL_LINE(GPIOC, GPIOC_PIN12)
#define LINE_DRV_OC_ADJ             PAL_LINE(GPIOD, GPIOD_PIN2)
#define LINE_DRV_M_PWM              PAL_LINE(GPIOC, GPIOD_PIN9)
#define LINE_DRV_DEBUG              PAL_LINE(GPIOC, GPIOC_PIN6)


#define MTR_COMM_SEQ_NUM 8

static uint32_t last_isr_cycles = 0;
static uint32_t debug_isr_duration_cycles = 0;
static uint32_t end_cycles = 0;


/*
 * Phase PWM output
 */

 #define LINE_MTR_PWM_PHASE_AH PAL_LINE(GPIOA, GPIOA_ARD_D7)
 #define LINE_MTR_PWM_PHASE_AL PAL_LINE(GPIOA, GPIOA_ARD_D11)
 #define LINE_MTR_PWM_PHASE_BH PAL_LINE(GPIOA, GPIOA_ARD_D8)
 #define LINE_MTR_PWM_PHASE_BL PAL_LINE(GPIOB, GPIOB_ARD_A3)
 #define LINE_MTR_PWM_PHASE_CH PAL_LINE(GPIOA, GPIOA_ARD_D2)
 #define LINE_MTR_PWM_PHASE_CL PAL_LINE(GPIOB, GPIOB_PIN1)
 
 #define MTR_PWM_CHANNELS 3
 #define MTR_PWM_PHASE_A 0
 #define MTR_PWM_PHASE_B 1
 #define MTR_PWM_PHASE_C 2
 
 #define MTR_PWM_CLOCK_FREQ 42000000
 #define MTR_PWM_FREQ 100000 

 static volatile uint16_t mtr_pwm_current_duty = 0;

 static uint16_t mtr_get_duty(void) {
  return mtr_pwm_current_duty;
 }

 static void mtr_set_duty(uint16_t duty) {
  mtr_pwm_current_duty = duty;
 }
 
 static PWMConfig mtr_pwm_config = {
   MTR_PWM_CLOCK_FREQ,
   MTR_PWM_CLOCK_FREQ / MTR_PWM_FREQ,
   NULL,
   {
     {PWM_OUTPUT_ACTIVE_HIGH | PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH, NULL},
     {PWM_OUTPUT_ACTIVE_HIGH | PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH, NULL},
     {PWM_OUTPUT_ACTIVE_HIGH | PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH, NULL},
     {PWM_OUTPUT_DISABLED, NULL}
   },
   0,
   0,
   0, 
 };
 
 static const uint32_t pwm_channel_enable_register_lookup[MTR_PWM_CHANNELS*2] = {
   TIM_CCER_CC1NE,
   TIM_CCER_CC1E,
   TIM_CCER_CC2NE,
   TIM_CCER_CC2E,
   TIM_CCER_CC3NE,
   TIM_CCER_CC3E,
 };
 
 static void mtr_pwm_init(void) {
   palSetLineMode(LINE_MTR_PWM_PHASE_AH, PAL_MODE_ALTERNATE(1));
   palSetLineMode(LINE_MTR_PWM_PHASE_BH, PAL_MODE_ALTERNATE(1));
   palSetLineMode(LINE_MTR_PWM_PHASE_CH, PAL_MODE_ALTERNATE(1));
 
   palSetLineMode(LINE_MTR_PWM_PHASE_AL, PAL_MODE_ALTERNATE(1));
   palSetLineMode(LINE_MTR_PWM_PHASE_BL, PAL_MODE_ALTERNATE(1));
   palSetLineMode(LINE_MTR_PWM_PHASE_CL, PAL_MODE_ALTERNATE(1));
 
 
   palClearLine(LINE_MTR_PWM_PHASE_AH);
   palClearLine(LINE_MTR_PWM_PHASE_BH);
   palClearLine(LINE_MTR_PWM_PHASE_CH);
 
   palClearLine(LINE_MTR_PWM_PHASE_AL);
   palClearLine(LINE_MTR_PWM_PHASE_BL);
   palClearLine(LINE_MTR_PWM_PHASE_CL);
 
   /*
    * Start the PWM driver
    */
   pwmStart(&PWMD1, &mtr_pwm_config);
   pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 0));
   pwmEnableChannel(&PWMD1, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 0)); 
   pwmEnableChannel(&PWMD1, 2, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 0));
 }
 
 static void mtr_pwm_set_channel_enable(uint8_t channel, bool enable) {
   uint32_t reg_offset = channel * 2;
   if (enable) {
     TIM1->CCER |= pwm_channel_enable_register_lookup[reg_offset];
     TIM1->CCER |= pwm_channel_enable_register_lookup[reg_offset + 1];
   } else {
     TIM1->CCER &= ~pwm_channel_enable_register_lookup[reg_offset];
     TIM1->CCER &= ~pwm_channel_enable_register_lookup[reg_offset + 1];
   }
 }
 
 static void mtr_pwm_set_duty(uint8_t channel, uint16_t duty) {
   pwmEnableChannel(&PWMD1, channel, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, duty));
 }


/* Motor Velocity Measurement */
#define MTR_VELOCITY_TIMER &GPTD5
#define MTR_VELOCITY_TIMER_MAX_VALUE 65535
#define MTR_VELOCITY_TIMER_FREQ 1000000

static uint16_t mtr_velocity_last_ticks_us = 0;
static uint16_t mtr_velocity_interval_us = 0;

static GPTConfig mtr_velocity_timer_config = {
  .frequency = MTR_VELOCITY_TIMER_FREQ,
  .callback = NULL,
  .cr2 = 0,
  .dier = 0,
};

static uint16_t mtr_velocity_calculate_timer_interval(uint16_t current, uint16_t last, uint16_t max_value) {
  if (current >= last) {
    return current - last;
  } else {
    return (max_value - last + 1) + current;
  }
}

static void mtr_velocity_init(void) {
  gptStart(MTR_VELOCITY_TIMER, &mtr_velocity_timer_config);
  gptStartContinuous(MTR_VELOCITY_TIMER, 65535);
}


// Assume the update event will not shorter than 1us
static uint16_t mtr_velocity_get_interval_us(void) {
  uint16_t current_ticks = gptGetCounterX(MTR_VELOCITY_TIMER);
  mtr_velocity_interval_us = mtr_velocity_calculate_timer_interval(current_ticks, mtr_velocity_last_ticks_us, MTR_VELOCITY_TIMER_MAX_VALUE);
  return mtr_velocity_interval_us;
}

static void mtr_velocity_update(uint16_t current_ticks) {
  mtr_velocity_last_ticks_us = current_ticks;
}

/* Commutation Sequence */
typedef enum {
  MTR_COMM_SEQ_IN,
  MTR_COMM_SEQ_OUT,
  MTR_COMM_SEQ_FLOAT,
} mtr_commutation_sequence_t;

typedef enum {
  MTR_COMM_SEQ_DETECT_EDGE_RISING,
  MTR_COMM_SEQ_DETECT_EDGE_FALLING,
} mtr_commutation_sequence_detect_edge_t;

typedef enum {
  MTR_COMM_SEQ_IDX_000,
  MTR_COMM_SEQ_IDX_100,
  MTR_COMM_SEQ_IDX_110,
  MTR_COMM_SEQ_IDX_010,
  MTR_COMM_SEQ_IDX_011,
  MTR_COMM_SEQ_IDX_001,
  MTR_COMM_SEQ_IDX_101,
  MTR_COMM_SEQ_IDX_111,
} mtr_commutation_sequence_index_t;

typedef struct {
  mtr_commutation_sequence_t sequence[MTR_PHASE_NUM];
  mtr_commutation_sequence_detect_edge_t detect_edge;
} mtr_commutation_sequence_config_t;

mtr_commutation_sequence_index_t mtr_commutation_sequence_state = MTR_COMM_SEQ_IDX_000;

static const mtr_commutation_sequence_config_t mtr_commutation_sequence_config[MTR_COMM_SEQ_NUM] = {
  {
    .sequence = {MTR_COMM_SEQ_FLOAT, MTR_COMM_SEQ_FLOAT, MTR_COMM_SEQ_FLOAT}, // s000
    .detect_edge = MTR_COMM_SEQ_DETECT_EDGE_RISING,
  },
  {
    .sequence = {MTR_COMM_SEQ_IN, MTR_COMM_SEQ_OUT, MTR_COMM_SEQ_FLOAT}, // s100
    .detect_edge = MTR_COMM_SEQ_DETECT_EDGE_FALLING,
  },
  {
    .sequence = {MTR_COMM_SEQ_IN, MTR_COMM_SEQ_FLOAT, MTR_COMM_SEQ_OUT}, // s110
    .detect_edge = MTR_COMM_SEQ_DETECT_EDGE_RISING,
  },
  {
    .sequence = {MTR_COMM_SEQ_FLOAT, MTR_COMM_SEQ_IN, MTR_COMM_SEQ_OUT}, // s010
    .detect_edge = MTR_COMM_SEQ_DETECT_EDGE_FALLING,
  },
  {
    .sequence = {MTR_COMM_SEQ_OUT, MTR_COMM_SEQ_IN, MTR_COMM_SEQ_FLOAT}, // s011
    .detect_edge = MTR_COMM_SEQ_DETECT_EDGE_RISING,
  },
  {
    .sequence = {MTR_COMM_SEQ_OUT, MTR_COMM_SEQ_FLOAT, MTR_COMM_SEQ_IN}, // s001
    .detect_edge = MTR_COMM_SEQ_DETECT_EDGE_FALLING,
  },
  {
    .sequence = {MTR_COMM_SEQ_FLOAT, MTR_COMM_SEQ_OUT, MTR_COMM_SEQ_IN}, // s101
    .detect_edge = MTR_COMM_SEQ_DETECT_EDGE_RISING,
  },
  {
    .sequence = {MTR_COMM_SEQ_FLOAT, MTR_COMM_SEQ_FLOAT, MTR_COMM_SEQ_FLOAT}, // s111 
    .detect_edge = MTR_COMM_SEQ_DETECT_EDGE_FALLING,
  },
};

static void mtr_commutation_sequence_set_state(mtr_commutation_sequence_index_t state) {
  mtr_commutation_sequence_state = state;
  mtr_commutation_sequence_config_t config = mtr_commutation_sequence_config[state];
  for(int i=0; i<MTR_PHASE_NUM; i++) {
    switch(config.sequence[i]) {
      case MTR_COMM_SEQ_IN:
        mtr_pwm_set_channel_enable(i, true);
        mtr_pwm_set_duty(i, 5000 + mtr_get_duty()/2);
        break;
      case MTR_COMM_SEQ_OUT:
        mtr_pwm_set_channel_enable(i, true);
        mtr_pwm_set_duty(i, 5000 - mtr_get_duty()/2);
        break;
      case MTR_COMM_SEQ_FLOAT:
        mtr_pwm_set_channel_enable(i, false);
        break;
    }
  }
}


static uint16_t debug_cnt = 0;

static void mtr_commutation_zero_crossing_set_crossed_zero(bool crossed_zero);
static void mtr_commutation_sequence_transition_state(void) {
  mtr_commutation_sequence_index_t next_state = mtr_commutation_sequence_state + 1;
  if(mtr_commutation_sequence_state == MTR_COMM_SEQ_IDX_101) {
    next_state = MTR_COMM_SEQ_IDX_100;
  }
  else {
    next_state = mtr_commutation_sequence_state + 1;
  }
  mtr_commutation_sequence_set_state(next_state);
  mtr_commutation_zero_crossing_set_crossed_zero(false);
}

static bool mtr_commutation_sequence_is_valid_state(mtr_commutation_sequence_index_t state) {
  return (state != MTR_COMM_SEQ_IDX_000 && state != MTR_COMM_SEQ_IDX_111);
}



/* Back EMF ADC */
#define MTR_BACK_EMF_ADC_DRIVER &ADCD1
#define MTR_BACK_EMF_ADC_NUM_CHANNELS 4
#define MTR_BACK_EMF_ADC_BUF_DEPTH 1

#define MTR_BACK_EMF_ADC_PHASE_A_INDEX 0
#define MTR_BACK_EMF_ADC_PHASE_B_INDEX 1
#define MTR_BACK_EMF_ADC_PHASE_C_INDEX 2
#define MTR_BACK_EMF_ADC_VPDD_INDEX 3

#define LINE_MTR_VOLTAGE_PHASE_A PAL_LINE(GPIOA, GPIOA_ARD_A0)
#define LINE_MTR_VOLTAGE_PHASE_B PAL_LINE(GPIOA, GPIOA_ARD_A1)
#define LINE_MTR_VOLTAGE_PHASE_C PAL_LINE(GPIOA, LINE_ARD_A2)
#define LINE_MTR_VOLTAGE_VPDD PAL_LINE(GPIOC, LINE_ARD_A4)

#define MTR_BACK_EMF_ADC_CHANNEL_PHASE_A ADC_CHANNEL_IN0
#define MTR_BACK_EMF_ADC_CHANNEL_PHASE_B ADC_CHANNEL_IN1
#define MTR_BACK_EMF_ADC_CHANNEL_PHASE_C ADC_CHANNEL_IN4
#define MTR_BACK_EMF_ADC_CHANNEL_VPDD ADC_CHANNEL_IN11
#define MTR_BACK_EMF_ADC_CHANNEL_SAMPLING_CYCLES ADC_SAMPLE_56

#define MTR_BACK_EMF_ADC_TRIGGER_GPT_DRIVER &GPTD3
#define MTR_BACK_EMF_ADC_TRIGGER_GPT_CLOCK_FREQ 2000000
#define MTR_BACK_EMF_ADC_TRIGGER_GPT_FREQ 8000

static void mtr_commutation_zero_crossing_update(void);
static void mtr_back_emf_average_add_sample(void);
static void mtr_back_emf_adc_cb(ADCDriver *adcp) {
  (void)adcp;
  mtr_back_emf_average_add_sample();
  mtr_commutation_zero_crossing_update();
}

static adcsample_t mtr_back_emf_adc_samples[MTR_BACK_EMF_ADC_NUM_CHANNELS * MTR_BACK_EMF_ADC_BUF_DEPTH];

static ADCConversionGroup mtr_voltage_adc_groupConfig = {
  true,
  MTR_BACK_EMF_ADC_NUM_CHANNELS,
  mtr_back_emf_adc_cb,
  NULL,
  0,
  ADC_CR2_EXTEN_RISING | ADC_CR2_EXTSEL_SRC(8),
  ADC_SMPR1_SMP_AN11(MTR_BACK_EMF_ADC_CHANNEL_SAMPLING_CYCLES),
  ADC_SMPR2_SMP_AN0(MTR_BACK_EMF_ADC_CHANNEL_SAMPLING_CYCLES) | ADC_SMPR2_SMP_AN1(MTR_BACK_EMF_ADC_CHANNEL_SAMPLING_CYCLES) | ADC_SMPR2_SMP_AN4(MTR_BACK_EMF_ADC_CHANNEL_SAMPLING_CYCLES),
  0,
  0,
  0,
  0,
  ADC_SQR3_SQ1_N(MTR_BACK_EMF_ADC_CHANNEL_PHASE_A) | ADC_SQR3_SQ2_N(MTR_BACK_EMF_ADC_CHANNEL_PHASE_B) | ADC_SQR3_SQ3_N(MTR_BACK_EMF_ADC_CHANNEL_PHASE_C) | ADC_SQR3_SQ4_N(MTR_BACK_EMF_ADC_CHANNEL_VPDD),
};

static GPTConfig mtr_back_emf_adc_trigger_source_config = {
  .frequency = MTR_BACK_EMF_ADC_TRIGGER_GPT_CLOCK_FREQ, // 2MHz
  .callback = NULL,
  .cr2 = TIM_CR2_MMS_1,
  .dier = 0,
};

static void mtr_back_emf_init(void) {
  palSetLineMode(LINE_MTR_VOLTAGE_PHASE_A, PAL_MODE_INPUT_ANALOG);
  palSetLineMode(LINE_MTR_VOLTAGE_PHASE_B, PAL_MODE_INPUT_ANALOG);
  palSetLineMode(LINE_MTR_VOLTAGE_PHASE_C, PAL_MODE_INPUT_ANALOG);
  palSetLineMode(LINE_MTR_VOLTAGE_VPDD, PAL_MODE_INPUT_ANALOG);

  adcStart(MTR_BACK_EMF_ADC_DRIVER, NULL);
  gptStart(MTR_BACK_EMF_ADC_TRIGGER_GPT_DRIVER, &mtr_back_emf_adc_trigger_source_config);
  adcStartConversion(MTR_BACK_EMF_ADC_DRIVER, &mtr_voltage_adc_groupConfig, mtr_back_emf_adc_samples, MTR_BACK_EMF_ADC_BUF_DEPTH);
  gptStartContinuous(MTR_BACK_EMF_ADC_TRIGGER_GPT_DRIVER, MTR_BACK_EMF_ADC_TRIGGER_GPT_CLOCK_FREQ / MTR_BACK_EMF_ADC_TRIGGER_GPT_FREQ);
}

static float mtr_back_emf_get_voltage(uint8_t channel) {
  return (float)mtr_back_emf_adc_samples[channel] / 4095.0 * 3.3;
}

/*===========================================================================*/
/* Back-EMF voltage averaging for noise reduction                             */
/*===========================================================================*/
#define MTR_BACK_EMF_AVERAGE_BUFFER_SIZE 4  // Number of samples to average per channel
static float mtr_back_emf_average_buffer[MTR_BACK_EMF_ADC_NUM_CHANNELS][MTR_BACK_EMF_AVERAGE_BUFFER_SIZE];
static uint8_t mtr_back_emf_average_buffer_index = 0;
static uint8_t mtr_back_emf_average_buffer_count = 0;
static float mtr_back_emf_average[MTR_BACK_EMF_ADC_NUM_CHANNELS] = {0};

/**
 * @brief Add a new back-EMF voltage measurement to the averaging buffer for all channels
 */
static void mtr_back_emf_average_add_sample(void) {
  // Add new samples to buffer for all channels
  for (uint8_t ch = 0; ch < MTR_BACK_EMF_ADC_NUM_CHANNELS; ch++) {
    float voltage = mtr_back_emf_get_voltage(ch);
    mtr_back_emf_average_buffer[ch][mtr_back_emf_average_buffer_index] = voltage;
  }
  
  mtr_back_emf_average_buffer_index = (mtr_back_emf_average_buffer_index + 1) % MTR_BACK_EMF_AVERAGE_BUFFER_SIZE;
  
  // Update count (max out at buffer size)
  if (mtr_back_emf_average_buffer_count < MTR_BACK_EMF_AVERAGE_BUFFER_SIZE) {
    mtr_back_emf_average_buffer_count++;
  }
  
  // Calculate new averages for all channels
  for (uint8_t ch = 0; ch < MTR_BACK_EMF_ADC_NUM_CHANNELS; ch++) {
    float sum = 0.0f;
    for (uint8_t i = 0; i < mtr_back_emf_average_buffer_count; i++) {
      sum += mtr_back_emf_average_buffer[ch][i];
    }
    mtr_back_emf_average[ch] = sum / (float)mtr_back_emf_average_buffer_count;
  }
}

/**
 * @brief Get the current averaged back-EMF voltage for a specific channel
 * @param channel Channel number (0-3)
 * @return Averaged voltage in volts
 */
static float mtr_back_emf_average_get(uint8_t channel) {
  if (channel >= MTR_BACK_EMF_ADC_NUM_CHANNELS) {
    return 0.0f;
  }
  return mtr_back_emf_average[channel];
}

/**
 * @brief Reset the back-EMF voltage averaging buffer for all channels
 */
static void mtr_back_emf_average_reset(void) {
  mtr_back_emf_average_buffer_index = 0;
  mtr_back_emf_average_buffer_count = 0;
  for (uint8_t ch = 0; ch < MTR_BACK_EMF_ADC_NUM_CHANNELS; ch++) {
    mtr_back_emf_average[ch] = 0.0f;
    for (uint8_t i = 0; i < MTR_BACK_EMF_AVERAGE_BUFFER_SIZE; i++) {
      mtr_back_emf_average_buffer[ch][i] = 0.0f;
    }
  }
}

/* Commutation Zero Crossing Detection */
#define MTR_COMM_ZERO_CROSSING_DETECTION_THRESHOLD 0.5
#define MTR_COMM_ZERO_CROSSING_DELAY_TIMER &GPTD11
#define MTR_COMM_ZERO_LOWEST_INTERVAL_US 1000

static bool mtr_commutation_crossed_zero = false;

static void mtr_commutation_zero_crossing_set_crossed_zero(bool crossed_zero) {
  mtr_commutation_crossed_zero = crossed_zero;
}

static void mtr_commutation_zero_crossing_delay_timer_cb(GPTDriver *gpt) {
  (void)gpt;
  mtr_commutation_sequence_transition_state();
  
}

static GPTConfig mtr_commutation_zero_crossing_delay_timer_config = {
  .frequency = 1000000,
  .callback = mtr_commutation_zero_crossing_delay_timer_cb,
  .cr2 = 0,
  .dier = 0,
};



static bool mtr_commutation_zero_crossing_is_valid(void) {
  debug_cnt++;
  uint8_t float_phase = 0;
  for(int i=0; i<MTR_PHASE_NUM; i++) {
    if(mtr_commutation_sequence_config[mtr_commutation_sequence_state].sequence[i] == MTR_COMM_SEQ_FLOAT) {
      float_phase = i;
      break;
    }
  }
  float float_phase_voltage = mtr_back_emf_average_get(float_phase);
  float vpdd_voltage = mtr_back_emf_average_get(MTR_BACK_EMF_ADC_VPDD_INDEX);
  mtr_commutation_sequence_detect_edge_t detect_edge = mtr_commutation_sequence_config[mtr_commutation_sequence_state].detect_edge;
  if(detect_edge == MTR_COMM_SEQ_DETECT_EDGE_RISING) {
    return float_phase_voltage >= vpdd_voltage * MTR_COMM_ZERO_CROSSING_DETECTION_THRESHOLD;
  } else {
    return float_phase_voltage <= vpdd_voltage * MTR_COMM_ZERO_CROSSING_DETECTION_THRESHOLD;
  }
}

static void mtr_commutation_zero_crossing_commutate_with_delay_us(uint16_t delay_us) {
  gptStopTimer(MTR_COMM_ZERO_CROSSING_DELAY_TIMER);
  gptStartOneShotI(MTR_COMM_ZERO_CROSSING_DELAY_TIMER, delay_us);
}

static void mtr_velocity_interval_average_add_sample(uint16_t current_interval_us);
static uint16_t mtr_velocity_interval_average_get(void);
static void mtr_commutation_zero_crossing_update(void) {
  if(mtr_commutation_crossed_zero) {
    return;
  }

  if(!mtr_commutation_sequence_is_valid_state(mtr_commutation_sequence_state)) {
    return;
  }

  if(mtr_commutation_zero_crossing_is_valid()) {
    uint16_t current_interval_us = mtr_velocity_get_interval_us();
    if(current_interval_us < MTR_COMM_ZERO_LOWEST_INTERVAL_US) {
      return;
    }
    mtr_velocity_update(gptGetCounterX(MTR_VELOCITY_TIMER));
    mtr_velocity_interval_average_add_sample(current_interval_us);
    mtr_commutation_zero_crossing_set_crossed_zero(true);
    mtr_commutation_zero_crossing_commutate_with_delay_us((uint16_t)(mtr_velocity_interval_average_get()/2));
  }
}

static void mtr_commutation_zero_crossing_init(void) {
  gptStart(MTR_COMM_ZERO_CROSSING_DELAY_TIMER, &mtr_commutation_zero_crossing_delay_timer_config);
}

/*===========================================================================*/
/* Motor velocity interval averaging for noise reduction                       */
/*===========================================================================*/
#define MTR_VELOCITY_INTERVAL_AVERAGE_BUFFER_SIZE 48  // Number of velocity samples to average
static uint16_t mtr_velocity_interval_average_buffer[MTR_VELOCITY_INTERVAL_AVERAGE_BUFFER_SIZE];
static uint8_t mtr_velocity_interval_average_buffer_index = 0;
static uint8_t mtr_velocity_interval_average_buffer_count = 0;
static uint16_t mtr_velocity_interval_average = 0;
/**
 * @brief Add a new velocity interval measurement to the averaging buffer
 */
static void mtr_velocity_interval_average_add_sample(uint16_t current_interval_us) {
  mtr_velocity_interval_average_buffer[mtr_velocity_interval_average_buffer_index] = current_interval_us;
  
  mtr_velocity_interval_average_buffer_index = (mtr_velocity_interval_average_buffer_index + 1) % MTR_VELOCITY_INTERVAL_AVERAGE_BUFFER_SIZE;
  
  // Update count (max out at buffer size)
  if (mtr_velocity_interval_average_buffer_count < MTR_VELOCITY_INTERVAL_AVERAGE_BUFFER_SIZE) {
    mtr_velocity_interval_average_buffer_count++;
  }
  
  // Calculate new average
  uint32_t sum = 0;
  for (uint8_t i = 0; i < mtr_velocity_interval_average_buffer_count; i++) {
    sum += mtr_velocity_interval_average_buffer[i];
  }
  mtr_velocity_interval_average = (uint16_t)(sum / mtr_velocity_interval_average_buffer_count);
}

/**
 * @brief Get the current averaged velocity interval
 * @return Averaged velocity interval in microseconds
 */
static uint16_t mtr_velocity_interval_average_get(void) {
  return mtr_velocity_interval_average;
}

/**
 * @brief Reset the velocity interval averaging buffer
 */
static void mtr_velocity_interval_average_reset(void) {
  mtr_velocity_interval_average_buffer_index = 0;
  mtr_velocity_interval_average_buffer_count = 0;
  mtr_velocity_interval_average = 0;
  for (uint8_t i = 0; i < MTR_VELOCITY_INTERVAL_AVERAGE_BUFFER_SIZE; i++) {
    mtr_velocity_interval_average_buffer[i] = 0;
  }
}

static void cmd_run(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)chp;
  (void)argc;
  (void)argv;
  if(argc == 1) {
    uint16_t input_duty = atoi(argv[0]) * 100;
    LOG("mtr-run: %d\r\n", input_duty);
    if(input_duty != 0) {
      if(mtr_get_duty() == 0) {
        mtr_set_duty(input_duty);
        mtr_commutation_sequence_set_state(MTR_COMM_SEQ_IDX_100);
        mtr_back_emf_average_reset();
        mtr_velocity_interval_average_reset();
      } else {
        mtr_set_duty(input_duty);
      }
    }
    else {
      mtr_set_duty(0); 
      mtr_commutation_sequence_set_state(MTR_COMM_SEQ_IDX_000);
    }
  }
  else {
    chprintf(chp, "Usage: mtr-run <duty>\r\n");
  }
}

static void cmd_adc(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)chp;
  (void)argc;
  (void)argv;
  for(int i=0; i<MTR_BACK_EMF_ADC_NUM_CHANNELS; i++) {
    chprintf(chp, "adc%d: %f\r\n", i, mtr_back_emf_average_get(i));
  }
}

static void cmd_mtr_debug(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)chp;
  (void)argc;
  (void)argv;
  LOG("mtr-debug: %d\r\n", debug_cnt);
  LOG("mtr-commutation-cross-zero: %d\r\n", mtr_commutation_crossed_zero);
  LOG("mtr_commutation_sequence_state: %d\r\n", mtr_commutation_sequence_state);
  LOG("current valid state: %d\r\n", mtr_commutation_sequence_is_valid_state(mtr_commutation_sequence_state));
}

/*===========================================================================*/
/* Shell                                                                     */
/*===========================================================================*/
#define SHELL_WA_SIZE THD_WORKING_AREA_SIZE(2048)
static const ShellCommand shell_commands[] = {
  {"mtr-run", cmd_run},
  {"adc", cmd_adc},
  {"mtr-debug", cmd_mtr_debug},
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

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  palSetLineMode(LINE_DRV_N_FAULT, PAL_MODE_INPUT_PULLDOWN);
  palSetLineMode(LINE_DRV_N_OCTW, PAL_MODE_INPUT_PULLDOWN);
  palSetLineMode(LINE_DRV_ENABLE, PAL_MODE_OUTPUT_PUSHPULL);
  palSetLineMode(LINE_DRV_M_OC, PAL_MODE_OUTPUT_PUSHPULL);
  palSetLineMode(LINE_DRV_OC_ADJ, PAL_MODE_OUTPUT_PUSHPULL);
  palSetLineMode(LINE_DRV_M_PWM, PAL_MODE_OUTPUT_PUSHPULL);
  palSetLineMode(LINE_DRV_DEBUG, PAL_MODE_OUTPUT_PUSHPULL);

  palClearLine(LINE_DRV_M_OC);
  palSetLine(LINE_DRV_OC_ADJ);
  palClearLine(LINE_DRV_M_PWM);

  sdStart(&SD2, &sd2_config);
  shellInit();
  mtr_pwm_init();
  mtr_back_emf_init();
  mtr_velocity_init();
  mtr_commutation_zero_crossing_init();
  
  // Initialize averaging buffers
  mtr_back_emf_average_reset();
  mtr_velocity_interval_average_reset();

  while (true) {
    thread_t *shell_thd = chThdCreateFromHeap(NULL, SHELL_WA_SIZE, "shell", NORMALPRIO + 1, shellThread, (void *)&shell_cfg);
    chThdWait(shell_thd);
    chThdSleepMilliseconds(500);
  }
}