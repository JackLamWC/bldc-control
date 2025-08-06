#include "ch.h"
#include "hal.h"
#include "log.h"
#include "shell.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "chprintf.h"

#define LINE_DRV_ENABLE             PAL_LINE(GPIOC, GPIOC_PIN8)
#define LINE_DRV_N_FAULT            PAL_LINE(GPIOC, GPIOC_PIN10) 
#define LINE_DRV_N_OCTW             PAL_LINE(GPIOC, GPIOC_PIN11)
#define LINE_DRV_M_OC               PAL_LINE(GPIOC, GPIOC_PIN12)
#define LINE_DRV_OC_ADJ             PAL_LINE(GPIOD, GPIOD_PIN2)
#define LINE_DRV_M_PWM              PAL_LINE(GPIOC, GPIOD_PIN9)
#define LINE_DRV_DEBUG              PAL_LINE(GPIOC, GPIOC_PIN5)


/*===========================================================================*/
/* ADC Data Structure and Mailbox                                           */
/*===========================================================================*/
typedef struct {
  uint32_t phase_current[3];
  uint32_t position_current;
  systime_t timestamp;
} adc_data_t;

#define ADC_MAILBOX_SIZE 8
static msg_t adc_mailbox_buffer[ADC_MAILBOX_SIZE];
static mailbox_t adc_mailbox;
static event_source_t adc_event_source;

#define ADC_EVENT_FLAG (1 << 1)

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
  // Enable TIM2 clock
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

  // Configure TIM2 to use external clock mode
  TIM2->PSC = 0;                    
  TIM2->ARR = 1;                    // Count to 2 (0,1) then reset

  // Configure TIM2 as external clock mode
  TIM2->SMCR &= ~TIM_SMCR_SMS_Msk;
  TIM2->SMCR |= TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0;  // External clock (111)

  // Select TIM1 as trigger source
  TIM2->SMCR &= ~TIM_SMCR_TS_Msk;

  // Enable the update event for ADC triggering
  TIM2->CR2 &= ~TIM_CR2_MMS_Msk;
  TIM2->CR2 |= TIM_CR2_MMS_1;      // Update event as TRGO

  // Enable TIM2 update interrupt
  TIM2->DIER |= TIM_DIER_UIE;      // Enable Update Interrupt Enable
  
  // Enable TIM2 counter
  TIM2->CR1 |= TIM_CR1_CEN;
  
  // Configure TIM2 interrupt in NVIC
  NVIC_EnableIRQ(TIM2_IRQn);
  NVIC_SetPriority(TIM2_IRQn, 3);  // Lower priority than ADC

  // Enable the main output
  TIM1->BDTR |= TIM_BDTR_MOE;

  // Enable the TIM1 (master timer)
  TIM1->CR1 |= TIM_CR1_CEN;
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
uint32_t debug_counter = 0;

// Pool of ADC data structures to avoid memory allocation in interrupt
static adc_data_t adc_data_pool[ADC_MAILBOX_SIZE];
static uint8_t pool_index = 0;
static adc_data_t current_adc_data; // For shell commands

CH_IRQ_HANDLER(STM32_ADC_HANDLER) {
  if (ADC1->SR & ADC_SR_JEOC) {
    palClearLine(LINE_DRV_DEBUG);
    ADC1->SR &= ~ADC_SR_JEOC; //!!! NEED TO CLEAR BIT IN ISR
    debug_counter++;
    // Get next buffer from pool (circular buffer)
    adc_data_t *data = &adc_data_pool[pool_index];
    pool_index = (pool_index + 1) % ADC_MAILBOX_SIZE;
    
    // Fill ADC data
    data->phase_current[0] = ADC1->JDR1 & ADC_JDR1_JDATA_Msk;
    data->phase_current[1] = ADC1->JDR2 & ADC_JDR2_JDATA_Msk;
    data->phase_current[2] = ADC1->JDR3 & ADC_JDR3_JDATA_Msk;
    data->position_current = ADC1->JDR4 & ADC_JDR4_JDATA_Msk;
    data->timestamp = chVTGetSystemTimeX();
    
    // Also update current_adc_data for shell commands
    current_adc_data = *data;
    
    chSysLockFromISR();
    // Post data pointer to mailbox (non-blocking)
    if (chMBPostI(&adc_mailbox, (msg_t)data) == MSG_OK) {
      // If successful, trigger event to wake up worker thread
      chEvtBroadcastI(&adc_event_source);
    }
    // If mailbox is full, oldest data is lost (but interrupt doesn't block)
    chSysUnlockFromISR();
  }
}

// TIM2 Update Interrupt Handler
CH_FAST_IRQ_HANDLER(STM32_TIM2_HANDLER) {
  if (TIM2->SR & TIM_SR_UIF) {       
    TIM2->SR &= ~TIM_SR_UIF;         
    if(TIM2->CNT == 1) {
      palSetLine(LINE_DRV_DEBUG);
    }
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


  // Select TIM2 (PWM Slave) as trigger source on rising edge
  ADC1->CR2 &= ~(ADC_CR2_JEXTSEL_Msk | ADC_CR2_JEXTEN_Msk);
  ADC1->CR2 |= ADC_CR2_JEXTSEL_1 | ADC_CR2_JEXTSEL_0;
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
#define WORKER_WA_SIZE THD_WORKING_AREA_SIZE(1024)

static THD_WORKING_AREA(worker_wa, WORKER_WA_SIZE);
static THD_FUNCTION(worker_thread, arg) {
  (void)arg;
  
  event_listener_t adc_listener;
  
  // Register to listen for ADC events
  chEvtRegister(&adc_event_source, &adc_listener, ADC_EVENT_FLAG);
  
  while (true) {
    // Wait for event notification from ADC interrupt
    eventmask_t events = chEvtWaitAny(ALL_EVENTS);
    
    if (events & ADC_EVENT_FLAG) {
      msg_t msg;
      adc_data_t *adc_data;
      
      // Process all available data from mailbox
      // This handles bursts of ADC conversions efficiently
      while (chMBFetchTimeout(&adc_mailbox, &msg, TIME_IMMEDIATE) == MSG_OK) {
        adc_data = (adc_data_t *)msg;
        
        // Process ADC data - can do blocking operations safely here
        // ADC interrupt is not blocked, so real-time performance is maintained
        
        // Example processing:
        uint32_t total_current = adc_data->phase_current[0] + 
                                adc_data->phase_current[1] + 
                                adc_data->phase_current[2];
        
        // Process position data
        float position = adc_data->position_current;
        
        // You can add:
        // - Digital filtering (moving average, IIR, etc.)
        // - Control algorithms (PID, FOC, etc.)
        // - Data logging
        // - Fault detection
        // - Any blocking operations
        
        // The adc_data pointer is valid until the next ADC_MAILBOX_SIZE conversions
        // This gives you time to process without affecting interrupt timing
      }
      
      // Clear the event flag
      chEvtGetAndClearEvents(ADC_EVENT_FLAG);
    }
  }
}

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
  chprintf(chp, "TIM2->CR1 = 0x%08lx (should have CEN=1)\n", TIM2->CR1);
  chprintf(chp, "TIM1->CNT = %lu\n", TIM1->CNT);
  chprintf(chp, "TIM2->CNT = %lu (should be counting)\n", TIM2->CNT);
  
  chThdSleepMilliseconds(100);
  chprintf(chp, "TIM2->CNT after 100ms = %lu (should be different)\n", TIM2->CNT);
  
  chprintf(chp, "\nADC Trigger Configuration:\n");
  chprintf(chp, "ADC1->CR2 = 0x%08lx\n", ADC1->CR2);
  // Temporarily commented out - might be causing hang
  // chprintf(chp, "Expected JEXTSEL = 0x%lx (TIM2_TRGO)\n", (ADC1->CR2 & ADC_CR2_JEXTSEL_Msk) >> ADC_CR2_JEXTSEL_Pos);
  // chprintf(chp, "Expected JEXTEN = 0x%lx (Rising edge)\n", (ADC1->CR2 & ADC_CR2_JEXTEN_Msk) >> ADC_CR2_JEXTEN_Pos);
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
  
  chprintf(chp, "ADC Mailbox + Events Status:\n");
  chprintf(chp, "Mailbox size: %d\n", ADC_MAILBOX_SIZE);
  chprintf(chp, "Mailbox used slots: %d\n", chMBGetUsedCountI(&adc_mailbox));
  chprintf(chp, "Mailbox free slots: %d\n", chMBGetFreeCountI(&adc_mailbox));
  chprintf(chp, "Pool index: %d\n", pool_index);
  chprintf(chp, "\nLatest ADC Data:\n");
  chprintf(chp, "Phase A: %lu, Phase B: %lu, Phase C: %lu\n", 
           current_adc_data.phase_current[0],
           current_adc_data.phase_current[1], 
           current_adc_data.phase_current[2]);
  chprintf(chp, "Position: %lu\n", 
           current_adc_data.position_current,
           current_adc_data.position_current);
  chprintf(chp, "Timestamp: %lu\n", (uint32_t)current_adc_data.timestamp);
  chprintf(chp, "Debug counter: %lu\n", debug_counter);
}

static const ShellCommand shell_commands[] = {
  {"adc", cmd_get_adc},
  {"status", cmd_adc_status},
  {"test_adc", cmd_test_adc_manual},      // Manual ADC trigger test
  {"test_pwm", cmd_check_pwm_trigger},    // PWM timer test  
  {"registers", cmd_adc_registers},       // Register dump
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

  // Initialize mailbox for ADC data passing
  chMBObjectInit(&adc_mailbox, adc_mailbox_buffer, ADC_MAILBOX_SIZE);
  
  // Initialize event source for ADC notifications
  chEvtObjectInit(&adc_event_source);

  // Start worker thread for mailbox + event processing
  chThdCreateStatic(worker_wa, sizeof(worker_wa), NORMALPRIO + 2, worker_thread, NULL);

  adc_init();
  pwm_init();
  pwm_slave_init();
  pwm_set_duty_cycle(0, 10);
  pwm_set_duty_cycle(1, 10);
  pwm_set_duty_cycle(2, 10);


  palSetLineMode(LINE_DRV_N_FAULT, PAL_MODE_INPUT_PULLDOWN);
  palSetLineMode(LINE_DRV_N_OCTW, PAL_MODE_INPUT_PULLDOWN);
  palSetLineMode(LINE_DRV_ENABLE, PAL_MODE_OUTPUT_PUSHPULL);
  palSetLineMode(LINE_DRV_M_OC, PAL_MODE_OUTPUT_PUSHPULL);
  palSetLineMode(LINE_DRV_OC_ADJ, PAL_MODE_OUTPUT_PUSHPULL);
  palSetLineMode(LINE_DRV_M_PWM, PAL_MODE_OUTPUT_PUSHPULL);
  palSetLineMode(LINE_DRV_DEBUG, PAL_MODE_OUTPUT_PUSHPULL);
  palClearLine(LINE_DRV_DEBUG);

  while (true) {
    thread_t *shell_thd = chThdCreateFromHeap(NULL, SHELL_WA_SIZE, "shell", NORMALPRIO + 1, shellThread, (void *)&shell_cfg);
    chThdWait(shell_thd);
    chThdSleepMilliseconds(500);
  }
}