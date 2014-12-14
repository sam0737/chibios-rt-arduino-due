/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

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

/**
 * @file    SAM3XA/gpt_lld.c
 * @brief   STM32 GPT subsystem low level driver source.
 *
 * @addtogroup GPT
 * @{
 */

#include "ch.h"
#include "hal.h"

#if HAL_USE_GPT || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   GPTD1 driver identifier.
 * @note    The driver GPTD1 allocates the complex timer TC0 when enabled.
 */
#if SAM3XA_GPT_USE_TC0 || defined(__DOXYGEN__)
GPTDriver GPTD1 = {
    .peripheral_id = ID_TC0,
    .irq_id = TC0_IRQn,
    .counter = TC0,
    .channel = &TC0->TC_CHANNEL[0]
};
#endif

/**
 * @brief   GPTD2 driver identifier.
 * @note    The driver GPTD2 allocates the timer TC1 when enabled.
 */
#if SAM3XA_GPT_USE_TC1 || defined(__DOXYGEN__)
GPTDriver GPTD2 = {
    .peripheral_id = ID_TC1,
    .irq_id = TC1_IRQn,
    .counter = NULL,
    .channel = &TC0->TC_CHANNEL[1]
};
#endif

/**
 * @brief   GPTD3 driver identifier.
 * @note    The driver GPTD3 allocates the timer TC2 when enabled.
 */
#if SAM3XA_GPT_USE_TC2 || defined(__DOXYGEN__)
GPTDriver GPTD3 = {
    .peripheral_id = ID_TC2,
    .irq_id = TC2_IRQn,
    .counter = NULL,
    .channel = &TC0->TC_CHANNEL[2]
};
#endif

/**
 * @brief   GPTD4 driver identifier.
 * @note    The driver GPTD4 allocates the timer TC3 when enabled.
 */
#if SAM3XA_GPT_USE_TC3 || defined(__DOXYGEN__)
GPTDriver GPTD4 = {
    .peripheral_id = ID_TC3,
    .irq_id = TC3_IRQn,
    .counter = TC1,
    .channel = &TC1->TC_CHANNEL[0]
};
#endif

/**
 * @brief   GPTD5 driver identifier.
 * @note    The driver GPTD5 allocates the timer TC4 when enabled.
 */
#if SAM3XA_GPT_USE_TC4 || defined(__DOXYGEN__)
GPTDriver GPTD5 = {
    .peripheral_id = ID_TC4,
    .irq_id = TC4_IRQn,
    .counter = NULL,
    .channel = &TC1->TC_CHANNEL[1]
};
#endif

/**
 * @brief   GPTD6 driver identifier.
 * @note    The driver GPTD6 allocates the timer TC5 when enabled.
 */
#if SAM3XA_GPT_USE_TC5 || defined(__DOXYGEN__)
GPTDriver GPTD6 = {
    .peripheral_id = ID_TC5,
    .irq_id = TC5_IRQn,
    .counter = NULL,
    .channel = &TC1->TC_CHANNEL[2]
};
#endif

/**
 * @brief   GPTD7 driver identifier.
 * @note    The driver GPTD7 allocates the timer TC5 when enabled.
 */
#if SAM3XA_GPT_USE_TC6 || defined(__DOXYGEN__)
GPTDriver GPTD7 = {
    .peripheral_id = ID_TC6,
    .irq_id = TC6_IRQn,
    .counter = TC2,
    .channel = &TC2->TC_CHANNEL[0]
};
#endif

/**
 * @brief   GPTD8 driver identifier.
 * @note    The driver GPTD8 allocates the timer TC6 when enabled.
 */
#if SAM3XA_GPT_USE_TC7 || defined(__DOXYGEN__)
GPTDriver GPTD8 = {
    .peripheral_id = ID_TC7,
    .irq_id = TC7_IRQn,
    .counter = NULL,
    .channel = &TC2->TC_CHANNEL[1]
};
#endif

/**
 * @brief   GPTD9 driver identifier.
 * @note    The driver GPTD9 allocates the timer TC7 when enabled.
 */
#if SAM3XA_GPT_USE_TC8 || defined(__DOXYGEN__)
GPTDriver GPTD9 = {
    .peripheral_id = ID_TC8,
    .irq_id = TC8_IRQn,
    .counter = NULL,
    .channel = &TC2->TC_CHANNEL[2]
};
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Shared IRQ handler.
 *
 * @param[in] gptp      pointer to a @p GPTDriver object
 */
static void gpt_lld_serve_interrupt(GPTDriver *gptp) {
  uint32_t status = gptp->channel->TC_SR;
  (void)status;
  if (gptp->state == GPT_ONESHOT) {
    gptp->state = GPT_READY;                /* Back in GPT_READY state.     */
    // Timer is stopped by hardware RC comparison
  }
  gptp->config->callback(gptp);
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if SAM3XA_GPT_USE_TC0
/**
 * @brief   TC0 interrupt handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(SAM3XA_TC0_HANDLER) {
  CH_IRQ_PROLOGUE();
  gpt_lld_serve_interrupt(&GPTD1);
  CH_IRQ_EPILOGUE();
}
#endif /* SAM3XA_GPT_USE_TC0 */

#if SAM3XA_GPT_USE_TC1
/**
 * @brief   TC1 interrupt handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(SAM3XA_TC1_HANDLER) {
  CH_IRQ_PROLOGUE();
  gpt_lld_serve_interrupt(&GPTD2);
  CH_IRQ_EPILOGUE();
}
#endif /* SAM3XA_GPT_USE_TC1 */


#if SAM3XA_GPT_USE_TC2
/**
 * @brief   TC2 interrupt handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(SAM3XA_TC2_HANDLER) {
  CH_IRQ_PROLOGUE();
  gpt_lld_serve_interrupt(&GPTD3);
  CH_IRQ_EPILOGUE();
}
#endif /* SAM3XA_GPT_USE_TC2 */

#if SAM3XA_GPT_USE_TC3
/**
 * @brief   TC3 interrupt handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(SAM3XA_TC3_HANDLER) {
  CH_IRQ_PROLOGUE();
  gpt_lld_serve_interrupt(&GPTD4);
  CH_IRQ_EPILOGUE();
}
#endif /* SAM3XA_GPT_USE_TC3 */

#if SAM3XA_GPT_USE_TC4
/**
 * @brief   TC4 interrupt handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(SAM3XA_TC4_HANDLER) {
  CH_IRQ_PROLOGUE();
  gpt_lld_serve_interrupt(&GPTD5);
  CH_IRQ_EPILOGUE();
}
#endif /* SAM3XA_GPT_USE_TC4 */

#if SAM3XA_GPT_USE_TC5
/**
 * @brief   TC5 interrupt handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(SAM3XA_TC5_HANDLER) {
  CH_IRQ_PROLOGUE();
  gpt_lld_serve_interrupt(&GPTD6);
  CH_IRQ_EPILOGUE();
}
#endif /* SAM3XA_GPT_USE_TC5 */

#if SAM3XA_GPT_USE_TC6
/**
 * @brief   TC6 interrupt handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(SAM3XA_TC6_HANDLER) {
  CH_IRQ_PROLOGUE();
  gpt_lld_serve_interrupt(&GPTD7);
  CH_IRQ_EPILOGUE();
}
#endif /* SAM3XA_GPT_USE_TC6 */

#if SAM3XA_GPT_USE_TC7
/**
 * @brief   TC7 interrupt handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(SAM3XA_TC7_HANDLER) {
  CH_IRQ_PROLOGUE();
  gpt_lld_serve_interrupt(&GPTD8);
  CH_IRQ_EPILOGUE();
}
#endif /* SAM3XA_GPT_USE_TC7 */

#if SAM3XA_GPT_USE_TC8
/**
 * @brief   TC8 interrupt handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(SAM3XA_TC8_HANDLER) {
  CH_IRQ_PROLOGUE();
  gpt_lld_serve_interrupt(&GPTD9);
  CH_IRQ_EPILOGUE();
}
#endif /* SAM3XA_GPT_USE_TC8 */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level GPT driver initialization.
 *
 * @notapi
 */
void gpt_lld_init(void) {
#if SAM3XA_GPT_USE_TC0
  gptObjectInit(&GPTD1);
#endif

#if SAM3XA_GPT_USE_TC1
  gptObjectInit(&GPTD2);
#endif

#if SAM3XA_GPT_USE_TC2
  gptObjectInit(&GPTD3);
#endif

#if SAM3XA_GPT_USE_TC3
  gptObjectInit(&GPTD4);
#endif

#if SAM3XA_GPT_USE_TC4
  gptObjectInit(&GPTD5);
#endif

#if SAM3XA_GPT_USE_TC5
  gptObjectInit(&GPTD6);
#endif

#if SAM3XA_GPT_USE_TC6
  gptObjectInit(&GPTD7);
#endif

#if SAM3XA_GPT_USE_TC7
  gptObjectInit(&GPTD8);
#endif

#if SAM3XA_GPT_USE_TC8
  gptObjectInit(&GPTD9);
#endif
}

/**
 * @brief   Configures and activates the GPT peripheral.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 *
 * @notapi
 */
void gpt_lld_start(GPTDriver *gptp) {
  uint32_t clks = TC_CMR_TCCLKS_TIMER_CLOCK5;
  pmc_enable_peripheral_clock(gptp->peripheral_id);

  uint32_t irq_priority =
      gptp->config->irq_priority ? gptp->config->irq_priority :
      SAM3XA_TC_DEFAULT_IRQ_PRIORITY;
  chDbgCheck(CORTEX_IS_VALID_KERNEL_PRIORITY(gptp->config->irq_priority),
      "GPTP irq_priority");
  nvicEnableVector(gptp->irq_id, CORTEX_PRIORITY_MASK(irq_priority));

  if (gptp->config->bmr & TC_BMR_QDEN)
  {
    /* Quad decode mode */
    clks = TC_CMR_TCCLKS_XC0;
  } else
  {
    if (gptp->config->frequency == SAM3XA_GPT_TIMER_CLOCK1_FREQ) {
      clks = TC_CMR_TCCLKS_TIMER_CLOCK1;
    } else if (gptp->config->frequency == SAM3XA_GPT_TIMER_CLOCK2_FREQ) {
      clks = TC_CMR_TCCLKS_TIMER_CLOCK2;
    } else if (gptp->config->frequency == SAM3XA_GPT_TIMER_CLOCK3_FREQ) {
      clks = TC_CMR_TCCLKS_TIMER_CLOCK3;
    } else if (gptp->config->frequency == SAM3XA_GPT_TIMER_CLOCK4_FREQ) {
      clks = TC_CMR_TCCLKS_TIMER_CLOCK4;
    } else {
      chDbgAssert(0, "GPT invalid frequency selection", "invalid frequency");
    }
  }

  if (gptp->counter != NULL)
  {
    gptp->counter->TC_BMR = gptp->config->bmr;
    gptp->counter->TC_QIER = gptp->config->qier;
    gptp->counter->TC_FMR = gptp->config->fmr;
  }

  if (gptp->config->bmr & TC_BMR_QDEN)
  {
    /* Quad decode mode */
    gptp->channel->TC_CMR = clks;
    gptp->channel->TC_CCR = TC_CCR_CLKEN;
    if (gptp->counter != NULL)
    {
      // Start the counter
      gptp->counter->TC_BCR = TC_BCR_SYNC;
    }
  } else
  {
    /* Wave mode, Increment until RC */
    gptp->channel->TC_CMR = clks | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC;
    gptp->channel->TC_CCR = TC_CCR_CLKDIS;
  }
}

/**
 * @brief   Deactivates the GPT peripheral.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 *
 * @notapi
 */
void gpt_lld_stop(GPTDriver *gptp) {
  gptp->channel->TC_CCR = TC_CCR_CLKDIS;
  gptp->channel->TC_IDR = 0xFFFFFFFF;

  pmc_disable_peripheral_clock(gptp->peripheral_id);
  nvicDisableVector(gptp->irq_id);
}

/**
 * @brief   Starts the timer in continuous mode.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 * @param[in] interval  period in ticks
 *
 * @notapi
 */
void gpt_lld_start_timer(GPTDriver *gptp, gptcnt_t interval) {
  if (gptp->state == GPT_ONESHOT) {
    // One shot mode
    gptp->channel->TC_CMR |= TC_CMR_CPCSTOP;
  } else {
    // Continuous mode
    gptp->channel->TC_CMR &= ~TC_CMR_CPCSTOP;
  }
  gptp->channel->TC_RC = interval - 1;
  // Enable interrupt
  gptp->channel->TC_IER = TC_IER_CPCS;
  // Enable clock
  gptp->channel->TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
}

/**
 * @brief   Stops the timer.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 *
 * @notapi
 */
void gpt_lld_stop_timer(GPTDriver *gptp) {
  uint32_t status __attribute__((__unused__));
  // Disable clock
  gptp->channel->TC_CCR = TC_CCR_CLKDIS;
  // Stop and clear interrupt
  gptp->channel->TC_IDR = 0xFFFFFFFF;
  status = gptp->channel->TC_SR;
}

/**
 * @brief   Starts the timer in one shot mode and waits for completion.
 * @details This function specifically polls the timer waiting for completion
 *          in order to not have extra delays caused by interrupt servicing,
 *          this function is only recommended for short delays.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 * @param[in] interval  time interval in ticks
 *
 * @notapi
 */
void gpt_lld_polled_delay(GPTDriver *gptp, gptcnt_t interval) {
  // One shot mode
  gptp->channel->TC_CMR |= TC_CMR_CPCSTOP;
  gptp->channel->TC_RC = interval - 1;
  // Enable clock
  gptp->channel->TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;

  while (!(gptp->channel->TC_SR & TC_SR_CPCS))
    ;
}

#endif /* HAL_USE_GPT */

/** @} */
