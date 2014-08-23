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
 * @file    SAM3XA/pwm_lld.c
 * @brief   SAM3XA PWM subsystem low level driver header.
 *
 * @addtogroup PWM
 * @{
 */

#include "ch.h"
#include "hal.h"

#if HAL_USE_PWM || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   PWMD1 driver identifier.
 */
#if SAM3XA_PWM_USE_CH0 || defined(__DOXYGEN__)
PWMDriver PWMD1 = {
    .channel_id = 0,
    .pwm = PWM,
    .pwm_ch = &(PWM->PWM_CH_NUM[0])
};
#endif

/**
 * @brief   PWMD2 driver identifier.
 */
#if SAM3XA_PWM_USE_CH1 || defined(__DOXYGEN__)
PWMDriver PWMD2 = {
    .channel_id = 1,
    .pwm = PWM,
    .pwm_ch = &(PWM->PWM_CH_NUM[1])
};
#endif

/**
 * @brief   PWMD3 driver identifier.
 */
#if SAM3XA_PWM_USE_CH2 || defined(__DOXYGEN__)
PWMDriver PWMD3 = {
    .channel_id = 2,
    .pwm = PWM,
    .pwm_ch = &(PWM->PWM_CH_NUM[2])
};
#endif

/**
 * @brief   PWMD4 driver identifier.
 */
#if SAM3XA_PWM_USE_CH3 || defined(__DOXYGEN__)
PWMDriver PWMD4 = {
    .channel_id = 3,
    .pwm = PWM,
    .pwm_ch = &(PWM->PWM_CH_NUM[3])
};
#endif

/**
 * @brief   PWMD5 driver identifier.
 */
#if SAM3XA_PWM_USE_CH4 || defined(__DOXYGEN__)
PWMDriver PWMD5 = {
    .channel_id = 4,
    .pwm = PWM,
    .pwm_ch = &(PWM->PWM_CH_NUM[4])
};
#endif

/**
 * @brief   PWMD6 driver identifier.
 */
#if SAM3XA_PWM_USE_CH5 || defined(__DOXYGEN__)
PWMDriver PWMD6 = {
    .channel_id = 5,
    .pwm = PWM,
    .pwm_ch = &(PWM->PWM_CH_NUM[5])
};
#endif

/**
 * @brief   PWMD7 driver identifier.
 */
#if SAM3XA_PWM_USE_CH6 || defined(__DOXYGEN__)
PWMDriver PWMD7 = {
    .channel_id = 6,
    .pwm = PWM,
    .pwm_ch = &(PWM->PWM_CH_NUM[6])
};
#endif

/**
 * @brief   PWMD8 driver identifier.
 */
#if SAM3XA_PWM_USE_CH7 || defined(__DOXYGEN__)
PWMDriver PWMD8 = {
    .channel_id = 7,
    .pwm = PWM,
    .pwm_ch = &(PWM->PWM_CH_NUM[7])
};
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/**
 * @brief PWM IRQ Handler
 */
CH_IRQ_HANDLER(SAM3XA_PWM_HANDLER) {
  uint8_t sr;
  CH_IRQ_PROLOGUE();

  sr = PWM->PWM_ISR1;

#if SAM3XA_PWM_USE_CH0
  if (sr & (1 << 0))
    PWMD1.config->callback(&PWMD1);
#endif
#if SAM3XA_PWM_USE_CH1
  if (sr & (1 << 1))
    PWMD2.config->callback(&PWMD2);
#endif
#if SAM3XA_PWM_USE_CH2
  if (sr & (1 << 2))
    PWMD3.config->callback(&PWMD3);
#endif
#if SAM3XA_PWM_USE_CH3
  if (sr & (1 << 3))
    PWMD4.config->callback(&PWMD4);
#endif
#if SAM3XA_PWM_USE_CH4
  if (sr & (1 << 4))
    PWMD5.config->callback(&PWMD5);
#endif
#if SAM3XA_PWM_USE_CH5
  if (sr & (1 << 5))
    PWMD6.config->callback(&PWMD6);
#endif
#if SAM3XA_PWM_USE_CH6
  if (sr & (1 << 6))
    PWMD7.config->callback(&PWMD7);
#endif
#if SAM3XA_PWM_USE_CH7
  if (sr & (1 << 7))
    PWMD8.config->callback(&PWMD8);
#endif

  CH_IRQ_EPILOGUE();
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level PWM driver initialization.
 *
 * @notapi
 */
void pwm_lld_init(void) {
#if SAM3XA_PWM_USE_CH0
  pwmObjectInit(&PWMD1);
#endif
#if SAM3XA_PWM_USE_CH1
  pwmObjectInit(&PWMD2);
#endif
#if SAM3XA_PWM_USE_CH2
  pwmObjectInit(&PWMD3);
#endif
#if SAM3XA_PWM_USE_CH3
  pwmObjectInit(&PWMD4);
#endif
#if SAM3XA_PWM_USE_CH4
  pwmObjectInit(&PWMD5);
#endif
#if SAM3XA_PWM_USE_CH5
  pwmObjectInit(&PWMD6);
#endif
#if SAM3XA_PWM_USE_CH6
  pwmObjectInit(&PWMD7);
#endif
#if SAM3XA_PWM_USE_CH7
  pwmObjectInit(&PWMD8);
#endif

  // Enable clock and interrupt vector
  pmc_enable_peripheral_clock(ID_PWM);
  nvicEnableVector(PWM_IRQn, CORTEX_PRIORITY_MASK(SAM3XA_PWM_DEFAULT_IRQ_PRIORITY));
}

/**
 * @brief   Configures and activates the PWM peripheral.
 * @note    Starting a driver that is already in the @p PWM_READY state
 *          disables all the active channels.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 *
 * @notapi
 */
void pwm_lld_start(PWMDriver *pwmp) {
  uint32_t psc;
  uint8_t psc_bits;

  // Disable the channel
  pwmp->pwm->PWM_DIS = 1 << pwmp->channel_id;

  /* Timer configuration.*/
  psc = SystemCoreClock / pwmp->config->frequency;
  chDbgAssert(psc <= 1024 && psc >= 1 && ((psc - 1) & psc) == 0,
      "pwm_lld_start()", "invalid frequency");

  for (psc_bits = 0; psc > 1; psc_bits++, psc >>= 1);

  pwmp->pwm_ch->PWM_CPRD = pwmp->config->period;
  pwmp->pwm_ch->PWM_CMR =
      psc_bits |
      ((pwmp->config->channels[0].mode) & PWM_CHANNEL_POLARITY_HIGH ?
          PWM_CMR_CPOL : 0);

  peripheral_pin_apply(&pwmp->config->channels[0].h_pin);
  peripheral_pin_apply(&pwmp->config->channels[0].l_pin);

  if (pwmp->config->callback != NULL) {
    pwmp->pwm->PWM_IER1 = 1 << pwmp->channel_id;
  } else {
    pwmp->pwm->PWM_IDR1 = 1 << pwmp->channel_id;
  }
}

/**
 * @brief   Deactivates the PWM peripheral.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 *
 * @notapi
 */
void pwm_lld_stop(PWMDriver *pwmp) {
  pwmp->pwm->PWM_DIS = 1 << pwmp->channel_id;
  pwmp->pwm->PWM_IDR1 = 1 << pwmp->channel_id;

  peripheral_pin_reset(&pwmp->config->channels[0].h_pin);
  peripheral_pin_reset(&pwmp->config->channels[0].l_pin);
}

/**
 * @brief   Enables a PWM channel.
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @post    The channel is active using the specified configuration.
 * @note    The function has effect at the next cycle start.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 * @param[in] channel   PWM channel identifier (0...PWM_CHANNELS-1)
 * @param[in] width     PWM pulse width as clock pulses number
 *
 * @notapi
 */
void pwm_lld_enable_channel(PWMDriver *pwmp,
                            pwmchannel_t channel,
                            pwmcnt_t width) {
  (void)channel;
  uint8_t channel_mask = 1 << pwmp->channel_id;
  if (pwmp->pwm->PWM_SR & channel_mask) {
    // If already running, use CDTY UDP double buffer
    pwmp->pwm_ch->PWM_CDTYUPD = width;
  } else {
    pwmp->pwm_ch->PWM_CDTY = width;
    pwmp->pwm->PWM_ENA = channel_mask;
  }
}

/**
 * @brief   Disables a PWM channel.
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @post    The channel is disabled and its output line returned to the
 *          idle state.
 * @note    The function has effect at the next cycle start.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 * @param[in] channel   PWM channel identifier (0...PWM_CHANNELS-1)
 *
 * @notapi
 */
void pwm_lld_disable_channel(PWMDriver *pwmp, pwmchannel_t channel) {
  (void)channel;
  pwmp->pwm->PWM_DIS = 1 << pwmp->channel_id;
}

#endif /* HAL_USE_PWM */

/** @} */
