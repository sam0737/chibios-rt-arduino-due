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
 * @file    SAM3XA/pwm_lld.h
 * @brief   SAM3XA PWM subsystem low level driver header.
 *
 * @addtogroup PWM
 * @{
 */

#ifndef _PWM_LLD_H_
#define _PWM_LLD_H_

#if HAL_USE_PWM || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   Number of PWM channels per PWM driver.
 */
#define PWM_CHANNELS                            1

/**
 * @brief   Channel output begins with low level first (default)
 */
#define PWM_CHANNEL_POLARITY_LOW                0x00

/**
 * @brief   Channel output begins with high level first
 */
#define PWM_CHANNEL_POLARITY_HIGH               0x10

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */

/**
 * @brief   PWMD1 driver enable switch.
 * @details If set to @p TRUE the support for PWMD1 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(SAM3XA_PWM_USE_CH0) || defined(__DOXYGEN__)
#define SAM3XA_PWM_USE_CH0                  FALSE
#endif

/**
 * @brief   PWMD2 driver enable switch.
 * @details If set to @p TRUE the support for PWMD1 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(SAM3XA_PWM_USE_CH1) || defined(__DOXYGEN__)
#define SAM3XA_PWM_USE_CH1                  FALSE
#endif

/**
 * @brief   PWMD3 driver enable switch.
 * @details If set to @p TRUE the support for PWMD1 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(SAM3XA_PWM_USE_CH2) || defined(__DOXYGEN__)
#define SAM3XA_PWM_USE_CH2                  FALSE
#endif


/**
 * @brief   PWMD4 driver enable switch.
 * @details If set to @p TRUE the support for PWMD1 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(SAM3XA_PWM_USE_CH3) || defined(__DOXYGEN__)
#define SAM3XA_PWM_USE_CH3                  FALSE
#endif

/**
 * @brief   PWMD5 driver enable switch.
 * @details If set to @p TRUE the support for PWMD1 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(SAM3XA_PWM_USE_CH4) || defined(__DOXYGEN__)
#define SAM3XA_PWM_USE_CH4                  FALSE
#endif

/**
 * @brief   PWMD6 driver enable switch.
 * @details If set to @p TRUE the support for PWMD1 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(SAM3XA_PWM_USE_CH5) || defined(__DOXYGEN__)
#define SAM3XA_PWM_USE_CH5                  FALSE
#endif

/**
 * @brief   PWMD7 driver enable switch.
 * @details If set to @p TRUE the support for PWMD1 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(SAM3XA_PWM_USE_CH6) || defined(__DOXYGEN__)
#define SAM3XA_PWM_USE_CH6                  FALSE
#endif

/**
 * @brief   PWMD8 driver enable switch.
 * @details If set to @p TRUE the support for PWMD1 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(SAM3XA_PWM_USE_CH7) || defined(__DOXYGEN__)
#define SAM3XA_PWM_USE_CH7                  FALSE
#endif

/**
 * @brief   PWM interrupt priority level setting
 */
#if !defined(SAM3XA_PWM_DEFAULT_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define SAM3XA_PWM_DEFAULT_IRQ_PRIORITY             12
#endif
/** @} */

/*===========================================================================*/
/* Configuration checks.                                                     */
/*===========================================================================*/

#if !CORTEX_IS_VALID_KERNEL_PRIORITY(SAM3XA_PWM_DEFAULT_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to PWM"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief PWM mode type.
 */
typedef uint32_t pwmmode_t;

/**
 * @brief   PWM channel type.
 */
typedef uint8_t pwmchannel_t;

/**
 * @brief   PWM counter type.
 */
typedef uint16_t pwmcnt_t;

/**
 * @brief   PWM driver channel configuration structure.
 */
typedef struct {
  /**
   * @brief Channel active logic level.
   */
  pwmmode_t                 mode;

  /**
   * @brief Output H pin config
   */
  PeripheralPinConfig       h_pin;

  /**
   * @brief Output L pin config
   */
  PeripheralPinConfig       l_pin;
  /* End of the mandatory fields.*/
} PWMChannelConfig;

/**
 * @brief   PWM driver configuration structure.
 */
typedef struct {
  /**
   * @brief   Timer clock in Hz.
   * @note    The low level can use assertions in order to catch invalid
   *          frequency specifications.
   */
  uint32_t                  frequency;
  /**
   * @brief   PWM period in ticks.
   * @note    The low level can use assertions in order to catch invalid
   *          period specifications.
   */
  pwmcnt_t                  period;
  /**
   * @brief Periodic callback pointer.
   * @note  This callback is invoked on PWM counter reset. If set to
   *        @p NULL then the callback is disabled.
   */
  pwmcallback_t             callback;
  /**
   * @brief Channels configurations.
   */
  PWMChannelConfig          channels[PWM_CHANNELS];
  /* End of the mandatory fields.*/
} PWMConfig;

/**
 * @brief   Structure representing a PWM driver.
 */
struct PWMDriver {
  /**
   * @brief Driver state.
   */
  pwmstate_t                state;
  /**
   * @brief Current driver configuration data.
   */
  const PWMConfig           *config;
  /**
   * @brief   Current PWM period in ticks.
   */
  pwmcnt_t                  period;
#if defined(PWM_DRIVER_EXT_FIELDS)
  PWM_DRIVER_EXT_FIELDS
#endif
  /* SAM3XA channel id  */
  uint8_t                   channel_id;
  /* Pointer to the device register */
  Pwm                       *pwm;
  /* Pointer to the pwm_ch register */
  PwmCh_num                 *pwm_ch;
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Changes the period the PWM peripheral.
 * @details This function changes the period of a PWM unit that has already
 *          been activated using @p pwmStart().
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @post    The PWM unit period is changed to the new value.
 * @note    The function has effect at the next cycle start.
 * @note    If a period is specified that is shorter than the pulse width
 *          programmed in one of the channels then the behavior is not
 *          guaranteed.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 * @param[in] period    new cycle time in ticks
 *
 * @notapi
 */
#define pwm_lld_change_period(pwmp, period)                                 \
  ((pwmp)->pwm_ch->PWM_CPRD = (uint16_t)(period))

/**
 * @brief   Returns a PWM channel status.
 * @pre     The PWM unit must have been activated using @p pwmStart().
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 * @param[in] channel   PWM channel identifier (0...PWM_CHANNELS-1)
 *
 * @notapi
 */
#define pwm_lld_is_channel_enabled(pwmp, channel)                           \
  ((pwmp)->pwm->PWM_SR & (1 << (pwmp)->channel_id))

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if SAM3XA_PWM_USE_CH0 && !defined(__DOXYGEN__)
extern PWMDriver PWMD1;
#endif

#if SAM3XA_PWM_USE_CH1 && !defined(__DOXYGEN__)
extern PWMDriver PWMD2;
#endif

#if SAM3XA_PWM_USE_CH2 && !defined(__DOXYGEN__)
extern PWMDriver PWMD3;
#endif

#if SAM3XA_PWM_USE_CH3 && !defined(__DOXYGEN__)
extern PWMDriver PWMD4;
#endif

#if SAM3XA_PWM_USE_CH4 && !defined(__DOXYGEN__)
extern PWMDriver PWMD5;
#endif

#if SAM3XA_PWM_USE_CH5 && !defined(__DOXYGEN__)
extern PWMDriver PWMD6;
#endif

#if SAM3XA_PWM_USE_CH6 && !defined(__DOXYGEN__)
extern PWMDriver PWMD7;
#endif

#if SAM3XA_PWM_USE_CH7 && !defined(__DOXYGEN__)
extern PWMDriver PWMD8;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void pwm_lld_init(void);
  void pwm_lld_start(PWMDriver *pwmp);
  void pwm_lld_stop(PWMDriver *pwmp);
  void pwm_lld_enable_channel(PWMDriver *pwmp,
                              pwmchannel_t channel,
                              pwmcnt_t width);
  void pwm_lld_disable_channel(PWMDriver *pwmp, pwmchannel_t channel);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_PWM */

#endif /* _PWM_LLD_H_ */

/** @} */
