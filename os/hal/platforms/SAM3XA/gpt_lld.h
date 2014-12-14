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
 * @file    SAM3XA/gpt_lld.h
 * @brief   STM32 GPT subsystem low level driver header.
 *
 * @addtogroup GPT
 * @{
 */

#ifndef _GPT_LLD_H_
#define _GPT_LLD_H_

#if HAL_USE_GPT || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief Frequency selection - TIMER_CLOCK1 (SystemCoreClock / 2).
 */
#define SAM3XA_GPT_TIMER_CLOCK1_FREQ (SystemCoreClock / 2)

/**
 * @brief Frequency selection - TIMER_CLOCK2 (SystemCoreClock / 8).
 */
#define SAM3XA_GPT_TIMER_CLOCK2_FREQ (SystemCoreClock / 8)

/**
 * @brief Frequency selection - TIMER_CLOCK3 (SystemCoreClock / 32).
 */
#define SAM3XA_GPT_TIMER_CLOCK3_FREQ (SystemCoreClock / 32)

/**
 * @brief Frequency selection - TIMER_CLOCK4 (SystemCoreClock / 128).
 */
#define SAM3XA_GPT_TIMER_CLOCK4_FREQ (SystemCoreClock / 128)

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   GPTD1 driver enable switch.
 * @details If set to @p TRUE the support for GPTD1 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(SAM3XA_GPT_USE_TC0) || defined(__DOXYGEN__)
#define SAM3XA_GPT_USE_TC0                  FALSE
#endif

/**
 * @brief   GPTD2 driver enable switch.
 * @details If set to @p TRUE the support for GPTD2 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(SAM3XA_GPT_USE_TC1) || defined(__DOXYGEN__)
#define SAM3XA_GPT_USE_TC1                  FALSE
#endif

/**
 * @brief   GPTD3 driver enable switch.
 * @details If set to @p TRUE the support for GPTD3 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(SAM3XA_GPT_USE_TC2) || defined(__DOXYGEN__)
#define SAM3XA_GPT_USE_TC2                  FALSE
#endif

/**
 * @brief   GPTD4 driver enable switch.
 * @details If set to @p TRUE the support for GPTD4 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(SAM3XA_GPT_USE_TC3) || defined(__DOXYGEN__)
#define SAM3XA_GPT_USE_TC3                  FALSE
#endif

/**
 * @brief   GPTD5 driver enable switch.
 * @details If set to @p TRUE the support for GPTD5 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(SAM3XA_GPT_USE_TC4) || defined(__DOXYGEN__)
#define SAM3XA_GPT_USE_TC4                  FALSE
#endif

/**
 * @brief   GPTD6 driver enable switch.
 * @details If set to @p TRUE the support for GPTD6 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(SAM3XA_GPT_USE_TC5) || defined(__DOXYGEN__)
#define SAM3XA_GPT_USE_TC5                  FALSE
#endif

/**
 * @brief   GPTD7 driver enable switch.
 * @details If set to @p TRUE the support for GPTD7 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(SAM3XA_GPT_USE_TC6) || defined(__DOXYGEN__)
#define SAM3XA_GPT_USE_TC6                  FALSE
#endif

/**
 * @brief   GPTD8 driver enable switch.
 * @details If set to @p TRUE the support for GPTD8 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(SAM3XA_GPT_USE_TC7) || defined(__DOXYGEN__)
#define SAM3XA_GPT_USE_TC7                  FALSE
#endif

/**
 * @brief   GPTD9 driver enable switch.
 * @details If set to @p TRUE the support for GPTD9 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(SAM3XA_GPT_USE_TC8) || defined(__DOXYGEN__)
#define SAM3XA_GPT_USE_TC8                  FALSE
#endif

/**
 * @brief   PWM interrupt priority level setting
 */
#if !defined(SAM3XA_TC_DEFAULT_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define SAM3XA_TC_DEFAULT_IRQ_PRIORITY             12
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if !SAM3XA_GPT_USE_TC0 && !SAM3XA_GPT_USE_TC1 &&  \
    !SAM3XA_GPT_USE_TC2 && !SAM3XA_GPT_USE_TC3 &&  \
    !SAM3XA_GPT_USE_TC4 && !SAM3XA_GPT_USE_TC5 &&  \
    !SAM3XA_GPT_USE_TC6 && !SAM3XA_GPT_USE_TC7 &&  \
    !SAM3XA_GPT_USE_TC8
#error "GPT driver activated but no TC peripheral assigned"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   GPT frequency type.
 */
typedef uint32_t gptfreq_t;

/**
 * @brief   GPT counter type.
 */
typedef uint32_t gptcnt_t;

/**
 * @brief   Driver configuration structure.
 * @note    It could be empty on some architectures.
 */
typedef struct {
  /**
   * @brief   Timer clock in Hz.
   * @note    The low level can use assertions in order to catch invalid
   *          frequency specifications.
   */
  gptfreq_t                 frequency;
  /**
   * @brief   Timer callback pointer.
   * @note    This callback is invoked on GPT counter events.
   */
  gptcallback_t             callback;
  /* End of the mandatory fields.*/

  /**
   * @brief Block Mode Register.
   */
  uint32_t                  bmr;
  /**
   * @brief QDEC Interrupt Enable Register.
   */
  uint32_t                  qier;
  /**
   * @brief Fault Mode Register.
   */
  uint32_t                  fmr;

  /**
   * @brief IRQ priority. If not set, a default will be assigned.
   */
  uint32_t                  irq_priority;
} GPTConfig;

/**
 * @brief   Structure representing a GPT driver.
 */
struct GPTDriver {
  /**
   * @brief Driver state.
   */
  gptstate_t                state;
  /**
   * @brief Current configuration data.
   */
  const GPTConfig           *config;
#if defined(GPT_DRIVER_EXT_FIELDS)
  GPT_DRIVER_EXT_FIELDS
#endif
  /* End of the mandatory fields.*/
  /**
   * @brief Pointer to the Tc registers block.
   */
  Tc                        *counter;

  /**
   * @brief Pointer to the TcChannel registers block.
   */
  TcChannel                 *channel;

  /**
   * @brief The peripheral id of the TC device
   */
  uint8_t                   peripheral_id;

  /**
   * @brief The IRQ id of the TC device
   */
  uint8_t                   irq_id;
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Changes the interval of GPT peripheral.
 * @details This function changes the interval of a running GPT unit.
 * @pre     The GPT unit must have been activated using @p gptStart().
 * @pre     The GPT unit must have been running in continuous mode using
 *          @p gptStartContinuous().
 * @post    The GPT unit interval is changed to the new value.
 * @note    The function has effect at the next cycle start.
 *
 * @param[in] gptp      pointer to a @p GPTDriver object
 * @param[in] interval  new cycle time in timer ticks
 * @notapi
 */
#define gpt_lld_change_interval(gptp, interval)                               \
  do {                                                                        \
	  (gptp)->channel->TC_RC = ((interval) - 1);                                \
    (gptp)->channel->TC_CCR = TC_CCR_SWTRG;                                   \
  } while (0)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if SAM3XA_GPT_USE_TC0 && !defined(__DOXYGEN__)
extern GPTDriver GPTD1;
#endif

#if SAM3XA_GPT_USE_TC1 && !defined(__DOXYGEN__)
extern GPTDriver GPTD2;
#endif

#if SAM3XA_GPT_USE_TC2 && !defined(__DOXYGEN__)
extern GPTDriver GPTD3;
#endif

#if SAM3XA_GPT_USE_TC3 && !defined(__DOXYGEN__)
extern GPTDriver GPTD4;
#endif

#if SAM3XA_GPT_USE_TC4 && !defined(__DOXYGEN__)
extern GPTDriver GPTD5;
#endif

#if SAM3XA_GPT_USE_TC5 && !defined(__DOXYGEN__)
extern GPTDriver GPTD6;
#endif

#if SAM3XA_GPT_USE_TC6 && !defined(__DOXYGEN__)
extern GPTDriver GPTD7;
#endif

#if SAM3XA_GPT_USE_TC7 && !defined(__DOXYGEN__)
extern GPTDriver GPTD8;
#endif

#if SAM3XA_GPT_USE_TC8 && !defined(__DOXYGEN__)
extern GPTDriver GPTD9;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void gpt_lld_init(void);
  void gpt_lld_start(GPTDriver *gptp);
  void gpt_lld_stop(GPTDriver *gptp);
  void gpt_lld_start_timer(GPTDriver *gptp, gptcnt_t period);
  void gpt_lld_stop_timer(GPTDriver *gptp);
  void gpt_lld_polled_delay(GPTDriver *gptp, gptcnt_t interval);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_GPT */

#endif /* _GPT_LLD_H_ */

/** @} */
