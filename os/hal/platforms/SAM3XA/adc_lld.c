/*
    ChibiOS/RT - Copyright (C) 2013 Sam Wong

    This file is part of Arduino Due patch for ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

                                      ---

    A special exception to the GPL can be applied should you wish to distribute
    a combined work that includes ChibiOS/RT, without being obliged to provide
    the source code for any proprietary components. See the file exception.txt
    for full details of how and when the exception can be applied.
 */

/**
 * @file    SAM3XA/adc_lld.c
 * @brief   SAM3XA ADC Driver subsystem low level driver source.
 *
 * @addtogroup ADC
 * @{
 */

#include "ch.h"
#include "hal.h"

#if HAL_USE_ADC || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   ADC1 driver identifier.
 */
#if SAM3XA_ADC_USE_ADC1 || defined(__DOXYGEN__)
ADCDriver ADCD1;
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

CH_IRQ_HANDLER(SAM3XA_ADC_HANDLER) {
  uint32_t isr;
  CH_IRQ_PROLOGUE();

  isr = ADC->ADC_ISR;
  if (ADCD1.grpp) {
    /* ADC overflow condition, this could happen only if the DMA is unable to read data fast enough.*/
    if ((isr & ADC_ISR_GOVRE)) {
      _adc_isr_error_code(&ADCD1, ADC_ERR_OVERFLOW);
      adc_lld_stop_conversion(&ADCD1);
    } else if ((isr & ADC_ISR_RXBUFF)) {
      _adc_isr_full_code(&ADCD1);
    }
  }

  CH_IRQ_EPILOGUE();
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level ADC driver initialization.
 *
 * @notapi
 */
void adc_lld_init(void) {
#if SAM3XA_ADC_USE_ADC1
  /* Driver initialization.*/
  adcObjectInit(&ADCD1);
#endif /* PLATFORM_ADC_USE_ADC1 */
}

/**
 * @brief   Configures and activates the ADC peripheral.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 *
 * @notapi
 */
void adc_lld_start(ADCDriver *adcp) {
  uint32_t frequency;

  // Enable clock and interrupt vector
  pmc_enable_peripheral_clock(ID_ADC);
  nvicEnableVector(ADC_IRQn, CORTEX_PRIORITY_MASK(SAM3XA_ADC_DEFAULT_IRQ_PRIORITY));

  if (adcp->state == ADC_STOP) {
    /* Enables the peripheral.*/
    // Clear the ISR
    uint16_t dummy __attribute__((__unused__));
    uint8_t i;
    dummy = ADC->ADC_LCDR;
    for (i = 0; i < 16; i++) {
      dummy = ADC->ADC_CDR[i];
    }

    frequency = adcp->config->frequency ? adcp->config->frequency : 10*1000*1000;

    ADC->ADC_MR =
        ADC_MR_PRESCAL( ((SystemCoreClock / 2) + (frequency - 1)) / frequency - 1 ) |
        ADC_MR_STARTUP_SUT24 |
        ADC_MR_SLEEP_NORMAL |
        ADC_MR_LOWRES_BITS_12 |
        ADC_MR_USEQ_NUM_ORDER |
        ADC_MR_TRGEN_DIS |
        (adcp->config->use_sequence ? ADC_MR_USEQ_REG_ORDER : 0);

    ADC->ADC_SEQR1 = adcp->config->sequence1;
    ADC->ADC_SEQR2 = adcp->config->sequence2;

    // Enable Temp sensor on CH15
    ADC->ADC_ACR = ADC_ACR_TSON;
  }
  /* Configures the peripheral.*/

}

/**
 * @brief   Deactivates the ADC peripheral.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 *
 * @notapi
 */
void adc_lld_stop(ADCDriver *adcp) {
  if (adcp->state == ADC_READY) {
    /* Resets the peripheral.*/
    ADC->ADC_MR |= ADC_MR_SLEEP_SLEEP;
  }
}

/**
 * @brief   Starts an ADC conversion.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 *
 * @notapi
 */
void adc_lld_start_conversion(ADCDriver *adcp) {
  uint32_t i;

  // TODO: Only software triggering for now.
  ((ADCConversionGroup *)adcp->grpp)->circular = 0;
  adcp->depth = 1;

  /* Count the real number of activated channels in case the user got it wrong */
  ((ADCConversionGroup *)adcp->grpp)->num_channels = 0;
  for (i = 1; i < 0x10000 && i != 0; i <<= 1) {
    if (adcp->grpp->channel_mask & i)
      ((ADCConversionGroup *)adcp->grpp)->num_channels++;
  }

  /* Set the channels */
  ADC->ADC_CHER = adcp->grpp->channel_mask;

  ADC->ADC_IER = ADC_IER_GOVRE | ADC_IER_RXBUFF;
  pdc_setup_receive(ADC, adcp->grpp->num_channels, adcp->samples);

  /* Trigger */
  ADC->ADC_CR = ADC_CR_START;
}

/**
 * @brief   Stops an ongoing conversion.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 *
 * @notapi
 */
void adc_lld_stop_conversion(ADCDriver *adcp) {
  (void)adcp;
  ADC->ADC_IDR = 0xFFFFFFFF;
  ADC->ADC_CHDR = 0xFFFFFFFF;
}

#endif /* HAL_USE_ADC */

/** @} */
