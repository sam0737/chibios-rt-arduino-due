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
 * @file    SAM3XA/serial_lld.c
 * @brief   SAM3XA low level Serial driver code.
 *
 * @addtogroup SERIAL
 * @{
 */

#include "ch.h"
#include "hal.h"
#include "serial_lld.h"

#if HAL_USE_SERIAL || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief UART UART driver identifier.*/
#if SAM3XA_SERIAL_USE_UART || defined(__DOXYGEN__)
SerialDriver SD1 = {
    .peripheral_id = ID_UART,
    .irq_id = UART_IRQn,
    .is_usart = 0,
    .reg.uart = UART
};
#endif

/** @brief USART0 USART driver identifier.*/
#if SAM3XA_SERIAL_USE_USART0 || defined(__DOXYGEN__)
SerialDriver SD2 = {
    .peripheral_id = ID_USART0,
    .irq_id = USART0_IRQn,
    .is_usart = 1,
    .reg.usart = USART0
};
#endif

/** @brief USART1 USART driver identifier.*/
#if SAM3XA_SERIAL_USE_USART1 || defined(__DOXYGEN__)
SerialDriver SD3 = {
    .peripheral_id = ID_USART1,
    .irq_id = USART1_IRQn,
    .is_usart = 1,
    .reg.usart = USART1
};
#endif

/** @brief USART2 USART driver identifier.*/
#if SAM3XA_SERIAL_USE_USART2 || defined(__DOXYGEN__)
SerialDriver SD4 = {
    .peripheral_id = ID_USART2,
    .irq_id = USART2_IRQn,
    .is_usart = 1,
    .reg.usart = USART2
};
#endif

/** @brief USART3 USART driver identifier.*/
#if SAM3XA_SERIAL_USE_USART3 || defined(__DOXYGEN__)
SerialDriver SD5 = {
    .peripheral_id = ID_USART3,
    .irq_id = USART3_IRQn,
    .is_usart = 1,
    .reg.usart = USART3
};
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Status bits translation.
 *
 * @param[in] sr USART SR register value
 *
 * @return  The error flags.
 */
static flagsmask_t translate_errors(uint32_t sr) {
  flagsmask_t sts = 0;

  if (sr & UART_SR_OVRE)
    sts |= SD_OVERRUN_ERROR;
  if (sr & UART_SR_PARE)
    sts |= SD_PARITY_ERROR;
  if (sr & UART_SR_FRAME)
    sts |= SD_FRAMING_ERROR;
  return sts;
}

/**
 * @brief   USART de-initialization.
 * @details This function must be invoked with interrupts disabled.
 *
 * @param[in] sdp        pointer to the @p UARTDriver object
 */
static void usart_stop(SerialDriver *sdp) {
  Uart *u = sdp->reg.uart;
  nvicDisableVector(sdp->irq_id);

  u->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX | UART_CR_RSTSTA;
  u->UART_IDR = 0xFFFF;
  u->UART_BRGR = 0;

  peripheral_pin_reset(&sdp->config->tx_pin);
  peripheral_pin_reset(&sdp->config->rx_pin);
  if (sdp->is_usart) {
    peripheral_pin_reset(&sdp->config->rts_pin);
    peripheral_pin_reset(&sdp->config->cts_pin);
  }
}


/**
 * @brief   USART initialization.
 * @details This function must be invoked with interrupts disabled.
 *
 * @param[in] sdp       pointer to the @p SerialDriver object
 * @param[in] config    pointer to the @p SerialConfig object
 */
static void usart_start(SerialDriver *sdp, const SerialConfig *config, uint32_t irq_priority) {
  Uart* u = sdp->reg.uart;

  u->UART_CR = UART_CR_RXEN | UART_CR_TXEN | UART_CR_RSTSTA;
  u->UART_BRGR = SystemCoreClock / 16 / config->speed;

  pmc_enable_peripheral_clock(sdp->peripheral_id);
  sdp->reg.usart->US_MR = config->mr;
  if (sdp->is_usart) {
    sdp->reg.usart->US_RTOR = config->rtor;
    sdp->reg.usart->US_TTGR = config->ttgr;
    peripheral_pin_apply(&config->rts_pin);
    peripheral_pin_apply(&config->cts_pin);
  }
  peripheral_pin_apply(&config->tx_pin);
  peripheral_pin_apply(&config->rx_pin);

  // Enable interrupts for error conditions, and RX
  u->UART_IER = UART_IER_PARE | UART_IER_FRAME | UART_IER_OVRE | UART_IER_RXRDY;

  nvicEnableVector(sdp->irq_id, CORTEX_PRIORITY_MASK(irq_priority));
}

/**
 * @brief   USART common service routine.
 *
 * @param[in] sdp       pointer to the @p SerialDriver object
 */
static void serve_usart_irq(SerialDriver *sdp) {
  msg_t b;
  uint32_t isr;
  Uart *u = sdp->reg.uart;

  isr = u->UART_SR & u->UART_IMR;

  if (isr & (UART_SR_OVRE | UART_SR_PARE | UART_SR_FRAME)) {
    /* Clearing status.*/
    u->UART_CR = UART_CR_RSTSTA;
    flagsmask_t sts = translate_errors(isr);

    chSysLockFromIsr();
    chnAddFlagsI(sdp, sts);
    chSysUnlockFromIsr();
  }
  if (isr & UART_SR_TXRDY) {
    chSysLockFromIsr();
    b = sdRequestDataI(sdp);
    chSysUnlockFromIsr();
    if (b < Q_OK)
      u->UART_IDR = UART_IDR_TXRDY;
    else
      u->UART_THR = b;
  }
  if (isr & UART_SR_RXRDY) {
    chSysLockFromIsr();
    sdIncomingDataI(sdp, u->UART_RHR);
    chSysUnlockFromIsr();
  }
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if SAM3XA_SERIAL_USE_UART || defined(__DOXYGEN__)
#if !defined(SAM3XA_UART_HANDLER)
#error "SAM3XA_UART_HANDLER not defined"
#endif
/**
 * @brief   UART IRQ handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(SAM3XA_UART_HANDLER) {
  CH_IRQ_PROLOGUE();
  serve_usart_irq(&SD1);
  CH_IRQ_EPILOGUE();
}

/**
 * @brief   UART TX queue notify hook
 *
 * @notapi
 */
static void notifySD1(GenericQueue *qp) {
  (void)qp;
  UART->UART_IER = UART_IER_TXRDY;
}
#endif /* SAM3XA_SERIAL_USE_UART */

#if SAM3XA_SERIAL_USE_USART0 || defined(__DOXYGEN__)
#if !defined(SAM3XA_USART0_HANDLER)
#error "SAM3XA_USART0_HANDLER not defined"
#endif
/**
 * @brief   USART0 IRQ handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(SAM3XA_USART0_HANDLER) {
  CH_IRQ_PROLOGUE();
  serve_usart_irq(&SD2);
  CH_IRQ_EPILOGUE();
}

/**
 * @brief   USART0 TX queue notify hook
 *
 * @notapi
 */
static void notifySD2(GenericQueue *qp) {
  (void)qp;
  USART0->USART_IER = USART_IER_TXRDY;
}
#endif /* SAM3XA_SERIAL_USE_USART0 */

#if SAM3XA_SERIAL_USE_USART1 || defined(__DOXYGEN__)
#if !defined(SAM3XA_USART1_HANDLER)
#error "SAM3XA_USART1_HANDLER not defined"
#endif
/**
 * @brief   USART1 IRQ handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(SAM3XA_USART1_HANDLER) {
  CH_IRQ_PROLOGUE();
  serve_usart_irq(&SD3);
  CH_IRQ_EPILOGUE();
}

/**
 * @brief   USART1 TX queue notify hook
 *
 * @notapi
 */
static void notifySD3(GenericQueue *qp) {
  (void)qp;
  USART1->USART_IER = USART_IER_TXRDY;
}
#endif /* SAM3XA_SERIAL_USE_USART1 */

#if SAM3XA_SERIAL_USE_USART2 || defined(__DOXYGEN__)
#if !defined(SAM3XA_USART2_HANDLER)
#error "SAM3XA_USART2_HANDLER not defined"
#endif
/**
 * @brief   USART2 IRQ handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(SAM3XA_USART2_HANDLER) {
  CH_IRQ_PROLOGUE();
  serve_usart_irq(&SD4);
  CH_IRQ_EPILOGUE();
}


/**
 * @brief   USART2 TX queue notify hook
 *
 * @notapi
 */
static void notifySD4(GenericQueue *qp) {
  (void)qp;
  USART2->USART_IER = USART2_IER_TXRDY;
}
#endif /* SAM3XA_SERIAL_USE_USART2 */

#if SAM3XA_SERIAL_USE_USART3 || defined(__DOXYGEN__)
#if !defined(SAM3XA_USART3_HANDLER)
#error "SAM3XA_USART3_HANDLER not defined"
#endif
/**
 * @brief   USART3 IRQ handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(SAM3XA_USART3_HANDLER) {
  CH_IRQ_PROLOGUE();
  serve_usart_irq(&SD5);
  CH_IRQ_EPILOGUE();
}

/**
 * @brief   USART3 TX queue notify hook
 *
 * @notapi
 */
static void notifySD5(GenericQueue *qp) {
  (void)qp;
  USART3->USART_IER = USART_IER_TXRDY;
}
#endif /* SAM3XA_SERIAL_USE_USART3 */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level Serial driver initialization.
 *
 * @notapi
 */
void sd_lld_init(void) {
#if SAM3XA_SERIAL_USE_UART
  sdObjectInit(&SD1, NULL, notifySD1);
#endif
#if SAM3XA_SERIAL_USE_USART0
  sdObjectInit(&SD2, NULL, notifySD2);
#endif
#if SAM3XA_SERIAL_USE_USART1
  sdObjectInit(&SD3, NULL, notifySD3);
#endif
#if SAM3XA_SERIAL_USE_USART2
  sdObjectInit(&SD4, NULL, notifySD4);
#endif
#if SAM3XA_SERIAL_USE_USART3
  sdObjectInit(&SD5, NULL, notifySD5);
#endif
}

/**
 * @brief   Configures and activates the Serial peripheral.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 *
 * @notapi
 */
void sd_lld_start(SerialDriver *sdp, const SerialConfig *config) {
  uint32_t irq_priority =
      config->irq_priority ? config->irq_priority :
      SAM3XA_SERIAL_DEFAULT_IRQ_PRIORITY;
  chDbgCheck(CORTEX_IS_VALID_KERNEL_PRIORITY(config->irq_priority),
      "Serial irq_priority");

  sdp->config = config;
  usart_start(sdp, config, irq_priority);
}

/**
 * @brief   Deactivates the Serial peripheral.
 *
 * @param[in] sdp       pointer to the @p SerialDriver object
 *
 * @notapi
 */
void sd_lld_stop(SerialDriver *sdp) {
  usart_stop(sdp);
}

#endif /* HAL_USE_UART */

/** @} */
