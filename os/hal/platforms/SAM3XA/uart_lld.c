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
 * @file    SAM3XA/uart_lld.c
 * @brief   SAM3XA low level UART driver code.
 *
 * @addtogroup UART
 * @{
 */

#include "ch.h"
#include "hal.h"
#include "uart_lld.h"

#if HAL_USE_UART || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief UART UART driver identifier.*/
#if SAM3XA_UART_USE_UART || defined(__DOXYGEN__)
UARTDriver UARTD1 = {
    .peripheral_id = ID_UART,
    .irq_id = UART_IRQn,
    .is_usart = 0,
    .reg.uart = UART
};
#endif

/** @brief USART0 USART driver identifier.*/
#if SAM3XA_UART_USE_USART0 || defined(__DOXYGEN__)
UARTDriver UARTD2 = {
    .peripheral_id = ID_USART0,
    .irq_id = USART0_IRQn,
    .is_usart = 1,
    .reg.usart = USART0
};
#endif

/** @brief USART1 USART driver identifier.*/
#if SAM3XA_UART_USE_USART1 || defined(__DOXYGEN__)
UARTDriver UARTD3 = {
    .peripheral_id = ID_USART1,
    .irq_id = USART1_IRQn,
    .is_usart = 1,
    .reg.usart = USART1
};
#endif

/** @brief USART2 USART driver identifier.*/
#if SAM3XA_UART_USE_USART2 || defined(__DOXYGEN__)
UARTDriver UARTD4 = {
    .peripheral_id = ID_USART2,
    .irq_id = USART2_IRQn,
    .is_usart = 1,
    .reg.usart = USART2
};
#endif

/** @brief USART3 USART driver identifier.*/
#if SAM3XA_UART_USE_USART3 || defined(__DOXYGEN__)
UARTDriver UARTD5 = {
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
static uartflags_t translate_errors(uint32_t sr) {
  uartflags_t sts = 0;

  if (sr & UART_SR_OVRE)
    sts |= UART_OVERRUN_ERROR;
  if (sr & UART_SR_PARE)
    sts |= UART_PARITY_ERROR;
  if (sr & UART_SR_FRAME)
    sts |= UART_FRAMING_ERROR;
  return sts;
}

/**
 * @brief   Puts the receiver in the UART_RX_IDLE state.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 */
static void set_rx_idle_loop(UARTDriver *uartp) {
  // if receive char callback is defined, listen for RXRDY (receive ready).
  if (uartp->config->rxchar_cb != NULL)
    uartp->reg.uart->UART_IER = UART_IER_RXRDY;
}

/**
 * @brief   USART de-initialization.
 * @details This function must be invoked with interrupts disabled.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 */
static void usart_stop(UARTDriver *uartp) {
  nvicDisableVector(uartp->irq_id);

  pdc_disable_transfer(uartp->reg.uart);
  pdc_disable_receive(uartp->reg.uart);
  uartp->reg.uart->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX | UART_CR_RSTSTA;
  uartp->reg.uart->UART_IDR = 0xFFFF;
  uartp->reg.uart->UART_BRGR = 0;

  peripheral_pin_reset(&uartp->config->tx_pin);
  peripheral_pin_reset(&uartp->config->rx_pin);
  if (uartp->is_usart) {
    peripheral_pin_reset(&uartp->config->rts_pin);
    peripheral_pin_reset(&uartp->config->cts_pin);
    peripheral_pin_reset(&uartp->config->sck_pin);
  }
}


/**
 * @brief   USART initialization.
 * @details This function must be invoked with interrupts disabled.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 */
static void usart_start(UARTDriver *uartp, uint32_t irq_priority) {
  uint8_t mode;
  Uart* u = uartp->reg.uart;

  pmc_enable_peripheral_clock(uartp->peripheral_id);
  u->UART_CR = UART_CR_RXEN | UART_CR_TXEN | UART_CR_RSTSTA;

  mode = uartp->config->mr & US_MR_USART_MODE_Msk;

  u->UART_MR = uartp->config->mr;
  if (uartp->is_usart) {
    uartp->reg.usart->US_RTOR = uartp->config->rtor;
    uartp->reg.usart->US_TTGR = uartp->config->ttgr;
    peripheral_pin_apply(&uartp->config->rts_pin);
    peripheral_pin_apply(&uartp->config->cts_pin);
    peripheral_pin_apply(&uartp->config->sck_pin);
  }
  peripheral_pin_apply(&uartp->config->tx_pin);
  peripheral_pin_apply(&uartp->config->rx_pin);

  if (mode == US_MR_USART_MODE_SPI_MASTER ||
      mode == US_MR_USART_MODE_SPI_SLAVE)
  {
    u->UART_BRGR = SystemCoreClock / uartp->config->speed;
  } else
  {
    u->UART_BRGR = SystemCoreClock / 16 / uartp->config->speed;
  }

  // Enable interrupts for error conditions
  u->UART_IER = UART_IER_PARE | UART_IER_FRAME | UART_IER_OVRE;

  /* Starting the receiver idle loop.*/
  set_rx_idle_loop(uartp);

  nvicEnableVector(uartp->irq_id, CORTEX_PRIORITY_MASK(irq_priority));
}

/**
 * @brief   USART common service routine.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 */
static void serve_usart_irq(UARTDriver *uartp) {
  uint32_t isr;
  Uart *u = uartp->reg.uart;

  isr = u->UART_SR & u->UART_IMR;

  if (isr & (UART_SR_OVRE | UART_SR_PARE | UART_SR_FRAME)) {
    /* Clearing status.*/
    u->UART_CR = UART_CR_RSTSTA;
    if (uartp->config->rxerr_cb != NULL)
      uartp->config->rxerr_cb(uartp, translate_errors(isr));
  }
  if (isr & UART_SR_TXEMPTY) {
    uartp->txstate = UART_TX_COMPLETE;
    u->UART_IDR = UART_IDR_TXEMPTY;
    /* End of transmission, a callback is generated.*/
    if (uartp->config->txend1_cb != NULL)
      uartp->config->txend1_cb(uartp);

    // If the callback didn't explicitly change state then the transmitter
    // automatically returns to the idle state.
    if (uartp->txstate == UART_TX_COMPLETE)
      uartp->txstate = UART_TX_IDLE;
  }
  if (isr & UART_SR_ENDTX) {
    u->UART_IDR = UART_IDR_ENDTX;
    /* End of transmission, a callback is generated.*/
    if (uartp->config->txend2_cb != NULL)
      uartp->config->txend2_cb(uartp);
  }
  if (isr & UART_SR_ENDRX) {
    u->UART_IDR = UART_IDR_ENDRX;
    uartp->rxstate = UART_RX_COMPLETE;
    /* End of receive buffering, a callback is generated.*/
    if (uartp->config->rxend_cb != NULL)
      uartp->config->rxend_cb(uartp);

    // If the callback didn't explicitly change state then the transmitter
    // automatically returns to the idle state.
    if (uartp->rxstate == UART_RX_COMPLETE) {
      uartp->rxstate = UART_RX_IDLE;
      set_rx_idle_loop(uartp);
    }
  }
  if (isr & UART_SR_RXRDY) {
    /* End of receive buffering, a callback is generated.*/
    if (uartp->config->rxchar_cb != NULL)
      uartp->config->rxchar_cb(uartp, u->UART_RHR);
  }
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if SAM3XA_UART_USE_UART || defined(__DOXYGEN__)
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
  serve_usart_irq(&UARTD1);
  CH_IRQ_EPILOGUE();
}
#endif /* SAM3XA_UART_USE_UART */

#if SAM3XA_UART_USE_USART0 || defined(__DOXYGEN__)
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
  serve_usart_irq(&UARTD2);
  CH_IRQ_EPILOGUE();
}
#endif /* SAM3XA_UART_USE_USART0 */

#if SAM3XA_UART_USE_USART1 || defined(__DOXYGEN__)
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
  serve_usart_irq(&UARTD3);
  CH_IRQ_EPILOGUE();
}
#endif /* SAM3XA_UART_USE_USART1 */

#if SAM3XA_UART_USE_USART2 || defined(__DOXYGEN__)
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
  serve_usart_irq(&UARTD4);
  CH_IRQ_EPILOGUE();
}
#endif /* SAM3XA_UART_USE_USART2 */

#if SAM3XA_UART_USE_USART3 || defined(__DOXYGEN__)
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
  serve_usart_irq(&UARTD5);
  CH_IRQ_EPILOGUE();
}
#endif /* SAM3XA_UART_USE_USART3 */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level UART driver initialization.
 *
 * @notapi
 */
void uart_lld_init(void) {
#if SAM3XA_UART_USE_UART
  uartObjectInit(&UARTD1);
#endif
#if SAM3XA_UART_USE_USART0
  uartObjectInit(&UARTD2);
#endif
#if SAM3XA_UART_USE_USART1
  uartObjectInit(&UARTD3);
#endif
#if SAM3XA_UART_USE_USART2
  uartObjectInit(&UARTD4);
#endif
#if SAM3XA_UART_USE_USART3
  uartObjectInit(&UARTD5);
#endif
}

/**
 * @brief   Configures and activates the UART peripheral.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 *
 * @notapi
 */
void uart_lld_start(UARTDriver *uartp) {
  uint32_t irq_priority =
      uartp->config->irq_priority ? uartp->config->irq_priority :
      SAM3XA_UART_DEFAULT_IRQ_PRIORITY;
  chDbgCheck(CORTEX_IS_VALID_KERNEL_PRIORITY(uartp->config->irq_priority),
      "UART irq_priority");

  usart_start(uartp, irq_priority);
}

/**
 * @brief   Deactivates the UART peripheral.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 *
 * @notapi
 */
void uart_lld_stop(UARTDriver *uartp) {
  if (uartp->state == UART_READY) {
    usart_stop(uartp);
  }
}

/**
 * @brief   Starts a transmission on the UART peripheral.
 * @note    The buffers are organized as uint8_t arrays for data sizes below
 *          or equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 * @param[in] n         number of data frames to send
 * @param[in] txbuf     the pointer to the transmit buffer
 *
 * @notapi
 */
void uart_lld_start_send(UARTDriver *uartp, size_t n, const void *txbuf) {
  pdc_disable_transfer(uartp->reg.uart);
  uartp->reg.uart->UART_IER = UART_IER_ENDTX | UART_IER_TXEMPTY;
  pdc_setup_transfer(uartp->reg.uart, n, txbuf);
}

/**
 * @brief   Stops any ongoing transmission.
 * @note    Stopping a transmission also suppresses the transmission callbacks.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 *
 * @return              The number of data frames not transmitted by the
 *                      stopped transmit operation.
 *
 * @notapi
 */
size_t uart_lld_stop_send(UARTDriver *uartp) {
  uartp->reg.uart->UART_IDR = UART_IDR_ENDTX | UART_IDR_TXEMPTY;
  return pdc_disable_transfer(uartp->reg.uart);
}

/**
 * @brief   Starts a receive operation on the UART peripheral.
 * @note    The buffers are organized as uint8_t arrays for data sizes below
 *          or equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 * @param[in] n         number of data frames to send
 * @param[out] rxbuf    the pointer to the receive buffer
 *
 * @notapi
 */
void uart_lld_start_receive(UARTDriver *uartp, size_t n, void *rxbuf) {
  pdc_disable_receive(uartp->reg.uart);
  uartp->reg.uart->UART_IER = UART_IER_ENDRX;
  uartp->reg.uart->UART_IDR = UART_IDR_RXRDY;
  pdc_setup_receive(uartp->reg.uart, n, rxbuf);
}

/**
 * @brief   Stops any ongoing receive operation.
 * @note    Stopping a receive operation also suppresses the receive callbacks.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 *
 * @return              The number of data frames not received by the
 *                      stopped receive operation.
 *
 * @notapi
 */
size_t uart_lld_stop_receive(UARTDriver *uartp) {
  size_t n;

  uartp->reg.uart->UART_IDR = UART_IDR_ENDRX;
  n = pdc_disable_receive(uartp->reg.uart);

  set_rx_idle_loop(uartp);
  return n;
}

#endif /* HAL_USE_UART */

/** @} */
