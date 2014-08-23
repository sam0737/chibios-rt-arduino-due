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
 * @file    SAM3XA/uart_lld.h
 * @brief   SAM3XA low level UART driver header.
 *
 * @addtogroup UART
 * @{
 */

#ifndef _UART_LLD_H_
#define _UART_LLD_H_

#if HAL_USE_UART || defined(__DOXYGEN__)

#include "peripheral_config.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   UART driver on UART enable switch.
 * @details If setb to @p TRUE the support for UART is included.
 *          The device object is UARTD1.
 * @note    The default is @p FALSE.
 */
#if !defined(SAM3XA_UART_USE_UART) || defined(__DOXYGEN__)
#define SAM3XA_UART_USE_UART       FALSE
#endif

/**
 * @brief   UART driver on USART0 enable switch.
 * @details If set to @p TRUE the support for USART1 is included.
 *          The device object is UARTD2.
 * @note    The default is @p FALSE.
 */
#if !defined(SAM3XA_UART_USE_USART0) || defined(__DOXYGEN__)
#define SAM3XA_UART_USE_USART0     FALSE
#endif

/**
 * @brief   UART driver on USART1 enable switch.
 * @details If set to @p TRUE the support for USART1 is included.
 *          The device object is UARTD3.
 * @note    The default is @p FALSE.
 */
#if !defined(SAM3XA_UART_USE_USART1) || defined(__DOXYGEN__)
#define SAM3XA_UART_USE_USART1     FALSE
#endif

/**
 * @brief   UART driver on USART2 enable switch.
 * @details If set to @p TRUE the support for USART2 is included.
 *          The device object is UARTD4.
 * @note    The default is @p FALSE.
 */
#if !defined(SAM3XA_UART_USE_USART2) || defined(__DOXYGEN__)
#define SAM3XA_UART_USE_USART2     FALSE
#endif

/**
 * @brief   UART driver on USART3 enable switch.
 * @details If set to @p TRUE the support for USART3 is included.
 *          The device object is UARTD5.
 * @note    The default is @p FALSE.
 */
#if !defined(SAM3XA_UART_USE_USART3) || defined(__DOXYGEN__)
#define SAM3XA_UART_USE_USART3     FALSE
#endif

/**
 * @brief   UART default IRQ priority
 */
#define SAM3XA_UART_DEFAULT_IRQ_PRIORITY    12
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if SAM3XA_UART_USE_UART && !defined(UART)
#error "UART not present in the selected device"
#endif

#if SAM3XA_UART_USE_USART0 && !defined(USART0)
#error "USART0 not present in the selected device"
#endif

#if SAM3XA_UART_USE_USART1 && !defined(USART1)
#error "USART1 not present in the selected device"
#endif

#if SAM3XA_UART_USE_USART2 && !defined(USART2)
#error "USART2 not present in the selected device"
#endif

#if SAM3XA_UART_USE_USART3 && !defined(USART3)
#error "USART3 not present in the selected device"
#endif

#if !SAM3XA_UART_USE_UART && !SAM3XA_UART_USE_USART0 && \
    !SAM3XA_UART_USE_USART1 && !SAM3XA_UART_USE_USART2 && \
    !SAM3XA_UART_USE_USART3
#error "UART driver activated but no USART/UART peripheral assigned"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   UART driver condition flags type.
 */
typedef uint32_t uartflags_t;

/**
 * @brief   Structure representing an UART driver.
 */
typedef struct UARTDriver UARTDriver;

/**
 * @brief   Generic UART notification callback type.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 */
typedef void (*uartcb_t)(UARTDriver *uartp);

/**
 * @brief   Character received UART notification callback type.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 * @param[in] c         received character
 */
typedef void (*uartccb_t)(UARTDriver *uartp, uint16_t c);

/**
 * @brief   Receive error UART notification callback type.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 * @param[in] e         receive error mask
 */
typedef void (*uartecb_t)(UARTDriver *uartp, uartflags_t e);

/**
 * @brief   Driver configuration structure.
 * @note    It could be empty on some architectures.
 */
typedef struct {
  /**
   * @brief End of transmission buffer callback.
   */
  uartcb_t                  txend1_cb;
  /**
   * @brief Physical end of transmission callback.
   */
  uartcb_t                  txend2_cb;
  /**
   * @brief Receive buffer filled callback.
   */
  uartcb_t                  rxend_cb;
  /**
   * @brief Character received while out if the @p UART_RECEIVE state.
   */
  uartccb_t                 rxchar_cb;
  /**
   * @brief Receive error callback.
   */
  uartecb_t                 rxerr_cb;
  /* End of the mandatory fields.*/

  /**
   * @brief TX pin config
   */
  PeripheralPinConfig       tx_pin;

  /**
   * @brief RX pin config
   */
  PeripheralPinConfig       rx_pin;

  /**
   * @brief RTS pin config (Only for USART0, 1, 2)
   */
  PeripheralPinConfig       rts_pin;

  /**
   * @brief CTS pin config (Only for USART0, 1, 2)
   */
  PeripheralPinConfig       cts_pin;

  /**
   * @brief SCK pin config (Only for USART0, 1, 2)
   */
  PeripheralPinConfig       sck_pin;

  /**
   * @brief Bit rate.
   */
  uint32_t                  speed;

  /**
   * @brief IRQ priority. If not set, a default will be assigned.
   */
  uint32_t                  irq_priority;

  /**
   * @brief Initialization value for the UART/USART Mode Register (See UART_MR/US_MR).
   * @note  This control important information like parity bit.
   *        Most likely you want to set the parity bit.
   */
  uint32_t                  mr;
  /**
   * @brief Initialization value for the receiver time-out register
   *        (See US_RTOR)
   */
  uint32_t                  rtor;
  /**
   * @brief Initialization value for the transmitter timeguard register
   *        (See US_TTGR)
   */
  uint32_t                  ttgr;
} UARTConfig;

/**
 * @brief   Structure representing an UART driver.
 */
struct UARTDriver {
  /**
   * @brief Driver state.
   */
  uartstate_t               state;
  /**
   * @brief Transmitter state.
   */
  uarttxstate_t             txstate;
  /**
   * @brief Receiver state.
   */
  uartrxstate_t             rxstate;
  /**
   * @brief Current configuration data.
   */
  const UARTConfig          *config;
#if defined(UART_DRIVER_EXT_FIELDS)
  UART_DRIVER_EXT_FIELDS
#endif
  /* End of the mandatory fields.*/

  /**
   * @brief The peripheral id of the UART/USART device
   */
  uint8_t                   peripheral_id;

  /**
   * @brief The IRQ id of the UART/USART device
   */
  uint8_t                   irq_id;

  /**
   * @brief Determine if this is an USART or UART device
   */
  uint8_t                   is_usart;

  union {
    /**
     * @brief Pointer to the UART registers block.
     */
    Uart                    *uart;

    /**
     * @brief Pointer to the USART registers block.
     */
    Usart                   *usart;
  } reg;
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if SAM3XA_UART_USE_UART && !defined(__DOXYGEN__)
extern UARTDriver UARTD1;
#endif

#if SAM3XA_UART_USE_USART0 && !defined(__DOXYGEN__)
extern UARTDriver UARTD2;
#endif

#if SAM3XA_UART_USE_USART1 && !defined(__DOXYGEN__)
extern UARTDriver UARTD3;
#endif

#if SAM3XA_UART_USE_USART2 && !defined(__DOXYGEN__)
extern UARTDriver UARTD4;
#endif

#if SAM3XA_UART_USE_USART3 && !defined(__DOXYGEN__)
extern UARTDriver UARTD5;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void uart_lld_init(void);
  void uart_lld_start(UARTDriver *uartp);
  void uart_lld_stop(UARTDriver *uartp);
  void uart_lld_start_send(UARTDriver *uartp, size_t n, const void *txbuf);
  size_t uart_lld_stop_send(UARTDriver *uartp);
  void uart_lld_start_receive(UARTDriver *uartp, size_t n, void *rxbuf);
  size_t uart_lld_stop_receive(UARTDriver *uartp);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_UART */

#endif /* _UART_LLD_H_ */

/** @} */
