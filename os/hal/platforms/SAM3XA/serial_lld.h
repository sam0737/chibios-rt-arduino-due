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
 * @file    SAM3XA/serial_lld.h
 * @brief   SAM3XA low level serial driver header.
 *
 * @addtogroup SERIAL
 * @{
 */

#ifndef _SERIAL_LLD_H_
#define _SERIAL_LLD_H_

#if HAL_USE_SERIAL || defined(__DOXYGEN__)

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
 * @brief   SERIAL driver on SERIAL enable switch.
 * @details If setb to @p TRUE the support for SERIAL is included.
 *          The device object is SERIALD1.
 * @note    The default is @p FALSE.
 */
#if !defined(SAM3XA_SERIAL_USE_SERIAL) || defined(__DOXYGEN__)
#define SAM3XA_SERIAL_USE_SERIAL       FALSE
#endif

/**
 * @brief   SERIAL driver on USART0 enable switch.
 * @details If set to @p TRUE the support for USART1 is included.
 *          The device object is SERIALD2.
 * @note    The default is @p FALSE.
 */
#if !defined(SAM3XA_SERIAL_USE_USART0) || defined(__DOXYGEN__)
#define SAM3XA_SERIAL_USE_USART0     FALSE
#endif

/**
 * @brief   SERIAL driver on USART1 enable switch.
 * @details If set to @p TRUE the support for USART1 is included.
 *          The device object is SERIALD3.
 * @note    The default is @p FALSE.
 */
#if !defined(SAM3XA_SERIAL_USE_USART1) || defined(__DOXYGEN__)
#define SAM3XA_SERIAL_USE_USART1     FALSE
#endif

/**
 * @brief   SERIAL driver on USART2 enable switch.
 * @details If set to @p TRUE the support for USART2 is included.
 *          The device object is SERIALD4.
 * @note    The default is @p FALSE.
 */
#if !defined(SAM3XA_SERIAL_USE_USART2) || defined(__DOXYGEN__)
#define SAM3XA_SERIAL_USE_USART2     FALSE
#endif

/**
 * @brief   SERIAL driver on USART3 enable switch.
 * @details If set to @p TRUE the support for USART3 is included.
 *          The device object is SERIALD5.
 * @note    The default is @p FALSE.
 */
#if !defined(SAM3XA_SERIAL_USE_USART3) || defined(__DOXYGEN__)
#define SAM3XA_SERIAL_USE_USART3     FALSE
#endif

/**
 * @brief   SERIAL default IRQ priority
 */
#define SAM3XA_SERIAL_DEFAULT_IRQ_PRIORITY    12
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if SAM3XA_SERIAL_USE_UART && !defined(UART)
#error "UART not present in the selected device"
#endif

#if SAM3XA_SERIAL_USE_USART0 && !defined(USART0)
#error "USART0 not present in the selected device"
#endif

#if SAM3XA_SERIAL_USE_USART1 && !defined(USART1)
#error "USART1 not present in the selected device"
#endif

#if SAM3XA_SERIAL_USE_USART2 && !defined(USART2)
#error "USART2 not present in the selected device"
#endif

#if SAM3XA_SERIAL_USE_USART3 && !defined(USART3)
#error "USART3 not present in the selected device"
#endif

#if !SAM3XA_SERIAL_USE_UART && !SAM3XA_SERIAL_USE_USART0 && \
    !SAM3XA_SERIAL_USE_USART1 && !SAM3XA_SERIAL_USE_USART2 && \
    !SAM3XA_SERIAL_USE_USART3
#error "Serial driver activated but no USART/UART peripheral assigned"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Driver configuration structure.
 * @note    It could be empty on some architectures.
 */
typedef struct {
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
   * @brief Bit rate.
   */
  uint32_t                  speed;

  /**
   * @brief IRQ priority. If not set, a default will be assigned.
   */
  uint32_t                  irq_priority;

  /**
   * @brief Initialization value for the SERIAL/USART Mode Register (See SERIAL_MR/US_MR).
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
} SerialConfig;

/**
 * @brief   @p SerialDriver specific data.
 */
#define _serial_driver_data                                                 \
  _base_asynchronous_channel_data                                           \
  /* Driver state.*/                                                        \
  sdstate_t                 state;                                          \
  /* Input queue.*/                                                         \
  InputQueue                iqueue;                                         \
  /* Output queue.*/                                                        \
  OutputQueue               oqueue;                                         \
  /* Input circular buffer.*/                                               \
  uint8_t                   ib[SERIAL_BUFFERS_SIZE];                        \
  /* Output circular buffer.*/                                              \
  uint8_t                   ob[SERIAL_BUFFERS_SIZE];                        \
  /* End of the mandatory fields.*/                                         \
  /* Config */                                                              \
  const SerialConfig              *config;                                  \
  /* The peripheral id of the UART/USART device */                          \
  uint8_t                   peripheral_id;                                  \
  /* The IRQ id of the UART/USART device */                                 \
  uint8_t                   irq_id;                                         \
  /* if this is an USART or SERIAL device */                                \
  uint8_t                   is_usart;                                       \
  /* Pointer to the device register */                                      \
  union { Uart *uart; Usart *usart; } reg;                                  \


/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if SAM3XA_SERIAL_USE_UART && !defined(__DOXYGEN__)
extern SerialDriver SD1;
#endif

#if SAM3XA_SERIAL_USE_USART0 && !defined(__DOXYGEN__)
extern SerialDriver SD2;
#endif

#if SAM3XA_SERIAL_USE_USART1 && !defined(__DOXYGEN__)
extern SerialDriver SD3;
#endif

#if SAM3XA_SERIAL_USE_USART2 && !defined(__DOXYGEN__)
extern SerialDriver SD4;
#endif

#if SAM3XA_SERIAL_USE_USART3 && !defined(__DOXYGEN__)
extern SerialDriver SD5;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void sd_lld_init(void);
  void sd_lld_start(SerialDriver *sdp, const SerialConfig *config);
  void sd_lld_stop(SerialDriver *sdp);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_SERIAL */

#endif /* _SERIAL_LLD_H_ */

/** @} */
