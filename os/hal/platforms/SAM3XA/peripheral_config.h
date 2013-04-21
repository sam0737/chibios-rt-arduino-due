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
 * @file    SAM3XA/peripheral_config.h
 * @brief   Provides utilities for configuring PIO multiplexor for peripheral functions
 *
 * @note    Most SAM3XA peripheral supports routing its IO pin to more than 1 actual pin.
 *          This provides the structure to describe such settings
 *
 * @addtogroup SAM3XA_HAL_PERIPHERAL_CONFIG
 * @{
 */

#ifndef _SAM3XA_HAL_PERIPHERAL_CONFIG_
#define _SAM3XA_HAL_PERIPHERAL_CONFIG_

/**
 * @brief   PIO in Mode A (See PIO_ABSR)
 */
#define PIO_MODE_A          0

/**
 * @brief   PIO in Mode A (See PIO_ABSR)
 */
#define PIO_MODE_B          1

/**
 * @brief   Macro indicating driver should use the most common default options
 */
#define DEFAULT_PERIPHERAL_PIN_CONFIG ((PeripheralPinConfig) {0,0,0})

/**
 * @brief   Check if the supplied config is a default config, not a valid one
 */
#define is_default_peripheral_config(config) \
		((uint32_t) ((PeripheralPinConfig)config)->port != 0)


/**
 * @brief   Describe a peripheral pin config
 */
typedef struct {
  /**
   * @brief   The PIO port that the IO should be routed to
   */
  Pio         *port;

  /**
   * @brief   The pin number that the IO should be routed to
   */
  uint8_t     pin;

  /**
   * @brief   PIO_MODE_A, or PIO_MODE_B
   */
  uint8_t     mode;
} PeripheralPinConfig;

#ifdef __cplusplus
extern "C" {
#endif
void peripheral_pin_apply(const PeripheralPinConfig *config);
void peripheral_pin_reset(const PeripheralPinConfig *config);
#ifdef __cplusplus
}
#endif

#endif /* _SAM3XA_HAL_PERIPHERAL_CONFIG_ */

/** @} */
