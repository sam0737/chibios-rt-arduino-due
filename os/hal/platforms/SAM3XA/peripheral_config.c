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
 * @file    SAM3XA/peripheral_config.c
 * @brief   Provides utilities for configuring PIO multiplexor for peripheral functions
 *
 * @addtogroup SAM3XA_HAL_PERIPHERAL_CONFIG
 * @{
 */

#include "sam3xa_min.h"
#include "peripheral_config.h"

/**
 * @brief   Disable pin for PIO and apply the settings described in the config
 */
void peripheral_pin_apply(const PeripheralPinConfig *config)
{
  if (config->port == 0)
    return;

  config->port->PIO_ABSR =
      (config->port->PIO_ABSR & ~(1 << config->pin)) | // Mask out
      (config->mode << config->pin);                 // Apply new settings
  config->port->PIO_PDR = 1 << config->pin;
}

/**
 * @brief   Enable pin for PIO.
 */
void peripheral_pin_reset(const PeripheralPinConfig *config)
{
  if (config->port == 0)
    return;

  config->port->PIO_PER = 1 << config->pin;
}

/** @} */
