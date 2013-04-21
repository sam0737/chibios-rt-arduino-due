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
 * @file    SAM3XA/pdc.c
* @brief   Provides access to peripheral DMA controller (PDC)
 *
 * @addtogroup SAM3XA_HAL_PMC
 * @{
 */

#include "sam3xa_min.h"
#include "pdc.h"

/**
 * @brief Start transmitting txbuf to device through PDC
 * @note  User should enable corresponding interrupt for complete notification
 *        Caller must make sure this PDC is not active when call is made.
 *
 * @param device  The base address for the device
 * @param n       Length of txbuf
 * @param txbuf   Transmit buffer
 */
void pdc_setup_transfer(void *device, size_t n, const void *txbuf) {
  ((OffsetedPdc*)device)->PERIPH_TPR = (uint32_t)txbuf;
  ((OffsetedPdc*)device)->PERIPH_TCR = n;
  ((OffsetedPdc*)device)->PERIPH_PTCR = PERIPH_PTCR_TXTEN;
}


/**
 * @brief Start receiving from device to rxbuf through PDC
 * @note  Caller should enable corresponding interrupt for complete notification.
 *        Caller must make sure this PDC is not active when call is made.
 *
 * @param device  The base address for the device
 * @param n       Length of rxbuf
 * @param txbuf   Receive buffer
 */
void pdc_setup_receive(void *device, size_t n, const void *rxbuf) {
  ((OffsetedPdc*)device)->PERIPH_RPR = (uint32_t)rxbuf;
  ((OffsetedPdc*)device)->PERIPH_RCR = n;
  __DMB();
  ((OffsetedPdc*)device)->PERIPH_PTCR = PERIPH_PTCR_RXTEN;
}

/**
 * @brief Abort/disable the PDC transmit
 * @param device  The base address for the device
 * @return How many bytes were not written
 */
size_t pdc_disable_transfer(void *device) {
  ((OffsetedPdc*)device)->PERIPH_PTCR = PERIPH_PTCR_TXTDIS;
  __DMB();
  return ((OffsetedPdc*)device)->PERIPH_TCR;
}

/**
 * @brief Abort/disable the PDC receive
 * @param device  The base address for the device
 * @return How many bytes were not received
 */
size_t pdc_disable_receive(void *device) {
  ((OffsetedPdc*)device)->PERIPH_PTCR = PERIPH_PTCR_RXTDIS;
  __DMB();
  return ((OffsetedPdc*)device)->PERIPH_RCR;
}
/** @} */
