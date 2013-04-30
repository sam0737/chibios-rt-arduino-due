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

/*
 * Part of the code is adopted from Atmel SAM software package, the following
 * license applies.
 */

/* ----------------------------------------------------------------------------
 *         SAM Software Package License
 * ----------------------------------------------------------------------------
 * Copyright (c) 2011-2012, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following condition is met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

/**
 * @file    SAM3XA/pmc.c
 * @brief   Provides access to power management controller (PMC)
 *
 * @addtogroup SAM3XA_HAL_PMC
 * @{
 */

#include "sam3xa_min.h"
#include "pmc.h"

/**
 * @brief   PMC_WPMR Write Protect key for locking/unlocking
 */
#define PMC_WPMR_WPKEY_VALUE    PMC_WPMR_WPKEY((uint32_t) 0x504D43)

/**
 * @brief Enable or disable write protect of PMC registers.
 *
 * @param enable 1 to enable (lock), 0 to disable (unlock).
 */
void pmc_set_writeprotect(uint32_t enable)
{
  if (enable) {
    PMC->PMC_WPMR = PMC_WPMR_WPKEY_VALUE | PMC_WPMR_WPEN;
  } else {
    PMC->PMC_WPMR = PMC_WPMR_WPKEY_VALUE;
  }
}


/**
 * @brief Enable the peripheral clock for the specific device
 *
 * @param peripheral_id The specific device id to enable
 */
void pmc_enable_peripheral_clock(uint32_t peripheral_id)
{
  if (peripheral_id < 32) {
    PMC->PMC_PCER0 = 1 << peripheral_id;
  } else {
    peripheral_id -= 32;
    PMC->PMC_PCER1 = 1 << peripheral_id;
  }
}

/**
 * @brief Enable the peripheral clock set clock division for the
 *        specific device
 * @note  Only CAN0, CAN1 can use divider
 *
 * @param peripheral_id The specific device id to enable
 * @param div Clock division
 *            PMC_PCR_DIV_PERIPH_DIV_MCK, PMC_PCR_DIV_PERIPH_DIV2_MCK, PMC_PCR_DIV_PERIPH_DIV4_MCK
 */
void pmc_enable_peripheral_clock_with_div(uint32_t peripheral_id, uint32_t div)
{
  // Disable
  pmc_disable_peripheral_clock(peripheral_id);
  // Set DIV
  PMC->PMC_PCR = PMC_PCR_PID(peripheral_id) | div | PMC_PCR_CMD;
  // Re-enable
  pmc_enable_peripheral_clock(peripheral_id);
}

/**
 * @brief Disable the peripheral clock for the specific device
 *
 * @param peripheral_id The specific device id to disable
 */
void pmc_disable_peripheral_clock(uint32_t peripheral_id)
{
  if (peripheral_id < 32) {
    PMC->PMC_PCDR0 = 1 << peripheral_id;
  } else {
    peripheral_id -= 32;
    PMC->PMC_PCDR1 = 1 << peripheral_id;
  }
}

/**
 * \brief Enable UPLL clock.
 */
void pmc_enable_upll_clock(void)
{
  PMC->CKGR_UCKR = CKGR_UCKR_UPLLCOUNT(3) | CKGR_UCKR_UPLLEN;

  /* Wait UTMI PLL Lock Status */
  while (!(PMC->PMC_SR & PMC_SR_LOCKU));
}

/**
 * \brief Disable UPLL clock.
 */
void pmc_disable_upll_clock(void)
{
  PMC->CKGR_UCKR &= ~CKGR_UCKR_UPLLEN;
}

/**
 * \brief Is UPLL locked?
 *
 * \retval 0 Not locked.
 * \retval 1 Locked.
 */
uint32_t pmc_is_locked_upll(void)
{
  return (PMC->PMC_SR & PMC_SR_LOCKU);
}

/**
 * \brief Switch UDP (USB) clock source selection to UPLL clock.
 *
 * \param dw_usbdiv Clock divisor.
 */
void pmc_switch_udpck_to_upllck(uint32_t ul_usbdiv)
{
  PMC->PMC_USB = PMC_USB_USBS | PMC_USB_USBDIV(ul_usbdiv);
}

/**
 * \brief Enable UDP (USB) clock.
 */
void pmc_enable_udpck(void)
{
  PMC->PMC_SCER = PMC_SCER_UOTGCLK;
}

/**
 * \brief Disable UDP (USB) clock.
 */
void pmc_disable_udpck(void)
{
  PMC->PMC_SCDR = PMC_SCDR_UOTGCLK;
}

/** @} */
