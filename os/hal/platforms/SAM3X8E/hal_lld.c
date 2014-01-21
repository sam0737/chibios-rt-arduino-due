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
 * @file    SAM3X8E/hal_lld.c
 * @brief   HAL Driver subsystem low level driver source template.
 *
 * @addtogroup HAL
 * @{
 */

#include "ch.h"
#include "hal.h"

#include "sam3x8e.h"

#if SAM3XA_USB_USE_DMAC
#include "dmac.h"
#endif

#ifndef SAM3XA_SLOW_CLOCK_USE_CRYSTAL
#define SAM3XA_SLOW_CLOCK_USE_CRYSTAL FALSE
#endif

/* Clock settings (12MHz crystal to 84MHz) */
#ifndef SYS_BOARD_OSCOUNT
#define SYS_BOARD_OSCOUNT   (CKGR_MOR_MOSCXTST(0x8))
// PLLA = 168Mhz
#define SYS_BOARD_PLLAR     (CKGR_PLLAR_ONE \
                            | CKGR_PLLAR_MULA(0xdUL) \
                            | CKGR_PLLAR_PLLACOUNT(0x3fUL) \
                            | CKGR_PLLAR_DIVA(0x1UL))
#define SYS_BOARD_MCKR      (PMC_MCKR_PRES_CLK_2 | PMC_MCKR_CSS_PLLA_CLK)
#define SYS_BOARD_FINAL_FREQ    (84000000UL)
#endif

/* Clock Definitions */
#define SYS_UTMIPLL             (480000000UL)	/* UTMI PLL frequency */

#define SYS_CKGR_MOR_KEY_VALUE  CKGR_MOR_KEY(0x37) /* Key to unlock MOR register */

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

uint32_t SystemCoreClock = CHIP_FREQ_MAINCK_RC_4MHZ;

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @\brief Setup the microcontroller system. (CMSIS Convention)
 * Initialize the System and update the SystemFrequency variable.
 */
static void SystemInit(void)
{
	/* Set FWS according to SYS_BOARD_MCKR configuration */
	EFC0->EEFC_FMR = EEFC_FMR_FWS(4);
	EFC1->EEFC_FMR = EEFC_FMR_FWS(4);

	/* Initialize main oscillator */
	if (!(PMC->CKGR_MOR & CKGR_MOR_MOSCSEL)) {
		PMC->CKGR_MOR = SYS_CKGR_MOR_KEY_VALUE | SYS_BOARD_OSCOUNT |
			                     CKGR_MOR_MOSCRCEN | CKGR_MOR_MOSCXTEN;
		while (!(PMC->PMC_SR & PMC_SR_MOSCXTS)) {
		}
	}

	/* Switch to 3-20MHz Xtal oscillator */
	PMC->CKGR_MOR = SYS_CKGR_MOR_KEY_VALUE | SYS_BOARD_OSCOUNT |
	                           CKGR_MOR_MOSCRCEN | CKGR_MOR_MOSCXTEN | CKGR_MOR_MOSCSEL;

	while (!(PMC->PMC_SR & PMC_SR_MOSCSELS)) {
	}
 	PMC->PMC_MCKR = (PMC->PMC_MCKR & ~(uint32_t)PMC_MCKR_CSS_Msk) |
		                     PMC_MCKR_CSS_MAIN_CLK;
	while (!(PMC->PMC_SR & PMC_SR_MCKRDY)) {

	}
	/* Initialize PLLA */
	PMC->CKGR_PLLAR = SYS_BOARD_PLLAR;
	while (!(PMC->PMC_SR & PMC_SR_LOCKA)) {
	}

	/* Switch to main clock */
	PMC->PMC_MCKR = (SYS_BOARD_MCKR & ~PMC_MCKR_CSS_Msk) | PMC_MCKR_CSS_MAIN_CLK;
	while (!(PMC->PMC_SR & PMC_SR_MCKRDY)) {
	}

	/* Switch to PLLA */
	PMC->PMC_MCKR = SYS_BOARD_MCKR;
	while (!(PMC->PMC_SR & PMC_SR_MCKRDY)) {
	}

	SystemCoreClock = SYS_BOARD_FINAL_FREQ;

#if SAM3XA_SLOW_CLOCK_USE_CRYSTAL
	SUPC->SUPC_CR = SUPC_CR_KEY(0xA5) | SUPC_CR_XTALSEL_CRYSTAL_SEL;
#endif
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level HAL driver initialization.
 *
 * @notapi
 */
void hal_lld_init(void) {
	SysTick_Config(SystemCoreClock / CH_FREQUENCY);
	/* Set slow clock frequency to 32768 */
	RTT->RTT_MR = RTT_MR_RTPRES(1) | RTT_MR_RTTRST;
#if SAM3XA_USB_USE_DMAC
	dmac_lld_init();
#endif
}

void sam3x8e_clock_init(void) {
	SystemInit();
}

halrtcnt_t hal_lld_get_counter_value(void) {
  chSysLock();
  uint32_t tick1 = SysTick->VAL;
  uint32_t time = chTimeNow();
  uint32_t tick2 = SysTick->VAL;
  chSysUnlock();
  return (halrtcnt_t) (time + (tick1 > tick2 ? 1 : 0)) * (SystemCoreClock / CH_FREQUENCY) + tick1;
}

halrtcnt_t hal_lld_get_counter_frequency(void)
{
  return (halrtcnt_t) SystemCoreClock;
}

/** @} */
