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
 * @file    SAM3X8E/pal_lld.c
 * @brief   PAL subsystem low level driver template.
 *
 * @addtogroup PAL
 * @{
 */

#include "ch.h"
#include "hal.h"

#if HAL_USE_PAL || defined(__DOXYGEN__)

#include "sam3x8e.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/
void _pal_lld_init(const PALConfig *config)
{
	(void)config;

	// Enable direct write to ODSR
	PIOA->PIO_OWER = 0xffffffff;
	PIOB->PIO_OWER = 0xffffffff;
	PIOC->PIO_OWER = 0xffffffff;
	PIOD->PIO_OWER = 0xffffffff;

  pmc_enable_peripheral_clock(ID_PIOA);
  pmc_enable_peripheral_clock(ID_PIOB);
  pmc_enable_peripheral_clock(ID_PIOC);
  pmc_enable_peripheral_clock(ID_PIOD);
}

void _pal_lld_setgroupmode(ioportid_t port, ioportmask_t mask, iomode_t mode)
{
  // Enable PIO function
  (port)->PIO_PER = mask;

	switch (mode)
	{
	case PAL_MODE_RESET:
	case PAL_MODE_UNCONNECTED:
	case PAL_MODE_INPUT_PULLUP:
    (port)->PIO_ODR = mask;   // Output disabled
		(port)->PIO_PUER = mask;	// Pull-up enabled
    (port)->PIO_MDER = mask;  // Open-drain enabled
		break;

	case PAL_MODE_INPUT_ANALOG:
	case PAL_MODE_INPUT:
		(port)->PIO_ODR = mask;		// Output disabled
		(port)->PIO_PUDR = mask;	// Pull-up disabled
    (port)->PIO_MDDR = mask;  // Open-drain disabled
		break;

	case PAL_MODE_OUTPUT_PUSHPULL:
		(port)->PIO_OER = mask;		// Output enabled
		(port)->PIO_PUDR = mask;	// Pull-up disabled
    (port)->PIO_MDDR = mask;  // Open-drain disabled
		break;

	case PAL_MODE_OUTPUT_OPENDRAIN:
		(port)->PIO_OER = mask;		// Output enabled
		(port)->PIO_PUER = mask;	// Pull-up enabled
    (port)->PIO_MDER = mask;  // Open-drain enabled
		break;
	}
}

#endif /* HAL_USE_PAL */

/** @} */
