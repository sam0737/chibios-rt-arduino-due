/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

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
*/

/**
 * @file    ARMCMx/LPC111x/cmparams.h
 * @brief   ARM Cortex-M0 LPC111x specific parameters.
 *
 * @defgroup ARMCMx_LPC111x LPC111x specific parameters
 * @ingroup ARMCMx
 * @details This file contains the Cortex-M0 specific parameters for the
 *          LPC111x platform.
 * @{
 */

#ifndef _CMPARAMS_H_
#define _CMPARAMS_H_

/*===========================================================================*/
/* Constants parameters.                                                     */
/*===========================================================================*/

#define CORTEX_M0               0       /**< @brief Cortex-M0 variant.      */
#define CORTEX_M3               3       /**< @brief Cortex-M3 variant.      */

/**
 * @brief   Cortex core model.
 */
#define CORTEX_MODEL            CORTEX_M0

/**
 * @brief   Systick unit presence.
 */
#define CORTEX_HAS_ST           TRUE

/**
 * @brief   Memory Protection unit presence.
 */
#define CORTEX_HAS_MPU          FALSE

/**
 * @brief   Number of bits in priority masks.
 * @details The available number of priority levels is equal to
 *          (1 << @p CORTEX_PRIORITY_BITS).
 */
#define CORTEX_PRIORITY_BITS    2

/**
 * @brief   Priority to priority mask conversion macro.
 */
#define CORTEX_PRIORITY(n)      ((n) << (8 - CORTEX_PRIORITY_BITS))

/*===========================================================================*/
/* Configurable parameters.                                                  */
/*===========================================================================*/

/**
 * @brief   SYSTICK handler priority.
 */
#ifndef CORTEX_PRIORITY_SYSTICK
#define CORTEX_PRIORITY_SYSTICK CORTEX_PRIORITY(2)
#endif

#endif /* _CMPARAMS_H_ */

/** @} */