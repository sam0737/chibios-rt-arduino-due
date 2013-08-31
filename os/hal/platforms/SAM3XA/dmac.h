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

/**
 * @file    SAM3XA/dmac.h
 * @brief   Provides access to the direct memory access controller (DMAC)
 *
 * @addtogroup SAM3XA_HAL_DMAC
 * @{
 */

#ifndef _SAM3XA_HAL_DMAC_
#define _SAM3XA_HAL_DMAC_

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
 * @brief   DMAC default IRQ priority
 */
#define SAM3XA_DMAC_DEFAULT_IRQ_PRIORITY    12
/** @} */

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of an DMA completion callback.
 *
 * @param[in] ch        DMA channel number
 * @param[in] state     state pointer stored in prepare call
 */
typedef void (*dmacallback_t)(uint8_t ch, const void *state);

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
void dmac_lld_init(void);

void dmac_prepare_send(uint8_t ch, uint8_t per, dmacallback_t callback, const void* state,
    uint16_t size, const void* buf, const void* destination);

void dmac_prepare_send_dummy(uint8_t ch, uint8_t per,  dmacallback_t callback, const void* state,
    uint16_t size, uint32_t dummy, const void* destination);

void dmac_prepare_receive(uint8_t ch, uint8_t per, dmacallback_t callback, const void* state,
    uint16_t size, const void* buf, const void* source);

void dmac_prepare_receive_dummy(uint8_t ch, uint8_t per, dmacallback_t callback, const void* state,
    uint16_t size, const void* source);

void dmac_channel_start(uint8_t ch);
#ifdef __cplusplus
}
#endif

#endif /* _SAM3XA_HAL_DMAC_ */

/** @} */
