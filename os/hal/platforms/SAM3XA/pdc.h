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
 * @file    SAM3XA/pdc.h
 * @brief   Provides access to peripheral DMA controller (PDC)
 *
 * @addtogroup SAM3XA_HAL_PDC
 * @{
 */

#ifndef _SAM3XA_HAL_PDC_
#define _SAM3XA_HAL_PDC_

typedef struct {
  uint8_t Reserved[0x100]; /**< \brief Device base register */
  RwReg PERIPH_RPR;   /**< \brief (Offset: 0x100) Receive Pointer Register */
  RwReg PERIPH_RCR;   /**< \brief (Offset: 0x104) Receive Counter Register */
  RwReg PERIPH_TPR;   /**< \brief (Offset: 0x108) Transmit Pointer Register */
  RwReg PERIPH_TCR;   /**< \brief (Offset: 0x10C) Transmit Counter Register */
  RwReg PERIPH_RNPR;  /**< \brief (Offset: 0x110) Receive Next Pointer Register */
  RwReg PERIPH_RNCR;  /**< \brief (Offset: 0x114) Receive Next Counter Register */
  RwReg PERIPH_TNPR;  /**< \brief (Offset: 0x118) Transmit Next Pointer Register */
  RwReg PERIPH_TNCR;  /**< \brief (Offset: 0x11C) Transmit Next Counter Register */
  WoReg PERIPH_PTCR;  /**< \brief (Offset: 0x120) Transfer Control Register */
  RoReg PERIPH_PTSR;  /**< \brief (Offset: 0x124) Transfer Status Register */
} OffsetedPdc;

/* -------- PERIPH_PTCR : (Pdc Offset: 0x120) Transfer Control Register -------- */
#define PERIPH_PTCR_RXTEN (0x1u << 0) /**< \brief (PERIPH_PTCR) Receive Transfer Enable */
#define PERIPH_PTCR_RXTDIS (0x1u << 1) /**< \brief (PERIPH_PTCR) Receive Transfer Disable */
#define PERIPH_PTCR_TXTEN (0x1u << 8) /**< \brief (PERIPH_PTCR) Transmit Transfer Enable */
#define PERIPH_PTCR_TXTDIS (0x1u << 9) /**< \brief (PERIPH_PTCR) Transmit Transfer Disable */

/* -------- PERIPH_PTSR : (Pdc Offset: 0x124) Transfer Status Register -------- */
#define PERIPH_PTSR_RXTEN (0x1u << 0) /**< \brief (PERIPH_PTSR) Receive Transfer Enabled */
#define PERIPH_PTSR_TXTEN (0x1u << 8) /**< \brief (PERIPH_PTSR) Transmit Transfer Enabled */

#ifdef __cplusplus
extern "C" {
#endif
void pdc_setup_transfer(void *dev_base, size_t n, const void *txbuf);
void pdc_setup_receive(void *dev_base, size_t n, const void *rxbuf);
size_t pdc_disable_transfer(void *dev_base);
size_t pdc_disable_receive(void *dev_base);
#ifdef __cplusplus
}
#endif

#endif /* _SAM3XA_HAL_PDC_ */

/** @} */
