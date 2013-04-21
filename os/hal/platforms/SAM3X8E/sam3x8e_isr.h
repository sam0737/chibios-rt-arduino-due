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
 * @file    SAM3X8E/sam3x8e_isr.h
 * @brief   Provide definition mapping for ISR for SAM3X8E
 *
 * @addtogroup SAM3XA_ISR
 * @{
 */

#ifndef _SAM3X8E_ISR_
#define _SAM3X8E_ISR_

/*===========================================================================*/
/* Platform specific friendly IRQ names.                                     */
/*===========================================================================*/

/**
 * @name  IRQ VECTOR names
 * @{
 */
#define SAM3XA_SUPC_HANDLER      Vector40 /*  0 Supply Controller */
#define SAM3XA_RSTC_HANDLER      Vector44 /*  1 Reset Controller */
#define SAM3XA_RTC_HANDLER       Vector48 /*  2 Real Time Clock */
#define SAM3XA_RTT_HANDLER       Vector4C /*  3 Real Time Timer */
#define SAM3XA_WDT_HANDLER       Vector50 /*  4 Watchdog Timer */
#define SAM3XA_PMC_HANDLER       Vector54 /*  5 Power Management Controller */
#define SAM3XA_EFC0_HANDLER      Vector58 /*  6 Enhanced Flash Controller 0 */
#define SAM3XA_EFC1_HANDLER      Vector5C /*  7 Enhanced Flash Controller 1 */
#define SAM3XA_UART_HANDLER      Vector60 /*  8 Universal Asynchronous Receiver Transceiver */
#define SAM3XA_SMC_HANDLER       Vector64 /*  9 Static Memory Controller */
#define SAM3XA_PIOA_HANDLER      Vector68 /* 11 Parallel I/O Controller A, */
#define SAM3XA_PIOB_HANDLER      Vector6C /* 12 Parallel I/O Controller B */
#define SAM3XA_PIOC_HANDLER      Vector70 /* 13 Parallel I/O Controller C */
#define SAM3XA_PIOD_HANDLER      Vector74 /* 14 Parallel I/O Controller D */
#define SAM3XA_USART0_HANDLER    Vector78 /* 17 USART 0 */
#define SAM3XA_USART1_HANDLER    Vector7C /* 18 USART 1 */
#define SAM3XA_USART2_HANDLER    Vector80 /* 19 USART 2 */
#define SAM3XA_USART3_HANDLER    Vector84 /* 20 USART 3 */
#define SAM3XA_HSMCI_HANDLER     Vector88 /* 21 Multimedia Card Interface */
#define SAM3XA_TWI0_HANDLER      Vector8C /* 22 Two-Wire Interface 0 */
#define SAM3XA_TWI1_HANDLER      Vector90 /* 23 Two-Wire Interface 1 */
#define SAM3XA_SPI0_HANDLER      Vector94 /* 24 Serial Peripheral Interface */
#define SAM3XA_SSC_HANDLER       Vector98 /* 26 Synchronous Serial Controller */
#define SAM3XA_TC0_HANDLER       Vector9C /* 27 Timer Counter 0 */
#define SAM3XA_TC1_HANDLER       VectorA0 /* 28 Timer Counter 1 */
#define SAM3XA_TC2_HANDLER       VectorA4 /* 29 Timer Counter 2 */
#define SAM3XA_TC3_HANDLER       VectorA8 /* 30 Timer Counter 3 */
#define SAM3XA_TC4_HANDLER       VectorAC /* 31 Timer Counter 4 */
#define SAM3XA_TC5_HANDLER       VectorB0 /* 32 Timer Counter 5 */
#define SAM3XA_TC6_HANDLER       VectorB4 /* 33 Timer Counter 6 */
#define SAM3XA_TC7_HANDLER       VectorB8 /* 34 Timer Counter 7 */
#define SAM3XA_TC8_HANDLER       VectorBC /* 35 Timer Counter 8 */
#define SAM3XA_PWM_HANDLER       VectorC0 /* 36 Pulse Width Modulation Controller */
#define SAM3XA_ADC_HANDLER       VectorC4 /* 37 ADC Controller */
#define SAM3XA_DACC_HANDLER      VectorC8 /* 38 DAC Controller */
#define SAM3XA_DMAC_HANDLER      VectorCC /* 39 DMA Controller */
#define SAM3XA_UOTGHS_HANDLER    VectorD0 /* 40 USB OTG High Speed */
#define SAM3XA_TRNG_HANDLER      VectorD4 /* 41 True Random Number Generator */
#define SAM3XA_EMAC_HANDLER      VectorD8 /* 42 Ethernet MAC */
#define SAM3XA_CAN0_HANDLER      VectorDC /* 43 CAN Controller 0 */
#define SAM3XA_CAN1_HANDLER      VectorE0 /* 44 CAN Controller 1 */
/** }@ */

#endif /* _SAM3X8E_ISR_ */

/** @} */
