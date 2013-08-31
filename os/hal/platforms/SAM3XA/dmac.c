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
 * @file    SAM3XA/dmac.c
 * @brief   Provides access to the direct memory access controller (DMAC)
 *
 * @addtogroup SAM3XA_HAL_DMAC
 * @{
 */

#include "ch.h"
#include "hal.h"

#include "dmac.h"

#if SAM3XA_USB_USE_DMAC || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

typedef struct DmaChannelConfig DmaChannelConfig;

struct DmaChannelConfig {
  dmacallback_t callback;
  const void* state;
  uint32_t dummy;
};

static DmaChannelConfig channel_config[DMACCH_NUM_NUMBER];

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

CH_IRQ_HANDLER(SAM3XA_DMAC_HANDLER) {
  CH_IRQ_PROLOGUE();
  uint8_t i;
  uint8_t mask = DMAC_EBCISR_BTC0;
  uint8_t mask_disable = DMAC_CHDR_DIS0;

  uint32_t s = DMAC->DMAC_EBCISR;
  for (i = 0; i < DMACCH_NUM_NUMBER; i++, mask <<= 1, mask_disable <<= 1)
  {
    if (s & mask) {
      DMAC->DMAC_CHDR = mask_disable;
      if (channel_config[i].callback) {
        channel_config[i].callback(i, channel_config[i].state);
      }
    }
  }
  CH_IRQ_EPILOGUE();
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

void dmac_lld_init(void)
{
  pmc_enable_peripheral_clock(ID_DMAC);
  nvicEnableVector(DMAC_IRQn, CORTEX_PRIORITY_MASK(SAM3XA_DMAC_DEFAULT_IRQ_PRIORITY));

  DMAC->DMAC_EN = DMAC_EN_ENABLE;
}

void dmac_prepare_send(uint8_t ch, uint8_t per, dmacallback_t callback, const void* state,
    uint16_t size, const void* buf, const void* destination)
{
  DmaChannelConfig* config = &channel_config[ch];
  config->callback = callback;
  config->state = state;

  DmacCh_num* c = &DMAC->DMAC_CH_NUM[ch];
  c->DMAC_SADDR = (uint32_t)buf;
  c->DMAC_DADDR = (uint32_t)destination;
  c->DMAC_DSCR = 0;
  c->DMAC_CTRLA = DMAC_CTRLA_DST_WIDTH_BYTE | DMAC_CTRLA_SRC_WIDTH_BYTE |
      DMAC_CTRLA_BTSIZE(size);
  c->DMAC_CTRLB = DMAC_CTRLB_SRC_INCR_INCREMENTING | DMAC_CTRLB_DST_INCR_FIXED |
      DMAC_CTRLB_FC_MEM2PER_DMA_FC |
      DMAC_CTRLB_SRC_DSCR_FETCH_DISABLE | DMAC_CTRLB_DST_DSCR_FETCH_DISABLE;
  c->DMAC_CFG = DMAC_CFG_DST_PER(per) | DMAC_CFG_DST_H2SEL_HW;

  DMAC->DMAC_EBCIER = DMAC_EBCIER_BTC0 << ch;
}

void dmac_prepare_send_dummy(uint8_t ch, uint8_t per, dmacallback_t callback, const void* state,
    uint16_t size, uint32_t dummy, const void* destination)
{
  DmaChannelConfig* config = &channel_config[ch];
  config->callback = callback;
  config->state = state;
  config->dummy = dummy;

  DmacCh_num* c = &DMAC->DMAC_CH_NUM[ch];
  c->DMAC_SADDR = (uint32_t)&config->dummy;
  c->DMAC_DADDR = (uint32_t)destination;
  c->DMAC_DSCR = 0;
  c->DMAC_CTRLA = DMAC_CTRLA_DST_WIDTH_BYTE | DMAC_CTRLA_SRC_WIDTH_BYTE |
      DMAC_CTRLA_BTSIZE(size);
  c->DMAC_CTRLB = DMAC_CTRLB_SRC_INCR_FIXED | DMAC_CTRLB_DST_INCR_FIXED |
      DMAC_CTRLB_FC_MEM2PER_DMA_FC |
      DMAC_CTRLB_SRC_DSCR_FETCH_DISABLE | DMAC_CTRLB_DST_DSCR_FETCH_DISABLE;
  c->DMAC_CFG = DMAC_CFG_DST_PER(per) | DMAC_CFG_DST_H2SEL_HW;

  DMAC->DMAC_EBCIER = DMAC_EBCIER_BTC0 << ch;
}

void dmac_prepare_receive(uint8_t ch, uint8_t per, dmacallback_t callback, const void* state,
    uint16_t size, const void* buf, const void* source)
{
  DmaChannelConfig* config = &channel_config[ch];
  config->callback = callback;
  config->state = state;

  DmacCh_num* c = &DMAC->DMAC_CH_NUM[ch];
  c->DMAC_SADDR = (uint32_t)source;
  c->DMAC_DADDR = (uint32_t)buf;
  c->DMAC_DSCR = 0;
  c->DMAC_CTRLA = DMAC_CTRLA_DST_WIDTH_BYTE | DMAC_CTRLA_SRC_WIDTH_BYTE |
      DMAC_CTRLA_BTSIZE(size);
  c->DMAC_CTRLB = DMAC_CTRLB_SRC_INCR_FIXED | DMAC_CTRLB_DST_INCR_INCREMENTING |
      DMAC_CTRLB_FC_PER2MEM_DMA_FC |
      DMAC_CTRLB_SRC_DSCR_FETCH_DISABLE | DMAC_CTRLB_DST_DSCR_FETCH_DISABLE;
  c->DMAC_CFG = DMAC_CFG_SRC_PER(per) | DMAC_CFG_SRC_H2SEL_HW;

  DMAC->DMAC_EBCIER = DMAC_EBCIER_BTC0 << ch;
}

void dmac_prepare_receive_dummy(uint8_t ch, uint8_t per, dmacallback_t callback, const void* state,
    uint16_t size, const void* source)
{
  DmaChannelConfig* config = &channel_config[ch];
  config->callback = callback;
  config->state = state;

  DmacCh_num* c = &DMAC->DMAC_CH_NUM[ch];
  c->DMAC_SADDR = (uint32_t)source;
  c->DMAC_DADDR = (uint32_t)&config->dummy;
  c->DMAC_DSCR = 0;
  c->DMAC_CTRLA = DMAC_CTRLA_DST_WIDTH_BYTE | DMAC_CTRLA_SRC_WIDTH_BYTE |
      DMAC_CTRLA_BTSIZE(size);
  c->DMAC_CTRLB = DMAC_CTRLB_SRC_INCR_FIXED | DMAC_CTRLB_DST_INCR_FIXED |
      DMAC_CTRLB_FC_PER2MEM_DMA_FC |
      DMAC_CTRLB_SRC_DSCR_FETCH_DISABLE | DMAC_CTRLB_DST_DSCR_FETCH_DISABLE;
  c->DMAC_CFG = DMAC_CFG_SRC_PER(per) | DMAC_CFG_SRC_H2SEL_HW;

  DMAC->DMAC_EBCIER = DMAC_EBCIER_BTC0 << ch;
}


void dmac_channel_start(uint8_t ch)
{
  DMAC->DMAC_CHER = DMAC_CHER_ENA0 << ch;
}

#endif /* SAM3XA_USB_USE_DMAC */

/** @} */
