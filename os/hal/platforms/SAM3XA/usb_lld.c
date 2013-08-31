/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    SAM3XA/usb_lld.c
 * @brief   SAM3XA USB Driver subsystem low level driver source.
 *
 * @addtogroup USB
 * @{
 */

#include "ch.h"
#include "hal.h"
#include "chprintf.h"

#if 0
static volatile uint32_t g_x = 0;
#define USBDEBUG(...) do {\
	chprintf((BaseSequentialStream *)&SD1, "%d", g_x++); \
  chprintf((BaseSequentialStream *)&SD1, __VA_ARGS__); \
} while (0);
#else
#define USBDEBUG(...) do{ } while ( false );
#endif


#if HAL_USE_USB || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/**
 * The smallest EP that supports DMA
 */
#define DEVDMA_BASE 1

/**
 * Maximum DMA transfer size (32KiB)
 */
#define DEVDMA_ENDPOINT_MAX_TRANS 0x8000

/**
 * @brief A set of DMA descriptors for DMA queue transfer.
 */
static UotghsDevdma dma_desc[UOTGHSDEVDMA_NUMBER - DEVDMA_BASE + 1][2]
  __attribute__((aligned(0x10)));

/**
 * @brief   8-bit access to FIFO data register of selected endpoint.
 * @param   ep    Endpoint of which to access FIFO data register
 * @return  Volatile 8-bit data pointer to FIFO data register
 * @warning It is up to the user of this macro to make sure that all accesses
 *          are aligned with their natural boundaries
 * @warning It is up to the user of this macro to make sure that used HSB
 *          addresses are identical to the DPRAM internal pointer modulo 32 bits.
 */
#define get_endpoint_fifo_access8(ep) \
    (((volatile uint8_t (*)[0x8000])UOTGHS_RAM_ADDR)[(ep)])

/**
 * @brief   Is the endpoint capable of DMA
 * @param   ep    Endpoint number
 */
#define is_dma_ep(ep) \
    0
// (ep >= 1 && ep < UOTGHSDEVDMA_NUMBER)

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   USB1 driver identifier.
 */
#if SAM3XA_USB_USE_UOTGHS || defined(__DOXYGEN__)
USBDriver USBD1 = {
    .Uotghs = UOTGHS,
    .peripheral_id = ID_UOTGHS,
    .irq_id = UOTGHS_IRQn,
};
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/**
 * @brief   EP0 state.
 * @note    It is an union because IN and OUT endpoints are never used at the
 *          same time for EP0.
 */
static union {
  /**
   * @brief   IN EP0 state.
   */
  USBInEndpointState in;
  /**
   * @brief   OUT EP0 state.
   */
  USBOutEndpointState out;
} ep0_state;

/**
 * @brief   EP0 initialization structure.
 */
static const USBEndpointConfig ep0config = {
  USB_EP_MODE_TYPE_CTRL,
  _usb_ep0setup,
  _usb_ep0in,
  _usb_ep0out,
  0x40,
  0x40,
  &ep0_state.in,
  &ep0_state.out,
  .bank = 0
};

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/


/**
 * @brief   Writes to a dedicated packet buffer.
 *
 * @param[in] ep        Endpoint number
 * @param[in] buf       buffer where to fetch the packet data
 * @param[in] n         maximum number of bytes to copy. This value must
 *                      not exceed the maximum packet size for this endpoint.
 *
 * @notapi
 */
static void usb_packet_write_from_buffer(USBDriver *usbp, uint8_t ep,
                                         const uint8_t *buf,
                                         size_t n) {
  (void) usbp;
  if (is_dma_ep(ep))
  {
    UotghsDevdma *dma = &dma_desc[ep - DEVDMA_BASE][0];
    dma->UOTGHS_DEVDMANXTDSC = 0;
    dma->UOTGHS_DEVDMAADDRESS = (uint32_t)buf;
    dma->UOTGHS_DEVDMACONTROL = UOTGHS_DEVDMACONTROL_BUFF_LENGTH(n) |
        UOTGHS_DEVDMACONTROL_END_BUFFIT |
        // If all packet are full - let software to send ZLP
        // ((n % usbp->ep_size[ep]) != 0 ? UOTGHS_DEVDMACONTROL_END_B_EN : 0) |
        UOTGHS_DEVDMACONTROL_END_B_EN |
        UOTGHS_DEVDMACONTROL_CHANN_ENB;
  } else
  {
    uint8_t *p = (uint8_t*)&get_endpoint_fifo_access8(ep);
    while (n > 0) {
      //USBDEBUG("%.2x", *buf);
      *p++ = *buf++;
      n--;
    }
    //USBDEBUG("\r\n");
  }
}

/**
 * @brief   Writes to a dedicated packet buffer.
 *
 * @param[in] ep        Endpoint number
 * @param[in] buf       buffer where to fetch the packet data
 * @param[in] n         maximum number of bytes to copy. This value must
 *                      not exceed the maximum packet size for this endpoint.
 *
 * @notapi
 */
static void usb_packet_write_from_queue(USBDriver *usbp, uint8_t ep,
                                        OutputQueue *oqp, size_t n) {
  (void) usbp;
  size_t nb = n;
  if (is_dma_ep(ep)) {
    if (oqp->q_rdptr + n <= oqp->q_top)
    {
      UotghsDevdma *dma = &dma_desc[ep - DEVDMA_BASE][0];
      dma->UOTGHS_DEVDMAADDRESS = (uint32_t)oqp->q_rdptr;
      dma->UOTGHS_DEVDMACONTROL = UOTGHS_DEVDMACONTROL_BUFF_LENGTH(n) |
          UOTGHS_DEVDMACONTROL_END_BUFFIT |
          // If all packet are full - let software to send ZLP
          //((n % usbp->ep_size[ep]) != 0 ? UOTGHS_DEVDMACONTROL_END_B_EN : 0) |
          UOTGHS_DEVDMACONTROL_END_TR_EN | UOTGHS_DEVDMACONTROL_END_TR_IT |
          UOTGHS_DEVDMACONTROL_CHANN_ENB;
      if (oqp->q_rdptr >= oqp->q_top)
        oqp->q_rdptr = oqp->q_buffer;
      oqp->q_rdptr += n;
    } else {
      size_t first_n = oqp->q_top - oqp->q_rdptr;

      UotghsDevdma *dma = &dma_desc[ep - DEVDMA_BASE][0];
      UotghsDevdma *dma_next = &dma_desc[ep - DEVDMA_BASE][1];

      dma->UOTGHS_DEVDMANXTDSC = (uint32_t)dma_next;
      dma->UOTGHS_DEVDMAADDRESS = (uint32_t)oqp->q_rdptr;
      dma->UOTGHS_DEVDMACONTROL = UOTGHS_DEVDMACONTROL_BUFF_LENGTH(first_n) |
          UOTGHS_DEVDMACONTROL_LDNXT_DSC | UOTGHS_DEVDMACONTROL_CHANN_ENB;

      dma_next->UOTGHS_DEVDMAADDRESS = (uint32_t)oqp->q_rdptr;
      dma_next->UOTGHS_DEVDMACONTROL = UOTGHS_DEVDMACONTROL_BUFF_LENGTH(n - first_n) |
          UOTGHS_DEVDMACONTROL_END_BUFFIT |
          // If all packet are full - let software to send ZLP
          //(((n-first_n) % usbp->ep_size[ep]) != 0 ? UOTGHS_DEVDMACONTROL_END_B_EN : 0) |
          UOTGHS_DEVDMACONTROL_END_TR_EN | UOTGHS_DEVDMACONTROL_END_TR_IT |
          UOTGHS_DEVDMACONTROL_CHANN_ENB;

      oqp->q_rdptr = oqp->q_buffer + n - first_n;
    }
  } else {
    uint8_t *p = (uint8_t*)&get_endpoint_fifo_access8(ep);
    while (n > 0) {
      *p++ = *oqp->q_rdptr++;
      if (oqp->q_rdptr >= oqp->q_top)
        oqp->q_rdptr = oqp->q_buffer;
      n--;
    }
  }

  /* Updating queue. Note, the lock is done in this unusual way because this
     function can be called from both ISR and thread context so the kind
     of lock function to be invoked cannot be decided beforehand.*/
  chSysLock();

  oqp->q_counter += nb;
  while (notempty(&oqp->q_waiting))
    chSchReadyI(fifo_remove(&oqp->q_waiting))->p_u.rdymsg = Q_OK;

  chSysUnlock();
}

/**
 * @brief   Reads from a dedicated packet buffer.
 *
 * @param[in] p         pointer the USB fifo buffer
 * @param[out] buf      buffer where to copy the packet data
 * @param[in] n         maximum number of bytes to copy. This value must
 *                      not exceed the maximum packet size for this endpoint.
 *
 * @notapi
 */
static void usb_packet_read_to_buffer(USBDriver *usbp, uint8_t ep,
                                      uint8_t *buf, size_t n) {
  (void) usbp;
  uint8_t *p = (uint8_t*)&get_endpoint_fifo_access8(ep);
  while (n > 0) {
    *buf++ = *p++;
    //USBDEBUG("%.2x", c);
    n--;
  }
  //USBDEBUG("\r\n");
}

/**
 * @brief   Reads from a dedicated packet buffer.
 *
 * @param[in] p         pointer the USB fifo buffer
 * @param[in] iqp       pointer to an @p InputQueue object
 * @param[in] n         maximum number of bytes to copy. This value must
 *                      not exceed the maximum packet size for this endpoint.
 *
 * @notapi
 */
static void usb_packet_read_to_queue(USBDriver *usbp, uint8_t ep,
                                     InputQueue *iqp, size_t n) {
  (void) usbp;
  uint8_t *p = (uint8_t*)&get_endpoint_fifo_access8(ep);
  size_t nb = n;
  while (n > 0) {
    *iqp->q_wrptr++ = *p++;
    //USBDEBUG("%.2x", c);
    if (iqp->q_wrptr >= iqp->q_top)
      iqp->q_wrptr = iqp->q_buffer;
    n--;
  }
  //USBDEBUG("\r\n");

  /* Updating queue.*/
  chSysLockFromIsr();
  iqp->q_counter += nb;
  while (notempty(&iqp->q_waiting))
    chSchReadyI(fifo_remove(&iqp->q_waiting))->p_u.rdymsg = Q_OK;
  chSysUnlockFromIsr();
}

/**
 * @brief Initialize the UPLL and USB clock
 */
static void usb_init_clock(void){
  /* Enable UPLL (480Mhz) */
  pmc_enable_upll_clock();

  /* Prepare USB Clock for slow speed mode */
  pmc_switch_udpck_to_upllck(0); // div = 0+1
  pmc_enable_udpck();
}

/**
 * @brief   USB initialization.
 * @details This function must be invoked with interrupts disabled.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 */
static void usb_start(USBDriver *usbp, uint32_t irq_priority) {
  Uotghs* u = usbp->Uotghs;

  pmc_enable_peripheral_clock(usbp->peripheral_id);

  // Disable IP pin and force device mode
  u->UOTGHS_CTRL |= UOTGHS_CTRL_UIMOD;
  u->UOTGHS_CTRL &= ~UOTGHS_CTRL_UIDE;

  // Enable USB pad
  u->UOTGHS_CTRL &= ~UOTGHS_CTRL_OTGPADE;
  u->UOTGHS_CTRL |= UOTGHS_CTRL_OTGPADE;

  // Enable USB device
  u->UOTGHS_CTRL |= UOTGHS_CTRL_USBE;

  // Disable low speed (1.5Mbps) mode, and enable full/high speed auto detection
  u->UOTGHS_DEVCTRL &= ~UOTGHS_DEVCTRL_LS;
  u->UOTGHS_DEVCTRL &= ~UOTGHS_DEVCTRL_SPDCONF_Msk;
  u->UOTGHS_DEVCTRL |= UOTGHS_DEVCTRL_SPDCONF_Msk & UOTGHS_DEVCTRL_SPDCONF_NORMAL;

  // Unfreeze clock and wait
  u->UOTGHS_CTRL &= ~UOTGHS_CTRL_FRZCLK;
  while (!(u->UOTGHS_SR & UOTGHS_SR_CLKUSABLE));

  // Global Interrupt
  u->UOTGHS_DEVIDR = 0xFFFF;
  u->UOTGHS_DEVIER =
      UOTGHS_DEVIER_EORSTES | // Enable End Of Reset interrupt
      // Enable Start Of Frame interrupt if callback is setup
      (usbp->config->sof_cb != NULL ? UOTGHS_DEVIER_SOFES : 0) |
      UOTGHS_DEVIER_WAKEUPES |
      UOTGHS_DEVIER_SUSPES;

  nvicEnableVector(usbp->irq_id, CORTEX_PRIORITY_MASK(irq_priority));
}

/**
 * @brief   USB de-initialization.
 * @details This function must be invoked with interrupts disabled.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 */
static void usb_stop(USBDriver *usbp) {
  Uotghs* u = usbp->Uotghs;

  nvicDisableVector(usbp->irq_id);

  // Freeze clock
  u->UOTGHS_CTRL |= UOTGHS_CTRL_FRZCLK;

  // Disable USBE
  u->UOTGHS_CTRL &= ~UOTGHS_CTRL_USBE;
}

/**
 * @brief   USB Endpoint interrupt service routine
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        Endpoint number
 */
static void serve_usb_ep_irq(USBDriver *usbp, usbep_t ep) {
  Uotghs* u = usbp->Uotghs;
  const USBEndpointConfig *epcp = usbp->epc[ep];
  uint8_t isr = u->UOTGHS_DEVEPTISR[ep] & u->UOTGHS_DEVEPTIMR[ep];
  uint8_t i;
  // Setup event
  if (ep == 0 && (isr & UOTGHS_DEVEPTISR_RXSTPI))
  {
    uint8_t *ptr_src = (uint8_t *) &get_endpoint_fifo_access8(ep);
    for (i = 0; i < 8; ++i)
      usbp->setup[i] = *ptr_src++;
    u->UOTGHS_DEVEPTICR[ep] = UOTGHS_DEVEPTICR_RXSTPIC;

    uint8_t *buf = usbp->setup;
    (void) buf;
    USBDEBUG("S %.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x\r\n",
          buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);

    _usb_isr_invoke_setup_cb(usbp, ep);
  }

  // IN
  if (isr & UOTGHS_DEVEPTISR_TXINI)
  {
    USBInEndpointState *isp = usbp->epc[ep]->in_state;

    //USBDEBUG("TX %d>%d %d\r\n", ep, isp->txcnt, isp->txsize);
    if (isp->txcnt < isp->txsize)
    {
      usb_lld_prepare_transmit(usbp, ep);
      chSysLockFromIsr();
      usb_lld_start_in(usbp, ep);
      chSysUnlockFromIsr();
    } else {
      /* Transfer completed */
      u->UOTGHS_DEVEPTIDR[ep] = UOTGHS_DEVEPTIDR_TXINEC;
      _usb_isr_invoke_in_cb(usbp, ep);
    }
  }

  // OUT
  if (isr & UOTGHS_DEVEPTISR_RXOUTI)
  {
    USBOutEndpointState *osp = usbp->epc[ep]->out_state;

    uint8_t shortpacket = u->UOTGHS_DEVEPTISR[ep] & UOTGHS_DEVEPTISR_SHORTPACKET;
    u->UOTGHS_DEVEPTICR[ep] = UOTGHS_DEVEPTICR_RXOUTIC | UOTGHS_DEVEPTICR_SHORTPACKETC;

    uint16_t n = (u->UOTGHS_DEVEPTISR[ep] & UOTGHS_DEVEPTISR_BYCT_Msk)
        >> UOTGHS_DEVEPTISR_BYCT_Pos;
    osp->rxcnt += n;

    //USBDEBUG("RX %d>%d %d\r\n", ep, osp->rxcnt, osp->rxsize);
    if (epcp->out_state->rxqueued) {
      usb_packet_read_to_queue(usbp, ep,
                               epcp->out_state->mode.queue.rxqueue,
                               n);
    } else {
      usb_packet_read_to_buffer(usbp, ep,
                                epcp->out_state->mode.linear.rxbuf,
                                n);
      epcp->out_state->mode.linear.rxbuf  += n;
    }

    if (ep != 0)
      u->UOTGHS_DEVEPTIDR[ep] = UOTGHS_DEVEPTIDR_FIFOCONC;

    // Receiving ends when short packet is detected, or count reached
    if (!shortpacket && osp->rxcnt < osp->rxsize)
    {
      chSysLockFromIsr();
      usb_lld_start_out(usbp, ep);
      chSysUnlockFromIsr();
    } else {
      /* Transfer completed */
      u->UOTGHS_DEVEPTIDR[ep] = UOTGHS_DEVEPTIDR_RXOUTEC;
      _usb_isr_invoke_out_cb(usbp, ep);
    }
  }
}

/**
 * @brief   USB DMA interrupt service routine
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        Endpoint number
 */
static void serve_usb_dma_irq(USBDriver *usbp, usbep_t ep) {
  Uotghs* u = usbp->Uotghs;
  UotghsDevdma *dma = &u->UOTGHS_DEVDMA[ep - DEVDMA_BASE];
  // Clear flags
  uint32_t status = dma->UOTGHS_DEVDMASTATUS;
  (void) status;

  u->UOTGHS_DEVIDR = UOTGHS_DEVIDR_DMA_1 << (ep - 1);

  USBDEBUG("**DMA %d %.8x s%.8x m%.8x\r\n", ep, status, u->UOTGHS_DEVEPTISR[ep], u->UOTGHS_DEVEPTIMR[ep]);

  return;
  if (u->UOTGHS_DEVEPTISR[ep] & UOTGHS_DEVEPTISR_TXINI)
  {
    USBInEndpointState *isp = usbp->epc[ep]->in_state;
    if (isp->txcnt < isp->txsize)
    {
      usb_lld_prepare_transmit(usbp, ep);
      chSysLockFromIsr();
      usb_lld_start_in(usbp, ep);
      chSysUnlockFromIsr();
    } else {
      /* Transfer completed */
      _usb_isr_invoke_in_cb(usbp, ep);
    }
  } else if (u->UOTGHS_DEVEPTISR[ep] & UOTGHS_DEVEPTISR_RXOUTI)
  {
    const USBEndpointConfig *epcp = usbp->epc[ep];
    USBOutEndpointState *osp = usbp->epc[ep]->out_state;

    uint16_t n = (dma->UOTGHS_DEVDMACONTROL & UOTGHS_DEVDMACONTROL_BUFF_LENGTH_Msk)
        >> UOTGHS_DEVDMACONTROL_BUFF_LENGTH_Pos;
    osp->rxcnt -= n;

    USBDEBUG("DRX %d>%d %d\r\n", ep, osp->rxcnt, osp->rxsize);
    if (epcp->out_state->rxqueued) {
      chSysLockFromIsr();

      InputQueue *iqp = epcp->out_state->mode.queue.rxqueue;
      iqp->q_wrptr += osp->rxcnt;
      if (iqp->q_wrptr >= iqp->q_top)
        iqp->q_wrptr -= iqp->q_top - iqp->q_buffer;
      iqp->q_counter += osp->rxcnt;
      while (notempty(&iqp->q_waiting))
        chSchReadyI(fifo_remove(&iqp->q_waiting))->p_u.rdymsg = Q_OK;

      chSysUnlockFromIsr();
    } else {
      epcp->out_state->mode.linear.rxbuf += osp->rxcnt;
    }

    _usb_isr_invoke_out_cb(usbp, ep);
  }
}

/**
 * @brief   USB common service routine.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 */
static void serve_usb_irq(USBDriver *usbp) {
  Uotghs* u = usbp->Uotghs;
  uint32_t isr = u->UOTGHS_DEVISR & u->UOTGHS_DEVIMR;
  uint32_t c = 0;

  //toggle_tx();
  //USBDEBUG("*I %.8x\r\n", isr);
  // Reset
  if (isr & UOTGHS_DEVISR_EORST)
  {
    _usb_reset(usbp);
    _usb_isr_invoke_event_cb(usbp, USB_EVENT_RESET);
    u->UOTGHS_DEVICR = UOTGHS_DEVICR_EORSTC;
  }

  // SOF (Start Of Frame)
  if (isr & UOTGHS_DEVISR_SOF)
  {
    //_usb_isr_invoke_sof_cb(usbp);
    u->UOTGHS_DEVICR = UOTGHS_DEVICR_SOFC;
  }

  // Suspend
  if (isr & UOTGHS_DEVISR_SUSP)
  {
    _usb_isr_invoke_event_cb(usbp, USB_EVENT_SUSPEND);
    // Disable suspend interrupt
    u->UOTGHS_DEVICR = UOTGHS_DEVICR_SUSPC;
    u->UOTGHS_DEVIER = UOTGHS_DEVIER_WAKEUPES;
    u->UOTGHS_DEVIDR = UOTGHS_DEVIDR_SUSPEC;
  }

  // Wakeup
  if (isr & UOTGHS_DEVISR_WAKEUP)
  {
    _usb_isr_invoke_event_cb(usbp, USB_EVENT_WAKEUP);
    // Re-enable suspend interrupt, clear Wakeup flag
    u->UOTGHS_DEVICR = UOTGHS_DEVICR_WAKEUPC;
    u->UOTGHS_DEVIER = UOTGHS_DEVIER_SUSPES;
    u->UOTGHS_DEVIDR = UOTGHS_DEVIDR_WAKEUPEC;
  }

  for (c = 0; c <= USB_MAX_ENDPOINTS; c++)
  {
    if (isr & (UOTGHS_DEVISR_PEP_0 << c)) {
      serve_usb_ep_irq(usbp, c);
    }
    if (is_dma_ep(c)) {
      if (isr & (UOTGHS_DEVISR_DMA_1 << (c-1))) {
        serve_usb_dma_irq(usbp, c);
      }
    }
  }
}

/*===========================================================================*/
/* Driver interrupt handlers and threads.                                    */
/*===========================================================================*/

#if SAM3XA_USB_USE_UOTGHS || defined(__DOXYGEN__)
#if !defined(SAM3XA_UOTGHS_HANDLER)
#error "SAM3XA_UOTGHS_HANDLER not defined"
#endif
/**
 * @brief   UOTGHS IRQ handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(SAM3XA_UOTGHS_HANDLER) {
  CH_IRQ_PROLOGUE();
  serve_usb_irq(&USBD1);
  CH_IRQ_EPILOGUE();
}
#endif /* SAM3XA_UART_USE_UART */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level USB driver initialization.
 *
 * @notapi
 */
void usb_lld_init(void) {
  usb_init_clock();

#if SAM3XA_USB_USE_UOTGHS
  /* Driver initialization.*/
  usbObjectInit(&USBD1);
#endif /* SAM3XA_USB_USE_UOTGHS */
}

/**
 * @brief   Configures and activates the USB peripheral.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_start(USBDriver *usbp) {
  if (usbp->state == USB_STOP) {
    uint32_t irq_priority =
        usbp->config->irq_priority ? usbp->config->irq_priority :
        SAM3XA_USB_DEFAULT_IRQ_PRIORITY;
    chDbgCheck(CORTEX_IS_VALID_KERNEL_PRIORITY(usbp->config->irq_priority),
        "USB irq_priority");

    usb_start(usbp, irq_priority);
  }
}

/**
 * @brief   Deactivates the USB peripheral.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_stop(USBDriver *usbp) {

  if (usbp->state == USB_READY) {
    /* Resets the peripheral.*/

    /* Disables the peripheral.*/
    usb_stop(usbp);
  }
}

/**
 * @brief   USB low level reset routine.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_reset(USBDriver *usbp) {
  Uotghs* u = usbp->Uotghs;
  /* Post reset initialization.*/

  // Enable End Of Reset interrupt
  // Enable Start Of Frame interrupt if callback is setup
  u->UOTGHS_DEVIDR = 0xFFFF;
  u->UOTGHS_DEVIER =
      UOTGHS_DEVIER_EORSTES | // Enable End Of Reset interrupt
      // Enable Start Of Frame interrupt if callback is setup
      (usbp->config->sof_cb != NULL ? UOTGHS_DEVIER_SOFES : 0) |
      UOTGHS_DEVIER_WAKEUPES |
      UOTGHS_DEVIER_SUSPES;

  /* EP0 initialization.*/
  usbp->epc[0] = &ep0config;
  usb_lld_init_endpoint(usbp, 0);
}

/**
 * @brief   Sets the USB address.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_set_address(USBDriver *usbp) {
  (void)usbp;
  usbp->Uotghs->UOTGHS_DEVCTRL |= UOTGHS_DEVCTRL_UADD(usbp->address);
  usbp->Uotghs->UOTGHS_DEVCTRL |= UOTGHS_DEVCTRL_ADDEN;
}

/**
 * @brief   Enables an endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_init_endpoint(USBDriver *usbp, usbep_t ep) {
  uint32_t cfg;
  size_t ep_size;

  Uotghs* u = usbp->Uotghs;
  const USBEndpointConfig *epcp = usbp->epc[ep];
  uint8_t ep_type = epcp->ep_mode & USB_EP_MODE_TYPE;

  cfg = UOTGHS_DEVEPTCFG_AUTOSW;

  /* Setting the endpoint type.*/
  switch (ep_type) {
  case USB_EP_MODE_TYPE_ISOC:
    cfg |= UOTGHS_DEVEPTCFG_EPTYPE_ISO;
    break;
  case USB_EP_MODE_TYPE_BULK:
    cfg |= UOTGHS_DEVEPTCFG_EPTYPE_BLK;
    break;
  case USB_EP_MODE_TYPE_INTR:
    cfg |= UOTGHS_DEVEPTCFG_EPTYPE_INTRPT;
    break;
  default:
    cfg |= UOTGHS_DEVEPTCFG_EPTYPE_CTRL;
  }

  /* Bank size */
  if (ep_type != USB_EP_MODE_TYPE_CTRL) {
    if (epcp->in_maxsize > 0) {
      // TXIn Endpoint
      cfg |= UOTGHS_DEVEPTCFG_EPDIR_IN;
    }
    if (epcp->bank) {
      cfg |= ((epcp->bank - 1) << UOTGHS_DEVEPTCFG_EPBK_Pos) & UOTGHS_DEVEPTCFG_EPBK_Msk;
      // FIXME: Iso mode and NB trans is not tested yet
      // cfg |= ((epcp->bank) << UOTGHS_DEVEPTCFG_NBTRANS_Pos) & UOTGHS_DEVEPTCFG_NBTRANS_Msk;
      cfg |= UOTGHS_DEVEPTCFG_NBTRANS_1_TRANS;
    }

    /* Endpoint size */
    ep_size = epcp->in_maxsize + epcp->out_maxsize;
    if (ep_size >= 1024) { cfg |= UOTGHS_HSTPIPCFG_PSIZE_1024_BYTE; ep_size = 1024; }
    else if (ep_size >= 512) { cfg |= UOTGHS_HSTPIPCFG_PSIZE_512_BYTE; ep_size = 512; }
    else if (ep_size >= 256) { cfg |= UOTGHS_HSTPIPCFG_PSIZE_256_BYTE; ep_size = 256; }
    else if (ep_size >= 128) { cfg |= UOTGHS_HSTPIPCFG_PSIZE_128_BYTE; ep_size = 128; }
    else if (ep_size >= 64) { cfg |= UOTGHS_HSTPIPCFG_PSIZE_64_BYTE; ep_size = 64; }
    else if (ep_size >= 32) { cfg |= UOTGHS_HSTPIPCFG_PSIZE_32_BYTE; ep_size = 32; }
    else if (ep_size >= 16) { cfg |= UOTGHS_HSTPIPCFG_PSIZE_16_BYTE; ep_size = 16; }
    else { cfg |= UOTGHS_HSTPIPCFG_PSIZE_8_BYTE; ep_size = 8; }
    usbp->ep_size[ep] = ep_size;
  } else {
    usbp->ep_size[ep] = 64;
    cfg |= UOTGHS_HSTPIPCFG_PSIZE_64_BYTE;
  }

  /* Alloc and configure */
  cfg |= UOTGHS_DEVEPTCFG_ALLOC;

  u->UOTGHS_DEVEPTCFG[ep] = cfg;
  chDbgAssert(u->UOTGHS_DEVEPTISR[ep] & UOTGHS_DEVEPTISR_CFGOK,
      "UOTGHS ep config ok", "UOTGHS config check");

  USBDEBUG("--INIT EP%d %.8x\r\n", ep, cfg);

  /* Interrupt */
  if ((epcp->ep_mode & USB_EP_MODE_TYPE) == USB_EP_MODE_TYPE_CTRL)
    u->UOTGHS_DEVEPTIER[ep] = UOTGHS_DEVEPTIER_RXSTPES;

  u->UOTGHS_DEVIER = UOTGHS_DEVIER_PEP_0 << ep;
  if (ep < UOTGHSDEVDMA_NUMBER && ep > 0) {
    u->UOTGHS_DEVIER = UOTGHS_DEVIER_DMA_1 << (ep - 1);
  }

  /* Enable endpoint */
  u->UOTGHS_DEVEPT |= UOTGHS_DEVEPT_EPEN0 << ep;
}

/**
 * @brief   Disables all the active endpoints except the endpoint zero.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_disable_endpoints(USBDriver *usbp) {
  (void)usbp;
  uint8_t c;
  Uotghs* u = usbp->Uotghs;

  /* Disable all endpoints except EP0 */
  u->UOTGHS_DEVEPT &= ~(UOTGHS_DEVEPT_EPEN0);

  /* Deallocate FIFO */
  for (c = 0; c <= USB_MAX_ENDPOINTS; c++)
  {
    u->UOTGHS_DEVEPTCFG[c] &= ~UOTGHS_DEVEPTCFG_ALLOC;
  }
}

/**
 * @brief   Returns the status of an OUT endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 * @return              The endpoint status.
 * @retval EP_STATUS_DISABLED The endpoint is not active.
 * @retval EP_STATUS_STALLED  The endpoint is stalled.
 * @retval EP_STATUS_ACTIVE   The endpoint is active.
 *
 * @notapi
 */
usbepstatus_t usb_lld_get_status_out(USBDriver *usbp, usbep_t ep) {
  Uotghs* u = usbp->Uotghs;

  if (!(u->UOTGHS_DEVEPTCFG[ep] & UOTGHS_DEVEPTISR_CFGOK))
    return EP_STATUS_DISABLED;
  if (u->UOTGHS_DEVEPTCFG[ep] & UOTGHS_DEVEPTISR_STALLEDI)
    return EP_STATUS_STALLED;
  return EP_STATUS_ACTIVE;
}

/**
 * @brief   Returns the status of an IN endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 * @return              The endpoint status.
 * @retval EP_STATUS_DISABLED The endpoint is not active.
 * @retval EP_STATUS_STALLED  The endpoint is stalled.
 * @retval EP_STATUS_ACTIVE   The endpoint is active.
 *
 * @notapi
 */
usbepstatus_t usb_lld_get_status_in(USBDriver *usbp, usbep_t ep) {
  Uotghs* u = usbp->Uotghs;

  if (!(u->UOTGHS_DEVEPTCFG[ep] & UOTGHS_DEVEPTISR_CFGOK))
    return EP_STATUS_DISABLED;
  if (u->UOTGHS_DEVEPTCFG[ep] & UOTGHS_DEVEPTISR_STALLEDI)
    return EP_STATUS_STALLED;
  return EP_STATUS_ACTIVE;
}

/**
 * @brief   Reads a setup packet from the dedicated packet buffer.
 * @details This function must be invoked in the context of the @p setup_cb
 *          callback in order to read the received setup packet.
 * @pre     In order to use this function the endpoint must have been
 *          initialized as a control endpoint.
 * @post    The endpoint is ready to accept another packet.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 * @param[out] buf      buffer where to copy the packet data
 *
 * @notapi
 */
void usb_lld_read_setup(USBDriver *usbp, usbep_t ep, uint8_t *buf) {
  (void)usbp;
  (void)ep;
  (void)buf;

  // Already read in ISR
}

/**
 * @brief   Prepares for a receive operation.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_prepare_receive(USBDriver *usbp, usbep_t ep) {
  (void)usbp;
  (void)ep;

  // Use DMA? rxsize > 0. Supported channel.
  //USBOutEndpointState *osp = usbp->epc[ep]->out_state;

  //USBDEBUG("PR %d>%d\r\n", ep, osp->rxsize);
}

/**
 * @brief   Prepares for a transmit operation.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_prepare_transmit(USBDriver *usbp, usbep_t ep) {
  size_t n;
  size_t n_max = is_dma_ep(ep) ? DEVDMA_ENDPOINT_MAX_TRANS : usbp->ep_size[ep];

  Uotghs* u = usbp->Uotghs;
  while (!(u->UOTGHS_DEVEPTISR[ep] & UOTGHS_DEVEPTISR_TXINI));

  USBInEndpointState *isp = usbp->epc[ep]->in_state;
  n = isp->txsize - isp->txcnt;
  if (n > n_max)
    n = n_max;
  isp->txcnt += n;

  //USBDEBUG("PT %d>%d\r\n", ep ,n);

  if (isp->txqueued)
    usb_packet_write_from_queue(usbp, ep,
                                isp->mode.queue.txqueue, n);
  else {
    usb_packet_write_from_buffer(usbp, ep,
                                 isp->mode.linear.txbuf, n);
    isp->mode.linear.txbuf += n;
  }
}

/**
 * @brief   Starts a receive operation on an OUT endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_start_out(USBDriver *usbp, usbep_t ep) {
  Uotghs* u = usbp->Uotghs;
  u->UOTGHS_DEVEPTIER[ep] = UOTGHS_DEVEPTIER_RXOUTES;
  //USBDEBUG(" R %d\r\n", ep);
}

/**
 * @brief   Starts a transmit operation on an IN endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_start_in(USBDriver *usbp, usbep_t ep) {
  Uotghs* u = usbp->Uotghs;
  //USBDEBUG(" T %d\r\n", ep);
  if (is_dma_ep(ep)) {
    //toggle_rx();
    //USBDEBUG("MEOW");
    UotghsDevdma *dma = &UOTGHS->UOTGHS_DEVDMA[ep - DEVDMA_BASE];
    dma->UOTGHS_DEVDMANXTDSC = (uint32_t)&dma_desc[ep - DEVDMA_BASE][0];
    dma->UOTGHS_DEVDMACONTROL = UOTGHS_DEVDMACONTROL_LDNXT_DSC;
    //u->UOTGHS_DEVIER = UOTGHS_DEVIER_DMA_1 << (ep - 1);
    //USBDEBUG("M: %.8x\r\n", dma->UOTGHS_DEVDMASTATUS);
    //USBDEBUG("M: %.8x\r\n", dma->UOTGHS_DEVDMASTATUS);
  } else {
    u->UOTGHS_DEVEPTIER[ep] = UOTGHS_DEVEPTIER_TXINES;
    u->UOTGHS_DEVEPTICR[ep] = UOTGHS_DEVEPTICR_TXINIC;
    if (ep != 0)
      u->UOTGHS_DEVEPTIDR[ep] = UOTGHS_DEVEPTIDR_FIFOCONC;
  }
}

/**
 * @brief   Brings an OUT endpoint in the stalled state.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_stall_out(USBDriver *usbp, usbep_t ep) {
  Uotghs* u = usbp->Uotghs;
  u->UOTGHS_DEVEPTIER[ep] = UOTGHS_DEVEPTIER_STALLRQS | UOTGHS_DEVEPTIER_RSTDTS;
  USBDEBUG("STALL-O %d\r\n", ep);
}

/**
 * @brief   Brings an IN endpoint in the stalled state.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_stall_in(USBDriver *usbp, usbep_t ep) {
  Uotghs* u = usbp->Uotghs;
  u->UOTGHS_DEVEPTIER[ep] = UOTGHS_DEVEPTIER_STALLRQS | UOTGHS_DEVEPTIER_RSTDTS;
  USBDEBUG("STALL-I %d\r\n", ep);
}

/**
 * @brief   Brings an OUT endpoint in the active state.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_clear_out(USBDriver *usbp, usbep_t ep) {
  Uotghs* u = usbp->Uotghs;
  // Clear stall condition and reset data toggle
  u->UOTGHS_DEVEPTIDR[ep] = UOTGHS_DEVEPTIDR_STALLRQC;
}

/**
 * @brief   Brings an IN endpoint in the active state.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_clear_in(USBDriver *usbp, usbep_t ep) {
  Uotghs* u = usbp->Uotghs;
  // Clear stall condition and reset data toggle
  u->UOTGHS_DEVEPTIDR[ep] = UOTGHS_DEVEPTIDR_STALLRQC;
}

#endif /* HAL_USE_USB */

/** @} */
