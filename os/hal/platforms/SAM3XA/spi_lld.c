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
 * @file    SAM3XA/spi_lld.c
 * @brief   SPI Driver subsystem low level driver source.
 *
 * @addtogroup SPI
 * @{
 */

#include "ch.h"
#include "hal.h"

#if HAL_USE_SPI || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#if SAM3XA_USB_USE_DMAC
#define IS_USE_DMA(spip) (spip->config->use_dma)
#else
#define IS_USE_DMA(spip) (0)
#endif

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   SPI1 driver identifier.
 */
#if SAM3XA_SPI_USE_SPI1 || defined(__DOXYGEN__)
SPIDriver SPID1 = {
    .peripheral_id = ID_SPI0,
    .irq_id = SPI0_IRQn,
    .spi = SPI0,
#if SAM3XA_USB_USE_DMAC
    .dma_tx_per = 1,
    .dma_rx_per = 2
#endif
};
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

void serve_spi_irq(SPIDriver *spip) {
  uint8_t dummy __attribute__((__unused__));
  if (spip->rx_buf) {
    *spip->rx_buf++ = spip->spi->SPI_RDR;
  } else {
    dummy = spip->spi->SPI_RDR;
  }
  if (--spip->size == 0) {
    spip->spi->SPI_IDR = 0xFFFFFFFF;
    _spi_isr_code(spip);
  } else {
    if (spip->tx_buf)
      spip->spi->SPI_TDR = *(++spip->tx_buf);
    else
      spip->spi->SPI_TDR = 0xFFFF;
  }
}

void serve_spi_dma(uint8_t ch, const void *state)
{
  (void)ch;
  _spi_isr_code((SPIDriver*)state);
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if SAM3XA_SPI_USE_SPI1
CH_IRQ_HANDLER(SAM3XA_SPI0_HANDLER) {
  CH_IRQ_PROLOGUE();
  serve_spi_irq(&SPID1);
  CH_IRQ_EPILOGUE();
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level SPI driver initialization.
 *
 * @notapi
 */
void spi_lld_init(void) {
#if SAM3XA_SPI_USE_SPI1
  /* Driver initialization.*/
  spiObjectInit(&SPID1);
#endif
}

/**
 * @brief   Configures and activates the SPI peripheral.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_start(SPIDriver *spip) {
  uint8_t scbr =
    spip->config->speed == 0 ? 0xFF :
    SystemCoreClock < spip->config->speed ? 0x01 :
    SystemCoreClock / spip->config->speed > 0xFF ? 0xFF :
    SystemCoreClock / spip->config->speed;

  if (spip->state == SPI_STOP) {
    uint32_t irq_priority =
        spip->config->irq_priority ? spip->config->irq_priority :
        SAM3XA_SPI_DEFAULT_IRQ_PRIORITY;
    chDbgCheck(CORTEX_IS_VALID_KERNEL_PRIORITY(irq_priority),
        "SPI irq_priority");

    pmc_enable_peripheral_clock(spip->peripheral_id);
    nvicEnableVector(spip->irq_id, CORTEX_PRIORITY_MASK(irq_priority));

    spip->spi->SPI_CR = SPI_CR_SPIDIS;
    spip->spi->SPI_CR = SPI_CR_SWRST;
    spip->spi->SPI_CR = SPI_CR_SWRST;
    spip->spi->SPI_MR = SPI_MR_MSTR | SPI_MR_MODFDIS;
    spip->spi->SPI_CSR[0] = SPI_CSR_BITS_8_BIT | SPI_CSR_CSAAT |
        SPI_CSR_SCBR(scbr) | (
        spip->config->spi_mode == 1 ? 0 :
        spip->config->spi_mode == 2 ? SPI_CSR_CPOL | SPI_CSR_NCPHA :
        spip->config->spi_mode == 3 ? SPI_CSR_CPOL :
        SPI_CSR_NCPHA);

    peripheral_pin_apply(&spip->config->spck_pin);
    peripheral_pin_apply(&spip->config->miso_pin);
    peripheral_pin_apply(&spip->config->mosi_pin);
    palSetPad(spip->config->cs_pin.port, spip->config->cs_pin.pin);
    palSetPadMode(spip->config->cs_pin.port, spip->config->cs_pin.pin, PAL_MODE_OUTPUT_OPENDRAIN);

    spip->spi->SPI_CR = SPI_CR_SPIEN;
  } else {
    // MMC_SD will reconfigure us without stopping
    spip->spi->SPI_CSR[0] = SPI_CSR_BITS_8_BIT | SPI_CSR_CSAAT |
        SPI_CSR_SCBR(scbr) | (
        spip->config->spi_mode == 1 ? 0 :
        spip->config->spi_mode == 2 ? SPI_CSR_CPOL | SPI_CSR_NCPHA :
        spip->config->spi_mode == 3 ? SPI_CSR_CPOL :
        SPI_CSR_NCPHA);
  }
}

/**
 * @brief   Deactivates the SPI peripheral.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_stop(SPIDriver *spip) {

  if (spip->state == SPI_READY) {
    /* Resets the peripheral.*/
    spip->spi->SPI_CR = SPI_CR_SPIDIS | SPI_CR_SWRST;

    /* Disables the peripheral.*/
    peripheral_pin_reset(&spip->config->spck_pin);
    peripheral_pin_reset(&spip->config->miso_pin);
    peripheral_pin_reset(&spip->config->mosi_pin);
  }
}

/**
 * @brief   Asserts the slave select signal and prepares for transfers.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_select(SPIDriver *spip) {
  palClearPad(spip->config->cs_pin.port, spip->config->cs_pin.pin);
}

/**
 * @brief   Deasserts the slave select signal.
 * @details The previously selected peripheral is unselected.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_unselect(SPIDriver *spip) {
  palSetPad(spip->config->cs_pin.port, spip->config->cs_pin.pin);
}

/**
 * @brief   Ignores data on the SPI bus.
 * @details This asynchronous function starts the transmission of a series of
 *          idle words on the SPI bus and ignores the received data.
 * @post    At the end of the operation the configured callback is invoked.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to be ignored
 *
 * @notapi
 */
void spi_lld_ignore(SPIDriver *spip, size_t n) {
#if SAM3XA_USB_USE_DMAC
  if (IS_USE_DMA(spip)) {
    dmac_prepare_receive_dummy(spip->config->dma_rx_ch, spip->dma_rx_per,
        serve_spi_dma, spip,
        n, (void*)&spip->spi->SPI_RDR);
    dmac_prepare_send_dummy(spip->config->dma_tx_ch, spip->dma_tx_per,
        NULL, NULL,
        n, 0xFF, (void*)&spip->spi->SPI_TDR);
    dmac_channel_start(spip->config->dma_rx_ch);
    dmac_channel_start(spip->config->dma_tx_ch);
    return;
  }
#endif
  spip->size = n;
  spip->tx_buf = 0;
  spip->rx_buf = 0;

  spip->spi->SPI_IER = SPI_IER_RDRF;
  spip->spi->SPI_TDR = 0xFFFF;
}

/**
 * @brief   Exchanges data on the SPI bus.
 * @details This asynchronous function starts a simultaneous transmit/receive
 *          operation.
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below or
 *          equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to be exchanged
 * @param[in] txbuf     the pointer to the transmit buffer
 * @param[out] rxbuf    the pointer to the receive buffer
 *
 * @notapi
 */
void spi_lld_exchange(SPIDriver *spip, size_t n,
                      const void *txbuf, void *rxbuf) {
#if SAM3XA_USB_USE_DMAC
  if (IS_USE_DMA(spip)) {
    dmac_prepare_receive(spip->config->dma_rx_ch, spip->dma_rx_per,
        serve_spi_dma, spip,
        n, rxbuf, (void*)&spip->spi->SPI_RDR);
    dmac_prepare_send(spip->config->dma_tx_ch, spip->dma_tx_per,
        NULL, NULL,
        n, txbuf, (void*)&spip->spi->SPI_TDR);
    dmac_channel_start(spip->config->dma_rx_ch);
    dmac_channel_start(spip->config->dma_tx_ch);
    return;
  }
#endif
  spip->size = n;
  spip->rx_buf = rxbuf;
  spip->tx_buf = (void*)txbuf;

  spip->spi->SPI_IER = SPI_IER_RDRF;
  spip->spi->SPI_TDR = *(uint8_t*)txbuf;
}

/**
 * @brief   Sends data over the SPI bus.
 * @details This asynchronous function starts a transmit operation.
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below or
 *          equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to send
 * @param[in] txbuf     the pointer to the transmit buffer
 *
 * @notapi
 */
void spi_lld_send(SPIDriver *spip, size_t n, const void *txbuf) {
#if SAM3XA_USB_USE_DMAC
  if (IS_USE_DMA(spip)) {
    dmac_prepare_receive_dummy(spip->config->dma_rx_ch, spip->dma_rx_per,
        serve_spi_dma, spip,
        n, (void*)&spip->spi->SPI_RDR);
    dmac_prepare_send(spip->config->dma_tx_ch, spip->dma_tx_per,
        NULL, NULL,
        n, txbuf, (void*)&spip->spi->SPI_TDR);
    dmac_channel_start(spip->config->dma_rx_ch);
    dmac_channel_start(spip->config->dma_tx_ch);
    return;
  }
#endif
  spip->size = n;
  spip->rx_buf = 0;
  spip->tx_buf = (void*)txbuf;

  spip->spi->SPI_IER = SPI_IER_RDRF;
  spip->spi->SPI_TDR = *(uint8_t*)txbuf;
}

/**
 * @brief   Receives data from the SPI bus.
 * @details This asynchronous function starts a receive operation.
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below or
 *          equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to receive
 * @param[out] rxbuf    the pointer to the receive buffer
 *
 * @notapi
 */

void spi_lld_receive(SPIDriver *spip, size_t n, void *rxbuf) {
#if SAM3XA_USB_USE_DMAC
  if (IS_USE_DMA(spip)) {
    dmac_prepare_receive(spip->config->dma_rx_ch, spip->dma_rx_per,
        serve_spi_dma, spip,
        n, rxbuf, (void*)&spip->spi->SPI_RDR);
    dmac_prepare_send_dummy(spip->config->dma_tx_ch, spip->dma_tx_per,
        NULL, NULL,
        n, 0xFF, (void*)&spip->spi->SPI_TDR);
    dmac_channel_start(spip->config->dma_rx_ch);
    dmac_channel_start(spip->config->dma_tx_ch);
    return;
  }
#endif
  spip->size = n;
  spip->rx_buf = rxbuf;
  spip->tx_buf = 0;

  spip->spi->SPI_IER = SPI_IER_RDRF;
  spip->spi->SPI_TDR = 0xFFFF;
}

/**
 * @brief   Exchanges one frame using a polled wait.
 * @details This synchronous function exchanges one frame using a polled
 *          synchronization method. This function is useful when exchanging
 *          small amount of data on high speed channels, usually in this
 *          situation is much more efficient just wait for completion using
 *          polling than suspending the thread waiting for an interrupt.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] frame     the data frame to send over the SPI bus
 * @return              The received data frame from the SPI bus.
 */
uint16_t spi_lld_polled_exchange(SPIDriver *spip, uint16_t frame) {
  spip->spi->SPI_CR = SPI_CR_SPIEN;
  spip->spi->SPI_TDR = frame;
  while (!(spip->spi->SPI_SR & SPI_SR_RDRF));
  return spip->spi->SPI_RDR;
}

#endif /* HAL_USE_SPI */

/** @} */
