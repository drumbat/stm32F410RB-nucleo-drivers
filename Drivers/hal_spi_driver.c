#include <stdint.h>
#include "hal_spi_driver.h"

/****************************************************************/
/*                                                              */
/*            Helper Functions                                  */
/*                                                              */
/****************************************************************/

/**
* @brief     Enables the SPI device
* @param    *SPIx : Base address of the SPI
* @retval    None
*/

void hal_spi_enable(SPI_TypeDef *SPIx) {
    if ( !(SPIx->CR1 & SPI_REG_CR1_SPE) ) {
        SPIx->CR1 |= SPI_REG_CR1_SPE;
    }
}

/**
* @brief     Disbables the SPI device
* @param    *SPIx : Base address of the SPI
* @retval    None
*/

void hal_spi_disable(SPI_TypeDef *SPIx) {
    SPIx->CR1 &= ~SPI_REG_CR1_SPE;
}

/**
* @brief     Configures SPI clock phase and polarity
* @param    *SPIx : Base address of the SPI
* @param    phase : configures phase
* @param    polarity : configures polarity
* @retval    None
*/

void hal_spi_configure_phase_and_polarity(SPI_TypeDef *SPIx, uint32_t phase, uint32_t polarity) {
    if (phase) {
        SPIx->CR1 |= SPI_REG_CR1_CPHA;
    } else {
        SPIx->CR1 &= ~SPI_REG_CR1_CPHA;
    }

    if (polarity) {
        SPIx->CR1 |= SPI_REG_CR1_CPOL;
    } else {
        SPIx->CR1 &= ~SPI_REG_CR1_CPOL;
    }
}

/**
* @brief     Configures master of slave mode
* @param    *SPIx : Base address of the SPI
* @param     master : if 1, then configured for master
* @retval    None
*/

void hal_spi_configure_device_mode(SPI_TypeDef *SPIx, uint32_t master) {
    if (master) {
        SPIx->CR1 |= SPI_REG_CR1_MSTR;
    } else {
        SPIx->CR1 &= ~SPI_REG_CR1_MSTR;
    }
}

/**
* @brief     Configures SPI datasize and direction
* @param    *SPIx : Base address of the SPI
* @param    datasize : data size to be configured
* @param    lsbfirst : if 1, lsb will be sent first
* @retval    None
*/

void hal_spi_configure_datasize_and_direction(SPI_TypeDef *SPIx, uint32_t datasize_16, uint32_t lsbfirst) {
    if (datasize_16) {
        SPIx->CR1 |= SPI_REG_CR1_DFF;
    } else {
        SPIx->CR1 &= ~SPI_REG_CR1_DFF;
    }

    if (lsbfirst) {
        SPIx->CR1 |= SPI_REG_CR1_LSBFIRST;
    } else {
        SPIx->CR1 &= ~SPI_REG_CR1_LSBFIRST;
    }

}
/**
* @brief     Configures NSS pin of the master
* @param    *SPIx : Base address of the SPI
* @param     ssm_enable : if 1, enable the
*              SSM(software slave management)
*              bit and set the NSS pin to high
*
* @retval    None
*/

void hal_spi_configure_nss_master(SPI_TypeDef *SPIx, uint32_t ssm_enable) {

    #if 0
    if (ssm_enable) {
        SPIx->CR1 |=  SPI_REG_CR1_SSM ;
        SPIx->CR1 |= SPI_REG_CR1_SSI;
    } else {
        SPIx->CR1 &= ~SPI_REG_CR1_SSM;
    }

    #endif

        if (ssm_enable) {
        SPIx->CR1 |= ( SPI_REG_CR1_SSM | (1 << 8) );
    } else {
        SPIx->CR1 &= ~SPI_REG_CR1_SSM;
    }
}

/**
* @brief     Configures NSS pin of the slave
* @param    *SPIx : Base address of the SPI
* @param     ssm_enable : if 1, enable the
*              SSM(software slave management)
*              otherwise enable hardware slave
*              management
* @retval    None
*/
void hal_spi_configure_nss_slave(SPI_TypeDef *SPIx, uint32_t ssm_enable) {
    if (ssm_enable) {
        SPIx->CR1 |= SPI_REG_CR1_SSM;
    } else {
        SPIx->CR1 &= ~SPI_REG_CR1_SSM;
    }
}

 /**
  * @brief  Configures SPI baudrate
  * @param  *SPIx : Base address of the SPI
  * @param  pre_scalar_value : pre scalar value to be used to generate baudrate
  * @retval None
  */
void hal_spi_configure_baudrate(SPI_TypeDef *SPIx, uint32_t pre_scalar_value) {
    SPIx->CR1 |= pre_scalar_value;
}

 /**
  * @brief  Configures SPI direction
  * @param  *SPIx : Base address of the SPI
  * @param  direction : if 1, direction will be single line bi-directional else, 2 lines uni directional
  * @retval None
  */
void hal_spi_configure_direction(SPI_TypeDef *SPIx, uint32_t direction ) {
    if (direction ) {
        SPIx->CR1 |= SPI_REG_CR1_BIDIMODE;
    } else {
        SPIx->CR1 &= ~SPI_REG_CR1_BIDIMODE;
    }
}

/**
 * @brief  Checks whether bus is free or busy
 * @param  *SPIx : Base address of the SPI
 * @retval return 1, if bus is busy
 */
uint8_t hal_spi_is_busy(SPI_TypeDef *SPIx) {
    if (SPIx->SR & SPI_REG_SR_BUSY_FLAG ) {
        return SPI_IS_BUSY;
    } else {
       return SPI_IS_NOT_BUSY;
    }
}

/**
* @brief     Enables the Tx buffer empty interrupt (TXE)
* @param    *SPIx : Base address of the SPI
* @retval    None
*/
static void hal_spi_enable_txe_interrupt(SPI_TypeDef *SPIx) {
    SPIx->CR2 |= SPI_REG_CR2_TXEIE_ENABLE;
}

/**
* @brief     Disables the Tx buffer empty interrupt (TXE)
* @param    *SPIx : Base address of the SPI
* @retval    None
*/
static void hal_spi_disable_txe_interrupt(SPI_TypeDef *SPIx) {
    SPIx->CR2 &= ~SPI_REG_CR2_TXEIE_ENABLE;
}

/**
* @brief     Enables the Rx buffer empty interrupt (TXE)
* @param    *SPIx : Base address of the SPI
* @retval    None
*/
static void hal_spi_enable_rxne_interrupt(SPI_TypeDef *SPIx) {
    SPIx->CR2 |= SPI_REG_CR2_RXNEIE_ENABLE;
}

/**
* @brief     Disables the Rx buffer empty interrupt (TXE)
* @param    *SPIx : Base address of the SPI
* @retval    None
*/
static void hal_spi_disable_rxne_interrupt(SPI_TypeDef *SPIx) {
    SPIx->CR2 &= ~SPI_REG_CR2_RXNEIE_ENABLE;
}


/****************************************************************/
/*                                                              */
/*            3. Driver exposed APIs                            */
/*                                                              */
/****************************************************************/

/**
* @brief     API used to initialize the given SPI device
* @param    *SPIx : Base address of the SPI
* @retval    None
*/
void hal_spi_init(spi_handle_t *spi_handle) {

    /* configure the phase and polarity */
    hal_spi_configure_phase_and_polarity(spi_handle->Instance, spi_handle->Init.CLKPhase, spi_handle->Init.CLKPolarity );

    /* configure the SPI device mode */
    hal_spi_configure_device_mode(spi_handle->Instance, spi_handle->Init.Mode);

    /* configure the SPI data size */
    hal_spi_configure_datasize_and_direction(spi_handle->Instance, spi_handle->Init.DataSize, spi_handle->Init.FirstBit);

    /* configure the slave select line */
    if (spi_handle->Init.Mode == SPI_MASTER_MODE_SEL) {
        hal_spi_configure_nss_master(spi_handle->Instance, spi_handle->Init.NSS);
    } else {
        hal_spi_configure_nss_slave(spi_handle->Instance, spi_handle->Init.NSS);
    }

    /* configure the SPI device speed */
    hal_spi_configure_baudrate(spi_handle->Instance, spi_handle->Init.BaudRatePrescaler);

    /* configure the SPI device direction */
    hal_spi_configure_direction(spi_handle->Instance,spi_handle->Init.Direction);
}

/**
* @brief     API used to do master data transmission
* @param    *SPIx : Base address of the SPI
* @param    *buffer : pointer to the tx buffer
* @param     len : len of tx data
* @retval    None
*/
void hal_spi_master_tx(spi_handle_t *spi_handle, uint8_t *buffer, uint32_t len){

    spi_handle->pTxBuffPtr = buffer;
    spi_handle->TxXferCount = len;
    spi_handle->TxXferSize = len;

    spi_handle->State = HAL_SPI_STATE_BUSY_TX;

    hal_spi_enable(spi_handle->Instance);
    hal_spi_enable_txe_interrupt(spi_handle->Instance);


}

/**
* @brief     API used to do slave data reception
* @param    *SPIx : Base address of the SPI
* @param    *buffer : pointer to the rx buffer
* @param     len : len of rx data
* @retval    None
*/
void hal_spi_slave_rx(spi_handle_t *spi_handle, uint8_t *buffer, uint32_t len){

    /* Populate the rcv buffer pointer addres along with the size in the handle */
    spi_handle->pRxBuffPtr = buffer;
    spi_handle->RxXferCount = len;
    spi_handle->RxXferSize = len;

    /* Driver is busy in RX */
    spi_handle->State = HAL_SPI_STATE_BUSY_RX;

    /* enable the peripheral */
    hal_spi_enable(spi_handle->Instance);

    /* slave needs to receive data so enable the RXNE interrupt*/
    hal_spi_enable_rxne_interrupt(spi_handle->Instance);
}

/**
* @brief     API used to do master data reception
* @param    *SPIx : Base address of the SPI
* @param    *buffer : pointer to the rx buffer
* @param     len : len of rx data
* @retval    None
*/
uint8_t dummu_tx[2]={0};
void hal_spi_master_rx(spi_handle_t *spi_handle, uint8_t *buffer, uint32_t len){

    uint32_t val;

    /* this is a dummy tx */
    spi_handle->pTxBuffPtr = dummu_tx;
    spi_handle->TxXferCount = len;
    spi_handle->TxXferSize = len;

    /* data wil be rxed to rx_buffer */
    spi_handle->pRxBuffPtr = buffer;
    spi_handle->RxXferCount = len;
    spi_handle->RxXferSize = len;

    /* Driver is busy in RX */
    spi_handle->State = HAL_SPI_STATE_BUSY_RX;


    /* read data register once before enabling
    *  the RXNE interrupt to make sure DR is empty
    */

    val = spi_handle->Instance->DR;

    /* Now enable both TXE and RXNE Interrupt  */
    hal_spi_enable_rxne_interrupt(spi_handle->Instance);
    hal_spi_enable_txe_interrupt(spi_handle->Instance);

    hal_spi_enable(spi_handle->Instance);
}

/**
* @brief     API used to do slave data transmission
* @param    *SPIx : Base address of the SPI
* @param    *buffer : pointer to the tx buffer
* @param     len : len of tx data
* @retval    None
*/
uint8_t dummy_rx[10]={0};
void hal_spi_slave_tx(spi_handle_t *spi_handle, uint8_t *buffer, uint32_t len){

    /* populate the pointers with length information to TX the data */
    spi_handle->pTxBuffPtr = buffer;
    spi_handle->TxXferCount = len;
    spi_handle->TxXferSize = len;

    /* pointers to handle dummy rx (it's safe ot reuse the same pointer) */
    spi_handle->pRxBuffPtr = dummy_rx;
    spi_handle->RxXferCount = len;
    spi_handle->RxXferSize = len;

    /* Driver is busy doing TX */
    spi_handle->State = HAL_SPI_STATE_BUSY_TX;

    /* Now enable both TXE and RXNE Interrupt  */
    hal_spi_enable_rxne_interrupt(spi_handle->Instance);
    hal_spi_enable_txe_interrupt(spi_handle->Instance);

    hal_spi_enable(spi_handle->Instance);
}


/**
* @brief     close Tx transfer
* @param     hspi : pointer to a spi_handle_t structure that
*                   contains the configuration information for
*                   the SPI module.
* @retval    None
*/
static void hal_spi_close_tx_interrupt(spi_handle_t *hspi) {
    /* Disable TXE interrupt */
    hal_spi_disable_txe_interrupt(hspi->Instance);

    /* if master and if driver state is not HAL_SPI_STATE_BUSY_RX then make state=READY*/
    if (hspi->Init.Mode && (hspi->State != HAL_SPI_STATE_BUSY_RX)) {
        hspi->State = HAL_SPI_STATE_READY;
    }
}

/**
* @brief     handles TXE interrupt.
* @param     hspi: pointer to an spi_handle_t structure that
*                  contains the configuration information for
*                  the SPI module.
* @retval    None
*/

static void hal_spi_handle_tx_interrupt(spi_handle_t *hspi) {

    if (hspi->Init.DataSize == SPI_8BIT_DF_ENABLE) {
        /* Transmit data in 8 Bit mode */
        hspi->Instance->DR = (*hspi->pTxBuffPtr++);
        hspi->TxXferCount--; // we sent 1 byte
    } else {
        /* Transmit data in 16 Bit mode */
        hspi->Instance->DR = *((uint16_t*)hspi->pTxBuffPtr);
        hspi->pTxBuffPtr+=2;
        hspi->TxXferCount-=2; // we sent 2 bytes in one go
    }

    if (hspi->TxXferCount == 0) {
        /* reached end of transmission, so close the txe interrupt */
        hal_spi_close_tx_interrupt(hspi);
    }

}

/**
* @brief     close Rx transfer
* @param     hspi : pointer to a spi_handle_t structure that
*                   contains the configuration information for
*                   the SPI module.
* @retval    None
*/
static void hal_spi_close_rx_interrupt(spi_handle_t *hspi) {

    while (hal_spi_is_busy(hspi->Instance));
    /* Disable RXNE interrupt */
    hal_spi_disable_rxne_interrupt(hspi->Instance);
    hspi->State = HAL_SPI_STATE_READY;

}

/**
* @brief     handles RXNE interrupt.
* @param     hspi: pointer to an spi_handle_t structure that
*                  contains the configuration information for
*                  the SPI module.
* @retval    None
*/

static void hal_spi_handle_rx_interrupt(spi_handle_t *hspi) {

    if (hspi->Init.DataSize == SPI_8BIT_DF_ENABLE) {
        /* Recieve data in 16 Bit mode */
        /* NULL check */
        if (hspi->pRxBuffPtr++) {
            (*hspi->pRxBuffPtr++) = hspi->Instance->DR;
        }
        hspi->RxXferCount--;
    } else {
        /* Receive data in 16 Bit mode */
        *((uint16_t*)hspi->pTxBuffPtr) = hspi->Instance->DR;
        hspi->pRxBuffPtr+=2;
        hspi->RxXferCount-=2;
    }

    if (hspi->RxXferCount == 0) {
        /* reached end of transmission, so close the txe interrupt */
        hal_spi_close_rx_interrupt(hspi);
    }

}

/**
* @brief     This function handles SPI interrupt request.
* @param     hspi: pointer to an spi_handle_t structure that
*                  contains the configuration information for
*                  the SPI module.
* @retval    None
*/
void hal_i2c_spi_irq_handler(spi_handle_t *hspi){
    uint32_t tmp1 = 0, tmp2 = 0;

    /* check to see RXNE is set in the status register */
    tmp1 = (hspi->Instance->SR & SPI_REG_SR_RXNE_FLAG);
    /* check to see RXNEIE bit is enabled in the control register*/
    tmp2 = (hspi->Instance->CR2 & SPI_REG_CR2_RXNEIE_ENABLE);

    if ((tmp1 != RESET) && (tmp2 != RESET) ) {
        /* RXNE flag is set
         * handle the RX of data bytes
         */
        hal_spi_handle_rx_interrupt(hspi);
        return;
    }
    /* check to see TXE is set in the status register */
    tmp1 = (hspi->Instance->SR & SPI_REG_SR_TXE_FLAG);
    /* check to see TXEIE bit is enabled in the control register*/
    tmp2 = (hspi->Instance->CR2 & SPI_REG_CR2_TXEIE_ENABLE);

    if ((tmp1 != RESET) && (tmp2 != RESET) ) {
        /* TXE flag is set
         * handle the tX of data bytes
         */
        hal_spi_handle_tx_interrupt(hspi);
        return;
    }
}



























