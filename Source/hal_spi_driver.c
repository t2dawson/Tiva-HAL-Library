#include "hal_spi_driver.h"

//Function definitions:

static void hal_spi_configure_clock(SSI0_Type* SPIx, uint8_t phase, uint8_t polarity) {
	
	if(phase)
		SPIx->CR0 |= SPI_REG_CR0_CLK_PHASE;
	
	else 
			SPIx->CR0 &= (~SPI_REG_CR0_CLK_PHASE);
	
	if(polarity) 
		SPIx->CR0 |= SPI_REG_CR0_CLK_POLARITY;
		
	
	else 
			SPIx->CR0 &= (~SPI_REG_CR0_CLK_POLARITY);
}


static void hal_spi_configure_mode(SSI0_Type* SPIx, uint8_t mode) {
	
	if(!mode)
		SPIx->CR1 &= (~SPI_REG_CR1_MASTER_SLAVE_SELECT);

	
	else 
		SPIx->CR1  |= SPI_REG_CR1_MASTER_SLAVE_SELECT;
}

static void hal_spi_enable(SSI0_Type* SPIx) {
	
		SPIx->CR1 |= SPI_REG_CR1_SSI_ENABLE;
}

static void hal_spi_disable(SSI0_Type* SPIx) {
	
		SPIx->CR1 &= (~SPI_REG_CR1_SSI_ENABLE);
}

static void hal_spi_configure_clk_source(SSI0_Type* SPIx, uint32_t source) {
	
	SPIx->CC = source;
}

static void hal_spi_configure_baud_rate(SSI0_Type* SPIx, uint8_t serialClkRate, uint8_t prescaleDiv) {
	
	SPIx->CPSR = prescaleDiv;
	SPIx->CR0 |= (serialClkRate << 8);
}

static void hal_ssi_configure_protocol(SSI0_Type* SPIx, uint8_t protocol) {
	
	if(protocol == SSI_FORMAT_MICROWIRE) {
		SPIx->CR0 |= (1 << 5);
		SPIx->CR0 &= ~(1 << 4);
	}
	else if(protocol == SSI_FORMAT_TI_SSI) {
		
		SPIx->CR0 |= ( 1 << 4);
		SPIx->CR0 &= ~(1 << 5);
	}
	
	else 
		SPIx->CR0 &= ~((1 << 4) | (1 << 5));
}

static void hal_spi_configure_data_size(SSI0_Type* SPIx, uint32_t dataSize) {
	
	SPIx->CR0 &= SPI_RESET_DATA_SIZE_REG;
	SPIx->CR0 |= dataSize;
}

static void hal_spi_configure_loopback(SSI0_Type* SPIx, uint8_t loopback) {
	
	if(loopback)
		SPIx->CR1 |= SPI_REG_CR1_LOOPBACK;
	
	else 
		SPIx->CR1 &= (~SPI_REG_CR1_LOOPBACK);
	
}

void hal_spi_enable_txe_interrupt(SSI0_Type* SPIx){
		
		SPIx->CR1 |= ( 1 << 4);
		SPIx->IM |= (1 << 3);
}

void hal_spi_disable_txe_interrupt(SSI0_Type* SPIx){
		
		SPIx->CR1 &= ~ (1 << 4);
		SPIx->IM &= ~(1 << 3);
}

void hal_spi_enable_rxne_interrupt(SSI0_Type* SPIx) {
		
	SPIx->IM |= (1 << 2);
	
}

void hal_spi_disable_rxne_interrupt(SSI0_Type* SPIx) {
	
	SPIx->IM &= ~(1 << 2);
}

void hal_spi_init(spi_handle_t* spi_handle) {
		
		hal_spi_disable(spi_handle->instance);
		hal_spi_configure_mode(spi_handle->instance, spi_handle->init.mode);
		hal_spi_configure_clk_source(spi_handle->instance,spi_handle->init.clkSource);
		hal_spi_configure_baud_rate(spi_handle->instance, spi_handle->init.serialClkRate, spi_handle->init.clockPrescaleDiv);
		hal_spi_configure_clock(spi_handle->instance, spi_handle->init.clcPhase, spi_handle->init.clkPolarity);
		hal_ssi_configure_protocol(spi_handle->instance, SSI_FORMAT_SPI);
		hal_spi_configure_data_size(spi_handle->instance, spi_handle->init.dataSize);
		hal_spi_configure_loopback(spi_handle->instance, spi_handle->init.loopback);
		hal_spi_enable(spi_handle->instance);
}

void hal_spi_master_tx(spi_handle_t* spi_handle, uint16_t* buffer, uint32_t len) {
	
	spi_handle->pTxBuffPtr = buffer;
	spi_handle->TxCount = len;
	spi_handle->TxSize = len;
	spi_handle->state = HAL_SPI_STATE_BUSY_TX;
	
	hal_spi_enable(spi_handle->instance);
	
	hal_spi_enable_txe_interrupt(spi_handle->instance);
}

void hal_spi_master_rx(spi_handle_t* spi_handle, uint16_t* rcvBuffer, uint32_t len) {
	
	uint32_t val;
	
	// This is a dummy transfer buffer
	
	spi_handle->pTxBuffPtr = rcvBuffer;
	spi_handle->TxCount = len;
	spi_handle->TxSize = len;
	
	//data will be read in to rx buffer
	spi_handle->RxCount = len;
	spi_handle->RxSize = len;
	spi_handle->pRxBuffPtr = rcvBuffer;
	
	//set driver state to busy with recieve
	spi_handle->state = HAL_SPI_STATE_BUSY_RX;
	
	hal_spi_enable(spi_handle->instance);
	
	
	// read data register before enabling RXNE interrupt to ensure 
	// data register is empty
	
	val = spi_handle->instance->DR;
	
	//Enable TXE and RXNE interrupts
	hal_spi_enable_rxne_interrupt(spi_handle->instance);
	hal_spi_enable_txe_interrupt(spi_handle->instance);
}

void hal_spi_slave_tx(spi_handle_t* spi_handle, uint16_t* buffer, uint32_t len) {
	
	// fill in pointers and length variables for TX data
		spi_handle->pTxBuffPtr = buffer;
		spi_handle->TxSize = len;
		spi_handle->TxCount = len;
	
	// fill in pointers to dummy rx, can reuse the same pointer
		spi_handle->pRxBuffPtr = buffer;
		spi_handle->RxCount = len;
		spi_handle->RxSize = len;
	
	// set Driver state to BUSY_TX
	
		spi_handle->state = HAL_SPI_STATE_BUSY_TX;
	
	// enable spi
		hal_spi_enable(spi_handle->instance);
	
	//Enable TXE and RXNE interrupts
		hal_spi_enable_rxne_interrupt(spi_handle->instance);
		hal_spi_enable_txe_interrupt(spi_handle->instance);
}	

void hal_spi_slave_rx(spi_handle_t* spi_handle, uint16_t* rcvBuffer, uint32_t len) {
	
		// fill in pointers and length of rx data
		spi_handle->pRxBuffPtr = rcvBuffer;
		spi_handle->RxCount = len;
		spi_handle->RxSize = len;
	
	// set Driver state to BUSY_RX
	
		spi_handle->state = HAL_SPI_STATE_BUSY_RX;
	
	// enable spi
		hal_spi_enable(spi_handle->instance);
	
	//Enable RXNE interrupts
		hal_spi_enable_rxne_interrupt(spi_handle->instance);
}

/* 
* @brief : handles the rxne interrupt 
* @param hspi: pointer to a spi_handle_t structure containing the spi configuration
* @retval : void
*/
static void hal_spi_handle_rx_interrupt(spi_handle_t* hspi) {
	
	(*hspi->pRxBuffPtr++) = hspi->instance->DR;
	hspi->RxCount -= 2;
	
	if(hspi->RxCount == 0) {
			
			while(hspi->instance->SR & SPI_REG_SR_BUSY_FLAG);
		
		hal_spi_disable_rxne_interrupt(hspi->instance);
		hspi->state = HAL_SPI_STATE_READY;
	}
}

/* 
* @brief : handles the txe interrupt 
* @param hspi: pointer to a spi_handle_t structure containing the spi configuration
* @retval : void
*/

static void hal_spi_handle_tx_interrupt(spi_handle_t* hspi) {
		
	hspi->instance->DR = (*hspi->pTxBuffPtr++);
	hspi->TxCount -= 2;
	
	if(hspi->TxCount == 0) {
			
			//when count is 0, transmission is complete so disable the tx interrupt
			hal_spi_disable_txe_interrupt(hspi->instance);
			
		//if master and if driver state is not BUSY_RX, change state to READY
		if (!hspi->init.mode && (hspi->state != HAL_SPI_STATE_BUSY_RX))
				hspi->state = HAL_SPI_STATE_READY;
	}
}

void hal_spi_irq_handler(spi_handle_t* hspi) {
	
	uint32_t flag1 = 0, flag2 = 0;
	
	//check to see if RXNE is set in status register
	
	flag1 = hspi->instance->SR;
	flag2 = hspi->instance->IM;
	
	if((flag1 & SPI_REG_SR_RX_FIFO_NOT_EMPTY) && (flag2 & SPI_REG_RXNE_INTERRUPT_ENABLE)) {
			
			hal_spi_handle_rx_interrupt(hspi);
			return;
	}
	
	else if ((flag1 & SPI_REG_SR_TX_FIFO_EMPTY) && (flag2 & SPI_REG_TXE_INTERRUPT_ENABLE)) {
		
		hal_spi_handle_tx_interrupt(hspi);
		return;
	}
}

void hal_spi_enable_global_interrupt(IRQn_Type irq) {
	
	__NVIC_EnableIRQ(irq);
}

void hal_spi_disable_global_interrupt(IRQn_Type irq) {
	
	__NVIC_DisableIRQ(irq);
}
