#include "hal_spi_driver.h"

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

