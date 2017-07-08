#ifndef __HAL_SPI_DRIVER_H
#define __HAL_SPI_DRIVER_H

// Microcontroller specific header file for Register Map
#include "tm4c123gh6pm.h"

/**************************************************************************************/
/*												1. Serial Peripheral Interface															*/
/*														Register Bit Definitions																*/
/*																																										*/
/**************************************************************************************/

/********************** Bit definitions for SPI_CR0 register*************/

#define SPI_REG_CR0_CLK_PHASE 						((uint32_t) 1 << 7)
#define SPI_CLK_PHASE_FIRST_EDGE			0
#define SPI_CLK_PHASE_SECOND_EDGE			1

#define SPI_REG_CR0_CLK_POLARITY					((uint32_t) 1 << 6)
#define SPI_CLK_POL_LOW								0
#define SPI_CLK_POL_HIGH							1

#define SPI_SPI_FORMAT
#define SPI_DATA_SIZE_4_BIT
#define SPI_DATA_SIZE_5_BIT
#define SPI_DATA_SIZE_6_BIT
#define SPI_DATA_SIZE_7_BIT
#define SPI_DATA_SIZE_8_BIT
#define SPI_DATA_SIZE_9_BIT
#define SPI_DATA_SIZE_10_BIT
#define SPI_DATA_SIZE_11_BIT
#define SPI_DATA_SIZE_12_BIT
#define SPI_DATA_SIZE_13_BIT
#define SPI_DATA_SIZE_14_BIT
#define SPI_DATA_SIZE_15_BIT
#define SPI_DATA_SIZE_16_BIT


/********************** Bit definitions for SPI_CR1 register*************/
#define SPI_REG_CR1_MASTER_SLAVE_SELECT 	((uint32_t) 1 << 2)
#define SPI_MASTER_MODE					0
#define SPI_SLAVE_MODE					1

#define SPI_REG_CR1_SSI_ENABLE						((uint32_t) 1 << 1)
#define SPI_SSI_ENABLE					1
#define SPI_SSI_DISABLE					0

#define SPI_REG_CR1_LOOPBACK							((uint32_t) 1 << 0)
#define SPI_LOOPBACK_ENABLE 	 	1
#define SPI_LOOPBACK_DISABLE		0
/********************** Bit definitions for SPI_STATUS register*************/

#define SPI_REG_SR_BUSY_FLAG						((uint32_t) 1 << 4)
#define SPI_REG_SR_RX_FIFO_FULL					((uint32_t) 1 << 3 )
#define SPI_REG_SR_RX_FIFO_NOT_EMPTY		((uint32_t) 1 << 2 )
#define SPI_REG_SR_TX_FIFO_NOT_FULL			((uint32_t) 1 << 1 )
#define SPI_REG_SR_TX_FIFO_EMPTY				((uint32_t) 1 << 0 ) 



/**********************Helper Macros and Clock Enable macro functions**********/
#define RESET 0
#define SET !RESET

#define SPI_0	SSI0
#define SPI_1 SSI1
#define SPI_2 SSI2
#define SPI_3 SSI3

#define __HAL_RCC_SPI0_CLK_ENABLE				(SYSCTL->RCGCSSI |= ( 1 << 0))
#define __HAL_RCC_SPI1_CLK_ENABLE				(SYSCTL->RCGCSSI |= ( 1 << 1))
#define __HAL_RCC_SPI2_CLK_ENABLE				(SYSCTL->RCGCSSI |= ( 1 << 2))
#define __HAL_RCC_SPI3_CLK_ENABLE				(SYSCTL->RCGCSSI |= ( 1 << 3))

#define __HAL_RCC_SPI0_CLK_DISABLE			(SYSCTL->RCGCSSI |= ( 0 << 0))
#define __HAL_RCC_SPI1_CLK_DISABLE			(SYSCTL->RCGCSSI |= ( 0 << 1))
#define __HAL_RCC_SPI2_CLK_DISABLE			(SYSCTL->RCGCSSI |= ( 0 << 2))
#define __HAL_RCC_SPI3_CLK_DISABLE			(SYSCTL->RCGCSSI |= ( 0 << 3))

/****************************************************************************************/
/*											2. Serial Peripheral Interface																	*/
/*												Configuration Data Structures																	*/
/*																																											*/
/****************************************************************************************/


/* @brief: SPI Commuincation State enumeration
*/
typedef enum {
	
	HAL_SPI_STATE_RESET 			= 0x00,  // SPI not yet initialized or disabled
	HAL_SPI_STATE_READY 			= 0x01,  // SPI initialized and ready for use
	HAL_SPI_STATE_BUSY 				= 0x02,	 // SPI process ongoing
	HAL_SPI_STATE_BUSY_TX 		= 0x12,	 // Data transmission process ongoing
	HAL_SPI_STATE_BUSY_RX			= 0x22,	 // Data recieve process ongoing
	HAL_SPI_STATE_BUSY_RX_TX 	= 0x32,	 // Data transmission and recieve process ongoing
	HAL_SPI_STATE_ERROR				= 0x03   // SPI error state

} spi_state_t;



/* @brief: SPI Initialization Data Structure
*/
typedef struct {
	
	uint32_t mode;   // Specifies the SPI operating mode (Master or Slave)
	
	uint32_t dataSize;  // Specifies SPI Data Size
	
	uint32_t clkPolarity;  // Specifices the clock's steady state
	
	uint32_t clcPhase; // Specifies the clock's active edge for bit capture
	
	uint32_t baudRatePrescalar; // Specifies the Baud Rate prescalar value used to configure TX and RX clock

} spi_init_t;


/*	@brief: SPI Handle Data Structure
*/
typedef struct {
	
	SSI0_Type 	*instance;  // Holds register base addresses for SPI
	
	spi_init_t 	init;  // Used to configure the SPI parameters
	
	uint16_t 		*pTxBuffPtr; //Pointer to the TX buffer
	
	uint16_t 		 TxSize;			// SPI transfer size
	
	uint16_t 		 TxCount;			// SPI transfer counter
	
	uint16_t		*pRxBuffPtr;	// Pointer to the RX Buffer
	
	uint16_t 		*RxSize;			// SPI recieve size
	
	uint16_t 			RxCount;		// SPI recieve counter
	
 spi_state_t		state;			// SPI communication current state

} spi_handle_t ;


/************************************************************************************/
/*									3. Serial Peripheral Interface	  															*/
/*												Driver exposed APIs																				*/
/*																																									*/
/************************************************************************************/

/**
*	@brief: API used to intialize the SPI device
* @param  spi_handle: Pointer to the SPI handle structure
*	@retval: none
*/
void hal_spi_init(spi_handle_t* spi_handle);


/**
* @brief: API used to perform master data tramsission
*	@param  spi_handle: Pointer to the SPI handle structure
* @param  buffer : Pointer to the tx buffer
*	@param len: length of data to write
*	@retval: none
*/
void hal_spi_master_tx(spi_handle_t* spi_handle, uint16_t* buffer, uint32_t len);


/**
* @brief: API used to perform master data reception
*	@param  spi_handle: Pointer to the SPI handle structure
* @param  rcvBuffer : Pointer to the rx buffer
*	@param  len: length of data to read
*	@retval: none
*/
void hal_spi_master_rx(spi_handle_t* spi_handle, uint16_t* rcvBuffer, uint32_t len);


/**
* @brief: API used to perform slave data tranmission
*	@param  spi_handle: Pointer to the SPI handle structure
* @param  buffer : Pointer to the tx buffer
*	@param len: length of data to write
*	@retval: none
*/
void hal_spi_slave_tx(spi_handle_t* spi_handle, uint16_t* buffer, uint32_t len);


/**
* @brief: API used to perform slave data reception
*	@param spi_handle: Pointer to the SPI handle structure
* @param rcvBuffer : Pointer to the rx buffer
*	@param len: length of data to read
*	@retval: none
*/
void hal_spi_slave_rx(spi_handle_t* spi_handle, uint16_t* rcvBuffer, uint32_t len);



/**
*	@brief: This function handles the SPI interrupt request.
*	@param  hspi: pointer to the spi_handle_t structure containing 
*								SPI conifiguration information
*
*	@retval: none
*/
void hal_spi_irq_handler(spi_handle_t* hspi);


#endif //__HAL_SPI_DRIVER_H