#ifndef __HAL_I2C_DRIVER_H
#define __HAL_I2C_DRIVER_H

//TODO: implement interrupt handlers
//TODO: handle repeated start case
//TODO: clear Data register before writing for both slave and master

#include "tm4c123gh6pm.h"

/************************************************************************************************/
/*																	I2C Register Bit Definitions
**
***********************************************************************************************/

//Bit definitions for MASTER SLAVE ADDRESS

#define I2C_MASTER_ADDR_REG_SEND_RECIEVE  ((uint32_t) 1 << 0)
#define I2C_MASTER_SEND										0
#define I2C_MASTER_RECIEVE								1

//Bit definitions for MASTER CONTROL REGISTER

#define I2C_MASTER_CONTROL_REG_HIGH_SPEED_ENABLE			((uint32_t) 1 << 4)
#define I2C_MASTER_HIGH_SPEED_ENABLE			1
#define I2C_MASTER_HIGH_SPEED_DISABLE			0

#define I2C_MASTER_GENERATE_START					((uint32_t) 1 << 1)
#define I2C_MASTER_GENERATE_STOP					((uint32_t) 1 << 2)

#define I2C_MASTER_CONTROL_REG_DATA_ACK_ENABLE		((uint32_t) 1 << 3)
#define I2C_MASTER_DATA_ACK_ENABLE				1
#define I2C_MASTER_DATA_ACK_DISABLE				0

#define I2C_MASTER_CONTROL_REG_ENABLE							((uint32_t) 1 << 0)
#define I2C_MASTER_ENABLE									1
#define I2C_MASTER_DISABLE								0

//Bit definitions for MASTER STATUS REGISTER
#define I2C_MASTER_STATUS_REG_CLK_TIMEOUT_ERROR 	((uint32_t) 1 << 7)
#define I2C_MASTER_TIMEOUT_ERROR									1
#define I2C_MASTER_NO_TIMEOUT_ERROR								0

#define I2C_MASTER_STATUS_REG_BUS_BUSY 						((uint32_t) 1 << 6)
#define I2C_MASTER_BUS_BUSY												1
#define I2C_MASTER_BUS_IDLE												0

#define I2C_MASTER_STATUS_REG_I2C_IDLE					 	((uint32_t) 1 << 5)
#define I2C_MASTER_I2C_IDLE												1
#define I2C_MASTER_I2C_NOT_IDLE										0

#define I2C_MASTER_STATUS_REG_ARBITRATION_LOST 		((uint32_t) 1 << 4)
#define I2C_MASTER_ARBITRATION_LOST								1
#define I2C_MASTER_ARBITRATION_WON								0

#define I2C_MASTER_STATUS_REG_ACK_DATA 						((uint32_t) 1 << 3)
#define I2C_MASTER_DATA_ACKNOWLEDGED							1
#define I2C_MASTER_DATA_NOT_ACKNOWLEDGED					0

#define I2C_MASTER_STATUS_REG_ACK_ADDRESS 				((uint32_t) 1 << 2)
#define I2C_MASTER_ADDRESS_ACKNOWLEDGED							1
#define I2C_MASTER_ADDRESS_NOT_ACKNOWLEDGED					0

#define I2C_MASTER_STATUS_REG_ERROR 							((uint32_t) 1 << 1)
#define I2C_MASTER_ERROR													1
#define I2C_MASTER_NO_ERROR												0

#define I2C_MASTER_STATUS_REG_I2C_BUSY 						((uint32_t) 1 << 0)
#define I2C_BUSY																	1
#define I2C_IDLE																	0
//Bit definitions for MASTER TIME PERIOD REGISTER

#define I2C_MASTER_TIME_REG_HIGH_SPEED_ENABLE 		((uint32_t)  1 << 7)
#define I2C_MASTER_TIME_HIGH_SPEED_ENABLE		1
#define I2C_MASTER_TIME_HIGH_SPEED_DISABLE 	0

//Bit definitions for MASTER INTERRUPT MASK

#define I2C_MASTER_INTERRUPT_REG_CLOCK_TIMEOUT	((uint32_t) 1 << 1)
#define I2C_MASTER_INTERRUPT_REG_IMASK						((uint32_t) 1 << 0)

//Bit definitions for MASTER CONFIGURATION REGISTER

#define I2C_MASTER_CONFIG_REG_GLITCH_FILTER_ENABLE				((uint32_t) 1 << 6)
#define I2C_GLITCH_FILTER_ENABLE													1
#define I2C_GLITCH_FILTER_DISABLE													0

#define I2C_MASTER_CONFIG_REG_SLAVE_FUNCTION_ENABLE				((uint32_t) 1 << 5)
#define I2C_SLAVE_MODE_ENABLE															1
#define I2C_SLAVE_MODE_DISABLE														0

#define I2C_MASTER_CONFIG_REG_MASTER_FUNCTION_ENABLE			((uint32_t) 1 << 4)
#define I2C_MASTER_MODE_ENABLE														1
#define I2C_MASTER_MODE_DISABLE														0

#define I2C_MASTER_CONFIG_REG_LOOPBACK_ENABLE							((uint32_t) 1 << 0)
#define I2C_LOOPBACK_ENABLE																1
#define I2C_LOOPBACK_DISABLE															0

//Bit definitions for SLAVE CONTROL REGISTER

#define I2C_SLAVE_CONTROL_REG_DEVICE_ACTIVE							((uint32_t) 1 << 0)
#define I2C_SLAVE_ACTIVATE															1
#define I2C_SLAVE_DEACTIVATE														0

//Bit definitions for SLAVE STATUS REGISTER

#define I2C_SLAVE_STATUS_REG_OAR2_MATCH									((uint32_t) 1 << 3)
#define I2C_SLAVE_OAR2_NOT_MATCHED											0
#define I2C_SLAVE_OAR2_MATCHED_ACKNOWLEDGED							1

#define I2C_SLAVE_STATUS_REG_FIRST_BYTE_RECIEVED				((uint32_t) 1 << 2)
#define I2C_SLAVE_FIRST_BYTE_NOT_RECIEVED								0
#define I2C_SLAVE_FIRST_BYTE_RECIEVED										1

#define I2C_SLAVE_STATUS_REG_TRANSMIT_REQUEST					((uint32_t) 1 << 1)
#define I2C_SLAVE_TRANSMIT_REQUEST_PENDING						1
#define I2C_SLAVE_NO_TRANSMIT_REQUEST									0

#define I2C_SLAVE_STATUS_REG_RECIEVE_REQUEST					((uint32_t) 1 << 0)
#define I2C_SLAVE_NO_RECIEVE_REQUEST									0
#define I2C_SLAVE_RECIEVE_REQUEST_PENDING							1

//Bit definitions for SLAVE INTERRUPT MASK

#define I2C_SLAVE_INTERRUPT_REG_STOP_CONDITION_IMASK	((uint32_t) 1 << 2)
#define I2C_SLAVE_INTERRUPT_REG_START_CONDITION_IMASK	((uint32_t) 1 << 1)
#define I2C_SLAVE_INTERRUPT_REG_DATA_IMASK						((uint32_t) 1 << 0)

//Bit definitions for SLAVE OAR2

#define I2C_SLAVE_OAR2_REG_OAR2_ENABLE								((uint32_t) 1 << 7)
#define I2C_SLAVE_OAR2_ENABLE													1
#define I2C_SLAVE_OAR2_DISABLE												0

//Bit definitions for SLAVE ACK CONTROL

#define I2C_SLAVE_ACK_CONTROL_REG_OVERRIDE_VALUE			((uint32_t) 1 << 1)
#define I2C_SLAVE_ACK_VALUE														0
#define I2C_SLAVE_NACK_VALUE													1

#define I2C_SLAVE_ACK_CONTROL_REG_OVERRIDE_ENABLE			((uint32_t) 1 << 0)
#define I2C_SLAVE_ACK_ENABLE													1
#define I2C_SLAVE_ACK_DISABLE													0

//Bit definitions for I2C PERIPHERAL PROPERTIES

#define I2C_PERIPHERAL_PROPERTIES_REG_HIGH_SPEED				((uint32_t) 1 << 0)
#define I2C_PERIPHERAL_PROPERTY_HIGH_SPEED_CAPABLE			1
#define I2C_PERIPHERAL_PROPERTY_HIGH_SPEED_NOT_CAPABLE	0

//Bit definitions for I2C PERIPHERAL CONFIGURATION
#define I2C_PERIPHERAL_CONFIG_REG_HIGH_SPEED			((uint32_t) 1 << 0)
#define I2C_PERIPHERAL_HIGH_SPEED_ENABLE					1
#define I2C_PERIPHERAL_HIGH_SPEED_DISABLE					0


//Redefintion of I2C peripheral structures and useful macros

#define I2C_0 I2C0
#define I2C_1 I2C1
#define I2C_2 I2C2
#define I2C_3	I2C3

#define I2C_HIGH_SPEED_OPERATION 				1
#define I2C_NON_HIGH_SPEED_OPERATION	  0
//I2C Peripheral Clock Control Macros

#define __HAL_RCC_I2C0_CLK_ENABLE				(SYSCTL->RCGCI2C |= (1 << 0))
#define __HAL_RCC_I2C0_CLK_DISABLE			(SYSCTL->RCGCI2C &=  ~(1 << 0))
#define __HAL_RCC_I2C1_CLK_ENABLE				(SYSCTL->RCGCI2C |= (1 << 1))
#define __HAL_RCC_I2C1_CLK_DISABLE			(SYSCTL->RCGCI2C &=  ~(1 << 1))
#define __HAL_RCC_I2C2_CLK_ENABLE				(SYSCTL->RCGCI2C |= (1 << 2))
#define __HAL_RCC_I2C2_CLK_DISABLE			(SYSCTL->RCGCI2C &=  ~(1 << 2))
#define __HAL_RCC_I2C3_CLK_ENABLE				(SYSCTL->RCGCI2C |= (1 << 3))
#define __HAL_RCC_I2C3_CLK_DISABLE			(SYSCTL->RCGCI2C &=  ~(1 << 3))

/********************************************************************************************************/
/*																					I2C Data Structures
**
********************************************************************************************************/

/**
*	@brief: I2C State enumeration
*/

typedef enum {
	
	HAL_I2C_STATE_RESET,		//i2c not yet initialized or disabled
	HAL_I2C_STATE_READY,		//i2c initialized and ready
	HAL_I2C_STATE_BUSY,			//i2c internal process ongoing
	HAL_I2C_STATE_BUSY_TX,	//data trasmission in progress
	HAL_I2C_STATE_BUSY_RX,	//data reception in progress
	HAL_I2C_STATE_ERROR,		//i2c error occurred
	HAL_I2C_STATE_ERROR_ADDR,// Address was not acknowledged
	HAL_I2C_STATE_ERROR_DATA, // Data transmitted was not acknowledged
	HAL_I2C_STATE_ERROR_ARBLOST //Arbitration lost during reception of data
} hal_i2c_state_t;

/**
*	@brief: I2C initialization structure
*/

typedef struct {
		
	uint8_t ownAddress : 7;  		//defines the primary slave address of the peripheral
	
	uint8_t ownAddress2 : 7;		//defines the alternate slave address of the peripheral
	
	uint8_t sclPeriod : 7;	//defines the number of system clock cycles in a SCL cycle
	
	unsigned char slaveAckValue : 1;		// determines whether an ACK is sent on valid data or NACK is sent on invalid data
	
	unsigned char slaveAckEnable : 1;		// determines whether any ACK or NACK is sent by the slave
	
	unsigned char altAddressEnable : 1;
	
	unsigned char speed : 1;	// determines whether the peripheral operates in High Speed mode or Standard/Fast/Fast Plus mode 
	
	uint8_t clockTimeOutLimit;					//the time limit for which the clock can be kept low
	
} i2c_init_t;


/**
*	@brief: I2C handle structure
*/

typedef struct {
	
	I2C0_Type*  						instance;			//Pointer to structure containing I2C register addresses
	
	i2c_init_t 							init;					//I2C initialization structure
	
	hal_i2c_state_t					state;				//Current state of I2C peripheral
	
	uint8_t*								pBuffPtr;		//I2C data buffer
	
	uint32_t								transferSize;	//I2C data size to be sent or recieved
	
	uint32_t								transferCount;	//data transfer counter
		
} i2c_handle_t;

//Helper Macros
#define RESET 	0
#define SET  		!RESET


/******************************************************************************************************/
/*																			Driver exposed APIs
**
******************************************************************************************************/

/**
*	@brief: Initializes the I2C peripheral
*	@param *handle: handle structure of the I2C peripheral that needs to be intialized
*	@retval: none
*/
void hal_i2c_init(i2c_handle_t* handle);

/**
*	@brief: Master data transmission API
*	@param *handle: handle structure of the I2C peripheral
* @param slaveAddress: address to be commuincated with
*	@param *buffer: pointer to the tx buffer
*	@param len: size of data to be transferred
*	@retval: none
*/
int hal_i2c_master_tx_poll(i2c_handle_t* handle, uint8_t slaveAddress, uint8_t* buffer, uint32_t len);

/**
*	@brief: Master data reception API
*	@param *handle: handle structure of the I2C peripheral
* @param slaveAddress: address to be commuincated with
*	@param *buffer: pointer to the tx buffer
*	@param len: size of data to be transferred
*	@retval: none
*/
int hal_i2c_master_rx_poll(i2c_handle_t* handle, uint8_t slaveAddress, uint8_t* buffer, uint32_t len);

/**
*	@brief: Slave data transmission API
*	@param *handle: handle structure of the I2C peripheral
*	@param *buffer: pointer to the rx buffer
*	@param len: size of data to be recieved
*	@retval: none
*/
void hal_i2c_slave_tx_poll(i2c_handle_t* handle, uint8_t* buffer, uint32_t len);

/**
*	@brief: Slave data reception API
*	@param *handle: handle structure of the I2C peripheral
*	@param *buffer: pointer to the rx buffer
*	@param len: size of data to be transferred
*	@retval: none
*/
void hal_i2c_slave_rx_poll(i2c_handle_t* handle, uint8_t* buffer, uint32_t len);


//Interrupt handlers are currently not implemented. 
//These will be updated to be used with Rx and Tx for master and slave through interrupts as well as error detection
/**
*	@brief: Handles I2C master interrupts	
*	@param: *handle: handle structure of the I2C peripheral
*	@retval: none
*/
void hal_i2c_master_interrupt_handler(i2c_handle_t* handle);

/**
*	@brief: Handles I2C slave interrupts	
*	@param: *handle: handle structure of the I2C peripheral
*	@retval: none
*/
void hal_i2c_slave_interrupt_handler(i2c_handle_t* handle);

#endif // __HAL_I2C_DRIVER_H
