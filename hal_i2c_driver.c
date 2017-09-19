#include "hal_i2c_driver.h"

//static function prototypes
static uint8_t i2c_master_read_last_poll(i2c_handle_t* handle);
static uint8_t i2c_master_write_last_poll(i2c_handle_t* handle);
static void i2c_slave_write_single_poll(i2c_handle_t* handle);
static void i2c_slave_read_single_poll(i2c_handle_t* handle);
static uint8_t i2c_master_error_handler(i2c_handle_t* handle);

//Function definitions
//i2c initialization function
void hal_i2c_init(i2c_handle_t* handle) {
	
		handle->instance->SOAR |= (uint32_t)(handle->init.ownAddress);
		handle->instance->SOAR2 |= ((uint32_t)(handle->init.ownAddress2)) | ((uint32_t)(handle->init.altAddressEnable << 7));
		handle->instance->MTPR |= ((uint32_t)(handle->init.sclPeriod));
	
		if(handle->instance->PP) {
			handle->instance->MTPR |= ((uint32_t)(handle->init.speed << 7));
			handle->instance->MCS |= ((uint32_t)(handle->init.speed << 4));
			handle->instance->PC |= (uint32_t)(handle->init.speed);
		}
		
		handle->instance->SACKCTL |= ((uint32_t)(handle->init.slaveAckValue << 1)) | ((uint32_t)(handle->init.slaveAckEnable));
		
		if(handle->init.clockTimeOutLimit < 0x02) handle->init.clockTimeOutLimit = 0x02;
		handle->instance->MCLKOCNT |= (uint32_t)(handle->init.clockTimeOutLimit);
		
		handle->state = HAL_I2C_STATE_READY;
}

//master single and multi-byte data transmission through polling
int hal_i2c_master_tx_poll(i2c_handle_t* handle, uint8_t slaveAddress, uint8_t* buffer, uint32_t len) {
	
	if(len < 1)
		return 0;
	
	handle->pBuffPtr = buffer; //set up the pointers, len, count and state of the I2C handle
	handle->transferCount = len;
	handle->transferSize = len;
	handle->state = HAL_I2C_STATE_BUSY_TX;
	
	handle->instance->MSA |= slaveAddress << 1 ;
	handle->instance->MSA &= ~((uint32_t)(I2C_MASTER_RECIEVE)); //set the slave address and write bit

	while(handle->transferCount > 1) {
		
		if(handle->transferCount == len) { //start the transmission of the first byte
			
			handle->instance->MDR |= (uint32_t) *(handle->pBuffPtr++);
			while(handle->instance->MCS & I2C_MASTER_STATUS_REG_BUS_BUSY);
			handle->instance->MCS &= ~(I2C_MASTER_GENERATE_STOP);
			handle->instance->MCS |= I2C_MASTER_GENERATE_START | I2C_MASTER_ENABLE;
		}
		
		while(handle->instance->MCS & I2C_MASTER_STATUS_REG_I2C_BUSY); //wait until operation completes
		
		if(i2c_master_error_handler(handle)) { //handle error in the operation by sending stop bit if arbitration is not lost
			
			if(!handle->instance->MCS & I2C_MASTER_STATUS_REG_ARBITRATION_LOST) {
				handle->instance->MCS |= (uint32_t) I2C_MASTER_GENERATE_STOP;
				handle->instance->MCS &= ~((uint32_t) I2C_MASTER_GENERATE_START | I2C_MASTER_ENABLE);
			}
			return (handle->transferSize - handle->transferCount);
		}
		
		handle->transferCount--;
		
		if(handle->transferCount > 1) { //keep sending bytes without the stop bit until the nth index. 
		handle->instance->MDR |= (uint32_t) *(handle->pBuffPtr++);
		handle->instance->MCS &= ~((uint32_t) I2C_MASTER_GENERATE_START | I2C_MASTER_GENERATE_STOP);
		handle->instance->MCS |= (uint32_t) I2C_MASTER_ENABLE;
		}
	}
	
	handle->transferCount-= i2c_master_write_last_poll(handle); //send the last byte of data
	return (handle->transferSize - handle->transferCount);
}

//master single and multi-byte data reception through polling
int hal_i2c_master_rx_poll(i2c_handle_t* handle, uint8_t slaveAddress, uint8_t* buffer, uint32_t len) {
	
	if(len < 1)
		return 0;
	
	handle->pBuffPtr = buffer; // set buffer pointers, len, count and state of the I2C handle
	handle->transferCount = len;
	handle->transferSize = len;
	handle->state = HAL_I2C_STATE_BUSY_RX;
	
	handle->instance->MSA |= (slaveAddress << 1) | ((uint32_t)(I2C_MASTER_RECIEVE)); //set the address and read bit
	while(handle->instance->MCS & I2C_MASTER_STATUS_REG_BUS_BUSY); //wait until bus is free
	
	while(handle->transferCount > 1) {
		
		if(handle->transferCount == len) { //start of the transfer of the first byte
	
			handle->instance->MCS &= ~(I2C_MASTER_GENERATE_STOP);
			handle->instance->MCS |= I2C_MASTER_GENERATE_START | I2C_MASTER_ENABLE | I2C_MASTER_CONTROL_REG_DATA_ACK_ENABLE;
		}
		
		while(handle->instance->MCS & I2C_MASTER_STATUS_REG_I2C_BUSY); //wait until operation completes
		if(i2c_master_error_handler(handle)) { //handle error by sending stop bit if arbitration is not lost
			
			if(!handle->instance->MCS & I2C_MASTER_STATUS_REG_ARBITRATION_LOST) {
				handle->instance->MCS |= (uint32_t) I2C_MASTER_GENERATE_STOP;
				handle->instance->MCS &= ~((uint32_t) I2C_MASTER_GENERATE_START | I2C_MASTER_ENABLE);
			}
			return (handle->transferSize - handle->transferCount);
		}
		
		handle->transferCount--;
		*(handle->pBuffPtr++) = handle->instance->MDR; //read data if no errors occurred
		if(handle->transferCount > 1) { //only keep sending acknowledge bytes until the n-1th index
		handle->instance->MCS &= ~((uint32_t) I2C_MASTER_GENERATE_START | I2C_MASTER_GENERATE_STOP);
		handle->instance->MCS |= (uint32_t) (I2C_MASTER_ENABLE | I2C_MASTER_CONTROL_REG_DATA_ACK_ENABLE);
		}
	}
	
	handle->transferCount-= i2c_master_read_last_poll(handle); // read last byte of data
	return (handle->transferSize - handle->transferCount);
	
}

//slave single and multi-byte data transmission through polling
void hal_i2c_slave_tx_poll(i2c_handle_t* handle, uint8_t* buffer, uint32_t len) {
	
	handle->pBuffPtr = buffer; //set buffer pointers, len, count and state of the peripheral structure
	handle->transferSize = len;
	handle->transferCount = len;
	handle->state = HAL_I2C_STATE_BUSY_TX;
	
	handle->instance->MCR |= I2C_MASTER_CONFIG_REG_SLAVE_FUNCTION_ENABLE;	//enable slave functionality and disable master functionality
	handle->instance->MCR &= ~(I2C_MASTER_CONFIG_REG_MASTER_FUNCTION_ENABLE);
	while(handle->transferCount-- > 0) {	//while there are bytes to send, keep send single bytes and incrementing the buffer pointer
	
		i2c_slave_write_single_poll(handle);
		handle->pBuffPtr++;
	}
	i2c_slave_write_single_poll(handle);	//send the last byte of data
	handle->state = HAL_I2C_STATE_READY;
	
}

//slave single and mult-byte data reception through polling
void hal_i2c_slave_rx_poll(i2c_handle_t* handle, uint8_t* buffer, uint32_t len) {
	
	handle->pBuffPtr = buffer; //set buffer pointers, len, count and state of the peripheral structure
	handle->transferSize = len;
	handle->transferCount = len;
	handle->state = HAL_I2C_STATE_BUSY_RX;
	
	handle->instance->MCR |= I2C_MASTER_CONFIG_REG_SLAVE_FUNCTION_ENABLE;   //enable slave functionality and disable master functionality
	handle->instance->MCR &= ~(I2C_MASTER_CONFIG_REG_MASTER_FUNCTION_ENABLE);
	
	while(handle->transferCount-- > 0) {	//while there are bytes to recieve, keep reading single bytes and incrementing the buffer pointer
	
		i2c_slave_read_single_poll(handle);
		handle->pBuffPtr++;
	}
	i2c_slave_read_single_poll(handle);	//read the last byte
	handle->state = HAL_I2C_STATE_READY; //set state to ready after last byte has been read
}

//master final byte write procedure
static uint8_t i2c_master_write_last_poll(i2c_handle_t* handle) {

	handle->instance->MDR |= (uint32_t)handle->pBuffPtr;
	
	if(handle->transferSize > 1) { 
			
		handle->instance->MCS &= ~(I2C_MASTER_GENERATE_START); //start last trasnmission
		handle->instance->MCS |= (I2C_MASTER_GENERATE_STOP | I2C_MASTER_ENABLE);
	}
	
	else {
				while(handle->instance->MCS & I2C_MASTER_STATUS_REG_BUS_BUSY); //wait while bus is busy
				handle->instance->MCS |= (uint32_t) I2C_MASTER_GENERATE_START | I2C_MASTER_GENERATE_STOP | I2C_MASTER_ENABLE; //start the operation
	}	
		
	while(handle->instance->MCS & I2C_MASTER_STATUS_REG_I2C_BUSY); //wait while I2C is busy

	if(i2c_master_error_handler(handle))	
				return 0;

	handle->state = HAL_I2C_STATE_READY;
	return 1;
}

static uint8_t i2c_master_read_last_poll(i2c_handle_t* handle) {
	
	if(handle->transferSize > 1) {
		
		handle->instance->MCS &= ~(I2C_MASTER_GENERATE_START | I2C_MASTER_CONTROL_REG_DATA_ACK_ENABLE); // start last transfer
		handle->instance->MCS |= (I2C_MASTER_GENERATE_STOP | I2C_MASTER_ENABLE);
	}
		
	else {
				handle->instance->MCS &= ~(I2C_MASTER_CONTROL_REG_DATA_ACK_ENABLE);
				handle->instance->MCS |= (uint32_t) I2C_MASTER_GENERATE_START | I2C_MASTER_GENERATE_STOP | I2C_MASTER_ENABLE; //start the operation
	}	
	
	while(handle->instance->MCS & I2C_MASTER_STATUS_REG_I2C_BUSY); // wait until operation is complete
	
			if(i2c_master_error_handler(handle)) //check if error has occured
			return 0;
	
	*(handle->pBuffPtr) = handle->instance->MDR;	//if no error has occured, read the recieved data		
		handle->state = HAL_I2C_STATE_READY;
	return 1;
}

//slave single byte write procedure
static void i2c_slave_write_single_poll(i2c_handle_t* handle) {
	
	if(!handle->instance->SCSR & I2C_SLAVE_CONTROL_REG_DEVICE_ACTIVE) // if slave is inactive, activate it
		handle->instance->SCSR |= I2C_SLAVE_ACTIVATE;
	while(!handle->instance->SCSR & I2C_SLAVE_STATUS_REG_TRANSMIT_REQUEST); //wait for transmit request
	handle->instance->SDR |= (uint32_t) *(handle->pBuffPtr);	//write data to be transmitted
}

//slave single byte read procedure
static void i2c_slave_read_single_poll(i2c_handle_t* handle) {
	
	if(!handle->instance->SCSR & I2C_SLAVE_CONTROL_REG_DEVICE_ACTIVE)	//if slave is inactive, activate it
		handle->instance->SCSR |= I2C_SLAVE_ACTIVATE;
	while(!handle->instance->SCSR & I2C_SLAVE_STATUS_REG_RECIEVE_REQUEST);	//wait for recieve request
	*(handle->pBuffPtr) = handle->instance->SDR;	//read the recived data
}

//error handler for master
static uint8_t i2c_master_error_handler(i2c_handle_t* handle) {
	
	if(handle->instance->MCS & I2C_MASTER_STATUS_REG_ERROR) { //check if error has occured 
			
			if(handle->instance->MCS & I2C_MASTER_STATUS_REG_ACK_ADDRESS)	//if error is due to address acknowledgement set the state to indicate this
				handle->state = HAL_I2C_STATE_ERROR_ADDR;	
			
			else if (handle->instance->MCS & I2C_MASTER_STATUS_REG_ACK_DATA) 
				handle->state = HAL_I2C_STATE_ERROR_DATA; //if error is due to data acknowledgement set the state to indicate this
			
			else		//otherwise set a generic error state. This could mean both kinds of error have occurred 
				handle->state = HAL_I2C_STATE_ERROR; 
			
			return 1;
}
	return 0;
}
