#ifndef __HAL_GPIO_DRIVER_H
#define __HAL_GPIO_DRIVER_H

//Include the MCU specific header file 
#include "tm4c123gh6pm.h"

/**********************************************
		
	1. Macros used for GPIO pin Initialization

***********************************************/
// GPIO pin direction settings:
#define GPIO_PIN_INPUT_DIR					((uint32_t)0x00)
#define GPIO_PIN_OUTPUT_DIR				  ((uint32_t)0x01)

// GPIO pin mode settings:
#define GPIO_PIN_ALT_FUNC					  ((uin32_t) 0x00)
#define GPIO_PIN_NORMAL_FUNC			  ((uint32_t) 0x01)

//GPIO Output type settings:
#define GPIO_PIN_PULL_DOWN		((uint32_t)0x01)
#define GPIO_PIN_OPEN_DRAIN		((uint32_t)0x00)
#define GPIO_PIN_PULL_UP			((uint32_t)0x01)


//GPIO Digital settings:
#define GPIO_PIN_DIGITAL_ENABLE			((uint32_t)0x01)
#define GPIO_PIN_DIGITAL_DISABLE    ((uint32_t)0x00)

//GPIO Analog settings:

#define GPIO_PIN_ANALOG_ENABLE			((uint32_t)0x01)
#define GPIO_PIN_ANALOG_DISABLE  	  ((uint32_t)0x00)

//GPIO Drive Strength settings:
#define GPIO_PIN_2MA_DRIVE					((uint32_t) 0x00)
#define GPIO_PIN_4MA_DRIVE					((uint32_t) 0x01)
#define GPIO_PIN_8MA_DRIVE					((uint32_t) 0x02)


//Redefinition of GPIO APB Ports
#define GPIO_PORT_A 		GPIOA
#define GPIO_PORT_B 		GPIOB
#define GPIO_PORT_C 		GPIOC
#define GPIO_PORT_D 		GPIOD
#define GPIO_PORT_E 		GPIOE
#define GPIO_PORT_F 		GPIOF

//Redefinition of GPIO AHB Ports
#define GPIO_PORT_A_AHB 	GPIOA_AHB
#define GPIO_PORT_B_AHB 	GPIOB_AHB
#define GPIO_PORT_C_AHB 	GPIOC_AHB
#define GPIO_PORT_D_AHB 	GPIOD_AHB
#define GPIO_PORT_E_AHB 	GPIOE_AHB
#define GPIO_PORT_F_AHB 	GPIOF_AHB


/*********************************************

	2. Macros used to enable/disable GPIO ClK


**********************************************/
#define __HAL_RCC_GPIOA_CLK_ENABLE()		(SYSCTL->RCGCGPIO|= (1 << 0))
#define __HAL_RCC_GPIOA_CLK_DISABLE()		(SYSCTL->RCGCGPIO|= (0 << 0))
#define __HAL_RCC_GPIOB_CLK_ENABLE()		(SYSCTL->RCGCGPIO|= (1 << 1))
#define __HAL_RCC_GPIOB_CLK_DISABLE()		(SYSCTL->RCGCGPIO|= (0 << 1))
#define __HAL_RCC_GPIOC_CLK_DISABLE()		(SYSCTL->RCGCGPIO|= (1 << 2))
#define __HAL_RCC_GPIOC_CLK_ENABLE()		(SYSCTL->RCGCGPIO|= (0 << 2))
#define __HAL_RCC_GPIOD_CLK_DISABLE()		(SYSCTL->RCGCGPIO|= (1 << 3))
#define __HAL_RCC_GPIOD_CLK_ENABLE()		(SYSCTL->RCGCGPIO|= (0 << 3))
#define __HAL_RCC_GPIOE_CLK_DISABLE()		(SYSCTL->RCGCGPIO|= (1 << 4))
#define __HAL_RCC_GPIOE_CLK_ENABLE()		(SYSCTL->RCGCGPIO|= (0 << 4))
#define __HAL_RCC_GPIOF_CLK_DISABLE()		(SYSCTL->RCGCGPIO|= (1 << 5))
#define __HAL_RCC_GPIOF_CLK_ENABLE()		(SYSCTL->RCGCGPIO|= (0 << 5))


/***********************************************

	3. Data structure for configuring GPIO 
	
************************************************/

typedef struct {
	
	uint32_t pin;				// holds the pin number of the gpio to be configured
	
	uint32_t function;	// holds the configuration value to choose between GPIO and Alternate function modes
	
	uint32_t  direction;	// determines whether pin is input or output
		
	uint32_t  pull_up;  	// determines whether pull-up or pull down resistor in attached or not
	
	uint32_t open_drain;	// determines if the pin is in open drain configuration or not
	
	uint32_t alternate_function;	//determines the alternate function of the gpio pin if function is set to alternate function.
	
	uint32_t digital;				// determines whether the pin is digital or analog
	
	uint32_t drive_strength; // determines the drive strength (2ma,4ma or 8ma) of the gpio pin
	
}	GPIO_Config_t ;



/**********************************************************

	4. Driver exposed APIS

**********************************************************/
/**
*	@description: Initializes the gpio pin(s)
*	@param: *GPIOx: GPIO Port Address 
*	@param: *gpio_pin_conf: Pointer to structure containing configuration details
*	@retval: none
*/
void hal_gpio_init(GPIOA_Type* GPIOx, GPIO_Config_t* gpio_pin_conf);


/**
*	@description: Initializes the gpio pin(s)
*	@param: *GPIOx: GPIO Port Address 
* @param: uint16_t: the GPIO pin number to read from
*	@retval: uint8_t: value read from pin
*/
uint8_t hal_gpio_read(GPIOA_Type* GPIOx, uint16_t pin);


/**
*	@description: Initializes the gpio pin(s)
*	@param: *GPIOx: GPIO Port Address 
*	@param: uint16_t: the GPIO pin number to write to
* @param: uint8_t: the value to write to the GPIO pin
*	@retval: none
*/
void hal_gpio_write(GPIOA_Type* GPIOx, uint16_t pin, uint8_t val);


#endif //__HAL_GPIO_DRIVER_H