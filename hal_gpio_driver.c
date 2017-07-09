#include "hal_gpio_driver.h"


void hal_gpio_init(GPIOA_Type* GPIOx, GPIO_Config_t* gpio_pin_conf) {
	
			GPIOx->DIR |= (gpio_pin_conf->direction << gpio_pin_conf->pin);
			
			GPIOx->AFSEL |= (gpio_pin_conf->function << gpio_pin_conf->pin);
			
			if(gpio_pin_conf->function)
				GPIOx->PCTL |= (gpio_pin_conf->alternate_function << (gpio_pin_conf->pin * 4));
			
			if(gpio_pin_conf->drive_strength == GPIO_PIN_4MA_DRIVE)
				GPIOx->DR4R |= (1 << gpio_pin_conf->pin);
			
			else if(gpio_pin_conf->drive_strength == GPIO_PIN_8MA_DRIVE)
				GPIOx->DR8R |= (1 << gpio_pin_conf->pin);
			
			else
				GPIOx->DR2R |= (1 << gpio_pin_conf->pin);
			
			GPIOx->ODR |= (gpio_pin_conf->open_drain << gpio_pin_conf->pin);
	
			if(gpio_pin_conf->pull_up == GPIO_PIN_PULL_UP) 
				GPIOx->PUR |= (1 << gpio_pin_conf->pin);
			
			else if (gpio_pin_conf->pull_up == GPIO_PIN_PULL_DOWN)
				GPIOx->PDR |= (1 << gpio_pin_conf->pin);
		
			GPIOx->DEN |= (gpio_pin_conf->digital_enable << gpio_pin_conf->pin);
			GPIOx->AMSEL |= (gpio_pin_conf->analog_enable << gpio_pin_conf->pin);			
}

void hal_gpio_write(GPIOA_Type* GPIOx, uint8_t pin, uint8_t val) {
	
	if(val)
		GPIOx->DATA |= ( 1 << pin);
	
	else
		GPIOx->DATA &= ~(1 << pin);
}

uint8_t hal_gpio_read(GPIOA_Type* GPIOx, uint8_t pin) {
	
	uint8_t value;
	
	//Read from the GPIODATA register and shift the value by the pin number
	// to get the appropriate reading
	
	value = ((GPIOx->DATA >> pin) & 0x01);
	
	return value;
	
}

void hal_configure_gpio_interrupt(GPIOA_Type* GPIOx, uint8_t pin, edge_select_t edge) {
	
	if(edge == NO_EDGE_LOW_LEVEL) { 
		GPIOx->IS |= (1 << pin);
		GPIOx->IEV &= ~( 1 << pin);
	}
	
	else if ( edge == NO_EDGE_HIGH_LEVEL) {
			
			GPIOx->IS |= ( 1 << pin);
			GPIOx->IEV |= ( 1 << pin);
	}
	
	else {
		
		GPIOx->IS &= ~1 << pin);
		
		if ( edge == RISING_AND_FALLING_EDGE) 
			GPIOx->IBE |= (1 << pin);
		
		else if (edge == FALLING_EDGE) {
			
			GPIOx->IBE &= ~(1 << pin);
			GPIOx->IEV &= (1 << pin);
		}
		
		else {
			
			GPIOx->IBE &= ~( 1 << pin);
			GPIOx->IEV |= (1 << pin);
		}	
	}
	
}

void hal_enable_gpio_interrupt(GPIOA_Type* GPIOx, uint8_t pin, IRQn_Type irq) {
	
	GPIOx->IM |= ( 1 << pin);
	__NVIC_EnableIRQ(irq);
}

void hal_disable_gpio_interrupt(GPIOA_Type* GPIOx, uint8_t pin, IRQn_Type irq) {
	
	GPIOx->IM &= ~( 1 << pin);
	__NVIC_DisableIRQ(irq);
}

void hal_clear_gpio_interrupt(GPIOA_Type* GPIOx, uint8_t pin) {
	
	
	
}