#include "hal_gpio_driver.h"


GPIO_Config_t led_config;

int main (void) {
	
	/*int i ;
	__HAL_RCC_GPIOF_CLK_ENABLE();
	led_config.pin = 1;
	led_config.function = 0;
	led_config.direction = 1;
	led_config.open_drain = 0;
	led_config.pull_up = GPIO_PIN_PULL_UP;
	led_config.analog_enable = 0;
	led_config.digital_enable = 1;
	
	hal_gpio_init(GPIO_PORT_F, &led_config);
	
	while(1) {
	
	hal_gpio_write(GPIO_PORT_F, 1, 1);
	
	for( i = 0; i < 100000; i++);
	
	hal_gpio_write(GPIO_PORT_F, 1, 0);
	hal_gpio_write(GPIO_PORT_F, 2 ,1);
		
	for ( i = 0; i < 100000; i++);
		
	hal_gpio_write(GPIO_PORT_F, 2, 0);
	hal_gpio_write(GPIO_PORT_F, 3 , 1);
	
	} */
	
	return 0;
}
