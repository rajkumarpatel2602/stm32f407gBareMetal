/*
 * 002_led_button.c
 *
 * Toggle LED on a button press event.
 *
 *  Created on: 13-Apr-2021
 *      Author: rajkumar
 */

#include <stdint.h>
#include <stm32f407xx.h>
#include <stm32f407xx_gpio_driver.h>

void delay(void){
	for(int32_t i=0; i<250000; i++);//this delay is reasonable delay to avoid key debouncing.
}

int main(void)

{
	GPIO_Handle_t led_GPIO;
	led_GPIO.pGPIOx = GPIOD;
	led_GPIO.GPIO_PinConfig.GPIO_PinMode 		= GPIO_MODE_OUT;
	led_GPIO.GPIO_PinConfig.GPIO_PinNumber		= GPIO_PIN_NO_12;
	led_GPIO.GPIO_PinConfig.GPIO_PinOpType		= GPIO_OP_TYPE_PP;
	led_GPIO.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_NO_PUPD;
	led_GPIO.GPIO_PinConfig.GPIO_PinSpeed		= GPIO_SPEED_FAST;

	GPIO_Handle_t button_GPIO;
	button_GPIO.pGPIOx = GPIOA;
	button_GPIO.GPIO_PinConfig.GPIO_PinMode 		= GPIO_MODE_IN;
	button_GPIO.GPIO_PinConfig.GPIO_PinNumber		= GPIO_PIN_NO_0;
	button_GPIO.GPIO_PinConfig.GPIO_PinSpeed		= GPIO_SPEED_FAST;
	button_GPIO.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_NO_PUPD;// button has already PD avaialble in ckt

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Init(&led_GPIO);
	GPIO_Init(&button_GPIO);

	while(1){
		uint8_t val=0;
		val = GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0);
		if(val)
		{
			delay();
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		}
	}
	return 0;
}
