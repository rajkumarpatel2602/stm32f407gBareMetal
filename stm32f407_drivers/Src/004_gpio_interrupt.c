/*
 * 004_gpio_interrupt.c
 *
 *  Created on: 17-Apr-2021
 *      Author: rajkumar
 */

void main(void){

	return;
}

void EXTI0_IRQHandler(void){
	//handle the interrupt
	GPIO_IRQHandling(0);
}
