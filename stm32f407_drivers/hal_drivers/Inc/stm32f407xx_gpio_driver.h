/*
 * stm32f407xx_gpio_driver.h
 *
 * Driver specific header file
 *
 *  Created on: 12-Apr-2021
 *      Author: Rajkumar
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include <stm32f407xx.h> // MCU specific header file

/*
 * This is a Configuration structure for a GPIO pin
 */
typedef struct{
	uint8_t GPIO_PinNumber;		/*!< possible values from @GPIO_PIN_NUMBERS >*/
	uint8_t GPIO_PinMode;		/*!< possible values from @GPIO_PIN_MODES >*/
	uint8_t GPIO_PinSpeed;		/*!< possible values from @GPIO_PIN_SPEED >*/
	uint8_t GPIO_PinPuPdControl;/*!< possible values from @GPIO_PIN_PUPD_CONTROL >*/
	uint8_t GPIO_PinOpType;		/*!< possible values from @GPIO_PIN_OPTYPE >*/
	uint8_t GPIO_PinAltFunMode;	/*!< possible values from @GPIO_PIN_ALT_FUN_MODE >*/
}GPIO_PinConfig_t;

/*
 *  This is the Handle structure for the GPIO pin
 */
typedef struct{
	GPIO_RegDef_t *pGPIOx; /* GPIO port which the pin belong */
	GPIO_PinConfig_t GPIO_PinConfig; /* GPIO pin configuration settings */
}GPIO_Handle_t;

/*
 * @GPIO_PIN_NUMBER
 * GPIO pin numbers
 */

#define GPIO_PIN_NO_0           0x0
#define GPIO_PIN_NO_1           0x1
#define GPIO_PIN_NO_2           0x2
#define GPIO_PIN_NO_3           0x3
#define GPIO_PIN_NO_4           0x4
#define GPIO_PIN_NO_5           0x5
#define GPIO_PIN_NO_6           0x6
#define GPIO_PIN_NO_7           0x7
#define GPIO_PIN_NO_8           0x8
#define GPIO_PIN_NO_9           0x9
#define GPIO_PIN_NO_10          0xa
#define GPIO_PIN_NO_11          0xb
#define GPIO_PIN_NO_12          0xc
#define GPIO_PIN_NO_13          0xd
#define GPIO_PIN_NO_14			0xe
#define GPIO_PIN_NO_15			0xf

/*
 * 	@GPIO_PIN_MODES
 * 	GPIO pin possible modes
 */
#define GPIO_MODE_IN		0x0
#define GPIO_MODE_OUT		0x1
#define GPIO_MODE_ALTFN		0x2
#define GPIO_MODE_ANALOG	0x3
#define GPIO_MODE_IT_FT		0x4
#define GPIO_MODE_IT_RT		0x5
#define GPIO_MODE_IT_RFT	0x6

/*
 * 	@GPIO_PIN_OPTYPE
 * 	GPIO output type macros
 */
#define GPIO_OP_TYPE_PP		0x0
#define GPIO_OP_TYPE_OD		0x1

/*
 * 	@GPIO_PIN_SPEED
 * 	GPIO Speed registration
 */
#define GPIO_SPEED_LOW		0x0
#define GPIO_SPEED_MEDIUM	0x1
#define GPIO_SPEED_FAST		0x2
#define GPIO_SPEED_HIGH		0x3

/*
 * 	@GPIO_PIN_PUPD_CONTROL
 * 	GPIO push-pull resistor mode
 */
#define GPIO_NO_PUPD		0x0
#define GPIO_PIN_PU			0x1
#define GPIO_PIN_PD			0x2

/**********************************************
 * API prototypes
 **********************************************/

/*
 *
 * 	Peripheral clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi);

/*
 * Initialize or Deinitialize GPIO
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);/* peripheral reset resistor in RCC will help */

/*
 * Data read and write GPIO
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ configuration and handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
