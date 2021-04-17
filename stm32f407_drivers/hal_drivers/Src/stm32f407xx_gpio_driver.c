/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: 12-Apr-2021
 *      Author: rajkumar
 */

#include <stm32f407xx_gpio_driver.h>
/*
 *	Peripheral clock setup
 *
 */

/****************************************************************************
 *
 * @fn				- GPIO_PeriClockControl
 *
 * @brief			- This function enables/disable clock for the port
 *
 * @param[in]		- Base address of the gpio peripheral
 * @param[in]		- ENABLE / DISABLE
 *
 * @return			- None
 *
 * @Note			- None
 *
 ****************************************************************************/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}else if(pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}else if(pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}else if(pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		}else if(pGPIOx == GPIOG){
			GPIOG_PCLK_EN();
		}else if(pGPIOx == GPIOH){
			GPIOH_PCLK_EN();
		}else if(pGPIOx == GPIOI){
			GPIOI_PCLK_EN();
		}
	} else {
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		}else if(pGPIOx == GPIOB){
			GPIOB_PCLK_DI();
		}else if(pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		}else if(pGPIOx == GPIOD){
			GPIOD_PCLK_DI();
		}else if(pGPIOx == GPIOE){
			GPIOE_PCLK_DI();
		}else if(pGPIOx == GPIOF){
			GPIOF_PCLK_DI();
		}else if(pGPIOx == GPIOG){
			GPIOG_PCLK_DI();
		}else if(pGPIOx == GPIOH){
			GPIOH_PCLK_DI();
		}else if(pGPIOx == GPIOI){
			GPIOI_PCLK_DI();
		}
	}
}

/*
 * Initialize or Deinitialize GPIO
 */

/****************************************************************************
 *
 * @fn				-
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-
 *
 ****************************************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
	uint32_t temp=0;

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// the non interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER |= temp;
	}else{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_FT){
			EXTI->FTSR |= (0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_RT){
			//config RTSR
			EXTI->RTSR |= (0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_RFT){
			//config FTSR and RTSR
			EXTI->RTSR |= (0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		//configure GPIO port selection in SYSCFG_EXTICR
		SYSCNFG_PCLK_EN();

		uint8_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/4;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%4;

		uint8_t portCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG->EXTICR[temp1] |=  portCode << (4 * temp2);


		//enable the exti interrupt delivery using IMR
		EXTI->IMR |= (0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	//2. configure the speed
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	//3.PUPD configurations
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUUPDR |= temp;

	//4.OP type configurations
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOpType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OPTYPE &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OPTYPE |= temp;

	//5.ALTFUN mode
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode == GPIO_MODE_ALTFN){
		uint8_t reg_type	=	(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)/8;
		uint8_t mod_val  	= 	(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) % 8;
		pGPIOHandle->pGPIOx->AFR[reg_type] &= ~(0xf << (4 * mod_val));
		pGPIOHandle->pGPIOx->AFR[reg_type] = pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * mod_val);
	}

	return;
}

/****************************************************************************
 *
 * @fn				-
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-
 *
 ****************************************************************************/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)/* peripheral reset resistor in RCC will help */
{
	if(pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	}else if(pGPIOx == GPIOB){
		GPIOB_REG_RESET();
	}else if(pGPIOx == GPIOC){
		GPIOC_REG_RESET();
	}else if(pGPIOx == GPIOD){
		GPIOD_REG_RESET();
	}else if(pGPIOx == GPIOE){
		GPIOE_REG_RESET();
	}else if(pGPIOx == GPIOF){
		GPIOF_REG_RESET();
	}else if(pGPIOx == GPIOG){
		GPIOG_REG_RESET();
	}else if(pGPIOx == GPIOH){
		GPIOH_REG_RESET();
	}else if(pGPIOx == GPIOI){
		GPIOI_REG_RESET();
	}
	return;
}
/*
 * Data read and write GPIO
 */

/****************************************************************************
 *
 * @fn				-
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-
 *
 ****************************************************************************/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	uint8_t value;
	value = (uint8_t)(((pGPIOx->IDR) >> PinNumber) & 0x1);
	return value;
}

/****************************************************************************
 *
 * @fn				-
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-
 *
 ****************************************************************************/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}

/****************************************************************************
 *
 * @fn				-
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-
 *
 ****************************************************************************/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){
	if(Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (0x1 << PinNumber);
	}else{
		pGPIOx->ODR &= ~(0x1 << PinNumber);
	}
	return;
}

/****************************************************************************
 *
 * @fn				-
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-
 *
 ****************************************************************************/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value){
	pGPIOx->ODR =Value;
	return;
}

/****************************************************************************
 *
 * @fn				-
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-
 *
 ****************************************************************************/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	pGPIOx->ODR ^= (0x1 << PinNumber);
	return;
}

/*
 * IRQ configuration and handling
 */

/****************************************************************************
 *
 * @fn				-
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-
 *
 ****************************************************************************/
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority){
	uint8_t iprNum = IRQNumber/4;
	uint8_t ipr_offset = IRQNumber%4;
	*(NVIC_IPR + iprNum * 4) |= (IRQPriority) << ((8 * (ipr_offset)) + (8 - NR_PR_BITS_IMPLEMENTED));
}

/****************************************************************************
 *
 * @fn				-
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-
 *
 ****************************************************************************/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE){
		if(IRQNumber <= 31){
			//ISER0 register
			*NVIC_ISER0 |= (1 << (IRQNumber));
		} else if (IRQNumber >=32 && IRQNumber <=63){
			//ISER1 register
			*NVIC_ISER1 |= 1 << (IRQNumber%32);
		} else if (IRQNumber >=64 && IRQNumber <=95){
			//ISER0 register
			*NVIC_ISER2 |= 1 << (IRQNumber%64);
		}
	}
	if(EnOrDi == DISABLE){
		if(IRQNumber <= 31){
			*NVIC_ICER0	|= 1 << (IRQNumber);
		} else if (IRQNumber >=32 && IRQNumber <=63){
			*NVIC_ICER0	|= 1 << (IRQNumber % 32);
		} else if (IRQNumber >=64 && IRQNumber <=95){
			*NVIC_ICER0	|= 1 << (IRQNumber % 64);
		}
	}

	return;
}

/****************************************************************************
 *
 * @fn				-
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-
 *
 ****************************************************************************/
void GPIO_IRQHandling(uint8_t PinNumber){
	if(EXTI->PR & ( 1 << PinNumber )){
		EXTI->PR &
	}
	return;
}
