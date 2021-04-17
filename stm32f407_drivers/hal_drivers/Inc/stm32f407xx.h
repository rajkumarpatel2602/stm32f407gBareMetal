/*
 * stm32f407xx.h
 *
 *  Created on: 12-Apr-2021
 *      Author: Rajkumar M Patel
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include<stdint.h>
#define	__vo volatile
#define NR_PR_BITS_IMPLEMENTED	4

/*
 * Processor specific macros are here
 */

#define NVIC_ISER0			(__vo uint32_t *)0xE000E100
#define NVIC_ISER1			(__vo uint32_t *)0xE000E104
#define NVIC_ISER2			(__vo uint32_t *)0xE000E108
#define NVIC_ISER3			(__vo uint32_t *)0xE000E10C

#define NVIC_ICER0			(__vo uint32_t *)0XE000E180
#define NVIC_ICER1			(__vo uint32_t *)0XE000E184
#define NVIC_ICER2			(__vo uint32_t *)0XE000E188
#define NVIC_ICER3			(__vo uint32_t *)0XE000E18C

#define NVIC_IPR			(__vo uint32_t *)0xE000E400

/*
 * IRQ Number of stm32f407x MCU and EXTI line mapping
 */

#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40

/*
 * Base address of flash and SRAM memories
 * HAL_ prefix can also be used.
 */
#define FLASH_BASEADDR		0x08000000U /* Base address of main memory */
#define SRAM1_BASEADDR		0x20000000U /* Base address of sram1 */
#define SRAM2_BASEADDR		SRAM1_BASEADDR + 0x0001C000U /* base address of sram2 */
#define ROM					0x1FFF0000U /* system memory base address */
#define OTP_AREA			0x1FFF 7800 /* OTP bank base address */
#define SRAM 				SRAM1_BASEADDR /* SRAM base address */

/*
 *	Base address of different bus domains
 */

#define PERIPH_BASE 			0x40000000U /* peripheral base address */
#define APB1PERIPH_BASEADDR		PERIPH_BASE /* apb1 base address */
#define APB2PERIPH_BASEADDR		0x40010000U /* apb2 base address */
#define AHB1PERIPH_BASEADDR		0x40020000U /* ahb1 base address */
#define AHB2PERIPH_BASEADDR		0x50000000U /* ahb2 base address */

/*
 * Defining base addresses of various peripherals on ahb1
 */

#define GPIOA_BASEADDR		(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR		(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR		(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR		(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR		(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR		(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR		(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR		(AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR		(AHB1PERIPH_BASEADDR + 0x2000)

#define RCC_BASEADDR		(AHB1PERIPH_BASEADDR + 0x3800)
/*
 * Defining base addresses of various peripherals on apb1
 */

#define I2C1_BASEADDR		(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR		(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR		(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR		(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR		(APB1PERIPH_BASEADDR + 0x3C00)

#define UART4_BASEADDR		(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR		(APB1PERIPH_BASEADDR + 0x5000)

#define USART2_BASEADDR		(APB1PERIPH_BASEADDR + 0x4400)
#define USAER3_BASEADDR		(APB1PERIPH_BASEADDR + 0x4800)

/*
 * Defining base addresses of various peripherals on apb2
 */

#define EXTI_BASEADDR		(APB2PERIPH_BASEADDR + 0x3C00)

#define SPI1_BASEADDR		(APB2PERIPH_BASEADDR + 0x3000)

#define SYSCFG_BASEADDR		(APB2PERIPH_BASEADDR + 0x3800)

#define USART1_BASEADDR		(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR		(APB2PERIPH_BASEADDR + 0x1400)

/*
 * GPIO Peripheral register definition structures
 */

typedef struct {
	__vo uint32_t MODER;
	__vo uint32_t OPTYPE;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BdRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2]; /* AFR[0]=LOW REG and AFR[1] = HIGH REG */
}GPIO_RegDef_t;

/*
 * RCC Peripheral register definition structures
 */

typedef struct{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	__vo uint32_t RESERVED0;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	__vo uint32_t RESERVED1[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	__vo uint32_t RESERVED2;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	__vo uint32_t RESERVED3[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	__vo uint32_t RESERVED4;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	__vo uint32_t RESERVED5[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
}RCC_RegDef_t;

/*
 * EXTI Peripheral register definition structures
 */

typedef struct{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
}EXTI_RegDef_t;


/*
 * SYSCFG peripheral register definition structures //TBD
 */
typedef struct{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	uint32_t RESERVED1[2];
	__vo uint32_t CMPCR;
}SYSCFG_RegDef_t;

/*
 * peripheral definitions (peripheral base addresses type casted to xxxx_RegDef_t)
 */

#define GPIOA			((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB			((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC			((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD			((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE			((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF			((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG			((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH			((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI			((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC				((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI			((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG			((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

/*
 * PORTCODE generator
 */

#define GPIO_BASEADDR_TO_CODE(x)	((x == GPIOA)? 0x00 	 \
									:(x == GPIOB) ? 0x01 \
									:(x == GPIOC) ? 0x02 \
									:(x == GPIOD) ? 0x03 \
                                    :(x == GPIOE) ? 0x04 \
                                    :(x == GPIOF) ? 0x05 \
                                    :(x == GPIOG) ? 0x06 \
                                    :(x == GPIOH) ? 0x07 : 0x08)
/*
 * Clock Enable Macros for GPIOx peripherals
 */

/* AHB1PERI clk enable */
#define GPIOA_PCLK_EN()		RCC->AHB1ENR |= ( 1 << 0 )
#define GPIOB_PCLK_EN()		RCC->AHB1ENR |= ( 1 << 1 )
#define GPIOC_PCLK_EN()		RCC->AHB1ENR |= ( 1 << 2 )
#define GPIOD_PCLK_EN()		RCC->AHB1ENR |= ( 1 << 3 )
#define GPIOE_PCLK_EN()		RCC->AHB1ENR |= ( 1 << 4 )
#define GPIOF_PCLK_EN()		RCC->AHB1ENR |= ( 1 << 5 )
#define GPIOG_PCLK_EN()		RCC->AHB1ENR |= ( 1 << 6 )
#define GPIOH_PCLK_EN()		RCC->AHB1ENR |= ( 1 << 7 )
#define GPIOI_PCLK_EN()		RCC->AHB1ENR |= ( 1 << 8 )

/* APB1PERI clk enable */
#define I2C1_PCLK_EN()			RCC->APB1ENR |= ( 1 << 21 )
#define I2C2_PCLK_EN()			RCC->APB1ENR |= ( 1 << 22 )
#define I2C3_PCLK_EN()			RCC->APB1ENR |= ( 1 << 23 )
#define UART4_PCLK_EN()			RCC->APB1ENR |= ( 1 << 19 )
#define UART5_PCLK_EN()			RCC->APB1ENR |= ( 1 << 20 )
#define USART2_PCLK_EN()		RCC->APB1ENR |= ( 1 << 18 )
#define USART3_PCLK_EN()		RCC->APB1ENR |= ( 1 << 17 )
#define SPI2_PCLK_EN()			RCC->APB1ENR |= ( 1 << 14 )
#define SPI3_PCLK_EN()			RCC->APB1ENR |= ( 1 << 15 )

/* APB2PERI clk enable */
#define SPI1_PCLK_EN()			RCC->APB2ENR |= ( 1 << 12 )
#define SYSCNFG_PCLK_EN() 		RCC->APB2ENR |= ( 1 << 14 )

#define USART1_PCLK_EN()		RCC->APB2ENR |= ( 1 << 1 )
#define USART6_PCLK_EN()		RCC->APB2ENR |= ( 1 << 5 )

/*
 * Disable clock for all peripheral
 */

/* AHB1PERI clk disable */
#define GPIOA_PCLK_DI()			RCC->AHB1ENR &= ~( 1 << 0 )
#define GPIOB_PCLK_DI()			RCC->AHB1ENR &= ~( 1 << 1 )
#define GPIOC_PCLK_DI()			RCC->AHB1ENR &= ~( 1 << 2 )
#define GPIOD_PCLK_DI()			RCC->AHB1ENR &= ~( 1 << 3 )
#define GPIOE_PCLK_DI()			RCC->AHB1ENR &= ~( 1 << 4 )
#define GPIOF_PCLK_DI()			RCC->AHB1ENR &= ~( 1 << 5 )
#define GPIOG_PCLK_DI()			RCC->AHB1ENR &= ~( 1 << 6 )
#define GPIOH_PCLK_DI()			RCC->AHB1ENR &= ~( 1 << 7 )
#define GPIOI_PCLK_DI()			RCC->AHB1ENR &= ~( 1 << 8 )

/* APB1PERI clk disable */
#define I2C1_PCLK_DI()			RCC->APB1ENR &= ~( 1 << 21 )
#define I2C2_PCLK_DI()			RCC->APB1ENR &= ~( 1 << 22 )
#define I2C3_PCLK_DI()			RCC->APB1ENR &= ~( 1 << 23 )
#define UART4_PCLK_DI()			RCC->APB1ENR &= ~( 1 << 19 )
#define UART5_PCLK_DI()			RCC->APB1ENR &= ~( 1 << 20 )
#define USART2_PCLK_DI()		RCC->APB1ENR &= ~( 1 << 18 )
#define USART3_PCLK_DI()		RCC->APB1ENR &= ~( 1 << 17 )
#define SPI2_PCLK_DI()			RCC->APB1ENR &= ~( 1 << 14 )
#define SPI3_PCLK_DI()			RCC->APB1ENR &= ~( 1 << 15 )

/* APB2PERI clk disable */
#define SPI1_PCLK_DI()			RCC->APB2ENR &= ~( 1 << 12 )
#define SYSCNFG_PCLK_DI() 		RCC->APB2ENR &= ~( 1 << 14 )

#define USART1_PCLK_DI()		RCC->APB2ENR &= ~( 1 << 1 )
#define USART6_PCLK_DI()		RCC->APB2ENR &= ~( 1 << 5 )

/* Reset Peripherals */

#define GPIOA_REG_RESET()		do{(RCC->AHB1RSTR |= 1 << 0x0); (RCC->AHB1RSTR &= ~(1 << 0x0));}while(0)
#define GPIOB_REG_RESET()		do{(RCC->AHB1RSTR |= 1 << 0x1); (RCC->AHB1RSTR &= ~(1 << 0x1));}while(0)
#define GPIOC_REG_RESET()		do{(RCC->AHB1RSTR |= 1 << 0x2); (RCC->AHB1RSTR &= ~(1 << 0x2));}while(0)
#define GPIOD_REG_RESET()		do{(RCC->AHB1RSTR |= 1 << 0x3); (RCC->AHB1RSTR &= ~(1 << 0x3));}while(0)
#define GPIOE_REG_RESET()		do{(RCC->AHB1RSTR |= 1 << 0x4); (RCC->AHB1RSTR &= ~(1 << 0x4));}while(0)
#define GPIOF_REG_RESET()		do{(RCC->AHB1RSTR |= 1 << 0x5); (RCC->AHB1RSTR &= ~(1 << 0x5));}while(0)
#define GPIOG_REG_RESET()		do{(RCC->AHB1RSTR |= 1 << 0x6); (RCC->AHB1RSTR &= ~(1 << 0x6));}while(0)
#define GPIOH_REG_RESET()		do{(RCC->AHB1RSTR |= 1 << 0x7); (RCC->AHB1RSTR &= ~(1 << 0x7));}while(0)
#define GPIOI_REG_RESET()		do{(RCC->AHB1RSTR |= 1 << 0x8); (RCC->AHB1RSTR &= ~(1 << 0x8));}while(0)


/* Generic Macros */
#define ENABLE 			1
#define DISABLE 		0
#define SET 			ENABLE
#define RESET 			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET
#endif /* INC_STM32F407XX_H_ */
