/*
 * stm32f446xx.h
 *
 *  Created on: Jul 8, 2022
 *      Author: MarcusT
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_


/*
* ARM Cortex M4 Processor NVIC ISERx register Addresses
*/

#define NVIC_ISER0				(( volatile uint32_t*) 0xE000E100)
#define NVIC_ISER1				(( volatile uint32_t*) 0xE000E104)
#define NVIC_ISER2				(( volatile uint32_t*) 0xE000E108)
#define NVIC_ISER3				(( volatile uint32_t*) 0xE000E10C)

/*
* ARM Cortex M4 Processor NVIC ICERx register Addresses
*/

#define NVIC_ICER0				(( volatile uint32_t*) 0xE000E180)
#define NVIC_ICER1				(( volatile uint32_t*) 0xE000E184)
#define NVIC_ICER2				(( volatile uint32_t*) 0xE000E188)
#define NVIC_ICER3				(( volatile uint32_t*) 0xE000E18C)

/*
* ARM Cortex M4 Processor Priority Register Addresses Calculation
*/

#define NVIC_PR_BASE_ADDR				(( volatile uint32_t*) 0xE000E400)

#define NO_PR_BITS_IMPLEMENTED 			4


/* Base addresses of flash and sram emory */

#define FLASH_BASEADDR                0X08000000U /* (UINT32_T) 0x0800..., SEEK FLASH MEMORY = MAIN MEMORY */
#define SRAM1_BASEADDR                0X20000000U  /* SRAM1 base address */
#define SRAM2_BASEADDR                0x2001C000U  /* SRAM1 base address */
#define ROM_BASEADDR                  0x1FFF0000U  /* = System memory */
#define SRAM                          SRAM1_BASEADDR /* Macro for */

/*
* AHBx and APBx Bus peripheral base addresses
*/

#define PERIPH_BASEADDR               0x40000000U
#define APB1PERIPH_BASEADDR           PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR           0x40010000U               
#define AHB1PERIPH_BASEADDR           0x40020000U
#define AHB2PERIPH_BASEADDR           0x50000000U

/*
* peripheral base addresses which are hanging on AHB1 base
*/

#define GPIOA_BASEADDR                (AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR                (AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR                (AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR                (AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR                (AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR                (AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR                (AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR                (AHB1PERIPH_BASEADDR + 0x1C00)
#define RCC_BASEADDR                  (AHB1PERIPH_BASEADDR + 0x3800)

/*
* peripheral base addresses which are hanging on APB1 base    --- Episódio  81
*/

#define I2C1_BASEADDR                 (APB1PERIPH_BASEADDR + 0x5400) 
#define I2C2_BASEADDR                 (APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR                 (APB1PERIPH_BASEADDR + 0x5C00)
  
#define SPI2_BASEADDR                 (APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR                 (APB1PERIPH_BASEADDR + 0x3C00)
  
#define USART2_BASEADDR               (APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR               (APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR                (APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR                (APB1PERIPH_BASEADDR + 0x5000)

/*
* peripheral base addresses which are hanging on APB2 base
*/

#define SPI1_BASEADDR                 (APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR                 (APB2PERIPH_BASEADDR + 0x3400)
#define USART1_BASEADDR               (APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR               (APB2PERIPH_BASEADDR + 0x1400)
#define EXIT_BASEADDR                 (APB2PERIPH_BASEADDR + 0x3C00)
#define SYSCFG_BASEADDR               (APB2PERIPH_BASEADDR + 0x3800)

/*
* peripheral register definition struct for GPIO
*/
  
typedef struct{
  volatile uint32_t MODER;                    /* GPIO port mode register,               Address offset 0x00 */
  volatile uint32_t OTYPER;                   /* GPIO port output type register,        Address offset 0x04 */
  volatile uint32_t OSPEEDR;                  /* GPIO port output type register, 		Address offset 0x08 */
  volatile uint32_t PUPDR;                    /* GPIO port pull-up/pull-down register,  Address offset 0x0C */
  volatile uint32_t IDR;					  /* GPIO port input data register 			Address offset 0x10 */
  volatile uint32_t ODR;					  /* GPIO port output data register 		Address offset 0x14 */
  volatile uint32_t BSRR;                     /* GPIO port bit set/reset register, 		Address offset 0x18 */
  volatile uint32_t LCKR;                     /* GPIO port configuration lock register, Address offset 0x1C */
  volatile uint32_t AFR [2];                  /* AFR[0]: GPIO alternate function low register, AFR[1]: GPIO alternate function high register, Address offset 0x20 - 0x24 */
}GPIO_RegDef_t;


/*
* peripheral register definition struct for GPIO  . - Ep:85
*/

typedef struct{   
  volatile uint32_t CR;                        /*RCC clock control register,         Address offset 0x00 */
  volatile uint32_t PLLCFGR;                   /*RCC PLL configuration register,     Address offset 0x04 */
  volatile uint32_t CFGR;                      /*RCC clock configuration register,   Address offset 0x08 */
  volatile uint32_t CIR;                       /*RCC clock interrupt register,       Address offset 0x0C */
  volatile uint32_t AHB1RSTR;                  
  volatile uint32_t AHB2RSTR;
  volatile uint32_t AHB3RSTR;
  uint32_t RCC_RESERVED0;                     /*Reserved, 0x1C */
  volatile uint32_t APB1RSTR;
  volatile uint32_t APB2RSTR;
  uint32_t RCC_RESERVED1;
  uint32_t RCC_RESERVED2;
  volatile uint32_t AHB1ENR;
  volatile uint32_t AHB2ENR;
  volatile uint32_t AHB3ENR;
  uint32_t RCC_RESERVED3;
  volatile uint32_t APB1ENR;
  volatile uint32_t APB2ENR;
  uint32_t RCC_RESERVED4;
  uint32_t RCC_RESERVED5;
  volatile uint32_t AHB1LPENR;
  volatile uint32_t AHB2LPENR;
  volatile uint32_t AHB3LPENR;
  uint32_t RCC_RESERVED6;
  volatile uint32_t APB1LPENR;
  volatile uint32_t APB2LPENR;
  uint32_t RCC_RESERVED7;
  uint32_t RCC_RESERVED8;
  volatile uint32_t BDCR;
  volatile uint32_t CSR;
  uint32_t RCC_RESERVED9;
  uint32_t RCC_RESERVED10;
  volatile uint32_t SSCGR;
  volatile uint32_t PLLI2SCFGR;
  volatile uint32_t PLLSAICFGR;
  volatile uint32_t DCKCFGR;
  volatile uint32_t CKGATENR;
  volatile uint32_t DCKCFGR2;  
}RCC_RegDef_t;

typedef struct
{
  volatile uint32_t IMR;                 /*!< Something with EXTI,         Address offset 0x00 */
  volatile uint32_t EMR;                 /*!< Something with EXTI,         Address offset 0x04 */
  volatile uint32_t RTSR;                 /*!< Something with EXTI,         Address offset 0x08 */
  volatile uint32_t FTSR;                 /*!< Something with EXTI,         Address offset 0x0C */
  volatile uint32_t SWIER;                 /*!< Something with EXTI,         Address offset 0x10 */
  volatile uint32_t PR;                 /*!< Something with EXTI,         Address offset 0x14 */    
}EXTI_RegDef_t;


typedef struct
{
  volatile uint32_t MEMRMP;                /*!< Something with EXTI,         Address offset 0x00 */
  volatile uint32_t PMC;                   /*!< Something with EXTI,         Address offset 0x04 */
  volatile uint32_t EXTICR[4];             /*!< Something with EXTI,         Address offset 0x08-0X14 */
  uint32_t RESERVED1[2];                   /*!< Something with SYSCFG,         Address offset 0x1C */
  volatile uint32_t CMPCR;                 /*!< Something with EXTI,         Address offset 0x20 */
  uint32_t RESERVED2[2];                   /*!< Something with SYSCFG,         Address offset 0x28 */
  volatile uint32_t CFGR;                  /*!< Something with EXTI,         Address offset 0x2C */    
}SYSCFG_RegDef_t;



typedef struct
{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t CRCPR;
	volatile uint32_t RXCRCR;
	volatile uint32_t TXCRCR;
	volatile uint32_t I2SCFGR;
	volatile uint32_t I2SPR;
}SPI_RegDef_t;



/*
* peripheral definitions ( Peripheral base addresses typecast to GPIO_RegDef_t)
*/
#define GPIOA                         ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB                         ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC                         ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD                         ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE                         ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF                         ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG                         ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH                         ((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC                           ((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI                          ((EXTI_RegDef_t*)EXIT_BASEADDR)
#define SYSCFG                        ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1						  ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2						  ((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3						  ((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4						  ((SPI_RegDef_t*)SPI4_BASEADDR)


/*
* Clock Enable Macros for GPIOx peripherals
*/
#define GPIOA_PCLK_EN()                RCC->AHB1ENR |= (1 << 0)
#define GPIOB_PCLK_EN()                RCC->AHB1ENR |= (1 << 1)
#define GPIOC_PCLK_EN()                RCC->AHB1ENR |= (1 << 2)
#define GPIOD_PCLK_EN()                RCC->AHB1ENR |= (1 << 3)
#define GPIOE_PCLK_EN()                RCC->AHB1ENR |= (1 << 4)
#define GPIOF_PCLK_EN()                RCC->AHB1ENR |= (1 << 5)
#define GPIOG_PCLK_EN()                RCC->AHB1ENR |= (1 << 6)
#define GPIOH_PCLK_EN()                RCC->AHB1ENR |= (1 << 7)

/*
* Clock Enable Macros for I2CX peripherals
*/
#define I2C1_PCLK_EN()                 RCC->APB1ENR |= (1 << 21)
#define I2C2_PCLK_EN()                 RCC->APB1ENR |= (1 << 22)
#define I2C3_PCLK_EN()                 RCC->APB1ENR |= (1 << 23)

/*
* Clock Enable Macros for ISPx peripherals
*/
#define SPI1_PCLK_EN()                 RCC->APB2ENR |= (1 << 12)
#define SPI2_PCLK_EN()                 RCC->APB1ENR |= (1 << 14)
#define SPI3_PCLK_EN()                 RCC->APB1ENR |= (1 << 15)
#define SPI4_PCLK_EN()                 RCC->APB2ENR |= (1 << 13)

/*
* Clock Enable Macros for USARTx peripherals
*/
#define USART1_PCLK_EN()               RCC->APB2ENR |= (1 << 4)
#define USART2_PCLK_EN()               RCC->APB1ENR |= (1 << 17)
#define USART3_PCLK_EN()               RCC->APB1ENR |= (1 << 18)
#define UART4_PCLK_EN()                RCC->APB1ENR |= (1 << 19)
#define UART5_PCLK_EN()                RCC->APB1ENR |= (1 << 20)
#define USART6_PCLK_EN()               RCC->APB2ENR |= (1 << 5)

/*
* Clock Enable Macros for SYSCFG peripherals
*/
#define SYSCFG_PCLK_EN()               RCC->APB2ENR |= (1 << 14)

/*
* Clock Disable Macros for GPIOx peripherals
*/
#define GPIOA_PCLK_DI()                RCC->AHB1ENR &= ~(1 << 0)
#define GPIOB_PCLK_DI()                RCC->AHB1ENR &= ~(1 << 1)
#define GPIOC_PCLK_DI()                RCC->AHB1ENR &= ~(1 << 2)
#define GPIOD_PCLK_DI()                RCC->AHB1ENR &= ~(1 << 3)
#define GPIOE_PCLK_DI()                RCC->AHB1ENR &= ~(1 << 4)
#define GPIOF_PCLK_DI()                RCC->AHB1ENR &= ~(1 << 5)
#define GPIOG_PCLK_DI()                RCC->AHB1ENR &= ~(1 << 6)
#define GPIOH_PCLK_DI()                RCC->AHB1ENR &= ~(1 << 7)

/*
* Clock Disable Macros for I2CX peripherals
*/
#define I2C1_PCLK_DI()                 RCC->APB1ENR &= ~(1 << 21)
#define I2C2_PCLK_DI()                 RCC->APB1ENR &= ~(1 << 22)
#define I2C3_PCLK_DI()                 RCC->APB1ENR &= ~(1 << 23)

/*
* Clock Disable Macros for ISPx peripherals
*/
#define SPI1_PCLK_DI()                 RCC->APB2ENR &= ~(1 << 12)
#define SPI2_PCLK_DI()                 RCC->APB1ENR &= ~(1 << 14)
#define SPI3_PCLK_DI()                 RCC->APB1ENR &= ~(1 << 15)
#define SPI4_PCLK_DI()                 RCC->APB2ENR &= ~(1 << 13)

/*
* Clock Disable Macros for USARTx peripherals
*/
#define USART1_PCLK_DI()               RCC->APB2ENR &= ~(1 << 4)
#define USART2_PCLK_DI()               RCC->APB1ENR &= ~(1 << 17)
#define USART3_PCLK_DI()               RCC->APB1ENR &= ~(1 << 18)

#define UART4_PCLK_DI()                RCC->APB1ENR &= ~(1 << 19)
#define UART5_PCLK_DI()                RCC->APB1ENR &= ~(1 << 20)

#define USART6_PCLK_DI()               RCC->APB2ENR &= ~(1 << 5)

/*
* Clock Disable Macros for SYSCFG peripherals
*/
#define SYSCFG_PCLK_DI()               RCC->APB2ENR &= ~(1 << 14)


/*
 * Macros to reset GPIOx peripheral
 */

#define GPIOA_REG_RESET()                do{ (RCC->AHB1ENR |= (1 << 0)); (RCC->AHB1ENR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()                do{ (RCC->AHB1ENR |= (1 << 1)); (RCC->AHB1ENR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()                do{ (RCC->AHB1ENR |= (1 << 2)); (RCC->AHB1ENR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()                do{ (RCC->AHB1ENR |= (1 << 3)); (RCC->AHB1ENR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()                do{ (RCC->AHB1ENR |= (1 << 4)); (RCC->AHB1ENR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()                do{ (RCC->AHB1ENR |= (1 << 5)); (RCC->AHB1ENR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()                do{ (RCC->AHB1ENR |= (1 << 6)); (RCC->AHB1ENR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()                do{ (RCC->AHB1ENR |= (1 << 7)); (RCC->AHB1ENR &= ~(1 << 7)); }while(0)

/*
 * return port code for given GPIOx base address made for SYSCFG_EXTIx
 */

#define GPIO_BASEADDR_TO_CODE(x)		 ( (x == GPIOA) ? 0 : \
										   (x == GPIOB) ? 1 : \
										   (x == GPIOC) ? 2 : \
										   (x == GPIOD) ? 3 : \
										   (x == GPIOE) ? 4 : \
										   (x == GPIOF) ? 5 : \
										   (x == GPIOG) ? 6 : \
										   (x == GPIOH) ? 7 : 0 )


/*
 * Macros to reset SPIx peripheral
 */
#define SPI1_REG_RESET()                do{ (RCC->APB2ENR |= (1 << 12)); (RCC->APB2ENR &= ~(1 << 12)); }while(0)
#define SPI2_REG_RESET()                do{ (RCC->APB1ENR |= (1 << 14)); (RCC->APB1ENR &= ~(1 << 14)); }while(0)
#define SPI3_REG_RESET()                do{ (RCC->APB1ENR |= (1 << 15)); (RCC->APB1ENR &= ~(1 << 15)); }while(0)
#define SPI4_REG_RESET()                do{ (RCC->APB2ENR |= (1 << 13)); (RCC->APB2ENR &= ~(1 << 13)); }while(0)



/*
 *  IRQ (Interrupt Request) number for MCU
 *  EXTI line interrupt - Position in VT(NVIC)
 */

#define IRQ_NO_EXTI0           6
#define IRQ_NO_EXTI1           7
#define IRQ_NO_EXTI2           8
#define IRQ_NO_EXTI3           9
#define IRQ_NO_EXTI4           10
#define IRQ_NO_EXTI9_5         23
#define IRQ_NO_EXTI15_10       40
#define IRQ_NO_SPI1            35
#define IRQ_NO_SPI2            36
#define IRQ_NO_SPI3            51

/*
 *  IRQ priority number 0 - 15
 */

#define NVIC_IRQ_PRI0		0
#define NVIC_IRQ_PRI1		1
#define NVIC_IRQ_PRI2		2
#define NVIC_IRQ_PRI3		3
#define NVIC_IRQ_PRI4		4
#define NVIC_IRQ_PRI5		5
#define NVIC_IRQ_PRI6		6
#define NVIC_IRQ_PRI7		7
#define NVIC_IRQ_PRI8		8
#define NVIC_IRQ_PRI9		9
#define NVIC_IRQ_PRI10		10
#define NVIC_IRQ_PRI11		11
#define NVIC_IRQ_PRI12		12
#define NVIC_IRQ_PRI13		13
#define NVIC_IRQ_PRI14		14
#define NVIC_IRQ_PRI15		15


// some generic macros
#define ENABLE          1
#define DISABLE         0
#define SET             ENABLE
#define RESET           DISABLE
#define GPIO_PIN_SET    SET
#define GPIO_PIN_RESET  RESET
#define HIGH            ENABLE
#define LOW             DISABLE
#define FLAG_RESET		RESET
#define FLAG_SET		SET



/*
 * Bit position definition of SPI peripheral
 */
#define SPI_CR1_REG_CPHA_BIT					0
#define SPI_CR1_REG_CPOL_BIT					1
#define SPI_CR1_REG_MASTER_SELEC_BIT			2
#define SPI_CR1_REG_BR_BIT						3
#define SPI_CR1_REG_SPI_EN_BIT					6
#define SPI_CR1_REG_LSB_FIRST_BIT				7
#define SPI_CR1_REG_SSI_BIT						8
#define SPI_CR1_REG_SSM_BIT						9
#define SPI_CR1_REG_RXONLY_BIT					10
#define SPI_CR1_REG_DFF_BIT						11
#define SPI_CR1_REG_CRC_NEXT_BIT				12
#define SPI_CR1_REG_CRC_EN_BIT					13
#define SPI_CR1_REG_BIDI_OE_BIT					14
#define SPI_CR1_REG_BIDI_MODE_BIT				15

#define SPI_CR2_REG_RX_DMA_BIT					0
#define SPI_CR2_REG_TX_DMA_BIT					1
#define SPI_CR2_REG_SSOE_BIT					2
#define SPI_CR2_REG_FRF_BIT						4
#define SPI_CR2_REG_ERRIE_BIT					5
#define SPI_CR2_REG_RXNEIE_BIT					6
#define SPI_CR2_REG_TXEIE_BIT					7

#define SPI_SR_REG_RXNE_BIT						0
#define SPI_SR_REG_TXE_BIT						1
#define SPI_SR_REG_CHSIDE_BIT					2
#define SPI_SR_REG_UDR_BIT						3
#define SPI_SR_REG_CRCERR_BIT					4
#define SPI_SR_REG_MODF_BIT						5
#define SPI_SR_REG_OVR_BIT						6
#define SPI_SR_REG_BSY_BIT						7
#define SPI_SR_REG_FRE_BIT						8


#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_spi_driver.h"














#endif /* INC_STM32F446XX_H_ */
