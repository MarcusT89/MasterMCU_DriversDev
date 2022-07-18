/*
 * stm32f446xx.h
 *
 *  Created on: Jul 8, 2022
 *      Author: MarcusT
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

/* Base addresses of flash and sram emory */

#define FLASH_BASEADDR                0X08000000U /* (UINT32_T) 0x0800..., SEEK FLASH MEMORY = MAIN MEMORY */
#define SRAM1_BASEADDR                0X20000000U  /* SRAM1 base address */
#define SRAM2_BASEADDR                0x2001C000U  /* SRAM1 base address */
#define ROM_BASEADDR                  0x1FFF0000U  /* = System memory */
#define SRAM                          SRAM1_BASEADDR /* Macro for 

/*
* AHBx and APBx Bus peripheral base addresses
*/

#define PERIPH_BASEADDR               0x40000000U
#define APB1PERIPH_BASEADDR           PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR           0x40010000U               
#define AHB1PERIPH_BASEADDR           0x40020000U
#define AHB2PERIPH_BASEADDR           0x50000000U

/*
* peripheral base addresses whitch are hanging on AHB1 base
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
* peripheral base addresses whitch are hanging on APB1 base    --- Episódio  81 
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
* peripheral base addresses whitch are hanging on APB2 base
*/

#define SPI1_BASEADDR                 (APB2PERIPH_BASEADDR + 0x3000) 
#define USART1_BASEADDR               (APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR               (APB2PERIPH_BASEADDR + 0x1400)
#define EXIT_BASEADDR                 (APB2PERIPH_BASEADDR + 0x3C00)
#define SYSCFG_BASEADDR               (APB2PERIPH_BASEADDR + 0x3800)

/*
* peripheral register definition struct for GPIO
*/
  
typedef struct{
  volatile uint32_t MODER;                    /* GPIO port mode register,              Address offset 0x00 */
  volatile uint32_t OTYPER;                   /* GPIO port output speed register,      Address offset 0x08 */
  volatile uint32_t OSPEEDR;                  /* GPIO port pull-up/pull-down register, Address offset 0x0C */
  volatile uint32_t PUPDR;
  volatile uint32_t IDR;
  volatile uint32_t ODR;
  volatile uint32_t BSRR;
  volatile uint32_t LCKR;
  volatile uint32_t AFR [2];                  /* AFR[0]: GPIO alternate function low register, AFR[1]: GPIO alternate function high register, Address offset 0x20 - 0x24 */
}GPIO_RegDef_t;

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


/* --------------------- RCC ----------------------*/

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

#define RCC                           ((RCC_RegDef_t*)RCC_BASEADDR)

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
#define I2C1_PCLK_EN()                 RCC->APB1ENR |= (1 << 23)

/*
* Clock Enable Macros for ISPx peripherals
*/
#define SPI1_PCLK_EN()                 RCC->APB2ENR |= (1 << 12)
#define SPI2_PCLK_EN()                 RCC->APB1ENR |= (1 << 14)
#define SPI3_PCLK_EN()                 RCC->APB1ENR |= (1 << 15)

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
#define I2C1_PCLK_DI()                 RCC->APB1ENR &= ~(1 << 23)

/*
* Clock Disable Macros for ISPx peripherals
*/
#define SPI1_PCLK_DI()                 RCC->APB2ENR &= ~(1 << 12)
#define SPI2_PCLK_DI()                 RCC->APB1ENR &= ~(1 << 14)
#define SPI3_PCLK_DI()                 RCC->APB1ENR &= ~(1 << 15)

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























#endif /* INC_STM32F446XX_H_ */
