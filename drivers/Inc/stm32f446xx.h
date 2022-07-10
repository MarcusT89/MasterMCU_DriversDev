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

#define PERIPH_BASE                   0x40000000U
#define APB1PERIPH_BASE               PERIPH_BASE
#define APB2PERIPH_BASE               0x40010000U               
#define AHB1PERIPH_BASE               0x40020000U
#define AHB2PERIPH_BASE               0x50000000U

/*
* peripheral base addresses whitch are hanging on AHB1 base
*/

#define GPIOA_BASEADDR                (AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR                (AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR                (AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR                (AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR                (AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASEADDR                (AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASEADDR                (AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASEADDR                (AHB1PERIPH_BASE + 0x1C00)

/*
* peripheral base addresses whitch are hanging on APB1 base
*/

#define I2C1_BASEADDR                 (APB1PERIPH_BASE + 0x0000) /* Episódio  81 /*


#endif /* INC_STM32F446XX_H_ */
