/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: 18/07/2022
 *      Author: MarcusT
 */

#include <stdint.h>
#include "stm32f446xx_gpio_driver.h"




/**
 * @brief Peripheral Clock Setup
 *
 * Peripheral Clock Setup of a given a pointer GPIO
 *
 * @param pGPIOx Pointer to a GPIO reg
 * @param ENorDI Enalbe or disable
 *
 * @return void.
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t ENorDI){
  
  if(ENorDI == ENABLE)
  {
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
    }    
  }
  else 
  {
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
    }    
  }
  
}


/**
 * @brief Function to set registers of a GPIO PIN
 *
 * Function to set registers of a GPIO PIN
 *
 * @param *pGPIOHandle Pointer to GPIO handle
 *
 * @return void.
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
  uint32_t temp = 0; // temp register
  
  //1. configure Gpio pin Mode (aula: 95)
  if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){ // look at the " GPIO pin possible mode" in this.h 0 to 3 are non interrupting modes
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
    pGPIOHandle->pGPIOx->MODER |= temp; //setting
  }
  else
  {
	  // for interrupt  mode
	  if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_FT)
	  {
		  //1. configure the FTSR
		  EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		  // clear the corresponding RTSR bit
		  EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	  }
	  else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_RT)
	  {
		  //1. configure the RTSR
		  EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		  // clear the corresponding FTSR bit
		  EXTI->FTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	  }
	  else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_FRT)
	  {
		  //1. configure the FTSR
		  EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		  // and configure RTSR bit
		  EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	  }
	  // 2. Configure the GPIO port selection in SYSCFG_EXTICR
	  uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4; // (AULA:111)
	  uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
	  uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
	  SYSCFG_PCLK_EN();
	  SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);

	  // 3. enable the EXTI interrupt delivery using IMR
	  EXTI->IMR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

  }
  
  //2. configure speed
  temp = 0;
  temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
  pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
  pGPIOHandle->pGPIOx->OSPEEDR |= temp;
  
  //3. configure popup popdown setting
  temp = 0;
  temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
  pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);  
  pGPIOHandle->pGPIOx->PUPDR |= temp;
    
  //4. configure optype
  temp = 0;
  temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType <<  pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
  pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
  pGPIOHandle->pGPIOx->OTYPER |= temp;
  
  //5. configure alternate functionality
  temp = 0;
  if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ALTFN)
  {
    uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
    uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
    pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
    pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2) );
  }
}

/**
 * @brief Function to reset registers of a GPIO PIN
 *
 * Function to reset registers of a GPIO PIN
 *
 * @param *pGPIOx Pointer to GPIO port register RESET
 *
 * @return void.
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
  if(pGPIOx == GPIOA)
  {
    GPIOA_REG_RESET();
  }
  else if(pGPIOx == GPIOB){
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
  }    

}      

/*
* Read and Write
*/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
  uint8_t value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
  return value;
}


uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	uint16_t value = (uint16_t)(pGPIOx->IDR) ;
	return value;

}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value){

	if (value == SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}
	if (value == RESET)
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value){

	pGPIOx->ODR = value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

	pGPIOx->ODR ^= (1 << PinNumber);
}

/*
* IRQ configuration and ISR handling
*/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENorDI){
	if (ENorDI == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );
		}
		else if (IRQNumber > 31 && IRQNumber < 64)
		{
			//program ISER1 register // 32 to 63
			*NVIC_ISER1 |= ( 1 << IRQNumber % 32 );
		}
		else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			//program ISER2 register //64 to 95
			*NVIC_ISER2 |= ( 1 << IRQNumber % 64 );
		}

	} else
	{
		if(IRQNumber <= 31)
				{
					//program ICER0 register
					*NVIC_ICER0 |= ( 1 << IRQNumber );
				}
				else if (IRQNumber > 31 && IRQNumber < 64)
				{
					//program ICER1 register // 32 to 63
					*NVIC_ICER1 |= ( 1 << IRQNumber % 32 );
				}
				else if (IRQNumber >= 64 && IRQNumber < 96)
				{
					//program ICER2 register //64 to 95
					*NVIC_ICER2 |= ( 1 << IRQNumber % 64 );
				}
	}
}


void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	//1. lets find out the ipr regsiter
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = ( 8 * iprx_section ) + (8 + NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + (iprx * 4)) |= (IRQPriority << shift_amount);
}

void GPIO_IRQHandling(uint8_t PinNumber){
	// clear the exti pr register corresponding to the pin number
	if (EXTI->PR & (1 << PinNumber))
	{
		//clear
		EXTI->PR |= (1 << PinNumber);
	}
}




