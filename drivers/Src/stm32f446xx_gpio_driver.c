/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: 18/07/2022
 *      Author: MarcusT
 */


#include "stm32f446xx_gpio_driver.c"




/*
* Peripheral Clock Setup
*/
void GPIO_PeriClockControl(GPIO_regDef_t *pGPIOx, uint8_t ENorDI){     
  
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


/*
* Peripheral Init and DeInit
*/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
  uint32_t temp = 0; // temp register
  
  //1. configure Gpio pin Mode
  if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->MODER |= temp;
  }
  else {
    // for interrups
  }
  
  //2. configure speed
  temp = 0;
  tmep = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); 
  pGPIOHandle->pGPIOx->OSPEEDR |= temp;
  
  //3. configure popup popdown setting
  temp = 0;
  tmep = pGPIOHandle->GPIO_PinConfig.PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); 
  pGPIOHandle->pGPIOx->PUPDR |= temp;
    
  //4. configure optype
  temp = 0;
  tmep = pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType <<  pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); 
  pGPIOHandle->pGPIOx->OTYPER |= temp;
  //5. configure alt functionality
  
}




void GPIO_DeInit(GPIO_regDef_t *pGPIOx){

}      

/*
* Read and Write
*/
uint8_t GPIO_ReadFromInputPin(GPIO_regDef_t *pGPIOx, uint8_t PinNumber){

}
uint16_t GPIO_ReadFromInputPort(GPIO_regDef_t *pGPIOx){

}
void GPIO_WriteToOutputPin(GPIO_regDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){

}
void GPIO_WriteToOutputPort(GPIO_regDef_t *pGPIOx, uint16_t Value){

}
void GPIO_ToggleOutputPin(GPIO_regDef_t *pGPIOx, uint8_t PinNumber){

}

/*
* IRQ configuration and ISR handling
*/
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t ENorDI){

}
void GPIO_IRQHandling(uint8_t PinNumber){

}




