/*
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: 18/07/2022
 *      Author: MarcusT
 */

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_


#include "stm32f446xx.h"

/*
* This is configuration stucture for a GPIO pin
*/
typedef struct{
  uint8_t GPIO_PinNumber;
  uint8_t GPIO_PinMode;
  uint8_t GPIO_PinSpeed;
  uint8_t GPIO_PinPuPdControl;
  uint8_t GPIO_PinOPType;
  uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

/*
* Handle stucture for GPIO pin
*/
typedef struct{
  GPIO_RegDef_t *pGPIOx;                     /* this holds the base address of the GPIO port to which the pin belongs */
  GPIO_PinConfig_t GPIO_PinConfig;           /* This holds GPIO pin configuration settings */
}GPIO_Handle_t;


//   --------------------  API Prototypes  --------------------------- // ep. 90

/*
* Peripheral Clock Setup
*/
void GPIO_PeriClockControl(GPIO_regDef_t *pGPIOx, uint8_t EnorDi);     //don't forget to create a #definition for enable disable

/*
* Peripheral Init and DeInit
*/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_regDef_t *pGPIOx);       // <----- here!

/*
* Read and Write
*/
uint8_t GPIO_ReadFromInputPin(GPIO_regDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_regDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_regDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_regDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_regDef_t *pGPIOx, uint8_t PinNumber);

/*
* IRQ configuration and ISR handling
*/
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t ENorDI);
void GPIO_IRQHandling(uint8_t PinNumber);


#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
