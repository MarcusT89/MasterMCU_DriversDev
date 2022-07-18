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
void GPIO_ReadFromInputPin(void);
void GPIO_ReadFromInputPort(void);
void GPIO_WriteToOutputPin(void);
void GPIO_WriteToOutputPort(void);
void GPIO_ToggleOutputPin(void);

/*
* IRQ configuration and ISR handling
*/
void GPIO_IRQConfig(void);
void GPIO_IRQHandling(void);


#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
