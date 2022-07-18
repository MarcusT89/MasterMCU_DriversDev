/**
*   Info for file stm32f446xx_gpio_driver.h
*   file created in folder: ../Drivers/Inc/
*
*/

(...) /* "ifndef header builder info" */

#include "stm32f466xx.h"

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

typedef stuct{
  GPIO_RegDef_t *pGPIOx;                     /* this holds the base address of the GPIO port to which the pin belongs */
  GPIO_PinConfig_t GPIO_PinConfig;           /* Yhis holds GPIO pin configuration settings */



}GPIO_Handle_t;



/**
*   Info for file stm32f446xx_gpio_driver.c
*   file created in folder: ../Drivers/Src/
*
*/


#include "stm32f466xx_gpio_driver.h"







