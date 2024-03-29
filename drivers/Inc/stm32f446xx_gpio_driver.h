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
* This is configuration structure for a GPIO pin
*/
typedef struct{
  uint8_t GPIO_PinNumber;               /*!< Possible variables from @GPIO_PIN_PINNUM >*/
  uint8_t GPIO_PinMode;                 /*!< Possible variables from @GPIO_PIN_MODES >*/
  uint8_t GPIO_PinSpeed;                /*!< Possible variables from @GPIO_PIN_SPEED >*/
  uint8_t GPIO_PinPuPdControl;          /*!< Possible variables from @GPIO_PIN_PUPDR >*/
  uint8_t GPIO_PinOPType;               /*!< Possible variables from @GPIO_PIN_OPTYPE >*/
  uint8_t GPIO_PinAltFunMode;           /*!< Possible variables from @GPIO_PIN_ALTERNATE_FUNCTION >*/
}GPIO_PinConfig_t;

/*
* Handle structure for GPIO pin
*/
typedef struct{
  GPIO_RegDef_t *pGPIOx;                     /* this holds the base address of the GPIO port to which the pin belongs */
  GPIO_PinConfig_t GPIO_PinConfig;           /* This holds GPIO pin configuration settings */
}GPIO_Handle_t;



/*
* @GPIO_PIN_PINNUM
* GPIO pin Number
*/
#define GPIO_PIN_NO_0            0
#define GPIO_PIN_NO_1            1
#define GPIO_PIN_NO_2            2
#define GPIO_PIN_NO_3            3
#define GPIO_PIN_NO_4            4
#define GPIO_PIN_NO_5            5
#define GPIO_PIN_NO_6            6
#define GPIO_PIN_NO_7            7
#define GPIO_PIN_NO_8            8
#define GPIO_PIN_NO_9            9
#define GPIO_PIN_NO_10           10
#define GPIO_PIN_NO_11           11
#define GPIO_PIN_NO_12           12
#define GPIO_PIN_NO_13           13
#define GPIO_PIN_NO_14           14
#define GPIO_PIN_NO_15           15

/*
* @GPIO_PIN_MODES
* GPIO pin possible mode
*/
#define GPIO_MODE_IN              0
#define GPIO_MODE_OUT             1
#define GPIO_MODE_ALTFN           2
#define GPIO_MODE_ANALOG          3 // 0 to 3 are non interrupting mode! - see this when configuring PIN mode
#define GPIO_MODE_IT_FT           4
#define GPIO_MODE_IT_RT           5
#define GPIO_MODE_IT_FRT          6

/*
* @GPIO_PIN_OPTYPE
* GPIO pin possible mode
*/
#define GPIO_OTYPER_PP             0
#define GPIO_OTYPER_OD             1

/*
* @GPIO_PIN_SPEED
* GPIO pin possible output speed
*/
#define GPIO_SPEED_LOW              0
#define GPIO_SPEED_MEDIUM           1
#define GPIO_SPEED_HIGH             2
#define GPIO_SPEED_VERYHIGH         3

/*
* @GPIO_PIN_PUPDR
* GPIO pin pullup and pulldown config macros
*/
#define GPIO_PIN_NO_PUPD           0
#define GPIO_PIN_PU                1
#define GPIO_PIN_PD                2



//   --------------------  API Prototypes  --------------------------- // ep. 90



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
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t ENorDI);     //don't forget to create a #definition for enable disable

/*
* Peripheral Init and DeInit
*/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
* Read and Write
*/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
* IRQ configuration and ISR handling
*/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENorDI);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);


#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
