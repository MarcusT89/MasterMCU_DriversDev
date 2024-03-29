/*
 * 003_LED_Toogle_button_ Ext.c
 *
 * 	ep.105 test BOARD led BLINK WITH push button CONFIGURATION - Externally
 *
 *
 *  Created on: 19/09/2022
 *      Author: MarcusT
 */




#include <stdint.h>
#include "stm32f446xx.h"


void Delay(uint32_t delay)
{
	for(uint32_t i=0; i<delay; i++);
}


int main (void){				// TESTE THIS FINCTION ON OSCILOSCOPE SEE UP

	GPIO_Handle_t GpioLed;
	GPIO_Handle_t GpioButton;

	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPER_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;

	GpioButton.pGPIOx = GPIOA;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioButton.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPER_PP;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU; // --> activate internal pull up if not installed externally

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioLed);
	GPIO_Init(&GpioButton);

	while(1)
	{
		if (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_8))
		{
			Delay(500000);
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
		}

	}


	return 0;
}

