/*
 * 001_LED_Toogle.c
 *
 * 	ep.102 e 103 test BOARD led BLINK WITH PUSPULL AND OPEN DRAIN CONFIGURATION*
 *
 *
 *  Created on: 16/08/2022
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

	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPER_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioLed);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_6);
		Delay(500000);
	}


	return 0;
}

