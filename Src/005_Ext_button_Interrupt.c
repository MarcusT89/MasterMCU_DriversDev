/*
 * 005_Ext_button_Interrupt.c
 *
 * Testing Interrupt with external button.
 * connecting button to PD5 and toggle LED whenever interrupt os triggered by the button
 * interrupt should be triggered during falling edge of button press.
 *
 * Aula: 115
 *
 *  Created on: 08/11/2022
 *      Author: MarcusT
 */


#include "string.h"
#include <stdint.h>
#include "stm32f446xx.h"


void Delay(uint32_t delay)
{
	for(uint32_t i=0; i<delay; i++);
}


int main (void){				// TESTE THIS FINCTION ON OSCILOSCOPE SEE UP

	GPIO_Handle_t GpioLed;
	GPIO_Handle_t GpioButton;

	memset(&GpioLed,0,sizeof(GpioLed));
	memset(&GpioButton,0,sizeof(GpioButton));

	// internal LED config
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPER_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;

	// External button config
	GpioButton.pGPIOx = GPIOA;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	//GpioButton.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPER_PP;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU; // --> activate internal pull up if not installed externally

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioLed);
	GPIO_Init(&GpioButton);

	// IRQ config
<	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);

	while (1);

}

void EXTI9_5_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_NO_8);
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_6);
}


