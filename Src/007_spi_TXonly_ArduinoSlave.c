/*
 * 007_spi_TXonly_ArduinoSlave.c
 *
 *  Created on: 10/12/2022
 *      Author: MarcusT
 *
 * 	Alternate function:
 * 	SPI2 MOSI --> PB15
 *	SPI2 MISO --> PB14
 *	SPI2 SCLK --> PB13
 *	SPI2 NSS  --> PB12
 */


#include <stdint.h>
#include <string.h>
#include "stm32f446xx.h"

void Delay(uint32_t delay)
{
	for(uint32_t i=0; i<delay; i++);
}

void SPI_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPER_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PIN_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	//GPIO_PeriClockControl(GPIOB, ENABLE);  //--> called this function in the periph init function

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);
	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);
	//MOSI
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	//GPIO_Init(&SPIPins);
	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);

}

void SPI2_Init(void)
{
	SPI_Handle_t SPI2_handle;

	SPI2_handle.pSPIx = SPI2;
	SPI2_handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2_handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2_handle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8; // 8mhz
	SPI2_handle.SPI_Config.SPI_DEF = SPI_DFF_8BITS;
	SPI2_handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2_handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2_handle.SPI_Config.SPI_SSM = SPI_SSM_DI; // ssm disabled - Hardware management

	//SPI_PeriClockControl(SPI2, ENABLE); //--> called this function in the periph Init function

	SPI_Init(&SPI2_handle);

}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GpioButton;

	GpioButton.pGPIOx = GPIOC;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioButton.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPER_PP;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;

	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GpioButton);
}




int main(void)
{
	char user_data [] = "Hello World";

	GPIO_ButtonInit();

	//initialize GPIO to SPI2
	SPI_GPIOInits();

	// initialize SPI2
	SPI2_Init();

	//this makes the NSS signal internally high and avoid MODF error -
	//this only makes sense when using SSM, SSI is required
	//   SPI_SSIConfig(SPI2, ENABLE);

	/*
	 * making SSOE = 1 deos NSS output enable
	 * the NNS pin is automatically managed by the hardware
	 * i.e when SPE=1, NSS will be pulled to low and NSS pin will be high when SPE=0
	 */
	SPI_SSOEConfig(SPI2, ENABLE);

	while(1)
	{
		//wait until the button i pressed
		while( ! GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13 ));

		Delay(500000/2);

		//enable SPI -> at status reg SPE flag
		SPI_PeripheralControl(SPI2, ENABLE);

		uint8_t dataLen = strlen(user_data);
		SPI_SendData(SPI2, &dataLen, strlen(dataLen));

		SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

		//wait until all data is sent - Busy flag from status reg
		while( SPI_GetFlagStatus(SPI2, SPI_SR_REG_BSY_BIT));

		//Disable SPI enable peripheral
		SPI_PeripheralControl(SPI2, DISABLE);
	}



	return 0;
}
