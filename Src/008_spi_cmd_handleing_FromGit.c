/*
 * 008_spi_cmd_handling_FromGit.c
 *
 *  Created on: 11/12/2022
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

//command codes the slave recognizes, exlpained at the header of the exercise
#define COMMAND_LED_CTRL            0X50
#define COMMAND_SENSOR_READ         0X51
#define COMMAND_LED_READ            0X52
#define COMMAND_PRINT               0X53
#define COMMAND_ID_READ             0X54

#define LED_ON                      1
#define LED_OFF                     0

//arduino analog pins
#define ANALOG_PIN0                 0
#define ANALOG_PIN1                 1
#define ANALOG_PIN2                 2
#define ANALOG_PIN3                 3
#define ANALOG_PIN4                 4

#define LED_PIN                     9


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
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);
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

uint8_t SPI_verifyResponse(uint8_t ackbyte)
{
	if(ackbyte == 0xF5)
		return 1;
	
	return 0;
}


int main(void)
{

	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;

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

        //1. CMD_LED_CTRL <pinNO(1)> <value(1)> ------------------
		
        uint8_t commandcode = COMMAND_LED_CTRL;
		uint8_t ackbyte;
		uint8_t args[2];


        SPI_SendData(SPI2,&commandcode,1); // slave will response ACK or NACK. Ver. apontamentos
		// do a dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2,&dummy_read,1);

		// send some dummy bits (1byte) to fetch the response form the salve 
		SPI_SendData(SPI2,&dummy_write,1);
		// need to read data to recive the ACK byte(send then read)
		SPI_ReceiveData(SPI2,&ackbyte,1);

		if ( SPI_verifyResponse(ackbyte) ) //check if its ACK
		{
			// send arguments 
			args[0]= LED_PIN;
			args[1]= LED_ON;
			SPI_SendData(SPI2,args,2);
		}
        



		//2. CMD_SENSOR_READ < analog pinNO(1)> -------------

		//wait until the button i pressed
		while( ! GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13 ));

		Delay(500000/2);

		commandcode = COMMAND_SENSOR_READ;
		SPI_SendData(SPI2,&commandcode,1);

		// do a dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2,&dummy_read,1);

		// send some dummy bits (1byte) to fetch the response form the salve 
		SPI_SendData(SPI2,&dummy_write,1);

		// need to read data to recive the ACK byte(send then read)
		SPI_ReceiveData(SPI2,&ackbyte,1);

		if ( SPI_verifyResponse(ackbyte) ) //check if its ACK
		{
			// send arguments 
			args[0]= ANALOG_PIN0;
			SPI_SendData(SPI2,args,1);

			// do a dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2,&dummy_read,1);

			// insert some delay so that the slave can ready the data
			Delay(500000/2);

			// send some dummy bits (1byte) to fetch the response form the salve 
			SPI_SendData(SPI2,&dummy_write,1);
			
			uint8_t analog_read;
			SPI_ReceiveData(SPI2,&analog_read,1);
		}


		//wait until all data is sent - Busy flag from status reg
		while( SPI_GetFlagStatus(SPI2, SPI_SR_REG_BSY_BIT));

		//Disable SPI enable peripheral
		SPI_PeripheralControl(SPI2, DISABLE);
        
	}



	return 0;
}
