/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: 18/11/2022
 *      Author: MarcusT
 */

#include <stdint.h>
#include "stm32f446xx_spi_driver.h"

/**
 * @brief Peripheral Clock Setup
 *
 * Peripheral Clock Setup of a given a pointer SPI
 *
 * @param pSPIx Pointer to a SPI reg
 * @param ENorDI Enalbe or disable
 *
 * @return void.
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t ENorDI)
{
	if(ENorDI == ENABLE)
	{
		if(pSPIx == SPI1){
		  SPI1_PCLK_EN();
		}else if(pSPIx == SPI2){
		  SPI2_PCLK_EN();
		}else if(pSPIx == SPI3){
		  SPI3_PCLK_EN();
		}else if(pSPIx == SPI4){
		  SPI4_PCLK_EN();
		}
	}
	else
	{
		if(pSPIx == SPI1){
		  SPI1_PCLK_DI();
		}else if(pSPIx == SPI2){
		  SPI2_PCLK_DI();
		}else if(pSPIx == SPI3){
		  SPI3_PCLK_DI();
		}else if(pSPIx == SPI4){
		  SPI4_PCLK_DI();
		}
	}
}


/*
* Peripheral Init and DeInit
* the logic here is just yo change values if they are not set be default!!
* Configure tempreg for all the CR1 reg
*/
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	// first configure SPI_CR1  reg
	uint32_t tempreg = 0;

	//1. conf the device mode - in bit field 2 of CR1 reg
	tempreg |= pSPIHandle->SPI_Config.SPI_DeviceMode << 2;

	//2. Conf Bus Config
	if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//bide mode should  bus cleared
		tempreg &= ~( 1 << 15);
	}
	else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//bidi mode should be set
		tempreg |= ( 1 << 15);
	}
	else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RX_ONLY)
	{
		// bidei mdoe should be cleared
		tempreg &= ~( 1 << 15);
		// rxonly bit musi be set
		tempreg |= ( 1 << 10);
	}

	//3. Config the clock speed (baud rate)
	tempreg |= pSPIHandle->SPI_Config.SPI_SclkSpeed << 3;

	//4. Config the DFF
	tempreg |= pSPIHandle->SPI_Config.SPI_DEF << 11;

	//4. Config the CPOL
	tempreg |= pSPIHandle->SPI_Config.SPI_CPOL << 1;

	//5. Config the CPHA
	tempreg |= pSPIHandle->SPI_Config.SPI_CPHA << 0;

	//6. Config the SSM
	tempreg |= pSPIHandle->SPI_Config.SPI_SSM << 9;

	pSPIHandle->pSPIx->CR1 = tempreg;
}

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}
	else if(pSPIx == SPI2){
		SPI2_REG_RESET();
	}else if(pSPIx == SPI3){
		SPI3_REG_RESET();
	}else if(pSPIx == SPI4){
		SPI4_REG_RESET();
	}
}


/*
 * Data send and receive
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint32_t *pTxBuffer, uint32_t Len)
{

}
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint32_t *pRxBuffer, uint32_t Len)
{

}

/*
* IRQ configuration and ISR handling
*/
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENorDI)
{

}
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{

}
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{

}
