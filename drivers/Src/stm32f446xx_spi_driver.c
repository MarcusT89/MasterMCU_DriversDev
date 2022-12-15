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

	// Peripheral clock  enable : para não estar sempre a chama-lo na função MAIN
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);


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

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}
/**
 * @brief SPI send data
 *
 * Peripheral SPI send data
 *
 * @param pSPIx Pointer to a SPI reg
 * @param *pTxBuffer
 * @param LEN
 *
 * @return void.
 *
 * @Note  this is a blocking call
 * 			also it's a polling based call - polling for the TXE flag to SET - this may hang  permanently
 *
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while (Len > 0)
	{
		//1. wait until SEreg TXEflag is SET = empty
		//while( ! (pSPIx->SR & (1 << 1)));  --- alternative in a function to check flag
		while(SPI_GetFlagStatus(pSPIx, SPI_SR_TXE_FLAG) == FLAG_RESET );

		//2. check the DFF bit in CR1
		if (pSPIx->CR1 & (1 << SPI_CR1_REG_DFF_BIT ))
		{
			//16 bit DFF
			//1. load the data in to the DR - Data register
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}
		else
		{
			//8 Bit DFF
			pSPIx->DR = *(pTxBuffer);
			Len--;
			pTxBuffer++;
		}
	}

}
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while (Len > 0)
		{
			//1. wait until SEreg RXNEflag is SET = empty
			while(SPI_GetFlagStatus(pSPIx, SPI_SR_REG_RXNE_BIT) == FLAG_RESET );

			//2. check the DFF bit in CR1
			if (pSPIx->CR1 & (1 << SPI_CR1_REG_DFF_BIT ))
			{
				//16 bit DFF
				//1. load the data from the DR (Data register) to de RxBuffer
				*((uint16_t*)pRxBuffer)= pSPIx->DR ;
				Len--;
				Len--;
				(uint16_t*)pRxBuffer++;
			}
			else
			{
				//8 Bit DFF
				*(pRxBuffer)= pSPIx->DR ;
				Len--;
				pRxBuffer++;
			}
		}

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


void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t ENorDI)
{
	if (ENorDI == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_REG_SPI_EN_BIT); //SPE
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_REG_SPI_EN_BIT); //SPE
	}

}


void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t ENorDI)
{
	if (ENorDI == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_REG_SSI_BIT);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_REG_SSI_BIT);
	}

}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t ENorDI)
{
	if (ENorDI == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_REG_SSOE_BIT);
	}
	else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_REG_SSOE_BIT);
	}

}

