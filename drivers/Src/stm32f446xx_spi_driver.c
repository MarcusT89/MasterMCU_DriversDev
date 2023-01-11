/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: 18/11/2022
 *      Author: MarcusT
 */

//#include <stdint.h>
#include "stm32f446xx_spi_driver.h"

/*
 * private function used in the driver function - only used here (static)
 * no need to be in tstm32f446xx_spi_driver.h header file
 *
 */
static void  spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void  spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void  spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);


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
	if (ENorDI == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );
		}
		else if (IRQNumber > 31 && IRQNumber < 64)
		{
			//program ISER1 register // 32 to 63
			*NVIC_ISER1 |= ( 1 << IRQNumber % 32 );
		}
		else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			//program ISER2 register //64 to 95
			*NVIC_ISER2 |= ( 1 << IRQNumber % 64 );
		}

	} else
	{
		if(IRQNumber <= 31)
				{
					//program ICER0 register
					*NVIC_ICER0 |= ( 1 << IRQNumber );
				}
				else if (IRQNumber > 31 && IRQNumber < 64)
				{
					//program ICER1 register // 32 to 63
					*NVIC_ICER1 |= ( 1 << IRQNumber % 32 );
				}
				else if (IRQNumber >= 64 && IRQNumber < 96)
				{
					//program ICER2 register //64 to 95
					*NVIC_ICER2 |= ( 1 << IRQNumber % 64 );
				}
	}
}

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. lets find out the ipr regsiter
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = ( 8 * iprx_section ) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}


uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
		//1 . Save the Tx buffer address and Len information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;
		//2.  Mark the SPI state as busy in transmission so that
		//    no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_REG_TXEIE_BIT );

	}


	return state;
}


uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX)
	{
		//1 . Save the Rx buffer address and Len information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;
		//2.  Mark the SPI state as busy in reception so that
		//    no other code can take over same SPI peripheral until reception is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//3. Enable the RXNEIE control bit to get interrupt whenever RXNEIE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_REG_RXNEIE_BIT );

	}


	return state;

}

void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1 , temp2;
	//first lets check for TXE
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_REG_TXE_BIT);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_REG_TXEIE_BIT);

	if( temp1 && temp2)
	{
		//handle TXE
		spi_txe_interrupt_handle(pHandle);
	}

	// check for RXNE
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_REG_RXNE_BIT);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_REG_RXNEIE_BIT);

	if( temp1 && temp2)
	{
		//handle RXNE
		spi_rxne_interrupt_handle(pHandle);
	}

	// check for ovr flag
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_REG_OVR_BIT);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_REG_ERRIE_BIT);

	if( temp1 && temp2)
	{
		//handle ovr error
		spi_ovr_err_interrupt_handle(pHandle);
	}

}


//some helper function implementations

static void  spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	// check the DFF bit in CR1
	if( (pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_REG_DFF_BIT) ) )
	{
		//16 bit DFF
		//1. load the data in to the DR
		pSPIHandle->pSPIx->DR =   *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}else
	{
		//8 bit DFF
		pSPIHandle->pSPIx->DR =   *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if(! pSPIHandle->TxLen)
	{
		//TxLen is zero , so close the spi transmission and inform the application that
		//TX is over.

		//this prevents interrupts from setting up of TXE flag
		SPI_CloseTransmisson(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
	}

}


static void  spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//do rxing as per the dff
	if(pSPIHandle->pSPIx->CR1 & ( 1 << 11))
	{
		//16 bit
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen -= 2;
		pSPIHandle->pRxBuffer++;
		pSPIHandle->pRxBuffer++;

	}else
	{
		//8 bit
		*(pSPIHandle->pRxBuffer) = (uint8_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}

	if(! pSPIHandle->RxLen)
	{
		//reception is complete
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
	}

}


static void  spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{

	uint8_t temp;
	//1. clear the ovr flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		// as defined in datasheet, to clear ovr, just need to read the DR ans SR reg.
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp; // because we just needed to read we used the temp variable. (void) is for error purpose for not using temp.
	//2. inform the application
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);

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


void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_REG_TXEIE_BIT);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;

}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_REG_RXNEIE_BIT);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;

}



void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;

}

__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{

	//This is a weak implementation . the user application may override this function.
}


