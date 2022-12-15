/*
 * stm32f446xx_spi_driver.h
 *
 *  Created on: 18/11/2022
 *      Author: MarcusT
 */

#ifndef INC_STM32F446XX_SPI_DRIVER_H_
#define INC_STM32F446XX_SPI_DRIVER_H_


#include "stm32f446xx.h"

/*
* This is configuration structure for SPIx
*/
typedef struct{
  uint8_t SPI_DeviceMode;             	  /*!<  >*/
  uint8_t SPI_BusConfig;                  /*!<  >*/
  uint8_t SPI_SclkSpeed;          	      /*!<  >*/
  uint8_t SPI_DEF;						  /*!<  >*/
  uint8_t SPI_CPOL;		         	      /*!<  >*/
  uint8_t SPI_CPHA;         			  /*!<  >*/
  uint8_t SPI_SSM;      	   			  /*!<  >*/
}SPI_PinConfig_t;

/*
* Handle structure for SPIx
*/
typedef struct{
  SPI_RegDef_t *pSPIx;                     /* this holds the base address of SPIx (x: 0, 1, 2)s */
  SPI_PinConfig_t SPI_Config;           /* This holds GPIO pin configuration settings */
}SPI_Handle_t;


/*
* @SPI_DeviceMode
*/
#define SPI_DEVICE_MODE_MASTER		1		/*!< SPI device mode in Master >*/
#define SPI_DEVICE_MODE_SLAVE		0		/*!< SPI device mode in Slave  >*/
/*
* @SPI_BusConfig
*/
#define SPI_BUS_CONFIG_FD				1		/*!< Full-Duplex Bus Mode  >*/
#define SPI_BUS_CONFIG_HD				2		/*!< Half-Duplex Bus Mode  >*/
#define SPI_BUS_CONFIG_SIMPLEX_RX_ONLY	3		/*!< Simplex TX ONLY Bus Mode >*/
//#define SPI_BUS_CONFIG_S_TX_ONLY				/*!<  THE SAME AS FULL DUPLEX  >*/
/*
* @SPI_SclkSpeed
*/
#define SPI_SCLK_SPEED_DIV2			0		/*!< defines pre-scale to 2 of SPI baud rate speed >*/
#define SPI_SCLK_SPEED_DIV4			1		/*!< defines pre-scale to 4 of SPI baud rate speed >*/
#define SPI_SCLK_SPEED_DIV8			2		/*!< defines pre-scale to 8 of SPI baud rate speed >*/
#define SPI_SCLK_SPEED_DIV16		3		/*!< defines pre-scale to 16 of SPI baud rate speed >*/
#define SPI_SCLK_SPEED_DIV32		4		/*!< defines pre-scale to 32 of SPI baud rate speed >*/
#define SPI_SCLK_SPEED_DIV64		5		/*!< defines pre-scale to 64 of SPI baud rate speed >*/
#define SPI_SCLK_SPEED_DIV128		6		/*!< defines pre-scale to 128 of SPI baud rate speed >*/
#define SPI_SCLK_SPEED_DIV256		7		/*!< defines pre-scale to 256 of SPI baud rate speed >*/
/*
* @SPI_DEF
*/
#define SPI_DFF_8BITS				0
#define SPI_DFF_16BITS				1
/*
* @SPI_CPOL
*/
#define SPI_CPOL_HIGH				1
#define SPI_CPHA_LOW				0
/*
* @SPI_CPHA
*/
#define SPI_CPHA_HIGH				1
#define SPI_CPOL_LOW				0
/*
* @SPI_SSM
*/
#define SPI_SSM_EN					1		/*!< Enable SSM - For Hardware Management >*/
#define SPI_SSM_DI					0		/*!< Disable SSM - For Software Management >*/

/*
* SPI relates status flags definitions
*/
#define SPI_SR_RXNE_FLAG			( 1 << SPI_SR_REG_RXNE_BIT)
#define SPI_SR_TXE_FLAG				( 1 << SPI_SR_REG_TXE_BIT )
#define SPI_SR_CHSIDE_FLAG			( 1 << SPI_SR_REG_CHSIDE_BIT )
#define SPI_SR_UDR_FLAG				( 1 << SPI_SR_REG_UDR_BIT )
#define SPI_SR_CRCERR_FLAG			( 1 << SPI_SR_REG_CRCERR_BIT)
#define SPI_SR_MODF_FLAG			( 1 << SPI_SR_REG_MODF_BIT )
#define SPI_SR_OVR_FLAG				( 1 << SPI_SR_REG_OVR_BIT )
#define SPI_SR_BUSY_FLAG			( 1 << SPI_SR_REG_BSY_BIT )
#define SPI_SR_FRE_FLAG				( 1 << SPI_SR_REG_FRE_BIT )


//   --------------------  API Prototypes  --------------------------- // ep. 136



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
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t ENorDI);     //don't forget to create a #definition for enable disable

/*
* Peripheral Init and DeInit
*/
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);


/*
 * Data send and receive
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

/*
* IRQ configuration and ISR handling
*/
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENorDI);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/*
*  Peripheral control
*/
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t ENorDI);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t ENorDI);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t ENorDI);

#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */
