/*
 * STM32WB55xx_SPI_Driver.h
 *
 *  Created on: 19-Apr-2020
 *      Author: Rahul
 */

#ifndef INC_STM32WB55XX_SPI_DRIVER_H_
#define INC_STM32WB55XX_SPI_DRIVER_H_

#include <STM32WB55xx.h>

/*
 * Configuration structure for SPIx peripheral
 */
typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_CRCL;		//DFF - in lecture
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;

/*
 * Handle structure for SPIx peripheral
 */
typedef struct
{
	SPI_RegDef_t 	*pSPIx;
	SPI_Config_t 	SPIConfig;
	uint8_t			*pTxBuffer;		/* !< To store the app. Tx buffer address > */
	uint8_t			*pRxBuffer;		/* !< To store the app. Rx buffer address > */
	uint32_t		TxLen;			/* !< To store the app. Tx length > */
	uint32_t		RxLen;			/* !< To store the app. Rx length > */
	uint8_t			TxState;		/* !< To store the app. Tx state > */
	uint8_t			RxState;		/* !< To store the app. Rx state > */
}SPI_Handle_t;


/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER		1
#define SPI_DEVICE_MODE_SLAVE		0

/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD					1	//Full Duplex
#define SPI_BUS_CONFIG_HD					2	//Half Duplex
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY		3	//Simplex Receive only

/*
 * @SPI_SclkSpeed -- For details we need to check the User manual of the board
 *					 Baudrate control details can be found in Control Register of SPI
 */
#define SPI_SCLK_SPEED_DIV2			0
#define SPI_SCLK_SPEED_DIV4			1
#define SPI_SCLK_SPEED_DIV8			2
#define SPI_SCLK_SPEED_DIV16		3
#define SPI_SCLK_SPEED_DIV32		4
#define SPI_SCLK_SPEED_DIV64		5
#define SPI_SCLK_SPEED_DIV128		6
#define SPI_SCLK_SPEED_DIV256		7

/*
 * @CRCL	-- Data Frame
 */
#define SPI_CRCL_8BITS		0 // By default the CRCL value will be 8 bits
#define SPI_CRCL_16BITS		1

/*
 * CPOL -- Clock POLarity
 */
#define SPI_CPOL_LOW		0
#define SPI_CPOL_HIGH		1

/*
 * CPHA -- Clock Phase
 */
#define SPI_CPHA_LOW		0
#define SPI_CPHA_HIGH		1

/*
 * @SPI_SSM
 */
#define SPI_SSM_DI		0		//By default it will be in Software Slave management is disabled
#define SPI_SSM_EN		1


/*
 * SPI related status flags definitions
 */
#define SPI_TXE_FLAG 		(1 << SPIx_SR_TXE)
#define SPI_RXNE_FLAG		(1 << SPIx_SR_RXNE)
#define SPI_CRCERR_FLAG		(1 << SPIx_SR_CRCERR)
#define SPI_MODF_FLAG		(1 << SPIx_SR_MODF)
#define SPI_OVR_FLAG		(1 << SPIx_SR_OVR)
#define SPI_BSY_FLAG		(1 << SPIx_SR_BSY)
#define SPI_FRE_FLAG		(1 << SPIx_SR_FRE)

/*
 * SPI Application States
 */
#define SPI_READY			0
#define SPI_BUSY_IN_RX		1
#define SPI_BUSY_IN_TX		2

/*
 * Possible SPI Application events
 * TODO: Should include all other events
 */
#define SPI_EVENT_TX_CMPLT		1
#define SPI_EVENT_RX_CMPLT		2
#define SPI_EVENT_OVR_ERR		3
#define SPI_EVENT_CRC_ERR		4


/*****************************************************************************************
 * 						APIs supported by this driver
 * 		For more information about the APIs, check the function definitions
 ****************************************************************************************/

/*
 * To enable/disable the peripheral clock the SPIx is hanging to
 */
void SPI_Peri_Clk_Control(SPI_RegDef_t *pSPIx, uint8_t EnOrDis);

/*
 * Initialize and De-initialize/reset the SPI
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data send and receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Length);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Length);

uint8_t SPI_SendDataINT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Length); //Interrupt based
uint8_t SPI_ReceiveDataINT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Length); //Interrupt based

/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterrupt_Config(uint8_t IRQNumber, uint8_t EnOrDis);
void SPI_IRQPriority_Config(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

/*
 * Other peripheral Control APIs
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDis);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDis);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDis);
uint8_t SPI_GetFLagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/*
 * Application Callback
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);
//This function has to be implemented by the User application.
//If the use doesn't implement it, then compiler will give the error.
//To stop the error, we have to create a weak implementation of the function in our driver C file.



#endif /* INC_STM32WB55XX_SPI_DRIVER_H_ */
