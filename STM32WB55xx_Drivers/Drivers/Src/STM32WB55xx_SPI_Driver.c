/*
 * STM32WB55xx_SPI_Driver.c
 *
 *  Created on: 19-Apr-2020
 *      Author: Rahul
 */


#include <STM32WB55xx_SPI_Driver.h> // The header file specific to SPI has to be included

/*
 * Helper functions.
 * These are private to this file only.
 * Therefore their prototype and implementation both are written in this file only. Not in the Header file.
 * So that the user cannot call these functions.
 *
 * Using "static" keyword will make sure that if by any chance the user is calling these helper functions
 * in his/her application, then the compiler issues the error
 */
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_interrupt_handle(SPI_Handle_t *pSPIHandle);


/*********************************************************************
 * @fn      		  - SPI_Peri_Clk_Control
 *
 * @brief             - This function enables or disables peripheral clock for the given SPI
 *
 * @param[in]         - Base address of the SPI peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  N/A
 */

void SPI_Peri_Clk_Control(SPI_RegDef_t *pSPIx, uint8_t EnOrDis)
{
	if(EnOrDis == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
	}
	else {
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}else if (pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
	}
}


/*
 * Initialize and De-initialize/reset the SPIx
 */

/*********************************************************************
 * @fn      		  - Initialize the SPIx
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//Very important step is to enable the clock of the SPI peripheral bus
	SPI_Peri_Clk_Control(pSPIHandle->pSPIx, ENABLE);

	//1. Configure the SPI_CR1 Register
	uint32_t tempreg = 0;

	//a. Configure the device mode
	tempreg |= (pSPIHandle->SPIConfig.SPI_DeviceMode << 2);

	//b. Configure the bus mode
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD )
	{
		// BIDI mode should be cleared
		tempreg &= ~(1 << 15);

	}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD) {
		// BIDI mode should be set
		tempreg |= (1 << 15);

	}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY){
		// BIDI mode should be cleared and
		//RXONLY bit must be set
		tempreg &= ~(1 << 15);
		tempreg |= (1 << 10);
	}

	//c. Configure the SClkSpeed
	tempreg |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPIx_CR1_BR);

	//d. Configure the CRCL (or DFF)
	tempreg |= (pSPIHandle->SPIConfig.SPI_CRCL << SPIx_CR1_CRCL);

	//e. Configure the CPOL
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPOL << SPIx_CR1_CPOL);

	//f. Configure the CPHA
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPHA << SPIx_CR1_CPHA);

	//g. Configure the SSM
	tempreg |= (pSPIHandle->SPIConfig.SPI_SSM << SPIx_CR1_SSM);

	//2. Assign the CR1 register with tempreg value
	pSPIHandle->pSPIx->CR1 = tempreg;

}


/*********************************************************************
 * @fn      		  - De-Initialize the SPIx
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}
	else if (pSPIx == SPI2) {
		SPI2_REG_RESET();
	}
}


/*********************************************************************
 * @fn      		  - Function to get the status of the flags SPIx
 *
 * @brief             -
 *
 * @param[in]         - Base address of the SPI peripheral
 * @param[in]         - Flag Name
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
uint8_t SPI_GetFLagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}


/*********************************************************************
 * @fn      		  - Data sending function of SPIx
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - This is a blocking call; It is a polling type API function
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Length)
{
	while(Length > 0)
	{
		//1. Wait until TXE is set
		// while( !(pSPIx->SR & (1 <<1 )) );
		while( (SPI_GetFLagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET) );

		//2. Check the DFF/CRCL field
		if( pSPIx->CR1 & (1 << SPIx_CR1_CRCL) )
		{
			//16-bit DFF/CRCL
			pSPIx->DR =	*((uint16_t *)pTxBuffer);
			Length--;
			Length--;
			(uint16_t *)pTxBuffer++;
		}
		else
		{
			//8-bit DFF/CRCL
			pSPIx->DR =	*pTxBuffer;
			Length--;
			pTxBuffer++;
		}
	}

}


/*********************************************************************
 * @fn      		  - Data receiving function of SPIx
 *
 * @brief             -
 *
 * @param[in]         - Base address of the SPI peripheral
 * @param[in]         - ENABLE/DISABLE
 *
 * @return            -
 *
 * @Note              -
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Length)
{
	while(Length > 0)
	{
		//1. Wait until RXNE flag is set
		// while( !(pSPIx->SR & (1 <<1 )) );
		while( (SPI_GetFLagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET) );

		//2. Check the DFF/CRCL field
		if( pSPIx->CR1 & (1 << SPIx_CR1_CRCL) )
		{
			//16-bit DFF/CRCL
			*((uint16_t *)pRxBuffer) = pSPIx->DR;
			Length--;
			Length--;
			(uint16_t *)pRxBuffer++;
		}
		else
		{
			//8-bit DFF/CRCL
			*pRxBuffer = pSPIx->DR;
			Length--;
			pRxBuffer++;
		}
	}
}


/*********************************************************************
 * @fn      		  - The function Enables/Disables the SPI peripheral
 *
 * @brief             -
 *
 * @param[in]         - Base address of the SPI peripheral
 * @param[in]         - ENABLE (1) / DISABLE (0)
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - SPE bit of SPIx->CR1 register is set/reset
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDis)
{
	if(EnOrDis == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPIx_CR1_SPE);
	}
	else if (EnOrDis == DISABLE)
	{
		pSPIx->CR1 &= ~(1 << SPIx_CR1_SPE);
	}
}


/*********************************************************************
 * @fn      		  - The function Enables/Disables the SSI
 *
 * @brief             - SSI should be set when the SSM is enabled.(Important)
 * 					  - This makes the NSS signal high and avoid MODEF error
 *
 * @param[in]         - Base address of the SPI peripheral
 * @param[in]         - ENABLE (1) / DISABLE (0)
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - SSI bit of SSIx->CR1 register is set/reset
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDis)
{
	if(EnOrDis == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPIx_CR1_SSI);
	}
	else if (EnOrDis == DISABLE)
	{
		pSPIx->CR1 &= ~(1 << SPIx_CR1_SSI);
	}
}


/*********************************************************************
 * @fn      		  - The function Enables/Disables the SSOE
 *
 * @brief             - This should be used when using Hardware Slave Management, i.e., SSM=0 .(Important)
 *					  - Making SSOE 1 does NSS output enable.
 *	 	 	 	 	  - The NSS pin is automatically managed by hardware.
 *	 	 	 	 	  - i.e., when SPE=1, NSS will be pulled to low, and NSS will be high when SPE=0
 * @param[in]         - Base address of the SPI peripheral
 * @param[in]         - ENABLE (1) / DISABLE (0)
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - SSOE bit of SSIx->CR2 register is set/reset
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDis)
{
	if(EnOrDis == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPIx_CR2_SSOE);
	}
	else if (EnOrDis == DISABLE)
	{
		pSPIx->CR2 &= ~(1 << SPIx_CR2_SSOE);
	}
}


/*********************************************************************
 * @fn      		  - Interrupt based SPI Send data function
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - Txstate of SPI
 *
 * @Note              -
 */
uint8_t SPI_SendDataINT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Length)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
		//1. Save the Tx buffer address and length is global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Length;

		//2. Mark the SPI state as busy in transmission,
		// so that no other code can take over the same SPI peripheral until transmission is completed.
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR.
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPIx_CR2_TXEIE);

		//4. Data transmission will be handled by the ISR code.
	}

	state = pSPIHandle->TxState;
	return state;
}


/*********************************************************************
 * @fn      		  - Interrupt based SPI Send data function
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - Rxstate of SPI
 *
 * @Note              -
 */
uint8_t SPI_ReceiveDataINT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Length)
{
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_TX)
	{
		//1. Save the Rx buffer address and length is global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Length;

		//2. Mark the SPI state as busy in transmission,
		// so that no other code can take over the same SPI peripheral until transmission is completed.
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//3. Enable the RXNEIE control bit to get interrupt whenever RXNE flag is set in SR.
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPIx_CR2_RXNEIE);

		//4. Data reception part will be handled by the ISR code.
	}

	state = pSPIHandle->TxState;
	return state;
}

/*********************************************************************
 * @fn      		  - To enable/disable the Interrupts of SPI
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
void SPI_IRQInterrupt_Config(uint8_t IRQNumber, uint8_t EnOrDis)
{
	//This configuration is done at the processor side
	if(EnOrDis == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//Program ISER0 register
			*NVIC_ISER0 = (1 << IRQNumber);

		}else if(IRQNumber >= 32 && IRQNumber <= 63)
		{
			//Program ISER1 register
			*NVIC_ISER1 = (1 << (IRQNumber % 32));

		}else if(IRQNumber >= 64 && IRQNumber <= 95)
		{
			//Program ISER2 register
			*NVIC_ISER2 = (1 << (IRQNumber % 64));
		}
	}else {
		if(IRQNumber <= 31)
		{
			//Program ICER0 register
			*NVIC_ICER0 = (1 << IRQNumber);

		}else if(IRQNumber >= 32 && IRQNumber <= 63)
		{
			//Program ICER1 register
			*NVIC_ICER1 = (1 << (IRQNumber % 32));

		}else if(IRQNumber >= 64 && IRQNumber <= 95)
		{
			//Program ICER2 register
			*NVIC_ICER1 = (1 << (IRQNumber % 64));
		}
	}

}


/*********************************************************************
 * @fn      		  - To set priorities to the Interrupts
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
void SPI_IRQPriority_Config(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//Find out the IPR register corresponding to IRQNumber
	uint8_t iprx = IRQNumber / 4;

	//Find out the section in the IPR register corresponding to IRQNumber
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	//Now store the Priority value in the iprx section.
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);

}

/*********************************************************************
 * @fn      		  - Interrupts Handler for SPIx
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp1, temp2;
	//Check for the flag responsible for the interrupt
	// 1. Check for TXE flag
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPIx_SR_TXE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPIx_CR2_TXEIE);

	if( temp1 && temp2 )
	{
		//Handle TXE (That is Send Data)
		spi_txe_interrupt_handle(pSPIHandle);		//Helper function. It will not be visible to user application
	}

	// 2. Check for RXNE flag
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPIx_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPIx_CR2_RXNEIE);

	if( temp1 && temp2 )
	{
		//Handle RXNE (That is Receive Data)
		spi_rxne_interrupt_handle(pSPIHandle);		//Helper function. It will not be visible to user application
	}

	// 3. Check for OVR flag
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPIx_SR_OVR);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPIx_CR2_ERRIE);

	if( temp1 && temp2 )
	{
		//Handle OVR (To clear the Over run flag)
		spi_ovr_interrupt_handle(pSPIHandle);		//Helper function. It will not be visible to user application
	}

	//I should implement other interrupts' handlers.
}

/*
 * Implementations of Helper functions.
 */
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	// Check the DFF/CRCL field
	if( pSPIHandle->pSPIx->CR1 & (1 << SPIx_CR1_CRCL) )
	{
		//16-bit DFF/CRCL
		pSPIHandle->pSPIx->DR = *((uint16_t *)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t *)pSPIHandle->pTxBuffer++;
	}
	else
	{
		//8-bit DFF/CRCL
		pSPIHandle->pSPIx->DR =	*pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if( !pSPIHandle->TxLen )
	{
		//TXE Length is zero.
		//So close the SPI transmission and inform the application that TX is completed.
		SPI_CloseTransmission(pSPIHandle);

		//2. Inform the application
		//Application event callback. The application has to implement this callback
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}

}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	// Check the DFF/CRCL field
	if( pSPIHandle->pSPIx->CR1 & (1 << SPIx_CR1_CRCL) )
	{
		//16-bit DFF/CRCL
		*((uint16_t *)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;
		(uint16_t *)pSPIHandle->pRxBuffer++;
	}
	else
	{
		//8-bit DFF/CRCL
		*pSPIHandle->pTxBuffer = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}

	if( !pSPIHandle->RxLen )
	{
		//RXE Length is zero.
		//So close the SPI reception and inform the application that RX is completed.
		SPI_CloseReception(pSPIHandle);

		//2. Inform the application
		//Application event callback. The application has to implement this callback
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}


}

static void spi_ovr_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//1. Clear the Over run flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		SPI_ClearOVRFlag(pSPIHandle->pSPIx);
	}

	//2. Inform the application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}



void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp; //type-casting the unused variable

}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	//1. Clear the TXEIE bit
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPIx_CR2_TXEIE);

	//2. Clear the variables related to TX in SPIHandle
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;

	//3. Change the SPI transmission state to READY.
	pSPIHandle->TxState = SPI_READY;

}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	//1. Clear the RXNEIE bit
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPIx_CR2_RXNEIE);

	//2. Clear the variables related to RX in SPIHandle
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;

	//3. Change the SPI reception state to READY.
	pSPIHandle->RxState = SPI_READY;

}

/*
 * Weak implementation of Application Callback function
 */
__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
	//If the user application doesn't implement this function, then this function will be called.
}
