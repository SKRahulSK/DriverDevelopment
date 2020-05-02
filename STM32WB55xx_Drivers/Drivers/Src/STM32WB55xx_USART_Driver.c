/*
 * STM32WB55xx_USART_Driver.c
 *
 *  Created on: 30-Apr-2020
 *      Author: Rahul
 */


#include <STM32WB55xx_USART_Driver.h>


/*********************************************************************
 * @fn      		  - USART_Peri_Clk_Control
 *
 * @brief             - This function enables or disables peripheral clock for the given USART
 *
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  N/A
 */
void USART_Peri_Clk_Control(uint8_t EnOrDis)
{
	if(EnOrDis == ENABLE)
	{
		USART1_PCLK_EN();
	}
	else {
		USART1_PCLK_DI();
	}

}

/*
 * Initialize and De-initialize/reset the USARTx
 */

/*********************************************************************
 * @fn      		  - Initialize the USARTx
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
void USART_Init(USART_Handle_t *pUSARTHandle)
{
	//Temporary variable
	uint32_t tempreg=0;

	/******************************** Configuration of CR1**************************************/

	//Implement the code to enable the Clock for given USART peripheral
	 USART_Peri_Clk_Control(ENABLE);

	//Enable USART Tx and Rx engines according to the USART_Mode configuration item
	if ( pUSARTHandle->USARTConfig.USART_Mode == USART_MODE_ONLY_RX)
	{
		//Implement the code to enable the Receiver bit field
		tempreg|= (1 << USARTx_CR1_RE);
	}else if (pUSARTHandle->USARTConfig.USART_Mode == USART_MODE_ONLY_TX)
	{
		//Implement the code to enable the Transmitter bit field
		tempreg |= ( 1 << USARTx_CR1_TE);

	}else if (pUSARTHandle->USARTConfig.USART_Mode == USART_MODE_TXRX)
	{
		//Implement the code to enable the both Transmitter and Receiver bit fields
		tempreg |= ( ( 1 << USARTx_CR1_RE) | ( 1 << USARTx_CR1_TE) );
	}

    //Implement the code to configure the Word length configuration item
	/*
	 * 	WORD LENGTH		M1		M0
	 * 	---------------------------
	 * 		8			0		0
	 * 		9			0		1
	 * 		7			1		0
	 *
	 * 	Therefore, USARTx_CR1_M1 = USART_WordLength/2 and USARTx_CR1_M0 = USART_WordLength%2
	 */
	tempreg |= (pUSARTHandle->USARTConfig.USART_WordLength/2 << USARTx_CR1_M1) | (pUSARTHandle->USARTConfig.USART_WordLength%2 << USARTx_CR1_M0);



    //Configuration of parity control bit fields
	if ( pUSARTHandle->USARTConfig.USART_ParityControl == USART_PARITY_EN_EVEN)
	{
		//Implement the code to enable the parity control
		tempreg |= ( 1 << USARTx_CR1_PCE);

		//Implement the code to enable EVEN parity
		//Not required because by default EVEN parity will be selected once you enable the parity control

	}else if (pUSARTHandle->USARTConfig.USART_ParityControl == USART_PARITY_EN_ODD )
	{
		//Implement the code to enable the parity control
	    tempreg |= ( 1 << USARTx_CR1_PCE);

	    //Implement the code to enable ODD parity
	    tempreg |= ( 1 << USARTx_CR1_PS);

	}

   //Program the CR1 register
	pUSARTHandle->pUSARTx->CR1 = tempreg;

	/******************************** Configuration of CR2**************************************/

	tempreg=0;

	//Implement the code to configure the number of stop bits inserted during USART frame transmission
	tempreg |= pUSARTHandle->USARTConfig.USART_NoOfStopBits << USARTx_CR2_STOP;

	//Program the CR2 register
	pUSARTHandle->pUSARTx->CR2 = tempreg;

	/******************************** Configuration of CR3**************************************/

	tempreg=0;

	//Configuration of USART hardware flow control
	if ( pUSARTHandle->USARTConfig.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		//Implement the code to enable CTS flow control
		tempreg |= ( 1 << USARTx_CR3_CTSE);


	}else if (pUSARTHandle->USARTConfig.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		//Implement the code to enable RTS flow control
		tempreg |= (1 << USARTx_CR3_RTSE);

	}else if (pUSARTHandle->USARTConfig.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		//Implement the code to enable both CTS and RTS Flow control
		tempreg |= ( (1 << USARTx_CR3_CTSE) | (1 << USARTx_CR3_RTSE) );
	}


	pUSARTHandle->pUSARTx->CR3 = tempreg;

	/********************** Configuration of BRR(Baudrate register)*****************************/

	//Call the function to configure the baud rate
	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USARTConfig.USART_BaudRate);

}


/*********************************************************************
 * @fn      		  - De-Initialize the USARTx
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
void USART_DeInit()
{
	USART1_REG_RESET();
}


/*********************************************************************
 * @fn      		  - USARTx enable/disable function
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
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDis)
{
	if(EnOrDis == ENABLE)
	{
		USART1->CR1 |= (1 << USARTx_CR1_UE);
	}
	else
	{
		USART1->CR1 &= ~(1 << USARTx_CR1_UE);
	}
}

/*********************************************************************
 * @fn      		  - Function to get the status of the flags USARTx
 *
 * @brief             -
 *
 * @param[in]         - Base address of the USART peripheral
 * @param[in]         - Flag Name
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t FlagName)
{
	if( pUSARTx->ISR & FlagName )
	{
		return FLAG_SET;
	}

	return FLAG_RESET;
}

/*********************************************************************
 * @fn      		  - Function to clear the status of the flags USARTx
 *
 * @brief             -
 *
 * @param[in]         - Base address of the USART peripheral
 * @param[in]         - Flag Name
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
void USART_ClearFLag(USART_RegDef_t *pUSARTx,uint16_t FlagName)
{
	pUSARTx->ISR &= ~(1 << FlagName);
}


/*********************************************************************
 * @fn      		  - USART_SetBaudRate
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -  Resolve all the TODOs

 */
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{

	//Variable to hold the APB clock
	uint32_t PCLKx;

	uint32_t USARTDIV;

	uint32_t tempreg=0;

	//Get the value of APB bus clock in to the variable PCLKx
	if(pUSARTx == USART1)
	{
		//USART1 is hanging on APB2 bus
		PCLKx = RCC_GetPCLK2Value();
	}
	else
	{
		//LPUART1 is hanging on APB1 bus
		PCLKx = RCC_GetPCLK1Value();
	}

	//Check for OVER8 configuration bit
	if(pUSARTx->CR1 & (1 << USARTx_CR1_OVER8))
	{
	    //OVER8 = 1 , over sampling by 8
		USARTDIV = ((2 * PCLKx) / BaudRate);

		//When OVER8 = 0, BRR = USARTDIV
		pUSARTx->BRR |= (uint16_t)USARTDIV;
	}
	else
	{
	   //over sampling by 16
		USARTDIV = (PCLKx / BaudRate);

		//BRR[2:0] = USARTDIV[3:0] shifted 1 bit to the right.
		//BRR[3] must be kept cleared
		tempreg |= ( (USARTDIV & 0xF) >> 1 );
		pUSARTx->BRR |= (uint16_t)tempreg;

		tempreg = 0;
		//BRR[15:4] = USARTDIV[15:4]
		tempreg |= ( (USARTDIV & 0xFFF0) );
		pUSARTx->BRR |= (uint16_t)tempreg;

	}

}



/*********************************************************************
 * @fn      		  - USART_SendData
 *
 * @brief             -
 *
 * @param[in]         - address of USART_Handle_t variable
 * @param[in]         - address of Tx buffer variable
 * @param[in]         - Length
 *
 * @return            -
 *
 * @Note              -

 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Length)
{
	uint16_t *pdata;
   //Loop over until "Length" number of bytes are transferred
	for(uint32_t i = 0 ; i < Length; i++)
	{
		//Implement the code to wait until TXE flag is set in the ISR
		while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TXE) );

         //Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if(pUSARTHandle->USARTConfig.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->TDR = (*pdata & (uint16_t)0x01FF);

			//check for USART_ParityControl
			if(pUSARTHandle->USARTConfig.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used in this transfer. so, 9bits of user data will be sent
				//Implement the code to increment pTxBuffer twice
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				//Parity bit is used in this transfer . so , 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		}
		else if(pUSARTHandle->USARTConfig.USART_WordLength == USART_WORDLEN_8BITS)
		{
			//This is 8bit data transfer
			pUSARTHandle->pUSARTx->TDR = (*pTxBuffer  & (uint8_t)0xFF);

			//The code to increment the buffer address
			pTxBuffer++;
		}
		else
		{
			//This is 7bit data transfer
			pUSARTHandle->pUSARTx->TDR = (*pTxBuffer  & (uint8_t)0x7F);

			//The code to increment the buffer address
			pTxBuffer++;
		}
	}

	//Implement the code to wait till TC flag is set in the ISR
	while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TC));

}


/*********************************************************************
 * @fn      		  - Interrupt based USART Send data function
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - Txstate of USART
 *
 * @Note              -
 */
uint8_t USART_SendDataINT()
{

}


/*********************************************************************
 * @fn      		  - Data receiving function of USARTx
 *
 * @brief             -
 *
 * @param[in]         - Base address of the USART peripheral
 * @param[in]         - Buffer to store the received data
 * @param[in]         - Length of Data Frame
 *
 * @return            -
 *
 * @Note              -
 */
void USART_ReceiveData(USART_RegDef_t *pUSARTx, uint8_t* *pRxBuffer, uint32_t Length)
{

}





/*********************************************************************
 * @fn      		  - To enable/disable the Interrupts of USART
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
void USART_IRQInterrupt_Config(uint8_t IRQNumber, uint8_t EnOrDis)
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
void USART_IRQPriority_Config(uint8_t IRQNumber, uint32_t IRQPriority)
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
 * @fn      		  - Interrupts Handler for USARTx
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
void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{

}



/*
 * Weak implementation of Application Callback function
 */
__weak void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t AppEv)
{
	//If the user application doesn't implement this function, then this function will be called.
}
