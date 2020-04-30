/*
 * STM32WB55xx_USART_Driver.h
 *
 *  Created on: 30-Apr-2020
 *      Author: Rahul
 */

#ifndef INC_STM32WB55XX_USART_DRIVER_H_
#define INC_STM32WB55XX_USART_DRIVER_H_


#include <STM32WB55xx.h>

/*
 * Configuration structure for USARTx
 */

typedef struct
{
	uint8_t USART_Mode;
	uint32_t USART_BaudRate;
	uint8_t USART_NoOfStopBits;
	uint8_t USART_WordLength;
	uint8_t USART_ParityControl;
	uint8_t USART_HWFlowControl;
}USART_Config_t;


/*
 * Handle structure for USARTx
 */

typedef struct
{
	USART_RegDef_t *pUSARTx;
	USART_Config_t USARTConfig;
}USART_Handle_t;


/*
 *@USART_Mode
 *Possible options for USART_Mode
 */
#define USART_MODE_ONLY_TX 	0
#define USART_MODE_ONLY_RX 	1
#define USART_MODE_TXRX  	2

/*
 *@USART_Baud
 *Possible options for USART_Baud
 */
#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 					2000000
#define SUART_STD_BAUD_3M 					3000000


/*
 *@USART_ParityControl
 *Possible options for USART_ParityControl
 */
#define USART_PARITY_EN_ODD   2
#define USART_PARITY_EN_EVEN  1
#define USART_PARITY_DISABLE   0

/*
 *@USART_WordLength
 *Possible options for USART_WordLength
 */
#define USART_WORDLEN_8BITS  0
#define USART_WORDLEN_9BITS  1
#define USART_WORDLEN_7BITS	 2

/*
 *@USART_NoOfStopBits
 *Possible options for USART_NoOfStopBits
 */
#define USART_STOPBITS_1     0
#define USART_STOPBITS_0_5   1
#define USART_STOPBITS_2     2
#define USART_STOPBITS_1_5   3

/*
 *@USART_HWFlowControl
 *Possible options for USART_HWFlowControl
 */
#define USART_HW_FLOW_CTRL_NONE    	0
#define USART_HW_FLOW_CTRL_CTS    	1
#define USART_HW_FLOW_CTRL_RTS    	2
#define USART_HW_FLOW_CTRL_CTS_RTS	3

/*
 * USART flags
 */

#define USART_FLAG_TXFE 		( 1 << USARTx_ISR_TXFE)
#define USART_FLAG_RXFNE 		( 1 << USARTx_ISR_RXFNE)
#define USART_FLAG_TC 			( 1 << USART_SR_TC)

/*
 * Application states
 */
#define USART_READY 		0
#define USART_BUSY_IN_RX 	1
#define USART_BUSY_IN_TX 	2

#define USART_EVENT_TX_CMPLT   	0
#define	USART_EVENT_RX_CMPLT   	1
#define	USART_EVENT_IDLE	   	2
#define	USART_EVENT_CTS        	3
#define	USART_EVENT_PE         	4
#define	USART_ERR_FE     	   	5
#define	USART_ERR_NE    	   	6
#define	USART_ERR_ORE    	   	7

/*****************************************************************************************
 * 						APIs supported by this driver
 * 		For more information about the APIs, check the function definitions
 ****************************************************************************************/
// Since there is only one USART communication peripheral (USART1),
// some of the functions defined here are directly using the USART1 macros without getting base address parameter in the function call.
/*
 * To enable/disable the peripheral clock the USARTx is hanging to
 */
void USART_Peri_Clk_Control(uint8_t EnOrDis);

/*
 * Initialize and De-initialize/reset the USARTx
 */
void USART_Init(USART_RegDef_t *pUSARTx, USART_Handle_t *pUSARTHandle);
void USART_DeInit();

/*
 * Data send and receive
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Length);
void USART_ReceiveData();

uint8_t USART_SendDataINT(); //Interrupt based
uint8_t USART_ReceiveDataINT(); //Interrupt based

/*
 * IRQ Configuration and ISR handling
 */
void USART_IRQInterrupt_Config(uint8_t IRQNumber, uint8_t EnOrDis);
void USART_IRQPriority_Config(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *pUSARTHandle);


/*
 * Other peripheral Control APIs
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDis);
uint8_t USART_GetFLagStatus(USART_RegDef_t *pUSARTx, uint32_t FlagName);
void USART_ClearFLag(USART_RegDef_t *pUSARTx, uint16_t StatuFlagName);

/*
 * Application callback
 */
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t AppEv);





#endif /* INC_STM32WB55XX_USART_DRIVER_H_ */
