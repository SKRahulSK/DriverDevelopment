/*
 * 015USART_Tx.c
 *
 *  Created on: 01-May-2020
 *      Author: Rahul
 */

#include <STM32WB55xx_USART_Driver.h>
#include <STM32WB55xx_Gpio_Driver.h>
#include <string.h>


char Txmsg[1024] = "USART Tx Testing!! \n";

USART_Handle_t USART1Handle;

void delay(void)
{
	for( uint32_t i = 0; i <= 500000/2; i++);
}

void USART1_GPIO_Init(void)
{
	/*
	 * GPIOA is used for USART1 here
	 * With Alternate Functionality Mode 7 (AF7)
	 * PA8	->	CLK
	 * PA9	->	Tx
	 * PA10	->	Rx
	 * PA11	->	CTS
	 * PA12	->	RTS
	 */

	GPIO_Handle_t GpioUSART;

	GpioUSART.pGPIOx = GPIOA;
	GpioUSART.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	GpioUSART.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_FAST;
	GpioUSART.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;
	GpioUSART.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	GpioUSART.GPIO_PinConfig.GPIO_PinAltFunMode = 7; //Alternate functionality mode 7 (from data sheet)

	//Tx pin configuration
	GpioUSART.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	GPIO_Init(&GpioUSART);

	//Rx pin configuration
	GpioUSART.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
	GPIO_Init(&GpioUSART);

}

void USART1_Init()
{
	USART1Handle.pUSARTx = USART1;
	USART1Handle.USARTConfig.USART_Mode = USART_MODE_ONLY_TX;
	USART1Handle.USARTConfig.USART_BaudRate = 115200;
	USART1Handle.USARTConfig.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	USART1Handle.USARTConfig.USART_NoOfStopBits = USART_STOPBITS_1_5;
	USART1Handle.USARTConfig.USART_ParityControl = USART_PARITY_DISABLE;
	USART1Handle.USARTConfig.USART_WordLength = USART_WORDLEN_8BITS;

	USART_Init(&USART1Handle);

}


void GPIO_Button_LED_Init()
{
	GPIO_Handle_t GpioLed, GpioButton;

	//Set the port you want to use
	GpioLed.pGPIOx = GPIOB; // LED port (This is internal BLUE LED)
	GpioButton.pGPIOx = GPIOC; // External button

	// Configure the LED pin
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_MEDIUM;
	GpioLed.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	//Configure the Button pin
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_MEDIUM;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	//Initialize the GPIOs
	GPIO_Init(&GpioLed);
	GPIO_Init(&GpioButton);

}


int main()
{

	GPIO_Button_LED_Init();

	USART1_GPIO_Init();

	USART1_Init();

	USART_PeripheralControl(USART1, ENABLE);


	while(1)
	{
		//GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_5,GPIO_PIN_RESET);

		//Wait till button is pressed
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_2));

		//to avoid button de-bouncing related issues
		delay();

		//Switch on the LED
		GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_5);

		//Send the msg to NodeMCU
		USART_SendData(&USART1Handle,(uint8_t*)Txmsg, strlen(Txmsg));

	}

	return 0;
}
