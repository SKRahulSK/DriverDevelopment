/*
 * 008SPICmdHandling.c
 *
 *  Created on: 25-Apr-2020
 *      Author: Rahul
 */

/*
 * 008SPI_Cmd_Handling.c
 *
 *  Created on: 24-Apr-2020
 *      Author: Rahul
 */

//Have to test the application thoroughly once again with Arduino / Node MCU connections
//Very important

#include <string.h>
#include <stdio.h>
#include <STM32WB55xx_SPI_Driver.h>
#include <STM32WB55xx_Gpio_Driver.h>

extern void initialise_monitor_handles(void);

/*
 * GPIO pins used for SPI2 communication
 * With Alternate Functionality Mode 5 (AF5)
 * PB12 - SPI2_NSS
 * PB13 - SPI2_SCLK
 * PB14 - SPI2_MISO
 * PB15 - SPI2_MOSI
 */

// External button
#define HIGH			1
#define LOW				0
#define BTN_PRESSED 	LOW

//Macros corresponding to slave(Arduino)
#define ARDUINO_LED_PIN			9

#define ARDUINO_LED_ON			1
#define ARDUINO_LED_OFF			0

#define ARDUINO_ANALOG_PIN_0	0
#define ARDUINO_ANALOG_PIN_1	1
#define ARDUINO_ANALOG_PIN_2	2
#define ARDUINO_ANALOG_PIN_3	3
#define ARDUINO_ANALOG_PIN_4	4


//Commands to be sent to slave(Arduino)
#define COMMAND_LED_CNTRL		0x50
#define COMMAND_SENSOR_READ		0x51
#define COMMAND_LED_READ		0x52
#define COMMAND_PRINT			0x53
#define COMMAND_ID_READ			0x54

/*
 * Software delay function
 */
void delay(void)
{
	for( uint32_t i = 0; i <= 500000/2; i++);
}


void SPI2_GPIOInits(void)
{
	//This function is used to initialize the GPIO pins to behave as SPI2 pins
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;

	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_MEDIUM;

	//SCLK pin
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI pin
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO pin
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

	//NSS pin
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);
}

void SPI2_Inits(void)
{
	//This function is used to initialize SPI2 peripheral
	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;

	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8; //Generates SCLK of 2MHz
	SPI2Handle.SPIConfig.SPI_CRCL = SPI_CRCL_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI; //Software Slave Management is disabled for NSS pin, i.e, Hardware Slave Management is enabled.

	SPI_Init(&SPI2Handle);
}


void GPIO_ButtonInit()
{
	GPIO_Handle_t GpioButton;
	GpioButton.pGPIOx = GPIOC; // External button

	//Configure the Button pin
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_MEDIUM;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	//Initialize the button
	GPIO_Init(&GpioButton);
}


uint8_t SPI_VerifyResponse(uint8_t AckByte)
{
	if(AckByte == 0xF5)
	{
		// It is ACK
		return 1;
	}

	return 0; //If NACK
}

void SPI_CommandFunction(uint8_t CommandCode)
{
	uint8_t dummyWrite = 0xff;
	uint8_t dummyRead, args[2], SensorValue, LEDStatus;
	uint8_t AckByte = 0x00;
	char PrintData[] = "Hello";

	if(CommandCode == COMMAND_LED_CNTRL)
	{
		//send command
		SPI_SendData(SPI2, &CommandCode, 1);

		//Do dummy read to clear off the RXNE flag
		SPI_ReceiveData(SPI2, &dummyRead, 1);

		// We should send some dummy bits (1 byte) to fetch response from the slave.
		// Since we are using 8 bit communication, dummy byte is 1 byte. If 16 bit, then dummy byte should by 2 bytes.
		SPI_SendData(SPI2, &dummyWrite, 1);
		SPI_ReceiveData(SPI2, &AckByte, 1);

		if( SPI_VerifyResponse(AckByte) )
		{
			//Send arguments
			args[0] = ARDUINO_LED_PIN;
			args[1] = ARDUINO_LED_ON;
			SPI_SendData(SPI2, args, 2);
			printf("COMMAND LED CONTROL is executed \n");
		}
	}
	else if (CommandCode == COMMAND_SENSOR_READ)
	{
		//send command
		SPI_SendData(SPI2, &CommandCode, 1);

		//Do dummy read to clear off the RXNE flag
		SPI_ReceiveData(SPI2, &dummyRead, 1);

		//Do dummy write to fetch ACK
		SPI_SendData(SPI2, &dummyWrite, 1);
		SPI_ReceiveData(SPI2, &AckByte, 1);

		if( SPI_VerifyResponse(AckByte) )
		{
			//Send arguments
			args[0] = ARDUINO_ANALOG_PIN_0;
			SPI_SendData(SPI2, args, 1);

			//Do dummy read to clear off the RXNE flag
			SPI_ReceiveData(SPI2, &dummyRead, 1);

			//Inserting delay, so that slave can be ready with the data. (ADC conversion can take some time in Arduino)
			delay();

			//Do dummy write to receive the actual sensor value
			SPI_SendData(SPI2, &dummyWrite, 1);
			SPI_ReceiveData(SPI2, &SensorValue, 1);
			printf("COMMAND LED CONTROL is executed \n");
			printf("Sensor Value: %d \n", SensorValue);
		}
	}
	else if (CommandCode == COMMAND_LED_READ)
	{
		//send command
		SPI_SendData(SPI2, &CommandCode, 1);

		//Do dummy read to clear off the RXNE flag
		SPI_ReceiveData(SPI2, &dummyRead, 1);

		//Do dummy write to fetch ACK
		SPI_SendData(SPI2, &dummyWrite, 1);
		SPI_ReceiveData(SPI2, &AckByte, 1);

		if( SPI_VerifyResponse(AckByte) )
		{
			//Send arguments
			args[0] = ARDUINO_LED_PIN;
			SPI_SendData(SPI2, args, 1);

			//Do dummy read to clear off the RXNE flag
			SPI_ReceiveData(SPI2, &dummyRead, 1);

			//Inserting delay, so that slave can be ready with the data. (ADC conversion can take some time in Arduino)
			delay();

			//Do dummy write to receive the actual sensor value
			SPI_SendData(SPI2, &dummyWrite, 1);
			SPI_ReceiveData(SPI2, &LEDStatus, 1);
			printf("COMMAND LED STATUS is executed \n");
			printf("LED Status: %s \n", (LEDStatus)?"ON":"OFF");
		}

	}
	else if (CommandCode == COMMAND_PRINT)
	{
		//send command
		SPI_SendData(SPI2, &CommandCode, 1);

		//Do dummy read to clear off the RXNE flag
		SPI_ReceiveData(SPI2, &dummyRead, 1);

		//Do dummy write to fetch ACK
		SPI_SendData(SPI2, &dummyWrite, 1);
		SPI_ReceiveData(SPI2, &AckByte, 1);

		if( SPI_VerifyResponse(AckByte) )
		{
			//Send arguments
			SPI_SendData(SPI2, (uint8_t*)PrintData, strlen(PrintData));

			printf("COMMAND PRINT is executed \n");
		}

	}
	else if (CommandCode == COMMAND_ID_READ)
	{
		//send command
		SPI_SendData(SPI2, &CommandCode, 1);

		//Do dummy read to clear off the RXNE flag
		SPI_ReceiveData(SPI2, &dummyRead, 1);

		//Do dummy write to fetch ACK
		SPI_SendData(SPI2, &dummyWrite, 1);
		SPI_ReceiveData(SPI2, &AckByte, 1);

		uint8_t id[11];
		uint32_t i=0;
		if( SPI_VerifyResponse(AckByte) )
		{
			//read 10 bytes id from the slave
			for(  i = 0 ; i < 10 ; i++)
			{
				//send dummy byte to fetch data from slave
				SPI_SendData(SPI2,&dummyWrite,1);
				SPI_ReceiveData(SPI2,&id[i],1);
			}

			id[11] = '\0';

			printf("COMMAND ID READ is executed \n");
			printf("COMMAND_ID received: %s \n",id);
		}

	}

}

int main(void)
{
	initialise_monitor_handles();

	printf("Application is running \n");

	GPIO_ButtonInit(); // To initialize the GPIO button

	SPI2_GPIOInits(); // To initialize the GPIO pins to SPI2 functionality

	SPI2_Inits(); //To initialize the SPI2 peripheral parameters

	printf("SPI is initialized \n");
	/*
	 * Making SSOE 1 does NSS output enable
	 * The NSS pin is automatically managed by hardware.
	 * i.e., when SPE=1, NSS will be pulled to low, and NSS will be high when SPE=0
	 */
	SPI_SSOEConfig(SPI2, ENABLE);

	for(;;)
	{
		//Wait until button is pressed
		printf("Waiting for the user to press the button to run the 1st command:\n");
		while( GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_2) );
		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		//Enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		//1. COMMAND_LED_CNTRL
		SPI_CommandFunction(COMMAND_LED_CNTRL);

		//2. COMMAND_SENSOR_READ
		//Wait until button is pressed
		printf("Waiting for the user to press the button to run the 2nd command:\n");
		while( GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_2) );
		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		SPI_CommandFunction(COMMAND_SENSOR_READ);

		//3. COMMAND_LED_READ
		//Wait until button is pressed
		printf("Waiting for the user to press the button to run the 3rd command:\n");
		while( GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_2) );
		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		SPI_CommandFunction(COMMAND_LED_READ);

		//4. COMMAND_PRINT
		//Wait until button is pressed
		printf("Waiting for the user to press the button to run the 4th command:\n");
		while( GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_2) );
		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		SPI_CommandFunction(COMMAND_PRINT);

		//5. COMMAND_ID_READ
		//Wait until button is pressed
		printf("Waiting for the user to press the button to run the 5th command:\n");
		while( GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_2) );
		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		SPI_CommandFunction(COMMAND_ID_READ);


		//Confirm that SPI is not busy
		while( SPI_GetFLagStatus(SPI2, SPI_BSY_FLAG));

		//Disable the SPI2 peripheral only after all the data is transferred successfully
		SPI_PeripheralControl(SPI2, DISABLE);
		printf("SPI communication is closed \n");

	}


	return 0;
}
