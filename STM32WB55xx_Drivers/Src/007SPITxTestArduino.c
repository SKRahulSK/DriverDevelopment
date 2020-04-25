/*
 * 006_2_SPITxTestArduino.c
 *
 *  Created on: 22-Apr-2020
 *      Author: Rahul
 */

//This has to be tested with Arduino /NodeMCU
//Very important


#include <string.h>
#include <STM32WB55xx_SPI_Driver.h>
#include <STM32WB55xx_Gpio_Driver.h>

/*
 * GPIO pins used for SPI2 communication
 * With Alternate Functionality Mode 5 (AF5)
 * PB12 - SPI2_NSS
 * PB13 - SPI2_SCLK
 * PB14 - SPI2_MISO
 * PB15 - SPI2_MOSI
 */


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
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	//GPIO_Init(&SPIPins);

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
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_LOW;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	//Initialize the button
	GPIO_Init(&GpioButton);
}


int main(void)
{

	char user_data[] = "Hello World";

	GPIO_ButtonInit(); // To initialize the GPIO button

	SPI2_GPIOInits(); // To initialize the GPIO pins to SPI2 functionality

	SPI2_Inits(); //To initialize the SPI2 peripheral parameters

	/*
	 * Making SSOE 1 does NSS output enable
	 * The NSS pin is automatically managed by hardware.
	 * i.e., when SPE=1, NSS will be pulled to low, and NSS will be high when SPE=0
	 */
	SPI_SSOEConfig(SPI2, ENABLE);

	for(;;)
	{
		//uint8_t press_value = GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_2);
		while( ! GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_2) ); //Check for button pressed

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		//Enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		//First send length information to slave.
		uint8_t dataLen = strlen(user_data);
		SPI_SendData(SPI2, &dataLen, 1);

		//Send data
		SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

		//Confirm that SPI is not busy
		while( SPI_GetFLagStatus(SPI2, SPI_BSY_FLAG));

		//Disable the SPI2 peripheral only after all the data is transferred successfully
		SPI_PeripheralControl(SPI2, DISABLE);

	}


	return 0;
}


