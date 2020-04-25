/*
 * 006SPITxTest.c
 *
 *  Created on: 20-Apr-2020
 *      Author: Rahul
 */
#include <string.h>
#include <STM32WB55xx_SPI_Driver.h>

/*
 * GPIO pins used for SPI2 communication
 * With Alternate Functionality Mode 5 (AF5)
 * PB12 - SPI2_NSS
 * PB13 - SPI2_SCLK
 * PB14 - SPI2_MISO
 * PB15 - SPI2_MOSI
 */

void SPI2_GPIOInits(void)
{
	//This function is used to initialize the GPIO pins to behave as SPI2 pins
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;

	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
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
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	//GPIO_Init(&SPIPins);
}

void SPI2_Inits(void)
{
	//This function is used to initialize SPI2 peripheral
	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;

	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2; //Generates SCLK of 8MHz
	SPI2Handle.SPIConfig.SPI_CRCL = SPI_CRCL_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_EN; //Software Slave Management is enabled for NSS pin

	SPI_Init(&SPI2Handle);
}


int main(void)
{

	char user_data[] = "Hello World";

	SPI2_GPIOInits(); // To initialize the GPIO pins to SPI2 functionality

	SPI2_Inits(); //To initialize the SPI2 peripheral parameters

	//This makes the NSS signal high and avoid MODEF error
	SPI_SSIConfig(SPI2, ENABLE);

	//Enable the SPI2 peripheral
	SPI_PeripheralControl(SPI2, ENABLE);

	//Send data
	SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

	//Confirm that SPI is not busy
	while( SPI_GetFLagStatus(SPI2, SPI_BSY_FLAG) );

	//Disable the SPI2 peripheral only after all the data is transferred successfully
	SPI_PeripheralControl(SPI2, DISABLE);

	while(1);

	return 0;
}
