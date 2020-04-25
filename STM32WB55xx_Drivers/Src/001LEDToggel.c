/*
 * 001LEDToggel.c
 *
 *  Created on: 12-Apr-2020
 *      Author: Rahul
 */

//#include <STM32WB55xx.h>
#include <STM32WB55xx_Gpio_Driver.h>

/*
 * Software delay function
 */
void delay(void)
{
	for( uint32_t i = 0; i <= 500000; i++);
}

extern void initialise_monitor_handles(void);

int main(void)
{
	initialise_monitor_handles();

	GPIO_Handle_t GpioLed;

	//Set the port you want to use
	GpioLed.pGPIOx = GPIOB;

	// Configure the LED pin
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_LOW;
	GpioLed.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	//Enable clock of the peripheral
	GPIO_Peri_Clk_Control(GpioLed.pGPIOx, ENABLE); //GPIO_Peri_Clk_Control(GPIOB, ENABLE)

	//Initialise the GPIO
	GPIO_Init(&GpioLed);

	//Toggle the LED
	for(;;)
	{
		GPIO_ToggleOutputPin(GpioLed.pGPIOx, GpioLed.GPIO_PinConfig.GPIO_PinNumber); //GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_5)
		delay();
	}

	return 0;
}

