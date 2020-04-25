/*
 * 002ButtonLEDToggle.c
 *
 *  Created on: 12-Apr-2020
 *      Author: Rahul
 */


//#include <STM32WB55xx.h>
#include <STM32WB55xx_Gpio_Driver.h>

#define HIGH			1
#define BTN_PRESSED 	HIGH

/*
 * Software delay function
 */
void delay(void)
{
	for( uint32_t i = 0; i <= 500000/2; i++);
}

extern void initialise_monitor_handles(void);

int main(void)
{
	initialise_monitor_handles();

	GPIO_Handle_t GpioLed, GpioButton;

	//Set the port you want to use
	GpioLed.pGPIOx = GPIOB; // LED port
	GpioButton.pGPIOx = GPIOD; // Button port (SW2)

	// Configure the LED pin
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1; //RED LED
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_LOW;
	GpioLed.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	//Configure the Button pin
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0; // SW2
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_LOW;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	//Enable clock of the respective peripherals
	GPIO_Peri_Clk_Control(GpioLed.pGPIOx, ENABLE); //GPIO_Peri_Clk_Control(GPIOB, ENABLE)
	GPIO_Peri_Clk_Control(GpioButton.pGPIOx, ENABLE); //GPIO_Peri_Clk_Control(GPIOB, ENABLE)

	//Initialize the GPIOs
	GPIO_Init(&GpioLed);
	GPIO_Init(&GpioButton);

	//Toggle the LED
	for(;;)
	{
		if(GPIO_ReadFromInputPin(GpioButton.pGPIOx, GpioButton.GPIO_PinConfig.GPIO_PinNumber) == BTN_PRESSED)
		{
			delay();
			GPIO_ToggleOutputPin(GpioLed.pGPIOx, GpioLed.GPIO_PinConfig.GPIO_PinNumber); //GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_1)
		}
	}

	return 0;
}

