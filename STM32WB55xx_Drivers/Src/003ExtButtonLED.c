/*
 * 002ButtonLEDToggle.c
 *
 *  Created on: 12-Apr-2020
 *      Author: Rahul
 */

#include <STM32WB55xx_Gpio_Driver.h>

#define HIGH			1
#define LOW				0
#define BTN_PRESSED 	LOW

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
	//initialise_monitor_handles();

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

	//Enable clock of the respective peripherals
	GPIO_Peri_Clk_Control(GpioLed.pGPIOx, ENABLE); //GPIO_Peri_Clk_Control(GPIOB, ENABLE)
	GPIO_Peri_Clk_Control(GpioButton.pGPIOx, ENABLE); //GPIO_Peri_Clk_Control(GPIOC, ENABLE)

	//Initialize the GPIOs
	GPIO_Init(&GpioLed);
	GPIO_Init(&GpioButton);

	GPIO_ToggleOutputPin(GpioLed.pGPIOx, GpioLed.GPIO_PinConfig.GPIO_PinNumber);
	delay();
	GPIO_ToggleOutputPin(GpioLed.pGPIOx, GpioLed.GPIO_PinConfig.GPIO_PinNumber);
	delay();

	// Configure the Output pin (For Logic Analyzer testing)
	GPIO_Handle_t GpioOutput;
	GpioOutput.pGPIOx = GPIOB;
	GpioOutput.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;
	GpioOutput.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioOutput.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_MEDIUM;
	GpioOutput.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;
	GpioOutput.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&GpioOutput);

	//Toggle the LED
	for(;;)
	{
		/*
		if(GPIO_ReadFromInputPin(GpioButton.pGPIOx, GpioButton.GPIO_PinConfig.GPIO_PinNumber) == BTN_PRESSED)
		{
			delay();
			GPIO_ToggleOutputPin(GpioLed.pGPIOx, GpioLed.GPIO_PinConfig.GPIO_PinNumber); //GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_1)
			GPIO_ToggleOutputPin(GpioOutput.pGPIOx, GpioOutput.GPIO_PinConfig.GPIO_PinNumber); //Logic Analyzer testing

		}
		 */
		while(GPIO_ReadFromInputPin(GpioButton.pGPIOx, GpioButton.GPIO_PinConfig.GPIO_PinNumber));
		delay();
		GPIO_ToggleOutputPin(GpioLed.pGPIOx, GpioLed.GPIO_PinConfig.GPIO_PinNumber); //GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_1)


	}

	return 0;
}

