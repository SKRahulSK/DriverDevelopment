/*
 * 004ButtonInterrupt.c
 *
 *  Created on: Apr 13, 2020
 *      Author: Rahul
 */

#include <string.h>

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
	initialise_monitor_handles();

	GPIO_Handle_t GpioLed, GpioButton;

	memset(&GpioLed, 0, sizeof(GpioLed)); //Initialize the members of the structure to zero. In order to avaoid bugs in the program
	memset(&GpioButton, 0, sizeof(GpioButton));

	//Set the port you want to use
	GpioLed.pGPIOx = GPIOB; // LED port (This is internal BLUE LED)
	GpioButton.pGPIOx = GPIOC; // External button

	// Configure the LED pin
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_LOW;
	GpioLed.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	//Configure the Button for External Interrupt Handling
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INT_FT; //Falling Edge trigger
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_LOW;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	//Enable clock of respective peripherals
	GPIO_Peri_Clk_Control(GpioLed.pGPIOx, ENABLE); //GPIO_Peri_Clk_Control(GPIOB, ENABLE)
	GPIO_Peri_Clk_Control(GpioButton.pGPIOx, ENABLE); //GPIO_Peri_Clk_Control(GPIOC, ENABLE)

	//Initialize the GPIOs
	GPIO_Init(&GpioLed);
	GPIO_Init(&GpioButton);

	//IRQ Configuration
	//This is for GPIOB Pin-2
	GPIO_IRQPriority_Config(IRQ_NO_EXTI_2, NVIC_IRQ_PRIORITY_10);
	GPIO_IRQInterrupt_Config(IRQ_NO_EXTI_2, ENABLE);

	//This is for GPIOB Pin-11
	//GPIO_IRQPriority_Config(IRQ_NO_EXTI_10_15, NVIC_IRQ_PRIORITY_15);
	//GPIO_IRQInterrupt_Config(IRQ_NO_EXTI_10_15, ENABLE);

	GPIO_ToggleOutputPin(GpioLed.pGPIOx, GpioLed.GPIO_PinConfig.GPIO_PinNumber);
	delay();
	delay();
	GPIO_ToggleOutputPin(GpioLed.pGPIOx, GpioLed.GPIO_PinConfig.GPIO_PinNumber);

	while(1);

	return 0;
}

/*
 * IRQ Handler for EXTI 10 to 15
 */
void EXTI10_15_IRQHandler(void)
{
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_11); //Clear the pending event from EXTI line
	GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_1);
}

/*
 * IRQ Handler for EXTI 2
 */
void EXTI2_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_NO_2); //Clear the pending event from EXTI line
	GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_5); //Toggle the BLUE LED
}
