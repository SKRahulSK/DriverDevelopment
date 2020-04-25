/*
 * STM32WB55xx_Gpio_Driver.h
 *
 *  Created on: Apr 10, 2020
 *      Author: Rahul
 */

#ifndef SRC_STM32WB55XX_GPIO_DRIVER_H_
#define SRC_STM32WB55XX_GPIO_DRIVER_H_


#include "STM32WB55xx.h" // The MCU specific headerfile has to be included


/*
 * This is a Configurable structure for a GPIO pin
 */

typedef struct
{
	uint8_t GPIO_PinNumber;			/*!< possible values from @GPIO_PIN_NUMBERS >*/
	uint8_t GPIO_PinMode;			/*!< possible values from @GPIO_PIN_MODES >*/
	uint8_t GPIO_PinSpeed;			/*!< possible values from @GPIO_PIN_SPEEDS >*/
	uint8_t GPIO_PinPuPdControl;	/*!< possible values from @GPIO_PIN_PUPD_CONFIG >*/
	uint8_t GPIO_PinOpType;			/*!< possible values from @GPIO_PIN_OP_TYPES >*/
	uint8_t GPIO_PinAltFunMode;		/*!< possible values from @GPIO_PIN_ALT_FUN_MODES >*/
}GPIO_PinConfig_t;


/*
 * This is a Handle structure for a GPIO pin
 */

typedef struct
{
	GPIO_RegDef_t *pGPIOx; //This holds the base address of the GPIO port to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;


/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin possible numbers
 */
#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15

/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN		0	//Input mode
#define GPIO_MODE_OUT		1	//Output mode
#define GPIO_MODE_ALTFN		2	//Alternate Functionality mode
#define GPIO_MODE_ANALOG	3	//Analog mode
#define GPIO_MODE_INT_FT	4	//Interrupt mode with Falling edge Trigger
#define GPIO_MODE_INT_RT	5	//Interrupt mode with Rising edge Trigger
#define GPIO_MODE_INT_RFT	6	//Interrupt mode with Rising and Falling edge Trigger

/*
 * @GPIO_PIN_OP_TYPES
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1

/*
 * @GPIO_PIN_SPEEDS
 * GPIO pin possible output speeds
 */
#define GPIO_OP_SPEED_LOW		0
#define GPIO_OP_SPEED_MEDIUM	1
#define GPIO_OP_SPEED_FAST		2
#define GPIO_OP_SPEED_HIGH		3

/*
 * @GPIO_PIN_PUPD_CONFIG
 * GPIO pin possible pull-up/pull-down configuration macros
 */
#define GPIO_NO_PUPD		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2


/*
 * @GPIO_PIN_ALT_FUN_MODES
 * GPIO pin possible alternate functionality modes
 */

/*****************************************************************************************
 * 						APIs supported by this driver
 * 		For more information about the APIs, check the function definitions
 ****************************************************************************************/

/*
 * To enable/disable the peripheral clock the GPIO port is hanging to
 */
void GPIO_Peri_Clk_Control(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDis);

/*
 * Initialize and De-initialize/reset the GPIO port
 */
void GPIO_Init(GPIO_Handle_t *GPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);	//To read from input pin
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);	//To read from input port
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);	//To write to output pin
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);	//To write to output port
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQInterrupt_Config(uint8_t IRQNumber, uint8_t EnOrDis);
void GPIO_IRQPriority_Config(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);



#endif /* SRC_STM32WB55XX_GPIO_DRIVER_H_ */
