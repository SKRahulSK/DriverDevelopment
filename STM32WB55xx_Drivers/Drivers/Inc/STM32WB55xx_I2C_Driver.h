/*
 * STM32WB55xx_I2C_Driver.h
 *
 *  Created on: 26-Apr-2020
 *      Author: Rahul
 */

#ifndef INC_STM32WB55XX_I2C_DRIVER_H_
#define INC_STM32WB55XX_I2C_DRIVER_H_

#include <STM32WB55xx.h>

/*
 * Configuration structure for I2Cx peripheral
 */
typedef struct
{
	uint32_t I2C_SCLSpeed;
	uint8_t	 I2C_DeviceAddress;		//The value will be mentioned by the user
	//uint8_t	 I2C_ACKControl;	//This is not required in STM32WB55
	//uint16_t I2C_FMDutyCycle;		//Fast Mode Duty Cycle - This is not required in STM32WB55
}I2C_Config_t;

/*
 * Handle structure for I2Cx peripheral
 */
typedef struct
{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;

}I2C_Handle_t;

/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM	100000		// Standard Mode 100KHz
#define I2C_SCL_SPEED_FM	400000		// Fast Mode 400KHz
#define I2C_SCL_SPEED_FMP	1000000		// Fast Mode Plust 1MHz
//We can create more macros depending on the interested clock speed
//For Ex:
#define I2C_SCL_SPEED_FM200K	200000		// Fast Mode 200KHz

/*
 * @I2C_ACKControl
 */
//This is not required in STM32WB55

/*
 * @I2C_FMDutyCycle		-- Fast Mode Duty Cycle
 */
//This is not required in STM32WB55


/*****************************************************************************************
 * 						APIs supported by this driver
 * 		For more information about the APIs, check the function definitions
 ****************************************************************************************/

/*
 * To enable/disable the peripheral clock the I2Cx is hanging to
 */
void I2C_Peri_Clk_Control(I2C_RegDef_t *pI2Cx, uint8_t EnOrDis);

/*
 * Initialize and De-initialize/reset the I2C
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*
 * Data send and receive
 */



/*
 * IRQ Configuration and ISR handling
 */
void I2C_IRQInterrupt_Config(uint8_t IRQNumber, uint8_t EnOrDis);
void I2C_IRQPriority_Config(uint8_t IRQNumber, uint32_t IRQPriority);


/*
 * Other peripheral Control APIs
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDis);
uint8_t I2C_GetFLagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);

/*
 * Application Callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);
//This function has to be implemented by the User application.
//If the use doesn't implement it, then compiler will give the error.
//To stop the error, we have to create a weak implementation of the function in our driver C file.


#endif /* INC_STM32WB55XX_I2C_DRIVER_H_ */
