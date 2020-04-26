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
	uint8_t	 I2C_DeviceAddress;
	uint8_t	 I2C_ACKControl;
	uint16_t I2C_FMDutyCycle;
}I2C_Config_t;

/*
 * Handle structure for I2Cx peripheral
 */
typedef struct
{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;

}I2C_Handle_t;



#endif /* INC_STM32WB55XX_I2C_DRIVER_H_ */
