/*
 * STM32WB55xx_RCC_Driver.h
 *
 *  Created on: 01-May-2020
 *      Author: Rahul
 */

#ifndef INC_STM32WB55XX_RCC_DRIVER_H_
#define INC_STM32WB55XX_RCC_DRIVER_H_

#include <STM32WB55xx.h>

//This returns the APB1 clock value
uint32_t RCC_GetPCLK1Value(void);

//This returns the APB2 clock value
uint32_t RCC_GetPCLK2Value(void);


uint32_t  RCC_GetPLLOutputClock(void);

#endif /* INC_STM32WB55XX_RCC_DRIVER_H_ */
