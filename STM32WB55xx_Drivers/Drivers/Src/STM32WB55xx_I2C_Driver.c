/*
 * STM32WB55xx_I2C_Driver.c
 *
 *  Created on: 26-Apr-2020
 *      Author: Rahul
 */


#include <STM32WB55xx_I2C_Driver.h> // The header file specific to I2C has to be included



/*********************************************************************
 * @fn      		  - I2C_Peri_Clk_Control
 *
 * @brief             - This function enables or disables peripheral clock for the given I2C
 *
 * @param[in]         - Base address of the I2C peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  N/A
 */

void I2C_Peri_Clk_Control(I2C_RegDef_t *pI2Cx, uint8_t EnOrDis)
{
	if(EnOrDis == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}
	else {
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		}
	}
}


/*
 * Initialize and De-initialize/reset the I2Cx
 */

/*********************************************************************
 * @fn      		  - Initialize the I2Cx
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{


}


/*********************************************************************
 * @fn      		  - De-Initialize the I2Cx
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if(pI2Cx == I2C1)
	{
		I2C1_REG_RESET();
	}
	else if (pI2Cx == I2C3) {
		I2C3_REG_RESET();
	}
}


/*********************************************************************
 * @fn      		  - The function Enables/Disables the I2C peripheral
 *
 * @brief             -
 *
 * @param[in]         - Base address of the I2C peripheral
 * @param[in]         - ENABLE (1) / DISABLE (0)
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - PE bit of I2Cx->CR1 register is set/reset
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDis)
{
	if(EnOrDis == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2Cx_CR1_PE);
	}
	else if (EnOrDis == DISABLE)
	{
		pI2Cx->CR1 &= ~(1 << I2Cx_CR1_PE);
	}
}


