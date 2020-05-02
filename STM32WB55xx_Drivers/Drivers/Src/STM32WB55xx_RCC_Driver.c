/*
 * STM32WB55xx_RCC_Driver.c
 *
 *  Created on: 01-May-2020
 *      Author: Rahul
 */

#include <STM32WB55xx_RCC_Driver.h>
#include <STM32WB55xx.h>

uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB_PreScaler[4] = {2,4,8,16};


/*********************************************************************
 * @fn      		  - RCC_GetPCLK1Value
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
uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1,SystemClk;

	uint8_t clksrc,temp,ahbp,apb1p;

	clksrc = ((RCC->CFGR >> 2) & 0x3);

	if(clksrc == 0 )
	{
		//MSI Oscillator 100KHz to 48MHz
		SystemClk = 100000;
	}
	else if(clksrc == 1)
	{
		//HSI16 oscillator
		SystemClk = 16000000;
	}
	else if (clksrc == 2)
	{
		//HSE is used
		SystemClk = 32000000;
	}
	else if (clksrc == 3) {
		//PLL is used
		SystemClk = RCC_GetPLLOutputClock();
	}

	//AHB Prescaler
	temp = (RCC->CFGR >> 4 ) & 0xF;
	if(temp >= 8)
	{
		ahbp = AHB_PreScaler[temp-8];
	}
	else if(temp == 1)
	{
		ahbp = 3;
	}
	else if (temp == 2) {
		ahbp = 5;
	}
	else if (temp == 5) {
		ahbp = 6;
	}
	else if (temp == 6) {
		ahbp = 10;
	}
	else if (temp == 7) {
		ahbp = 32;
	}
	else {
		ahbp = 1;
	}


	//APB1
	temp = ((RCC->CFGR >> 8 ) & 0x7);

	if(temp < 4)
	{
		apb1p = 1;
	}else
	{
		apb1p = APB_PreScaler[temp-4];
	}

	pclk1 =  (SystemClk / ahbp) /apb1p;


	return pclk1;
}



/*********************************************************************
 * @fn      		  - RCC_GetPCLK2Value
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
uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t SystemClk=0,temp,pclk2;
	uint8_t clksrc = ( RCC->CFGR >> 2) & 0X3;

	uint8_t ahbp,apb2p;

	if(clksrc == 0 )
	{
		//MSI Oscillator 100KHz to 48MHz
		//The MSI is used as system clock source after startup from Reset, configured at 4 MHz.
		SystemClk = 4000000;
	}
	else if(clksrc == 1)
	{
		//HSI16 oscillator
		SystemClk = 16000000;
	}
	else if (clksrc == 2)
	{
		//HSE is used
		SystemClk = 32000000;
	}
	else if (clksrc == 3) {
		//PLL is used
		SystemClk = RCC_GetPLLOutputClock();
	}

	//AHB Prescaler
	temp = (RCC->CFGR >> 4 ) & 0xF;
	if(temp >= 8)
	{
		ahbp = AHB_PreScaler[temp-8];
	}
	else if(temp == 1)
	{
		ahbp = 3;
	}
	else if (temp == 2) {
		ahbp = 5;
	}
	else if (temp == 5) {
		ahbp = 6;
	}
	else if (temp == 6) {
		ahbp = 10;
	}
	else if (temp == 7) {
		ahbp = 32;
	}
	else {
		ahbp = 1;
	}

	// APB2 Prescaler
	temp = (RCC->CFGR >> 11 ) & 0x7;
	if(temp < 0x04)
	{
		apb2p = 1;
	}else
	{
		apb2p = APB_PreScaler[temp-4];
	}

	pclk2 = (SystemClk / ahbp )/ apb2p;

	return pclk2;
}

/*********************************************************************
 * @fn      		  - RCC_GetPLLOutputClock
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
uint32_t  RCC_GetPLLOutputClock()
{

	return 0;
}
