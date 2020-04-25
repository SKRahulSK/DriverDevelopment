/*
 * STM32WB55xx_Gpio_Driver.c
 *
 *  Created on: Apr 10, 2020
 *      Author: Rahul
 */

#include "STM32WB55xx_Gpio_Driver.h" // The headerfile specific to Gpio has to be included


/*
 * To enable/disable the peripheral clock the GPIO port is hanging to
 */

/*********************************************************************
 * @fn      		  - GPIO_Peri_Clk_Control
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - Base address of the GPIO peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  N/A
 */

void GPIO_Peri_Clk_Control(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDis)
{
	if(EnOrDis == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PERI_CLK_EN();
		}else if (pGPIOx == GPIOB) {
			GPIOB_PERI_CLK_EN();
		}else if (pGPIOx == GPIOC) {
			GPIOC_PERI_CLK_EN();
		}else if (pGPIOx == GPIOD) {
			GPIOD_PERI_CLK_EN();
		}else if (pGPIOx == GPIOE) {
			GPIOE_PERI_CLK_EN();
		}else if (pGPIOx == GPIOH) {
			GPIOH_PERI_CLK_EN();
		}
	}else {
		if(pGPIOx == GPIOA)
		{
			GPIOA_PERI_CLK_DI();
		}else if (pGPIOx == GPIOB) {
			GPIOB_PERI_CLK_DI();
		}else if (pGPIOx == GPIOC) {
			GPIOC_PERI_CLK_DI();
		}else if (pGPIOx == GPIOD) {
			GPIOD_PERI_CLK_DI();
		}else if (pGPIOx == GPIOE) {
			GPIOE_PERI_CLK_DI();
		}else if (pGPIOx == GPIOH) {
			GPIOH_PERI_CLK_DI();
		}
	}

}


/*
 * Initialize and De-initialize/reset the GPIO port
 */

/*********************************************************************
 * @fn      		  - GPIO Initialize
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
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0; //temporary register/variable

	//Very important step is to enable the clock of the BUS for which the given GPIO port is hanging to
	GPIO_Peri_Clk_Control(pGPIOHandle->pGPIOx, ENABLE);

	// 1. Configure the mode of the GPIO pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//It is a non-interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //Clearing
		pGPIOHandle->pGPIOx->MODER |= temp; //Setting
	}else{

		//It is an interrupt mode
		//We should configure the GPIO pin as input mode to make it work as interrrupt mode.
		temp = (GPIO_MODE_IN << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //Clearing
		pGPIOHandle->pGPIOx->MODER |= temp; //Setting

		//A) First configure the Trigger registers accordingly
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_INT_FT)
		{
			//a) Configure the Falling-edge Trigger Selection Register (FTSR)
			EXTI->FTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//b) Clear the corresponding RTSR bit (To make sure that RT is not configured)
			EXTI->RTSR1 &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_INT_RT) {
			//a) Configure the Rising-edge Trigger Selection Register (RTSR)
			EXTI->RTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//b) Clear the corresponding FTSR bit (To make sure that FT is not configured)
			EXTI->FTSR1 &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);


		}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_INT_RFT) {
			//a) Configure both FTSR and RTSR
			EXTI->FTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//B) Configure the GPIO port selection in SYSCFG_EXTICR (System Config EXTernal Interrupt Control Register)
		//B.1)
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4; // the index of EXTICR register
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4; // the section of the corresponding EXTICR register where the PortCode has to be stored

		uint8_t PortCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx); //Port Code related to given GPIO port

		//Enable the clock for SYSCFG peripheral before configuring the registers
		//SYSCFG_PCLK_EN(); //In STM32WB55, the SYSCFG clock is already enabled. So no need for this. However, this is board specific.

		SYSCFG->EXTICR[temp1] = PortCode << (temp2 * 4);

		//C) Enable the EXTI interrupt delivery using IMR (Interrupt Mask Register)
		EXTI->IMR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		EXTI->EMR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // I have added (according to reference manual of wb55)

	}

	temp =0;

	// 2. Configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //Clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;

	// 3. Configure the push-pull or Open-drain settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //Clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;

	// 4. Configure the output type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOpType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x3 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //Clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp =0;

	// 5. Configure the alternate functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{

		//Then configure the registers corresponding to alternate functionality
		//temp1 variable holds the index of AFR[] variable
		//temp2 variable holds the number of times to shift....(should write it in understandable way)

		uint8_t temp1, temp2;
		temp1 =	temp2 = 0;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4*temp2)); //Clearing
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4*temp2)); //Setting

		temp1=temp2=0;
	}




}

/*********************************************************************
 * @fn      		  - De-Initialize GPIO port
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
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}else if (pGPIOx == GPIOB) {
		GPIOB_REG_RESET();
	}else if (pGPIOx == GPIOC) {
		GPIOC_REG_RESET();
	}else if (pGPIOx == GPIOD) {
		GPIOD_REG_RESET();
	}else if (pGPIOx == GPIOE) {
		GPIOE_REG_RESET();
	}else if (pGPIOx == GPIOH) {
		GPIOH_REG_RESET();
	}

}


/*
 * Data read and write
 */
/*********************************************************************
 * @fn      		  - Read from GPIO input pin
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - 0 or 1
 *
 * @Note              -
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t) ((pGPIOx->IDR >> PinNumber) & (0x00000001));
	return value;
}

/*********************************************************************
 * @fn      		  - Read from GPIO input port
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
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)	//To read from input port
{
	uint16_t value;
	value = (uint16_t) (pGPIOx->IDR);
	return value;

}

/*********************************************************************
 * @fn      		  - Write to GPIO output pin
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
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)	//To write to output pin
{
	if(Value == GPIO_PIN_SET)
	{
		// Write 1 to the output data register (ODR) at the bit field corresponding PinNumber.
		pGPIOx->ODR |= (1 << PinNumber);
	}else if (Value == GPIO_PIN_RESET) {
		// Write 0 to the output data register (ODR) at the bit filed corresponding PinNumber.
		pGPIOx->ODR &= ~(1 << PinNumber);
	}


}

/*********************************************************************
 * @fn      		  - Write to GPIO output port
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
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)	//To write to output port
{
	pGPIOx->ODR = Value;
}

/*********************************************************************
 * @fn      		  - Toggle GPIO output pin
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
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber); //To toggle the bit, we use XOR operation
}


/*
 * IRQ Configuration and ISR handling
 */

/*********************************************************************
 * @fn      		  - To enable/disable the Interrupt
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
void GPIO_IRQInterrupt_Config(uint8_t IRQNumber, uint8_t EnOrDis)
{
	//This configuration is done at the processor side
	if(EnOrDis == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//Program ISER0 register
			*NVIC_ISER0 = (1 << IRQNumber);

		}else if(IRQNumber >= 32 && IRQNumber <= 63)
		{
			//Program ISER1 register
			*NVIC_ISER1 = (1 << (IRQNumber % 32));

		}else if(IRQNumber >= 64 && IRQNumber <= 95)
		{
			//Program ISER2 register
			*NVIC_ISER2 = (1 << (IRQNumber % 64));
		}
	}else {
		if(IRQNumber <= 31)
		{
			//Program ICER0 register
			*NVIC_ICER0 = (1 << IRQNumber);

		}else if(IRQNumber >= 32 && IRQNumber <= 63)
		{
			//Program ICER1 register
			*NVIC_ICER1 = (1 << (IRQNumber % 32));

		}else if(IRQNumber >= 64 && IRQNumber <= 95)
		{
			//Program ICER2 register
			*NVIC_ICER1 = (1 << (IRQNumber % 64));
		}
	}

}

/*********************************************************************
 * @fn      		  - To set priorities to the Interrupts
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
void GPIO_IRQPriority_Config(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//Find out the IPR register corresponding to IRQNumber
	uint8_t iprx = IRQNumber / 4;

	//Find out the section in the IPR register corresponding to IRQNumber
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	//Now store the Priority value in the iprx section.
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);

}


/*********************************************************************
 * @fn      		  - The IRQ Handle function
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
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//Clear the EXTI PR register corresponding the PinNumber
	if(EXTI->PR1 & (1 << PinNumber))
	{
		//Clear the bit
		EXTI->PR1 |= (1 << PinNumber);
	}

}

