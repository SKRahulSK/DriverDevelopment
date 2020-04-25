/**
 ******************************************************************************
 * @file      startup_stm32wb55rgvx.s
 * @author    Auto-generated by STM32CubeIDE
 * @brief     STM32WB55RGVx device vector table for GCC toolchain.
 *            This module performs:
 *                - Set the initial SP
 *                - Set the initial PC == Reset_Handler,
 *                - Set the vector table entries with the exceptions ISR address
 *                - Branches to main in the C library (which eventually
 *                  calls main()).
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

.syntax unified
.cpu cortex-m4
.fpu softvfp
.thumb

.global g_pfnVectors
.global Default_Handler

/* start address for the initialization values of the .data section.
defined in linker script */
.word _sidata
/* start address for the .data section. defined in linker script */
.word _sdata
/* end address for the .data section. defined in linker script */
.word _edata
/* start address for the .bss section. defined in linker script */
.word _sbss
/* end address for the .bss section. defined in linker script */
.word _ebss

/**
 * @brief  This is the code that gets called when the processor first
 *          starts execution following a reset event. Only the absolutely
 *          necessary set is performed, after which the application
 *          supplied main() routine is called.
 * @param  None
 * @retval : None
*/

  .section .text.Reset_Handler
  .weak Reset_Handler
  .type Reset_Handler, %function
Reset_Handler:
  ldr   r0, =_estack
  mov   sp, r0          /* set stack pointer */

/* Copy the data segment initializers from flash to SRAM */
  ldr r0, =_sdata
  ldr r1, =_edata
  ldr r2, =_sidata
  movs r3, #0
  b LoopCopyDataInit

CopyDataInit:
  ldr r4, [r2, r3]
  str r4, [r0, r3]
  adds r3, r3, #4

LoopCopyDataInit:
  adds r4, r0, r3
  cmp r4, r1
  bcc CopyDataInit

/* Zero fill the bss segment. */
  ldr r2, =_sbss
  ldr r4, =_ebss
  movs r3, #0
  b LoopFillZerobss

FillZerobss:
  str  r3, [r2]
  adds r2, r2, #4

LoopFillZerobss:
  cmp r2, r4
  bcc FillZerobss

/* Call the clock system intitialization function.*/
  bl  SystemInit
/* Call static constructors */
  bl __libc_init_array
/* Call the application's entry point.*/
  bl main

LoopForever:
    b LoopForever

  .size Reset_Handler, .-Reset_Handler

/**
 * @brief  This is the code that gets called when the processor receives an
 *         unexpected interrupt.  This simply enters an infinite loop, preserving
 *         the system state for examination by a debugger.
 *
 * @param  None
 * @retval : None
*/
  .section .text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_Loop:
  b Infinite_Loop
  .size Default_Handler, .-Default_Handler

/******************************************************************************
*
* The STM32WB55RGVx vector table.  Note that the proper constructs
* must be placed on this to ensure that it ends up at physical address
* 0x0000.0000.
*
******************************************************************************/
  .section .isr_vector,"a",%progbits
  .type g_pfnVectors, %object
  .size g_pfnVectors, .-g_pfnVectors

g_pfnVectors:
  .word _estack
  .word Reset_Handler
  .word NMI_Handler
  .word HardFault_Handler
  .word	MemManage_Handler
  .word	BusFault_Handler
  .word	UsageFault_Handler
  .word	0
  .word	0
  .word	0
  .word	0
  .word	SVC_Handler
  .word	DebugMon_Handler
  .word	0
  .word	PendSV_Handler
  .word	SysTick_Handler
  .word	WWDG_IRQHandler              			/* Window Watchdog interrupt                                          */
  .word	PVD_IRQHandler               			/* PVD through EXTI[16] (C1IMR2[20])                                  */
  .word	RTC_TAMP_IRQHandler          			/* RTC/TAMP/CSS on LSE through EXTI line 19 interrupt                 */
  .word	RTC_WKUP_IRQHandler          			/* RTC wakeup interrupt through EXTI[19]                              */
  .word	FLASH_IRQHandler             			/* Flash global interrupt                                             */
  .word	RCC_IRQHandler               			/* RCC global interrupt                                               */
  .word	EXTI0_IRQHandler             			/* EXTI line 0 interrupt through EXTI[0]                              */
  .word	EXTI1_IRQHandler             			/* EXTI line 0 interrupt through EXTI[1]                              */
  .word	EXTI2_IRQHandler             			/* EXTI line 0 interrupt through EXTI[2]                              */
  .word	EXTI3_IRQHandler             			/* EXTI line 0 interrupt through EXTI[3]                              */
  .word	EXTI4_IRQHandler             			/* EXTI line 0 interrupt through EXTI[4]                              */
  .word	DMA1_Channel1_IRQHandler     			/* DMA1 Channel1 global interrupt                                     */
  .word	DMA1_Channel2_IRQHandler     			/* DMA1 Channel2 global interrupt                                     */
  .word	DMA1_Channel3_IRQHandler     			/* DMA1 Channel3 interrupt                                            */
  .word	DMA1_Channel4_IRQHandler     			/* DMA1 Channel4 interrupt                                            */
  .word	DMA1_Channel5_IRQHandler     			/* DMA1 Channel5 interrupt                                            */
  .word	DMA1_Channel6_IRQHandler     			/* DMA1 Channel6 interrupt                                            */
  .word	DMA1_Channel7_IRQHandler     			/* DMA1 Channel 7 interrupt                                           */
  .word	ADC1_IRQHandler              			/* ADC1 global interrupt                                              */
  .word	USB_HP_IRQHandler            			/* USB high priority interrupt                                        */
  .word	USB_LP_IRQHandler            			/* USB low priority interrupt (including USB wakeup)                  */
  .word	C2SEV_IRQHandler             			/* CPU2 SEV through EXTI[40]                                          */
  .word	COMP_IRQHandler              			/* COMP2 & COMP1 interrupt through AIEC[21:20]                        */
  .word	EXTI5_9_IRQHandler           			/* EXTI line [9:5] interrupt through EXTI[9:5]                        */
  .word	TIM1_BRK_IRQHandler          			/* Timer 1 break interrupt                                            */
  .word	TIM1_UP_IRQHandler           			/* Timer 1 Update                                                     */
  .word	TIM1_TRG_COM_TIM17_IRQHandler			/* TIM1 Trigger and Commutation interrupts and TIM17 global interrupt */
  .word	TIM1_CC_IRQHandler           			/* TIM1 Capture Compare interrupt                                     */
  .word	TIM2_IRQHandler              			/* TIM2 global interrupt                                              */
  .word	PKA_IRQHandler               			/* Private key accelerator interrupt                                  */
  .word	I2C1_EV_IRQHandler           			/* I2C1 event interrupt                                               */
  .word	I2C1_ER_IRQHandler           			/* I2C1 error interrupt                                               */
  .word	I2C3_EV_IRQHandler           			/* I2C3 event interrupt                                               */
  .word	I2C3_ER_IRQHandler           			/* I2C3 error interrupt                                               */
  .word	SPI1_IRQHandler              			/* SPI 1 global interrupt                                             */
  .word	SPI2_IRQHandler              			/* SPI1 global interrupt                                              */
  .word	USART1_IRQHandler            			/* USART1 global interrupt                                            */
  .word	LPUART1_IRQHandler           			/* LPUART1 global interrupt                                           */
  .word	SAI1_IRQHandler              			/* SAI1 global interrupt                                              */
  .word	TSC_IRQHandler               			/* TSC global interrupt                                               */
  .word	EXTI10_15_IRQHandler         			/* EXTI line [15:10] interrupt through EXTI[15:10]                    */
  .word	RTC_ALARM_IRQHandler         			/* RTC Alarms (A and B) interrupt through AIEC                        */
  .word	CRS_IT_IRQHandler            			/* CRS interrupt                                                      */
  .word	PWR_SOTF_IRQHandler          			/* PWR switching on the fly interrupt                                 */
  .word	IPCC_C1_RX_IT_IRQHandler     			/* IPCC CPU1 RX occupied interrupt                                    */
  .word	IPCC_C1_TX_IT_IRQHandler     			/* IPCC CPU1 TX free interrupt                                        */
  .word	HSEM_IRQHandler              			/* Semaphore interrupt 0 to CPU1                                      */
  .word	LPTIM1_IRQHandler            			/* LPtimer 1 global interrupt                                         */
  .word	LPTIM2_IRQHandler            			/* LPtimer 2 global interrupt                                         */
  .word	LCD_IRQHandler               			/* LCD global interrupt                                               */
  .word	QUADSPI_IRQHandler           			/* QSPI global interrupt                                              */
  .word	AES1_IRQHandler              			/* AES1 global interrupt                                              */
  .word	AES2_IRQHandler              			/* AES2 global interrupt                                              */
  .word	True_RNG_IRQHandler          			/* True random number generator interrupt                             */
  .word	FPU_IRQHandler               			/* Floating point unit interrupt                                      */
  .word	DMA2_CH1_IRQHandler          			/* DMA2 channel 1 interrupt                                           */
  .word	DMA2_CH2_IRQHandler          			/* DMA2 channel 2 interrupt                                           */
  .word	DMA2_CH3_IRQHandler          			/* DMA2 channel 3 interrupt                                           */
  .word	DMA2_CH4_IRQHandler          			/* DMA2 channel 4 interrupt                                           */
  .word	DMA2_CH5_IRQHandler          			/* DMA2 channel 5 interrupt                                           */
  .word	DMA2_CH6_IRQHandler          			/* DMA2 channel 6 interrupt                                           */
  .word	DMA2_CH7_IRQHandler          			/* DMA2 channel 7 interrupt                                           */
  .word	DMAMUX_OVR_IRQHandler        			/* DMAMUX overrun interrupt                                           */

/*******************************************************************************
*
* Provide weak aliases for each Exception handler to the Default_Handler.
* As they are weak aliases, any function with the same name will override
* this definition.
*
*******************************************************************************/

	.weak	NMI_Handler
	.thumb_set NMI_Handler,Default_Handler

	.weak	HardFault_Handler
	.thumb_set HardFault_Handler,Default_Handler

	.weak	MemManage_Handler
	.thumb_set MemManage_Handler,Default_Handler

	.weak	BusFault_Handler
	.thumb_set BusFault_Handler,Default_Handler

	.weak	UsageFault_Handler
	.thumb_set UsageFault_Handler,Default_Handler

	.weak	SVC_Handler
	.thumb_set SVC_Handler,Default_Handler

	.weak	DebugMon_Handler
	.thumb_set DebugMon_Handler,Default_Handler

	.weak	PendSV_Handler
	.thumb_set PendSV_Handler,Default_Handler

	.weak	SysTick_Handler
	.thumb_set SysTick_Handler,Default_Handler

	.weak	WWDG_IRQHandler
	.thumb_set WWDG_IRQHandler,Default_Handler

	.weak	PVD_IRQHandler
	.thumb_set PVD_IRQHandler,Default_Handler

	.weak	RTC_TAMP_IRQHandler
	.thumb_set RTC_TAMP_IRQHandler,Default_Handler

	.weak	RTC_WKUP_IRQHandler
	.thumb_set RTC_WKUP_IRQHandler,Default_Handler

	.weak	FLASH_IRQHandler
	.thumb_set FLASH_IRQHandler,Default_Handler

	.weak	RCC_IRQHandler
	.thumb_set RCC_IRQHandler,Default_Handler

	.weak	EXTI0_IRQHandler
	.thumb_set EXTI0_IRQHandler,Default_Handler

	.weak	EXTI1_IRQHandler
	.thumb_set EXTI1_IRQHandler,Default_Handler

	.weak	EXTI2_IRQHandler
	.thumb_set EXTI2_IRQHandler,Default_Handler

	.weak	EXTI3_IRQHandler
	.thumb_set EXTI3_IRQHandler,Default_Handler

	.weak	EXTI4_IRQHandler
	.thumb_set EXTI4_IRQHandler,Default_Handler

	.weak	DMA1_Channel1_IRQHandler
	.thumb_set DMA1_Channel1_IRQHandler,Default_Handler

	.weak	DMA1_Channel2_IRQHandler
	.thumb_set DMA1_Channel2_IRQHandler,Default_Handler

	.weak	DMA1_Channel3_IRQHandler
	.thumb_set DMA1_Channel3_IRQHandler,Default_Handler

	.weak	DMA1_Channel4_IRQHandler
	.thumb_set DMA1_Channel4_IRQHandler,Default_Handler

	.weak	DMA1_Channel5_IRQHandler
	.thumb_set DMA1_Channel5_IRQHandler,Default_Handler

	.weak	DMA1_Channel6_IRQHandler
	.thumb_set DMA1_Channel6_IRQHandler,Default_Handler

	.weak	DMA1_Channel7_IRQHandler
	.thumb_set DMA1_Channel7_IRQHandler,Default_Handler

	.weak	ADC1_IRQHandler
	.thumb_set ADC1_IRQHandler,Default_Handler

	.weak	USB_HP_IRQHandler
	.thumb_set USB_HP_IRQHandler,Default_Handler

	.weak	USB_LP_IRQHandler
	.thumb_set USB_LP_IRQHandler,Default_Handler

	.weak	C2SEV_IRQHandler
	.thumb_set C2SEV_IRQHandler,Default_Handler

	.weak	COMP_IRQHandler
	.thumb_set COMP_IRQHandler,Default_Handler

	.weak	EXTI5_9_IRQHandler
	.thumb_set EXTI5_9_IRQHandler,Default_Handler

	.weak	TIM1_BRK_IRQHandler
	.thumb_set TIM1_BRK_IRQHandler,Default_Handler

	.weak	TIM1_UP_IRQHandler
	.thumb_set TIM1_UP_IRQHandler,Default_Handler

	.weak	TIM1_TRG_COM_TIM17_IRQHandler
	.thumb_set TIM1_TRG_COM_TIM17_IRQHandler,Default_Handler

	.weak	TIM1_CC_IRQHandler
	.thumb_set TIM1_CC_IRQHandler,Default_Handler

	.weak	TIM2_IRQHandler
	.thumb_set TIM2_IRQHandler,Default_Handler

	.weak	PKA_IRQHandler
	.thumb_set PKA_IRQHandler,Default_Handler

	.weak	I2C1_EV_IRQHandler
	.thumb_set I2C1_EV_IRQHandler,Default_Handler

	.weak	I2C1_ER_IRQHandler
	.thumb_set I2C1_ER_IRQHandler,Default_Handler

	.weak	I2C3_EV_IRQHandler
	.thumb_set I2C3_EV_IRQHandler,Default_Handler

	.weak	I2C3_ER_IRQHandler
	.thumb_set I2C3_ER_IRQHandler,Default_Handler

	.weak	SPI1_IRQHandler
	.thumb_set SPI1_IRQHandler,Default_Handler

	.weak	SPI2_IRQHandler
	.thumb_set SPI2_IRQHandler,Default_Handler

	.weak	USART1_IRQHandler
	.thumb_set USART1_IRQHandler,Default_Handler

	.weak	LPUART1_IRQHandler
	.thumb_set LPUART1_IRQHandler,Default_Handler

	.weak	SAI1_IRQHandler
	.thumb_set SAI1_IRQHandler,Default_Handler

	.weak	TSC_IRQHandler
	.thumb_set TSC_IRQHandler,Default_Handler

	.weak	EXTI10_15_IRQHandler
	.thumb_set EXTI10_15_IRQHandler,Default_Handler

	.weak	RTC_ALARM_IRQHandler
	.thumb_set RTC_ALARM_IRQHandler,Default_Handler

	.weak	CRS_IT_IRQHandler
	.thumb_set CRS_IT_IRQHandler,Default_Handler

	.weak	PWR_SOTF_IRQHandler
	.thumb_set PWR_SOTF_IRQHandler,Default_Handler

	.weak	IPCC_C1_RX_IT_IRQHandler
	.thumb_set IPCC_C1_RX_IT_IRQHandler,Default_Handler

	.weak	IPCC_C1_TX_IT_IRQHandler
	.thumb_set IPCC_C1_TX_IT_IRQHandler,Default_Handler

	.weak	HSEM_IRQHandler
	.thumb_set HSEM_IRQHandler,Default_Handler

	.weak	LPTIM1_IRQHandler
	.thumb_set LPTIM1_IRQHandler,Default_Handler

	.weak	LPTIM2_IRQHandler
	.thumb_set LPTIM2_IRQHandler,Default_Handler

	.weak	LCD_IRQHandler
	.thumb_set LCD_IRQHandler,Default_Handler

	.weak	QUADSPI_IRQHandler
	.thumb_set QUADSPI_IRQHandler,Default_Handler

	.weak	AES1_IRQHandler
	.thumb_set AES1_IRQHandler,Default_Handler

	.weak	AES2_IRQHandler
	.thumb_set AES2_IRQHandler,Default_Handler

	.weak	True_RNG_IRQHandler
	.thumb_set True_RNG_IRQHandler,Default_Handler

	.weak	FPU_IRQHandler
	.thumb_set FPU_IRQHandler,Default_Handler

	.weak	DMA2_CH1_IRQHandler
	.thumb_set DMA2_CH1_IRQHandler,Default_Handler

	.weak	DMA2_CH2_IRQHandler
	.thumb_set DMA2_CH2_IRQHandler,Default_Handler

	.weak	DMA2_CH3_IRQHandler
	.thumb_set DMA2_CH3_IRQHandler,Default_Handler

	.weak	DMA2_CH4_IRQHandler
	.thumb_set DMA2_CH4_IRQHandler,Default_Handler

	.weak	DMA2_CH5_IRQHandler
	.thumb_set DMA2_CH5_IRQHandler,Default_Handler

	.weak	DMA2_CH6_IRQHandler
	.thumb_set DMA2_CH6_IRQHandler,Default_Handler

	.weak	DMA2_CH7_IRQHandler
	.thumb_set DMA2_CH7_IRQHandler,Default_Handler

	.weak	DMAMUX_OVR_IRQHandler
	.thumb_set DMAMUX_OVR_IRQHandler,Default_Handler

	.weak	SystemInit

/************************ (C) COPYRIGHT STMicroelectonics *****END OF FILE****/
