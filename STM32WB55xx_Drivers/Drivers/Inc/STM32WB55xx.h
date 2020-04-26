/*
 * STM32WB55xx.h
 *
 *  Created on: Apr 9, 2020
 *      Author: Rahul
 */

#ifndef INC_STM32WB55XX_H_
#define INC_STM32WB55XX_H_


#include <stdint.h> // This is included because, we have used uint32_t which is part of stdint.h
#include <stddef.h> // This is included because, we have used NULL which is part of stddef.h


#define __vo volatile
#define __weak __attribute__((weak)) 	//Weak function attribute

/****************Processor Specific Details********************/
/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 */
#define NVIC_ISER0			( (__vo uint32_t*) 0xE000E100 )
#define NVIC_ISER1			( (__vo uint32_t*) 0xE000E104 )
#define NVIC_ISER2			( (__vo uint32_t*) 0xE000E108 )
#define NVIC_ISER3			( (__vo uint32_t*) 0xE000E10C )

#define NVIC_ICER0			( (__vo uint32_t*) 0xE000E180 )
#define NVIC_ICER1			( (__vo uint32_t*) 0xE000E184 )
#define NVIC_ICER2			( (__vo uint32_t*) 0xE000E188 )
#define NVIC_ICER3			( (__vo uint32_t*) 0xE000E18C )

/*
 * ARM Cortex Mx Processor Priority Register Address calculation
 */
#define NVIC_IPR0			( (__vo uint32_t*) 0xE000E400 )
#define NVIC_PR_BASE_ADDR	NVIC_IPR0


#define NO_PR_BITS_IMPLEMENTED			4

/*
 *  Macros for all the possible NVIC Priority level
 */
#define NVIC_IRQ_PRIORITY_0				0
#define NVIC_IRQ_PRIORITY_1				1
#define NVIC_IRQ_PRIORITY_2				2
#define NVIC_IRQ_PRIORITY_3				3
#define NVIC_IRQ_PRIORITY_4				4
#define NVIC_IRQ_PRIORITY_5				5
#define NVIC_IRQ_PRIORITY_6				6
#define NVIC_IRQ_PRIORITY_7				7
#define NVIC_IRQ_PRIORITY_8				8
#define NVIC_IRQ_PRIORITY_9				9
#define NVIC_IRQ_PRIORITY_10			10
#define NVIC_IRQ_PRIORITY_11			11
#define NVIC_IRQ_PRIORITY_12			12
#define NVIC_IRQ_PRIORITY_13			13
#define NVIC_IRQ_PRIORITY_14			14
#define NVIC_IRQ_PRIORITY_15			15


/*
 * Base addresses of Flash and SRAM memories
 */
#define FLASH_BASEADDR					0x08000000U
#define SRAM1_BASEADDR					0x20000000U  // 192 KB
#define SRAM 							SRAM1_BASEADDR
#define SRAM2a_BASEADDR					0x20030000U // SRAM1_BASEADDR + 0x00030000U (SRAM1_BASEADDR + 192KB)
#define SRAM2b_BASEADDR					0x20038000U // SRAM2a_BASEADDR + 0x00008000U (SRAM2a_BASEADDR + 32KB)
#define ROM								0x1FFF0000U

/*
 * AHBx and APBx Bus Peripheral base addresses
 */
#define PERIPH_BASEADDR					0x40000000U //The peripheral addresses start from here.
#define APB1PERIPH_BASEADDR				PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR				0x40010000U
#define AHB1PERIPH_BASEADDR				0x40020000U
#define AHB2PERIPH_BASEADDR				0x48000000U
#define AHB4PERIPH_BASEADDR				0x58000000U
#define APB3PERIPH_BASEADDR				0x60000000U
#define AHB3PERIPH_BASEADDR				0x90000000U

/*
 * Base addresses of peripherals connected to AHB1 bus
 * TODO: Like this we should define base addresses all the peripheral hanging on AHB1 bus
 */
#define DMA1_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0000U)
#define DMA2_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0400U)


/*
 * Base addresses of peripherals connected to AHB2 bus
 * TODO: Like this we should define base addresses all the peripheral hanging on AHB2 bus
 */
#define GPIOA_BASEADDR					(AHB2PERIPH_BASEADDR + 0x0000U)
#define GPIOB_BASEADDR					(AHB2PERIPH_BASEADDR + 0x0400U)
#define GPIOC_BASEADDR					(AHB2PERIPH_BASEADDR + 0x0800U)
#define GPIOD_BASEADDR					(AHB2PERIPH_BASEADDR + 0x0C00U)
#define GPIOE_BASEADDR					(AHB2PERIPH_BASEADDR + 0x1000U)
#define GPIOH_BASEADDR					(AHB2PERIPH_BASEADDR + 0x1C00U)

/*
 * Base addresses of peripherals connected to APB1 bus
 * TODO: Like this we should define base addresses all the peripheral hanging to APB1 bus
 */
#define TIM2_BASEADDR					(APB1PERIPH_BASEADDR + 0x0000U)
#define SPI2_BASEADDR					(APB1PERIPH_BASEADDR + 0x3800U)
#define I2C1_BASEADDR					(APB1PERIPH_BASEADDR + 0x5400U)
#define I2C3_BASEADDR					(APB1PERIPH_BASEADDR + 0x5C00U)
#define LPUART1_BASEADDR				(APB1PERIPH_BASEADDR + 0x8000U)

/*
 * Base addresses of peripherals connected to APB2 bus
 * TODO: Like this we should define base addresses all the peripheral hanging to APB2 bus
 */
#define SYSCFG_BASEADDR					(APB2PERIPH_BASEADDR + 0x0000U)
#define SPI1_BASEADDR					(APB2PERIPH_BASEADDR + 0x3000U)
#define USART1_BASEADRR					(APB2PERIPH_BASEADDR + 0x3800U)


/*
 * Base addresses of peripherals connected to AHB4 bus
 * TODO: Like this we should define base addresses all the peripheral hanging to AHB4 bus
 */
#define EXTI_BASEADDDR					(AHB4PERIPH_BASEADDR + 0x0800U)
#define RCC_BASEADDR					(AHB4PERIPH_BASEADDR + 0x0000U)

/***************** Peripheral register definition structures ******************/
/*
 * Note: Registers of peripheral are specific to MCU
 */

typedef struct		//Generic register structure for GPIO ports
{
	/* Give a short description of each registers */
	__vo uint32_t MODER; 	// Address OFFSET = 0x00; GPIO port mode register
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];	// Instead of using AFR[0] is for AFRL and AFR[1] is for AFRH
	__vo uint32_t BRR;
}GPIO_RegDef_t;

/*
 * Peripheral register definition structure for RCC
 */
typedef struct
{
	__vo uint32_t CR; 			// RCC clock control register
	__vo uint32_t ICSCR;
	__vo uint32_t CFGR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t PLLSAI1CFGR;
	uint32_t Reserved;
	__vo uint32_t CIER;
	__vo uint32_t CIFR;
	__vo uint32_t CICR;
	__vo uint32_t SMPSCR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	uint32_t Reserved1;
	__vo uint32_t APB1RSTR1;
	__vo uint32_t APB1RSTR2;
	__vo uint32_t APB2RSTR;
	__vo uint32_t APB3RSTR;
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	uint32_t Reserved2;
	__vo uint32_t APB1ENR1;
	__vo uint32_t APB1ENR2;
	__vo uint32_t APB2ENR;
	uint32_t Reserved3;
	__vo uint32_t AHB1SMENR;
	__vo uint32_t AHB2SMENR;
	__vo uint32_t AHB3SMENR;
	uint32_t Reserved4;
	__vo uint32_t APB1SMENR1;
	__vo uint32_t APB1SMENR2;
	__vo uint32_t APB2SMENR;
	uint32_t Reserved5;
	__vo uint32_t CCIPR;
	uint32_t Reserved6;
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	__vo uint32_t CRRCR;
	__vo uint32_t HSECR;
	uint32_t Reserved7[26]; //Reserved, Address offset = 0x0A0 to 0x104
	__vo uint32_t EXTCFGR;
	uint32_t Reserved8[15];
	__vo uint32_t C2AHB1ENR;
	__vo uint32_t C2AHB2ENR;
	__vo uint32_t C2AHB3ENR;
	__vo uint32_t C2APB1ENR1;
	__vo uint32_t C2APB1ENR2;
	__vo uint32_t C2APB2ENR;
	__vo uint32_t C2APB3ENR;
	__vo uint32_t C2AHB1SMENR;
	__vo uint32_t C2AHB2SMENR;
	__vo uint32_t C2AHB33MENR;
	uint32_t Reserved9;
	__vo uint32_t C2APB1SMENR1;
	__vo uint32_t C2APB1SMENR2;
	__vo uint32_t C2APB2SMENR;
	__vo uint32_t C2APB3SMENR;
}RCC_RegDef_t;


/*
 * Peripheral register definition structure for EXTI
 */
typedef struct
{
	__vo uint32_t RTSR1;
	__vo uint32_t FTSR1;
	__vo uint32_t SWIER1;
	__vo uint32_t PR1;
	uint32_t Reserved[4];
	__vo uint32_t RTSR2;
	__vo uint32_t FTSR2;
	__vo uint32_t SWIER2;
	__vo uint32_t PR2;
	uint32_t Reserved1[20];
	__vo uint32_t IMR1;
	__vo uint32_t EMR1;
	uint32_t Reserved2[2];
	__vo uint32_t IMR2;
	__vo uint32_t EMR2;
	uint32_t Reserved3[10];
	__vo uint32_t C2IMR1;
	__vo uint32_t C2EMR1;
	uint32_t Reserved4[2];
	__vo uint32_t C2IMR2;
	__vo uint32_t C2EMR2;
}EXTI_RegDef_t;


/*
 * Peripheral register definition structure for SYSCFG
 */
typedef struct
{
	__vo uint32_t MEMRMP;
	__vo uint32_t CFGR1;
	__vo uint32_t EXTICR[4];
	__vo uint32_t SCSR;
	__vo uint32_t CFGR2;
	__vo uint32_t SWPR;
	__vo uint32_t SKR;
	__vo uint32_t SWPR2;
	uint32_t Reserved[52];
	__vo uint32_t IMR1;
	__vo uint32_t IMR2;
	__vo uint32_t C2IMR1;
	__vo uint32_t C2IMR2;
	__vo uint32_t SIPCR;
}SYSCFG_RegDef_t;


/*
 * Peripheral register definition structure for SPI
 */
typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
}SPI_RegDef_t;


/*
 * Peripheral register definition structure for I2C
 */
typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t OAR1;			// Own address 1 register
	__vo uint32_t OAR2;			// Own address 2 register
	__vo uint32_t TIMINGR;
	__vo uint32_t TIMEOUTR;
	__vo uint32_t ISR;			//Interrupt and Status register
	__vo uint32_t PECR;			//
	__vo uint32_t RXDR;			//Receiver Data register
	__vo uint32_t TXDR;			//Transmit Data register

}I2C_RegDef_t;


/*
 * Peripheral definitions (Peripheral base addresses type casted xx_RefDef_t)
 */
#define GPIOA	((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB	((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC	((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD	((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE	((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOH	((GPIO_RegDef_t*) GPIOH_BASEADDR)

#define RCC		((RCC_RegDef_t*) RCC_BASEADDR)

#define EXTI	((EXTI_RegDef_t*) EXTI_BASEADDDR)

#define SYSCFG	((SYSCFG_RegDef_t*) SYSCFG_BASEADDR)

//SPIx base address type casted:
#define SPI1	((SPI_RegDef_t*) SPI1_BASEADDR)
#define SPI2	((SPI_RegDef_t*) SPI2_BASEADDR)

//I2Cx base address type casted:
#define I2C1	((I2C_RegDef_t*)) I2C1_BASEADDR)
#define I2C3	((I2C_RegDef_t*)) I2C3_BASEADDR)


/*
 * Clock enable macros for GPIOx peripherals
 */
#define GPIOA_PERI_CLK_EN()	( RCC->AHB2ENR |= (1 << 0 ) )
#define GPIOB_PERI_CLK_EN() ( RCC->AHB2ENR |= (1 << 1 ) )
#define GPIOC_PERI_CLK_EN() ( RCC->AHB2ENR |= (1 << 2 ) )
#define GPIOD_PERI_CLK_EN() ( RCC->AHB2ENR |= (1 << 3 ) )
#define GPIOE_PERI_CLK_EN() ( RCC->AHB2ENR |= (1 << 4 ) )
#define GPIOH_PERI_CLK_EN() ( RCC->AHB2ENR |= (1 << 7 ) )

/*
 * Clock enable macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN()	( RCC->APB1ENR1 |= (1 << 21) )
#define I2C3_PCLK_EN()	( RCC->APB1ENR1 |= (1 << 23) )

/*
 * Clock enable macros for SPIx peripherals
 */
#define SPI1_PCLK_EN()	( RCC->APB2ENR |= (1 << 12) )
#define SPI2_PCLK_EN()	( RCC->APB1ENR1 |= (1 << 14) )

/*
 * Clock enable macros for USARTx peripherals
 */


/*
 * Clock enable macros for SYSCFG peripheral
 */
//Have to check this in the User Manual (I didn't find the Clock enable bit in peripheral APB2 to which SYSCFG is hanging
//#define SYSCFG_PCLK_EN() 	( RCC )

/*
 * Clock Disable macros for GPIOx peripherals
 */
#define GPIOA_PERI_CLK_DI()		( RCC->AHB2ENR &= ~(1 << 0 ) )
#define GPIOB_PERI_CLK_DI()		( RCC->AHB2ENR &= ~(1 << 1 ) )
#define GPIOC_PERI_CLK_DI()		( RCC->AHB2ENR &= ~(1 << 2 ) )
#define GPIOD_PERI_CLK_DI()		( RCC->AHB2ENR &= ~(1 << 3 ) )
#define GPIOE_PERI_CLK_DI()		( RCC->AHB2ENR &= ~(1 << 4 ) )
#define GPIOH_PERI_CLK_DI()		( RCC->AHB2ENR &= ~(1 << 7 ) )

/*
 * Clock disable macros for I2Cx peripherals
 */
#define I2C1_PCLK_DI()			( RCC->APB1ENR1 &= ~(1 << 21) )
#define I2C3_PCLK_DI()			( RCC->APB1ENR1 &= ~(1 << 23) )

/*
 * Clock disable macros for SPIx peripherals
 */
#define SPI1_PCLK_DI()			( RCC->APB2ENR &= ~(1 << 12) )
#define SPI2_PCLK_DI()			( RCC->APB1ENR1 &= ~(1 << 14) )

/*
 * Clock disable macros for USARTx peripherals
 */


/*
 * Clock disable macros for SYSCFG peripheral
 */


/*
 * Macros to reset GPIO peripherals
 */
#define GPIOA_REG_RESET()		do{ (RCC->AHB2RSTR |= (1 << 0)); (RCC->AHB2RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()		do{ (RCC->AHB2RSTR |= (1 << 1)); (RCC->AHB2RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()		do{ (RCC->AHB2RSTR |= (1 << 2)); (RCC->AHB2RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()		do{ (RCC->AHB2RSTR |= (1 << 3)); (RCC->AHB2RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()		do{ (RCC->AHB2RSTR |= (1 << 4)); (RCC->AHB2RSTR &= ~(1 << 4)); }while(0)
#define GPIOH_REG_RESET()		do{ (RCC->AHB2RSTR |= (1 << 7)); (RCC->AHB2RSTR &= ~(1 << 7)); }while(0)


//Macros to reset SPI peripherals
#define SPI1_REG_RESET()		do{ (RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12)); }while(0)
#define SPI2_REG_RESET()		do{ (RCC->APB1RSTR1 |= (1 << 14)); (RCC->APB1RSTR1 &= ~(1 << 14)); }while(0)


//Macros to reset I2C peripherals
#define I2C1_REG_RESET()		do{ (RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12)); }while(0)
#define I2C3_REG_RESET()		do{ (RCC->APB1RSTR1 |= (1 << 14)); (RCC->APB1RSTR1 &= ~(1 << 14)); }while(0)



//Macro to return the code corresponding to the given GPIO port
#define GPIO_BASEADDR_TO_CODE(x)		((x == GPIOA) ? 0:\
										 (x == GPIOB) ? 1:\
										 (x == GPIOC) ? 2:\
										 (x == GPIOD) ? 3:\
										 (x == GPIOE) ? 4:\
										 (x == GPIOH) ? 7:0)

/*
 * IR (Interrupt Request) Numbers
 */
#define IRQ_NO_EXTI_0		6
#define IRQ_NO_EXTI_1		7
#define IRQ_NO_EXTI_2		8
#define IRQ_NO_EXTI_3		9
#define IRQ_NO_EXTI_4		10
#define IRQ_NO_EXTI_5_9		23
#define IRQ_NO_EXTI_10_15	40

#define IRQ_NO_SPI1			25
#define IRQ_NO_SPI2			26


//Some Generic macros
#define ENABLE 			1
#define DISABLE 		0

#define SET 			ENABLE
#define RESET 			DISABLE

#define GPIO_PIN_SET 	SET
#define GPIO_PIN_RESET 	RESET

#define FLAG_RESET		RESET
#define FLAG_SET		SET


/****************************************************************
 * Bit position definitions of SPI peripheral
 ****************************************************************/
#define SPIx_CR1_CPHA		0
#define SPIx_CR1_CPOL		1
#define SPIx_CR1_MSTR		2	//Device Slave/Master Config
#define SPIx_CR1_BR			3
#define SPIx_CR1_SPE		6
#define SPIx_CR1_LSBFIRST	7
#define SPIx_CR1_SSI		8
#define SPIx_CR1_SSM		9
#define SPIx_CR1_RXONLY		10
#define SPIx_CR1_CRCL		11
#define SPIx_CR1_CRCNEXT	12
#define SPIx_CR1_CRCEN		13
#define SPIx_CR1_BIDIOE		14
#define SPIx_CR1_BIBIMODE	15


#define SPIx_CR2_RXDMAEN	0
#define SPIx_CR2_TXDMAEN	1
#define SPIx_CR2_SSOE		2
#define SPIx_CR2_NSSP		3
#define SPIx_CR2_FRF		4
#define SPIx_CR2_ERRIE		5
#define SPIx_CR2_RXNEIE		6
#define SPIx_CR2_TXEIE		7
#define SPIx_CR2_DS			8
#define SPIx_CR2_FRXTh		12
#define SPIx_CR2_LDMA_RX	13
#define SPIx_CR2_LDMA_TX	14


#define SPIx_SR_RXNE		0
#define SPIx_SR_TXE			1
#define SPIx_SR_CRCERR		4
#define SPIx_SR_MODF		5
#define SPIx_SR_OVR			6
#define SPIx_SR_BSY			7
#define SPIx_SR_FRE			8
#define SPIx_SR_FRLVL		9
#define SPIx_SR_FTLVL		11


/****************************************************************
 * Bit position definitions of I2C peripheral
 ****************************************************************/
//I2Cx Control Register 1
#define I2Cx_CR1_PE				0
#define I2Cx_CR1_TXIE			1
#define I2Cx_CR1_RXIE			2
#define I2Cx_CR1_ADDRIE			3
#define I2Cx_CR1_NACKIE			4
#define I2Cx_CR1_STOPIE			5
#define I2Cx_CR1_TCIE			6
#define I2Cx_CR1_ERRIE			7
#define I2Cx_CR1_DNF			8		//DNF[3:0]
#define I2Cx_CR1_ANFOFF			12
#define I2Cx_CR1_TXDMAEN		14
#define I2Cx_CR1_RXDMAEN		15
#define I2Cx_CR1_SBC			16
#define I2Cx_CR1_NOSTRETCH		17
#define I2Cx_CR1_WUPEN			18
#define I2Cx_CR1_GCEN			19
#define I2Cx_CR1_SMBHEN			20
#define I2Cx_CR1_SMBDEN			21
#define I2Cx_CR1_ALERTEN		22
#define I2Cx_CR1_PECEN			23

//I2Cx Control Register 2
#define I2Cx_CR2_SADD			0 		//SADD[9:0]
#define I2Cx_CR2_RD_WRN			10
#define I2Cx_CR2_ADD10			11
#define I2Cx_CR2_HEAD10R		12
#define I2Cx_CR2_START			13
#define I2Cx_CR2_STOP			14
#define I2Cx_CR2_NACK			15
#define I2Cx_CR2_NBYTES			16		//NBYTES[7:0]
#define I2Cx_CR2_RELOAD			24
#define I2Cx_CR2_AUTOEND		25
#define I2Cx_CR2_PECBYTE		26

//I2C Interrupt and Status Register
#define I2Cx_ISR_TXE			0
#define I2Cx_ISR_TXIS			1
#define I2Cx_ISR_RXNE			2
#define I2Cx_ISR_ADDR			3
#define I2Cx_ISR_NACKF			4
#define I2Cx_ISR_STOPF			5
#define I2Cx_ISR_TC				6
#define I2Cx_ISR_TCR			7
#define I2Cx_ISR_BERR			8
#define I2Cx_ISR_ARLO			9
#define I2Cx_ISR_OVR			10
#define I2Cx_ISR_PECERR			11
#define I2Cx_ISR_TIMOUT			12
#define I2Cx_ISR_ALERT			13
#define I2Cx_ISR_BUSY			15
#define I2Cx_ISR_DIR			16
#define I2Cx_ISR_ADDCODE		17		// ADDCODE[7]


//I2C Interrupt Clear Register
#define I2Cx_ICR_ADDRCF			3
#define I2Cx_ICR_NACKCF			4
#define I2Cx_ICR_STOPCF			5
#define I2Cx_ICR_BERRCF			8
#define I2Cx_ICR_ARLOCF			9
#define I2Cx_ICR_OVRCF			10
#define I2Cx_ICR_PECCF			11
#define I2Cx_ICR_TIMEOUTCF		12
#define I2Cx_ICR_ALERTCF		13




#include <STM32WB55xx_Gpio_Driver.h>
#include <STM32WB55xx_SPI_Driver.h>

#endif /* INC_STM32WB55XX_H_ */
