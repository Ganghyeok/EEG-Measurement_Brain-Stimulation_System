/*
 * stm32f407xx.h
 *
 *  Created on: Mar 7, 2021
 *      Author: Ganghyeok Lim
 */

#ifndef __STM32F407xx_H
#define __STM32F407xx_H


#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>



/**************************************************************************************************************
 * 																											  *
 * 													Generic Macros											  *
 * 																											  *
 **************************************************************************************************************/

#define __IO									volatile
#define __vo									volatile
#define __weak									__attribute__((weak))
#define __naked									__attribute__((naked))
#define HIGH									1
#define LOW										0
#define UP										1
#define DOWN									0
#define TRUE									1
#define FALSE									0
#define ENABLE									1
#define DISABLE									0
#define SET										1
#define RESET									0
#define CLEAR									0
#define GPIO_PIN_SET							SET
#define GPIO_PIN_RESET							RESET
#define FLAG_SET								SET
#define FLAG_RESET								RESET
#define BUTTON_PRESSED							GPIO_PIN_RESET
#define BUTTON_NOT_PRESSED						GPIO_PIN_SET
#define UNUSED(X)								(void)X			/* To avoid gcc/g++ warnings */


/**************************************************************************************************************
 * 																											  *
 * 											Peripheral Register Structures									  *
 * 																											  *
 **************************************************************************************************************/

/* Peripheral Register Structures of Core Hardware */

/**
  \ingroup    CMSIS_core_register
  \defgroup   CMSIS_NVIC  Nested Vectored Interrupt Controller (NVIC)
  \brief      Type definitions for the NVIC Registers
  @{
 */

/**
  \brief  Structure type to access the Nested Vectored Interrupt Controller (NVIC).
 */
typedef struct
{
  __IO 	uint32_t ISER[8U];               /*!< Offset: 0x000 (R/W)  Interrupt Set Enable Register */
        uint32_t RESERVED0[24U];
  __IO	uint32_t ICER[8U];               /*!< Offset: 0x080 (R/W)  Interrupt Clear Enable Register */
        uint32_t RSERVED1[24U];
  __IO 	uint32_t ISPR[8U];               /*!< Offset: 0x100 (R/W)  Interrupt Set Pending Register */
        uint32_t RESERVED2[24U];
  __IO 	uint32_t ICPR[8U];               /*!< Offset: 0x180 (R/W)  Interrupt Clear Pending Register */
        uint32_t RESERVED3[24U];
  __IO 	uint32_t IABR[8U];               /*!< Offset: 0x200 (R/W)  Interrupt Active bit Register */
        uint32_t RESERVED4[56U];
  __IO 	uint8_t  IPR[240U];               /*!< Offset: 0x300 (R/W)  Interrupt Priority Register (8Bit wide) */
        uint32_t RESERVED5[644U];
  __IO  uint32_t STIR;                   /*!< Offset: 0xE00 ( /W)  Software Trigger Interrupt Register */
}  NVIC_Type;


/**
  \ingroup  CMSIS_core_register
  \defgroup CMSIS_SCB     System Control Block (SCB)
  \brief    Type definitions for the System Control Block Registers
  @{
 */

/**
  \brief  Structure type to access the System Control Block (SCB).
 */
typedef struct
{
	__IO  uint32_t CPUID;                  /*!< Offset: 0x000 (R/ )  CPUID Base Register */
	__IO  uint32_t ICSR;                   /*!< Offset: 0x004 (R/W)  Interrupt Control and State Register */
	__IO  uint32_t VTOR;                   /*!< Offset: 0x008 (R/W)  Vector Table Offset Register */
	__IO  uint32_t AIRCR;                  /*!< Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register */
	__IO  uint32_t SCR;                    /*!< Offset: 0x010 (R/W)  System Control Register */
	__IO  uint32_t CCR;                    /*!< Offset: 0x014 (R/W)  Configuration Control Register */
	__IO  uint8_t  SHP[12U];               /*!< Offset: 0x018 (R/W)  System Handlers Priority Registers (4-7, 8-11, 12-15) */
	__IO  uint32_t SHCSR;                  /*!< Offset: 0x024 (R/W)  System Handler Control and State Register */
	__IO  uint32_t CFSR;                   /*!< Offset: 0x028 (R/W)  Configurable Fault Status Register */
	__IO  uint32_t HFSR;                   /*!< Offset: 0x02C (R/W)  HardFault Status Register */
	__IO  uint32_t DFSR;                   /*!< Offset: 0x030 (R/W)  Debug Fault Status Register */
	__IO  uint32_t MMFAR;                  /*!< Offset: 0x034 (R/W)  MemManage Fault Address Register */
	__IO  uint32_t BFAR;                   /*!< Offset: 0x038 (R/W)  BusFault Address Register */
	__IO  uint32_t AFSR;                   /*!< Offset: 0x03C (R/W)  Auxiliary Fault Status Register */
	__IO  uint32_t PFR[2U];                /*!< Offset: 0x040 (R/ )  Processor Feature Register */
	__IO  uint32_t DFR;                    /*!< Offset: 0x048 (R/ )  Debug Feature Register */
	__IO  uint32_t ADR;                    /*!< Offset: 0x04C (R/ )  Auxiliary Feature Register */
	__IO  uint32_t MMFR[4U];               /*!< Offset: 0x050 (R/ )  Memory Model Feature Register */
	__IO  uint32_t ISAR[5U];               /*!< Offset: 0x060 (R/ )  Instruction Set Attributes Register */
          uint32_t RESERVED0[5U];
    __IO  uint32_t CPACR;                  /*!< Offset: 0x088 (R/W)  Coprocessor Access Control Register */
} SCB_Type;


/**
  \ingroup  CMSIS_core_register
  \defgroup CMSIS_SysTick     System Tick Timer (SysTick)
  \brief    Type definitions for the System Timer Registers.
  @{
 */

/**
  \brief  Structure type to access the System Timer (SysTick).
 */
typedef struct
{
	__IO uint32_t CTRL;                   /*!< Offset: 0x000 (R/W)  SysTick Control and Status Register */
	__IO uint32_t LOAD;                   /*!< Offset: 0x004 (R/W)  SysTick Reload Value Register */
	__IO uint32_t VAL;                    /*!< Offset: 0x008 (R/W)  SysTick Current Value Register */
	__IO uint32_t CALIB;                  /*!< Offset: 0x00C (R/ )  SysTick Calibration Register */
} SysTick_Type;



/* Peripheral Register Structures of Processor Hardware */

/**
  * @brief Analog to Digital Converter
  */
typedef struct
{
	__IO uint32_t SR;     /*!< ADC status register,                         Address offset: 0x00 */
	__IO uint32_t CR1;    /*!< ADC control register 1,                      Address offset: 0x04 */
	__IO uint32_t CR2;    /*!< ADC control register 2,                      Address offset: 0x08 */
	__IO uint32_t SMPR1;  /*!< ADC sample time register 1,                  Address offset: 0x0C */
	__IO uint32_t SMPR2;  /*!< ADC sample time register 2,                  Address offset: 0x10 */
	__IO uint32_t JOFR1;  /*!< ADC injected channel data offset register 1, Address offset: 0x14 */
	__IO uint32_t JOFR2;  /*!< ADC injected channel data offset register 2, Address offset: 0x18 */
	__IO uint32_t JOFR3;  /*!< ADC injected channel data offset register 3, Address offset: 0x1C */
	__IO uint32_t JOFR4;  /*!< ADC injected channel data offset register 4, Address offset: 0x20 */
	__IO uint32_t HTR;    /*!< ADC watchdog higher threshold register,      Address offset: 0x24 */
	__IO uint32_t LTR;    /*!< ADC watchdog lower threshold register,       Address offset: 0x28 */
	__IO uint32_t SQR1;   /*!< ADC regular sequence register 1,             Address offset: 0x2C */
	__IO uint32_t SQR2;   /*!< ADC regular sequence register 2,             Address offset: 0x30 */
	__IO uint32_t SQR3;   /*!< ADC regular sequence register 3,             Address offset: 0x34 */
	__IO uint32_t JSQR;   /*!< ADC injected sequence register,              Address offset: 0x38*/
	__IO uint32_t JDR1;   /*!< ADC injected data register 1,                Address offset: 0x3C */
	__IO uint32_t JDR2;   /*!< ADC injected data register 2,                Address offset: 0x40 */
	__IO uint32_t JDR3;   /*!< ADC injected data register 3,                Address offset: 0x44 */
	__IO uint32_t JDR4;   /*!< ADC injected data register 4,                Address offset: 0x48 */
	__IO uint32_t DR;     /*!< ADC regular data register,                   Address offset: 0x4C */
} ADC_TypeDef;

typedef struct
{
	__IO uint32_t CSR;    /*!< ADC Common status register,                  Address offset: ADC1 base address + 0x300 */
	__IO uint32_t CCR;    /*!< ADC common control register,                 Address offset: ADC1 base address + 0x304 */
	__IO uint32_t CDR;    /*!< ADC common regular data register for dual
                             AND triple modes,                            Address offset: ADC1 base address + 0x308 */
} ADC_Common_TypeDef;


/**
  * @brief Digital to Analog Converter
  */
typedef struct
{
	__IO uint32_t CR;       /*!< DAC control register,                                    Address offset: 0x00 */
	__IO uint32_t SWTRIGR;  /*!< DAC software trigger register,                           Address offset: 0x04 */
	__IO uint32_t DHR12R1;  /*!< DAC channel1 12-bit right-aligned data holding register, Address offset: 0x08 */
	__IO uint32_t DHR12L1;  /*!< DAC channel1 12-bit left aligned data holding register,  Address offset: 0x0C */
	__IO uint32_t DHR8R1;   /*!< DAC channel1 8-bit right aligned data holding register,  Address offset: 0x10 */
	__IO uint32_t DHR12R2;  /*!< DAC channel2 12-bit right aligned data holding register, Address offset: 0x14 */
	__IO uint32_t DHR12L2;  /*!< DAC channel2 12-bit left aligned data holding register,  Address offset: 0x18 */
	__IO uint32_t DHR8R2;   /*!< DAC channel2 8-bit right-aligned data holding register,  Address offset: 0x1C */
	__IO uint32_t DHR12RD;  /*!< Dual DAC 12-bit right-aligned data holding register,     Address offset: 0x20 */
	__IO uint32_t DHR12LD;  /*!< DUAL DAC 12-bit left aligned data holding register,      Address offset: 0x24 */
	__IO uint32_t DHR8RD;   /*!< DUAL DAC 8-bit right aligned data holding register,      Address offset: 0x28 */
	__IO uint32_t DOR1;     /*!< DAC channel1 data output register,                       Address offset: 0x2C */
	__IO uint32_t DOR2;     /*!< DAC channel2 data output register,                       Address offset: 0x30 */
	__IO uint32_t SR;       /*!< DAC status register,                                     Address offset: 0x34 */
} DAC_TypeDef;

/**
  * @brief Debug MCU
  */
typedef struct
{
  __IO uint32_t IDCODE;  /*!< MCU device ID code,               Address offset: 0x00 */
  __IO uint32_t CR;      /*!< Debug MCU configuration register, Address offset: 0x04 */
  __IO uint32_t APB1FZ;  /*!< Debug MCU APB1 freeze register,   Address offset: 0x08 */
  __IO uint32_t APB2FZ;  /*!< Debug MCU APB2 freeze register,   Address offset: 0x0C */
}DBGMCU_TypeDef;


/**
  * @brief DMA Controller
  */
typedef struct
{
  __IO uint32_t CR;     /*!< DMA stream x configuration register      */
  __IO uint32_t NDTR;   /*!< DMA stream x number of data register     */
  __IO uint32_t PAR;    /*!< DMA stream x peripheral address register */
  __IO uint32_t M0AR;   /*!< DMA stream x memory 0 address register   */
  __IO uint32_t M1AR;   /*!< DMA stream x memory 1 address register   */
  __IO uint32_t FCR;    /*!< DMA stream x FIFO control register       */
} DMA_Stream_TypeDef;

typedef struct
{
  __IO uint32_t LISR;   /*!< DMA low interrupt status register,      Address offset: 0x00 */
  __IO uint32_t HISR;   /*!< DMA high interrupt status register,     Address offset: 0x04 */
  __IO uint32_t LIFCR;  /*!< DMA low interrupt flag clear register,  Address offset: 0x08 */
  __IO uint32_t HIFCR;  /*!< DMA high interrupt flag clear register, Address offset: 0x0C */
} DMA_TypeDef;


/**
  * @brief External Interrupt/Event Controller
  */
typedef struct
{
  __IO uint32_t IMR;    /*!< EXTI Interrupt mask register,            Address offset: 0x00 */
  __IO uint32_t EMR;    /*!< EXTI Event mask register,                Address offset: 0x04 */
  __IO uint32_t RTSR;   /*!< EXTI Rising trigger selection register,  Address offset: 0x08 */
  __IO uint32_t FTSR;   /*!< EXTI Falling trigger selection register, Address offset: 0x0C */
  __IO uint32_t SWIER;  /*!< EXTI Software interrupt event register,  Address offset: 0x10 */
  __IO uint32_t PR;     /*!< EXTI Pending register,                   Address offset: 0x14 */
} EXTI_TypeDef;

/**
  * @brief FLASH Registers
  */
typedef struct
{
  __IO uint32_t ACR;      /*!< FLASH access control register,   Address offset: 0x00 */
  __IO uint32_t KEYR;     /*!< FLASH key register,              Address offset: 0x04 */
  __IO uint32_t OPTKEYR;  /*!< FLASH option key register,       Address offset: 0x08 */
  __IO uint32_t SR;       /*!< FLASH status register,           Address offset: 0x0C */
  __IO uint32_t CR;       /*!< FLASH control register,          Address offset: 0x10 */
  __IO uint32_t OPTCR;    /*!< FLASH option control register ,  Address offset: 0x14 */
  __IO uint32_t OPTCR1;   /*!< FLASH option control register 1, Address offset: 0x18 */
} FLASH_TypeDef;


/**
  * @brief General Purpose I/O
  */
typedef struct
{
  __IO uint32_t MODER;    /*!< GPIO port mode register,               Address offset: 0x00      */
  __IO uint32_t OTYPER;   /*!< GPIO port output type register,        Address offset: 0x04      */
  __IO uint32_t OSPEEDR;  /*!< GPIO port output speed register,       Address offset: 0x08      */
  __IO uint32_t PUPDR;    /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
  __IO uint32_t IDR;      /*!< GPIO port input data register,         Address offset: 0x10      */
  __IO uint32_t ODR;      /*!< GPIO port output data register,        Address offset: 0x14      */
  __IO uint32_t BSRR;     /*!< GPIO port bit set/reset register,      Address offset: 0x18      */
  __IO uint32_t LCKR;     /*!< GPIO port configuration lock register, Address offset: 0x1C      */
  __IO uint32_t AFR[2];   /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
} GPIO_TypeDef;

/**
  * @brief System configuration controller
  */
typedef struct
{
  __IO uint32_t MEMRMP;       /*!< SYSCFG memory remap register,                      Address offset: 0x00      */
  __IO uint32_t PMC;          /*!< SYSCFG peripheral mode configuration register,     Address offset: 0x04      */
  __IO uint32_t EXTICR[4];    /*!< SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14 */
  uint32_t      RESERVED[2];  /*!< Reserved, 0x18-0x1C                                                          */
  __IO uint32_t CMPCR;        /*!< SYSCFG Compensation cell control register,         Address offset: 0x20      */
} SYSCFG_TypeDef;

/**
  * @brief Inter-integrated Circuit Interface
  */
typedef struct
{
  __IO uint32_t CR1;        /*!< I2C Control register 1,     Address offset: 0x00 */
  __IO uint32_t CR2;        /*!< I2C Control register 2,     Address offset: 0x04 */
  __IO uint32_t OAR1;       /*!< I2C Own address register 1, Address offset: 0x08 */
  __IO uint32_t OAR2;       /*!< I2C Own address register 2, Address offset: 0x0C */
  __IO uint32_t DR;         /*!< I2C Data register,          Address offset: 0x10 */
  __IO uint32_t SR1;        /*!< I2C Status register 1,      Address offset: 0x14 */
  __IO uint32_t SR2;        /*!< I2C Status register 2,      Address offset: 0x18 */
  __IO uint32_t CCR;        /*!< I2C Clock control register, Address offset: 0x1C */
  __IO uint32_t TRISE;      /*!< I2C TRISE register,         Address offset: 0x20 */
} I2C_TypeDef;


/**
  * @brief Power Control
  */
typedef struct
{
  __IO uint32_t CR;   /*!< PWR power control register,        Address offset: 0x00 */
  __IO uint32_t CSR;  /*!< PWR power control/status register, Address offset: 0x04 */
} PWR_TypeDef;

/**
  * @brief Reset and Clock Control
  */
typedef struct
{
  __IO uint32_t CR;            /*!< RCC clock control register,                                  Address offset: 0x00 */
  __IO uint32_t PLLCFGR;       /*!< RCC PLL configuration register,                              Address offset: 0x04 */
  __IO uint32_t CFGR;          /*!< RCC clock configuration register,                            Address offset: 0x08 */
  __IO uint32_t CIR;           /*!< RCC clock interrupt register,                                Address offset: 0x0C */
  __IO uint32_t AHB1RSTR;      /*!< RCC AHB1 peripheral reset register,                          Address offset: 0x10 */
  __IO uint32_t AHB2RSTR;      /*!< RCC AHB2 peripheral reset register,                          Address offset: 0x14 */
  __IO uint32_t AHB3RSTR;      /*!< RCC AHB3 peripheral reset register,                          Address offset: 0x18 */
  uint32_t      RESERVED0;     /*!< Reserved, 0x1C                                                                    */
  __IO uint32_t APB1RSTR;      /*!< RCC APB1 peripheral reset register,                          Address offset: 0x20 */
  __IO uint32_t APB2RSTR;      /*!< RCC APB2 peripheral reset register,                          Address offset: 0x24 */
  uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                               */
  __IO uint32_t AHB1ENR;       /*!< RCC AHB1 peripheral clock register,                          Address offset: 0x30 */
  __IO uint32_t AHB2ENR;       /*!< RCC AHB2 peripheral clock register,                          Address offset: 0x34 */
  __IO uint32_t AHB3ENR;       /*!< RCC AHB3 peripheral clock register,                          Address offset: 0x38 */
  uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                                    */
  __IO uint32_t APB1ENR;       /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x40 */
  __IO uint32_t APB2ENR;       /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x44 */
  uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                               */
  __IO uint32_t AHB1LPENR;     /*!< RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
  __IO uint32_t AHB2LPENR;     /*!< RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
  __IO uint32_t AHB3LPENR;     /*!< RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
  uint32_t      RESERVED4;     /*!< Reserved, 0x5C                                                                    */
  __IO uint32_t APB1LPENR;     /*!< RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
  __IO uint32_t APB2LPENR;     /*!< RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
  uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                               */
  __IO uint32_t BDCR;          /*!< RCC Backup domain control register,                          Address offset: 0x70 */
  __IO uint32_t CSR;           /*!< RCC clock control & status register,                         Address offset: 0x74 */
  uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                               */
  __IO uint32_t SSCGR;         /*!< RCC spread spectrum clock generation register,               Address offset: 0x80 */
  __IO uint32_t PLLI2SCFGR;    /*!< RCC PLLI2S configuration register,                           Address offset: 0x84 */
} RCC_TypeDef;


/**
  * @brief Serial Peripheral Interface
  */
typedef struct
{
  __IO uint32_t CR1;        /*!< SPI control register 1 (not used in I2S mode),      Address offset: 0x00 */
  __IO uint32_t CR2;        /*!< SPI control register 2,                             Address offset: 0x04 */
  __IO uint32_t SR;         /*!< SPI status register,                                Address offset: 0x08 */
  __IO uint32_t DR;         /*!< SPI data register,                                  Address offset: 0x0C */
  __IO uint32_t CRCPR;      /*!< SPI CRC polynomial register (not used in I2S mode), Address offset: 0x10 */
  __IO uint32_t RXCRCR;     /*!< SPI RX CRC register (not used in I2S mode),         Address offset: 0x14 */
  __IO uint32_t TXCRCR;     /*!< SPI TX CRC register (not used in I2S mode),         Address offset: 0x18 */
  __IO uint32_t I2SCFGR;    /*!< SPI_I2S configuration register,                     Address offset: 0x1C */
  __IO uint32_t I2SPR;      /*!< SPI_I2S prescaler register,                         Address offset: 0x20 */
} SPI_TypeDef;


/**
  * @brief TIM
  */
typedef struct
{
  __IO uint32_t CR1;         /*!< TIM control register 1,              Address offset: 0x00 */
  __IO uint32_t CR2;         /*!< TIM control register 2,              Address offset: 0x04 */
  __IO uint32_t SMCR;        /*!< TIM slave mode control register,     Address offset: 0x08 */
  __IO uint32_t DIER;        /*!< TIM DMA/interrupt enable register,   Address offset: 0x0C */
  __IO uint32_t SR;          /*!< TIM status register,                 Address offset: 0x10 */
  __IO uint32_t EGR;         /*!< TIM event generation register,       Address offset: 0x14 */
  __IO uint32_t CCMR1;       /*!< TIM capture/compare mode register 1, Address offset: 0x18 */
  __IO uint32_t CCMR2;       /*!< TIM capture/compare mode register 2, Address offset: 0x1C */
  __IO uint32_t CCER;        /*!< TIM capture/compare enable register, Address offset: 0x20 */
  __IO uint32_t CNT;         /*!< TIM counter register,                Address offset: 0x24 */
  __IO uint32_t PSC;         /*!< TIM prescaler,                       Address offset: 0x28 */
  __IO uint32_t ARR;         /*!< TIM auto-reload register,            Address offset: 0x2C */
  __IO uint32_t RCR;         /*!< TIM repetition counter register,     Address offset: 0x30 */
  __IO uint32_t CCR1;        /*!< TIM capture/compare register 1,      Address offset: 0x34 */
  __IO uint32_t CCR2;        /*!< TIM capture/compare register 2,      Address offset: 0x38 */
  __IO uint32_t CCR3;        /*!< TIM capture/compare register 3,      Address offset: 0x3C */
  __IO uint32_t CCR4;        /*!< TIM capture/compare register 4,      Address offset: 0x40 */
  __IO uint32_t BDTR;        /*!< TIM break and dead-time register,    Address offset: 0x44 */
  __IO uint32_t DCR;         /*!< TIM DMA control register,            Address offset: 0x48 */
  __IO uint32_t DMAR;        /*!< TIM DMA address for full transfer,   Address offset: 0x4C */
  __IO uint32_t OR;          /*!< TIM option register,                 Address offset: 0x50 */
} TIM_TypeDef;

/**
  * @brief Universal Synchronous Asynchronous Receiver Transmitter
  */
typedef struct
{
  __IO uint32_t SR;         /*!< USART Status register,                   Address offset: 0x00 */
  __IO uint32_t DR;         /*!< USART Data register,                     Address offset: 0x04 */
  __IO uint32_t BRR;        /*!< USART Baud rate register,                Address offset: 0x08 */
  __IO uint32_t CR1;        /*!< USART Control register 1,                Address offset: 0x0C */
  __IO uint32_t CR2;        /*!< USART Control register 2,                Address offset: 0x10 */
  __IO uint32_t CR3;        /*!< USART Control register 3,                Address offset: 0x14 */
  __IO uint32_t GTPR;       /*!< USART Guard time and prescaler register, Address offset: 0x18 */
} USART_TypeDef;


/**************************************************************************************************************
 * 																											  *
 * 											Peripheral Declarations											  *
 * 							(Peripheral base addresses type casted to xxx_TypeDef *)						  *
 * 																											  *
 **************************************************************************************************************/

/* Peripheral Declarations of Core Hardware */
#define SCS_BASE            (0xE000E000UL)                            /*!< System Control Space Base Address */
#define ITM_BASE            (0xE0000000UL)                            /*!< ITM Base Address */
#define DWT_BASE            (0xE0001000UL)                            /*!< DWT Base Address */
#define TPI_BASE            (0xE0040000UL)                            /*!< TPI Base Address */
#define CoreDebug_BASE      (0xE000EDF0UL)                            /*!< Core Debug Base Address */
#define SysTick_BASE        (SCS_BASE +  0x0010UL)                    /*!< SysTick Base Address */
#define NVIC_BASE           (SCS_BASE +  0x0100UL)                    /*!< NVIC Base Address */
#define SCB_BASE            (SCS_BASE +  0x0D00UL)                    /*!< System Control Block Base Address */

#define SCnSCB              ((SCnSCB_Type    *)     SCS_BASE      )   /*!< System control Register not in SCB */
#define SCB                 ((SCB_Type       *)     SCB_BASE      )   /*!< SCB configuration struct */
#define SysTick             ((SysTick_Type   *)     SysTick_BASE  )   /*!< SysTick configuration struct */
#define NVIC                ((NVIC_Type      *)     NVIC_BASE     )   /*!< NVIC configuration struct */
#define ITM                 ((ITM_Type       *)     ITM_BASE      )   /*!< ITM configuration struct */
#define DWT                 ((DWT_Type       *)     DWT_BASE      )   /*!< DWT configuration struct */
#define TPI                 ((TPI_Type       *)     TPI_BASE      )   /*!< TPI configuration struct */
#define CoreDebug           ((CoreDebug_Type *)     CoreDebug_BASE)   /*!< Core Debug configuration struct */

#define FPU_BASE            (SCS_BASE +  0x0F30UL)                    /*!< Floating Point Unit */
#define FPU                 ((FPU_Type       *)     FPU_BASE      )   /*!< Floating Point Unit */


/* Peripheral Declarations of Processor Hardware */
#define FLASH_BASE            0x08000000UL /*!< FLASH(up to 1 MB) base address in the alias region                         */
#define CCMDATARAM_BASE       0x10000000UL /*!< CCM(core coupled memory) data RAM(64 KB) base address in the alias region  */
#define SRAM1_BASE            0x20000000UL /*!< SRAM1(112 KB) base address in the alias region                              */
#define SRAM2_BASE            0x2001C000UL /*!< SRAM2(16 KB) base address in the alias region                              */
#define PERIPH_BASE           0x40000000UL /*!< Peripheral base address in the alias region                                */
#define BKPSRAM_BASE          0x40024000UL /*!< Backup SRAM(4 KB) base address in the alias region                         */
#define FSMC_R_BASE           0xA0000000UL /*!< FSMC registers base address                                                */
#define SRAM1_BB_BASE         0x22000000UL /*!< SRAM1(112 KB) base address in the bit-band region                          */
#define SRAM2_BB_BASE         0x22380000UL /*!< SRAM2(16 KB) base address in the bit-band region                           */
#define PERIPH_BB_BASE        0x42000000UL /*!< Peripheral base address in the bit-band region                             */
#define BKPSRAM_BB_BASE       0x42480000UL /*!< Backup SRAM(4 KB) base address in the bit-band region                      */
#define FLASH_END             0x080FFFFFUL /*!< FLASH end address                                                          */
#define FLASH_OTP_BASE        0x1FFF7800UL /*!< Base address of : (up to 528 Bytes) embedded FLASH OTP Area                */
#define FLASH_OTP_END         0x1FFF7A0FUL /*!< End address of : (up to 528 Bytes) embedded FLASH OTP Area                 */
#define CCMDATARAM_END        0x1000FFFFUL /*!< CCM data RAM end address                                                   */

/* Legacy defines */
#define SRAM_BASE             SRAM1_BASE
#define SRAM_BB_BASE          SRAM1_BB_BASE

/*!< Peripheral memory map */
#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000UL)
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000UL)
#define AHB2PERIPH_BASE       (PERIPH_BASE + 0x10000000UL)

/*!< APB1 peripherals */
#define TIM2_BASE             (APB1PERIPH_BASE + 0x0000UL)
#define TIM3_BASE             (APB1PERIPH_BASE + 0x0400UL)
#define TIM4_BASE             (APB1PERIPH_BASE + 0x0800UL)
#define TIM5_BASE             (APB1PERIPH_BASE + 0x0C00UL)
#define TIM6_BASE             (APB1PERIPH_BASE + 0x1000UL)
#define TIM7_BASE             (APB1PERIPH_BASE + 0x1400UL)
#define TIM12_BASE            (APB1PERIPH_BASE + 0x1800UL)
#define TIM13_BASE            (APB1PERIPH_BASE + 0x1C00UL)
#define TIM14_BASE            (APB1PERIPH_BASE + 0x2000UL)
#define RTC_BASE              (APB1PERIPH_BASE + 0x2800UL)
#define WWDG_BASE             (APB1PERIPH_BASE + 0x2C00UL)
#define IWDG_BASE             (APB1PERIPH_BASE + 0x3000UL)
#define I2S2ext_BASE          (APB1PERIPH_BASE + 0x3400UL)
#define SPI2_BASE             (APB1PERIPH_BASE + 0x3800UL)
#define SPI3_BASE             (APB1PERIPH_BASE + 0x3C00UL)
#define I2S3ext_BASE          (APB1PERIPH_BASE + 0x4000UL)
#define USART2_BASE           (APB1PERIPH_BASE + 0x4400UL)
#define USART3_BASE           (APB1PERIPH_BASE + 0x4800UL)
#define UART4_BASE            (APB1PERIPH_BASE + 0x4C00UL)
#define UART5_BASE            (APB1PERIPH_BASE + 0x5000UL)
#define I2C1_BASE             (APB1PERIPH_BASE + 0x5400UL)
#define I2C2_BASE             (APB1PERIPH_BASE + 0x5800UL)
#define I2C3_BASE             (APB1PERIPH_BASE + 0x5C00UL)
#define CAN1_BASE             (APB1PERIPH_BASE + 0x6400UL)
#define CAN2_BASE             (APB1PERIPH_BASE + 0x6800UL)
#define PWR_BASE              (APB1PERIPH_BASE + 0x7000UL)
#define DAC_BASE              (APB1PERIPH_BASE + 0x7400UL)

/*!< APB2 peripherals */
#define TIM1_BASE             (APB2PERIPH_BASE + 0x0000UL)
#define TIM8_BASE             (APB2PERIPH_BASE + 0x0400UL)
#define USART1_BASE           (APB2PERIPH_BASE + 0x1000UL)
#define USART6_BASE           (APB2PERIPH_BASE + 0x1400UL)
#define ADC1_BASE             (APB2PERIPH_BASE + 0x2000UL)
#define ADC2_BASE             (APB2PERIPH_BASE + 0x2100UL)
#define ADC3_BASE             (APB2PERIPH_BASE + 0x2200UL)
#define ADC123_COMMON_BASE    (APB2PERIPH_BASE + 0x2300UL)
/* Legacy define */
#define ADC_BASE               ADC123_COMMON_BASE
#define SDIO_BASE             (APB2PERIPH_BASE + 0x2C00UL)
#define SPI1_BASE             (APB2PERIPH_BASE + 0x3000UL)
#define SYSCFG_BASE           (APB2PERIPH_BASE + 0x3800UL)
#define EXTI_BASE             (APB2PERIPH_BASE + 0x3C00UL)
#define TIM9_BASE             (APB2PERIPH_BASE + 0x4000UL)
#define TIM10_BASE            (APB2PERIPH_BASE + 0x4400UL)
#define TIM11_BASE            (APB2PERIPH_BASE + 0x4800UL)

/*!< AHB1 peripherals */
#define GPIOA_BASE            (AHB1PERIPH_BASE + 0x0000UL)
#define GPIOB_BASE            (AHB1PERIPH_BASE + 0x0400UL)
#define GPIOC_BASE            (AHB1PERIPH_BASE + 0x0800UL)
#define GPIOD_BASE            (AHB1PERIPH_BASE + 0x0C00UL)
#define GPIOE_BASE            (AHB1PERIPH_BASE + 0x1000UL)
#define GPIOF_BASE            (AHB1PERIPH_BASE + 0x1400UL)
#define GPIOG_BASE            (AHB1PERIPH_BASE + 0x1800UL)
#define GPIOH_BASE            (AHB1PERIPH_BASE + 0x1C00UL)
#define GPIOI_BASE            (AHB1PERIPH_BASE + 0x2000UL)
#define CRC_BASE              (AHB1PERIPH_BASE + 0x3000UL)
#define RCC_BASE              (AHB1PERIPH_BASE + 0x3800UL)
#define FLASH_R_BASE          (AHB1PERIPH_BASE + 0x3C00UL)
#define DMA1_BASE             (AHB1PERIPH_BASE + 0x6000UL)
#define DMA1_Stream0_BASE     (DMA1_BASE + 0x010UL)
#define DMA1_Stream1_BASE     (DMA1_BASE + 0x028UL)
#define DMA1_Stream2_BASE     (DMA1_BASE + 0x040UL)
#define DMA1_Stream3_BASE     (DMA1_BASE + 0x058UL)
#define DMA1_Stream4_BASE     (DMA1_BASE + 0x070UL)
#define DMA1_Stream5_BASE     (DMA1_BASE + 0x088UL)
#define DMA1_Stream6_BASE     (DMA1_BASE + 0x0A0UL)
#define DMA1_Stream7_BASE     (DMA1_BASE + 0x0B8UL)
#define DMA2_BASE             (AHB1PERIPH_BASE + 0x6400UL)
#define DMA2_Stream0_BASE     (DMA2_BASE + 0x010UL)
#define DMA2_Stream1_BASE     (DMA2_BASE + 0x028UL)
#define DMA2_Stream2_BASE     (DMA2_BASE + 0x040UL)
#define DMA2_Stream3_BASE     (DMA2_BASE + 0x058UL)
#define DMA2_Stream4_BASE     (DMA2_BASE + 0x070UL)
#define DMA2_Stream5_BASE     (DMA2_BASE + 0x088UL)
#define DMA2_Stream6_BASE     (DMA2_BASE + 0x0A0UL)
#define DMA2_Stream7_BASE     (DMA2_BASE + 0x0B8UL)
#define ETH_BASE              (AHB1PERIPH_BASE + 0x8000UL)
#define ETH_MAC_BASE          (ETH_BASE)
#define ETH_MMC_BASE          (ETH_BASE + 0x0100UL)
#define ETH_PTP_BASE          (ETH_BASE + 0x0700UL)
#define ETH_DMA_BASE          (ETH_BASE + 0x1000UL)

/*!< AHB2 peripherals */
#define DCMI_BASE             (AHB2PERIPH_BASE + 0x50000UL)
#define RNG_BASE              (AHB2PERIPH_BASE + 0x60800UL)

/*!< FSMC Bankx registers base address */
#define FSMC_Bank1_R_BASE     (FSMC_R_BASE + 0x0000UL)
#define FSMC_Bank1E_R_BASE    (FSMC_R_BASE + 0x0104UL)
#define FSMC_Bank2_3_R_BASE   (FSMC_R_BASE + 0x0060UL)
#define FSMC_Bank4_R_BASE     (FSMC_R_BASE + 0x00A0UL)


/*!< Debug MCU registers base address */
#define DBGMCU_BASE           0xE0042000UL
/*!< USB registers base address */
#define USB_OTG_HS_PERIPH_BASE               0x40040000UL
#define USB_OTG_FS_PERIPH_BASE               0x50000000UL

#define USB_OTG_GLOBAL_BASE                  0x000UL
#define USB_OTG_DEVICE_BASE                  0x800UL
#define USB_OTG_IN_ENDPOINT_BASE             0x900UL
#define USB_OTG_OUT_ENDPOINT_BASE            0xB00UL
#define USB_OTG_EP_REG_SIZE                  0x20UL
#define USB_OTG_HOST_BASE                    0x400UL
#define USB_OTG_HOST_PORT_BASE               0x440UL
#define USB_OTG_HOST_CHANNEL_BASE            0x500UL
#define USB_OTG_HOST_CHANNEL_SIZE            0x20UL
#define USB_OTG_PCGCCTL_BASE                 0xE00UL
#define USB_OTG_FIFO_BASE                    0x1000UL
#define USB_OTG_FIFO_SIZE                    0x1000UL

#define UID_BASE                     0x1FFF7A10UL           /*!< Unique device ID register base address */
#define FLASHSIZE_BASE               0x1FFF7A22UL           /*!< FLASH Size register base address       */
#define PACKAGE_BASE                 0x1FFF7BF0UL           /*!< Package size register base address     */
/**
  * @}
  */

/** @addtogroup Peripheral_declaration
  * @{
  */
#define TIM2                ((TIM_TypeDef *) TIM2_BASE)
#define TIM3                ((TIM_TypeDef *) TIM3_BASE)
#define TIM4                ((TIM_TypeDef *) TIM4_BASE)
#define TIM5                ((TIM_TypeDef *) TIM5_BASE)
#define TIM6                ((TIM_TypeDef *) TIM6_BASE)
#define TIM7                ((TIM_TypeDef *) TIM7_BASE)
#define TIM12               ((TIM_TypeDef *) TIM12_BASE)
#define TIM13               ((TIM_TypeDef *) TIM13_BASE)
#define TIM14               ((TIM_TypeDef *) TIM14_BASE)
#define RTC                 ((RTC_TypeDef *) RTC_BASE)
#define WWDG                ((WWDG_TypeDef *) WWDG_BASE)
#define IWDG                ((IWDG_TypeDef *) IWDG_BASE)
#define I2S2ext             ((SPI_TypeDef *) I2S2ext_BASE)
#define SPI2                ((SPI_TypeDef *) SPI2_BASE)
#define SPI3                ((SPI_TypeDef *) SPI3_BASE)
#define I2S3ext             ((SPI_TypeDef *) I2S3ext_BASE)
#define USART2              ((USART_TypeDef *) USART2_BASE)
#define USART3              ((USART_TypeDef *) USART3_BASE)
#define UART4               ((USART_TypeDef *) UART4_BASE)
#define UART5               ((USART_TypeDef *) UART5_BASE)
#define I2C1                ((I2C_TypeDef *) I2C1_BASE)
#define I2C2                ((I2C_TypeDef *) I2C2_BASE)
#define I2C3                ((I2C_TypeDef *) I2C3_BASE)
#define CAN1                ((CAN_TypeDef *) CAN1_BASE)
#define CAN2                ((CAN_TypeDef *) CAN2_BASE)
#define PWR                 ((PWR_TypeDef *) PWR_BASE)
#define DAC1                ((DAC_TypeDef *) DAC_BASE)
#define DAC                 ((DAC_TypeDef *) DAC_BASE) /* Kept for legacy purpose */
#define TIM1                ((TIM_TypeDef *) TIM1_BASE)
#define TIM8                ((TIM_TypeDef *) TIM8_BASE)
#define USART1              ((USART_TypeDef *) USART1_BASE)
#define USART6              ((USART_TypeDef *) USART6_BASE)
#define ADC1                ((ADC_TypeDef *) ADC1_BASE)
#define ADC2                ((ADC_TypeDef *) ADC2_BASE)
#define ADC3                ((ADC_TypeDef *) ADC3_BASE)
#define ADC123_COMMON       ((ADC_Common_TypeDef *) ADC123_COMMON_BASE)
/* Legacy define */
#define ADC                  ADC123_COMMON
#define SDIO                ((SDIO_TypeDef *) SDIO_BASE)
#define SPI1                ((SPI_TypeDef *) SPI1_BASE)
#define SYSCFG              ((SYSCFG_TypeDef *) SYSCFG_BASE)
#define EXTI                ((EXTI_TypeDef *) EXTI_BASE)
#define TIM9                ((TIM_TypeDef *) TIM9_BASE)
#define TIM10               ((TIM_TypeDef *) TIM10_BASE)
#define TIM11               ((TIM_TypeDef *) TIM11_BASE)
#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE               ((GPIO_TypeDef *) GPIOE_BASE)
#define GPIOF               ((GPIO_TypeDef *) GPIOF_BASE)
#define GPIOG               ((GPIO_TypeDef *) GPIOG_BASE)
#define GPIOH               ((GPIO_TypeDef *) GPIOH_BASE)
#define GPIOI               ((GPIO_TypeDef *) GPIOI_BASE)
#define CRC                 ((CRC_TypeDef *) CRC_BASE)
#define RCC                 ((RCC_TypeDef *) RCC_BASE)
#define FLASH               ((FLASH_TypeDef *) FLASH_R_BASE)
#define DMA1                ((DMA_TypeDef *) DMA1_BASE)
#define DMA1_Stream0        ((DMA_Stream_TypeDef *) DMA1_Stream0_BASE)
#define DMA1_Stream1        ((DMA_Stream_TypeDef *) DMA1_Stream1_BASE)
#define DMA1_Stream2        ((DMA_Stream_TypeDef *) DMA1_Stream2_BASE)
#define DMA1_Stream3        ((DMA_Stream_TypeDef *) DMA1_Stream3_BASE)
#define DMA1_Stream4        ((DMA_Stream_TypeDef *) DMA1_Stream4_BASE)
#define DMA1_Stream5        ((DMA_Stream_TypeDef *) DMA1_Stream5_BASE)
#define DMA1_Stream6        ((DMA_Stream_TypeDef *) DMA1_Stream6_BASE)
#define DMA1_Stream7        ((DMA_Stream_TypeDef *) DMA1_Stream7_BASE)
#define DMA2                ((DMA_TypeDef *) DMA2_BASE)
#define DMA2_Stream0        ((DMA_Stream_TypeDef *) DMA2_Stream0_BASE)
#define DMA2_Stream1        ((DMA_Stream_TypeDef *) DMA2_Stream1_BASE)
#define DMA2_Stream2        ((DMA_Stream_TypeDef *) DMA2_Stream2_BASE)
#define DMA2_Stream3        ((DMA_Stream_TypeDef *) DMA2_Stream3_BASE)
#define DMA2_Stream4        ((DMA_Stream_TypeDef *) DMA2_Stream4_BASE)
#define DMA2_Stream5        ((DMA_Stream_TypeDef *) DMA2_Stream5_BASE)
#define DMA2_Stream6        ((DMA_Stream_TypeDef *) DMA2_Stream6_BASE)
#define DMA2_Stream7        ((DMA_Stream_TypeDef *) DMA2_Stream7_BASE)
#define ETH                 ((ETH_TypeDef *) ETH_BASE)
#define DCMI                ((DCMI_TypeDef *) DCMI_BASE)
#define RNG                 ((RNG_TypeDef *) RNG_BASE)
#define FSMC_Bank1          ((FSMC_Bank1_TypeDef *) FSMC_Bank1_R_BASE)
#define FSMC_Bank1E         ((FSMC_Bank1E_TypeDef *) FSMC_Bank1E_R_BASE)
#define FSMC_Bank2_3        ((FSMC_Bank2_3_TypeDef *) FSMC_Bank2_3_R_BASE)
#define FSMC_Bank4          ((FSMC_Bank4_TypeDef *) FSMC_Bank4_R_BASE)
#define DBGMCU              ((DBGMCU_TypeDef *) DBGMCU_BASE)
#define USB_OTG_FS          ((USB_OTG_GlobalTypeDef *) USB_OTG_FS_PERIPH_BASE)
#define USB_OTG_HS          ((USB_OTG_GlobalTypeDef *) USB_OTG_HS_PERIPH_BASE)

/**
  * @}
  */

/** @addtogroup Exported_constants
  * @{
  */

  /** @addtogroup Peripheral_Registers_Bits_Definition
  * @{
  */

/******************************************************************************/
/*                         Peripheral Registers_Bits_Definition               */
/******************************************************************************/

/* Software Triggered Interrupt Register Definitions */
#define NVIC_STIR_INTID_Pos                 0U                                         /*!< STIR: INTLINESNUM Position */
#define NVIC_STIR_INTID_Msk                (0x1FFUL /*<< NVIC_STIR_INTID_Pos*/)        /*!< STIR: INTLINESNUM Mask */

/* SCB CPUID Register Definitions */
#define SCB_CPUID_IMPLEMENTER_Pos          24U                                            /*!< SCB CPUID: IMPLEMENTER Position */
#define SCB_CPUID_IMPLEMENTER_Msk          (0xFFUL << SCB_CPUID_IMPLEMENTER_Pos)          /*!< SCB CPUID: IMPLEMENTER Mask */
#define SCB_CPUID_VARIANT_Pos              20U                                            /*!< SCB CPUID: VARIANT Position */
#define SCB_CPUID_VARIANT_Msk              (0xFUL << SCB_CPUID_VARIANT_Pos)               /*!< SCB CPUID: VARIANT Mask */
#define SCB_CPUID_ARCHITECTURE_Pos         16U                                            /*!< SCB CPUID: ARCHITECTURE Position */
#define SCB_CPUID_ARCHITECTURE_Msk         (0xFUL << SCB_CPUID_ARCHITECTURE_Pos)          /*!< SCB CPUID: ARCHITECTURE Mask */
#define SCB_CPUID_PARTNO_Pos                4U                                            /*!< SCB CPUID: PARTNO Position */
#define SCB_CPUID_PARTNO_Msk               (0xFFFUL << SCB_CPUID_PARTNO_Pos)              /*!< SCB CPUID: PARTNO Mask */
#define SCB_CPUID_REVISION_Pos              0U                                            /*!< SCB CPUID: REVISION Position */
#define SCB_CPUID_REVISION_Msk             (0xFUL /*<< SCB_CPUID_REVISION_Pos*/)          /*!< SCB CPUID: REVISION Mask */

/* SCB Interrupt Control State Register Definitions */
#define SCB_ICSR_NMIPENDSET_Pos            31U                                            /*!< SCB ICSR: NMIPENDSET Position */
#define SCB_ICSR_NMIPENDSET_Msk            (1UL << SCB_ICSR_NMIPENDSET_Pos)               /*!< SCB ICSR: NMIPENDSET Mask */
#define SCB_ICSR_PENDSVSET_Pos             28U                                            /*!< SCB ICSR: PENDSVSET Position */
#define SCB_ICSR_PENDSVSET_Msk             (1UL << SCB_ICSR_PENDSVSET_Pos)                /*!< SCB ICSR: PENDSVSET Mask */
#define SCB_ICSR_PENDSVCLR_Pos             27U                                            /*!< SCB ICSR: PENDSVCLR Position */
#define SCB_ICSR_PENDSVCLR_Msk             (1UL << SCB_ICSR_PENDSVCLR_Pos)                /*!< SCB ICSR: PENDSVCLR Mask */
#define SCB_ICSR_PENDSTSET_Pos             26U                                            /*!< SCB ICSR: PENDSTSET Position */
#define SCB_ICSR_PENDSTSET_Msk             (1UL << SCB_ICSR_PENDSTSET_Pos)                /*!< SCB ICSR: PENDSTSET Mask */
#define SCB_ICSR_PENDSTCLR_Pos             25U                                            /*!< SCB ICSR: PENDSTCLR Position */
#define SCB_ICSR_PENDSTCLR_Msk             (1UL << SCB_ICSR_PENDSTCLR_Pos)                /*!< SCB ICSR: PENDSTCLR Mask */
#define SCB_ICSR_ISRPREEMPT_Pos            23U                                            /*!< SCB ICSR: ISRPREEMPT Position */
#define SCB_ICSR_ISRPREEMPT_Msk            (1UL << SCB_ICSR_ISRPREEMPT_Pos)               /*!< SCB ICSR: ISRPREEMPT Mask */
#define SCB_ICSR_ISRPENDING_Pos            22U                                            /*!< SCB ICSR: ISRPENDING Position */
#define SCB_ICSR_ISRPENDING_Msk            (1UL << SCB_ICSR_ISRPENDING_Pos)               /*!< SCB ICSR: ISRPENDING Mask */
#define SCB_ICSR_VECTPENDING_Pos           12U                                            /*!< SCB ICSR: VECTPENDING Position */
#define SCB_ICSR_VECTPENDING_Msk           (0x1FFUL << SCB_ICSR_VECTPENDING_Pos)          /*!< SCB ICSR: VECTPENDING Mask */
#define SCB_ICSR_RETTOBASE_Pos             11U                                            /*!< SCB ICSR: RETTOBASE Position */
#define SCB_ICSR_RETTOBASE_Msk             (1UL << SCB_ICSR_RETTOBASE_Pos)                /*!< SCB ICSR: RETTOBASE Mask */
#define SCB_ICSR_VECTACTIVE_Pos             0U                                            /*!< SCB ICSR: VECTACTIVE Position */
#define SCB_ICSR_VECTACTIVE_Msk            (0x1FFUL /*<< SCB_ICSR_VECTACTIVE_Pos*/)       /*!< SCB ICSR: VECTACTIVE Mask */

/* SCB Vector Table Offset Register Definitions */
#define SCB_VTOR_TBLOFF_Pos                 7U                                            /*!< SCB VTOR: TBLOFF Position */
#define SCB_VTOR_TBLOFF_Msk                (0x1FFFFFFUL << SCB_VTOR_TBLOFF_Pos)           /*!< SCB VTOR: TBLOFF Mask */

/* SCB Application Interrupt and Reset Control Register Definitions */
#define SCB_AIRCR_VECTKEY_Pos              16U                                            /*!< SCB AIRCR: VECTKEY Position */
#define SCB_AIRCR_VECTKEY_Msk              (0xFFFFUL << SCB_AIRCR_VECTKEY_Pos)            /*!< SCB AIRCR: VECTKEY Mask */
#define SCB_AIRCR_VECTKEYSTAT_Pos          16U                                            /*!< SCB AIRCR: VECTKEYSTAT Position */
#define SCB_AIRCR_VECTKEYSTAT_Msk          (0xFFFFUL << SCB_AIRCR_VECTKEYSTAT_Pos)        /*!< SCB AIRCR: VECTKEYSTAT Mask */
#define SCB_AIRCR_ENDIANESS_Pos            15U                                            /*!< SCB AIRCR: ENDIANESS Position */
#define SCB_AIRCR_ENDIANESS_Msk            (1UL << SCB_AIRCR_ENDIANESS_Pos)               /*!< SCB AIRCR: ENDIANESS Mask */
#define SCB_AIRCR_PRIGROUP_Pos              8U                                            /*!< SCB AIRCR: PRIGROUP Position */
#define SCB_AIRCR_PRIGROUP_Msk             (7UL << SCB_AIRCR_PRIGROUP_Pos)                /*!< SCB AIRCR: PRIGROUP Mask */
#define SCB_AIRCR_SYSRESETREQ_Pos           2U                                            /*!< SCB AIRCR: SYSRESETREQ Position */
#define SCB_AIRCR_SYSRESETREQ_Msk          (1UL << SCB_AIRCR_SYSRESETREQ_Pos)             /*!< SCB AIRCR: SYSRESETREQ Mask */
#define SCB_AIRCR_VECTCLRACTIVE_Pos         1U                                            /*!< SCB AIRCR: VECTCLRACTIVE Position */
#define SCB_AIRCR_VECTCLRACTIVE_Msk        (1UL << SCB_AIRCR_VECTCLRACTIVE_Pos)           /*!< SCB AIRCR: VECTCLRACTIVE Mask */
#define SCB_AIRCR_VECTRESET_Pos             0U                                            /*!< SCB AIRCR: VECTRESET Position */
#define SCB_AIRCR_VECTRESET_Msk            (1UL /*<< SCB_AIRCR_VECTRESET_Pos*/)           /*!< SCB AIRCR: VECTRESET Mask */

/* SCB System Control Register Definitions */
#define SCB_SCR_SEVONPEND_Pos               4U                                            /*!< SCB SCR: SEVONPEND Position */
#define SCB_SCR_SEVONPEND_Msk              (1UL << SCB_SCR_SEVONPEND_Pos)                 /*!< SCB SCR: SEVONPEND Mask */
#define SCB_SCR_SLEEPDEEP_Pos               2U                                            /*!< SCB SCR: SLEEPDEEP Position */
#define SCB_SCR_SLEEPDEEP_Msk              (1UL << SCB_SCR_SLEEPDEEP_Pos)                 /*!< SCB SCR: SLEEPDEEP Mask */
#define SCB_SCR_SLEEPONEXIT_Pos             1U                                            /*!< SCB SCR: SLEEPONEXIT Position */
#define SCB_SCR_SLEEPONEXIT_Msk            (1UL << SCB_SCR_SLEEPONEXIT_Pos)               /*!< SCB SCR: SLEEPONEXIT Mask */

/* SCB Configuration Control Register Definitions */
#define SCB_CCR_STKALIGN_Pos                9U                                            /*!< SCB CCR: STKALIGN Position */
#define SCB_CCR_STKALIGN_Msk               (1UL << SCB_CCR_STKALIGN_Pos)                  /*!< SCB CCR: STKALIGN Mask */
#define SCB_CCR_BFHFNMIGN_Pos               8U                                            /*!< SCB CCR: BFHFNMIGN Position */
#define SCB_CCR_BFHFNMIGN_Msk              (1UL << SCB_CCR_BFHFNMIGN_Pos)                 /*!< SCB CCR: BFHFNMIGN Mask */
#define SCB_CCR_DIV_0_TRP_Pos               4U                                            /*!< SCB CCR: DIV_0_TRP Position */
#define SCB_CCR_DIV_0_TRP_Msk              (1UL << SCB_CCR_DIV_0_TRP_Pos)                 /*!< SCB CCR: DIV_0_TRP Mask */
#define SCB_CCR_UNALIGN_TRP_Pos             3U                                            /*!< SCB CCR: UNALIGN_TRP Position */
#define SCB_CCR_UNALIGN_TRP_Msk            (1UL << SCB_CCR_UNALIGN_TRP_Pos)               /*!< SCB CCR: UNALIGN_TRP Mask */
#define SCB_CCR_USERSETMPEND_Pos            1U                                            /*!< SCB CCR: USERSETMPEND Position */
#define SCB_CCR_USERSETMPEND_Msk           (1UL << SCB_CCR_USERSETMPEND_Pos)              /*!< SCB CCR: USERSETMPEND Mask */
#define SCB_CCR_NONBASETHRDENA_Pos          0U                                            /*!< SCB CCR: NONBASETHRDENA Position */
#define SCB_CCR_NONBASETHRDENA_Msk         (1UL /*<< SCB_CCR_NONBASETHRDENA_Pos*/)        /*!< SCB CCR: NONBASETHRDENA Mask */

/* SCB System Handler Control and State Register Definitions */
#define SCB_SHCSR_USGFAULTENA_Pos          18U                                            /*!< SCB SHCSR: USGFAULTENA Position */
#define SCB_SHCSR_USGFAULTENA_Msk          (1UL << SCB_SHCSR_USGFAULTENA_Pos)             /*!< SCB SHCSR: USGFAULTENA Mask */
#define SCB_SHCSR_BUSFAULTENA_Pos          17U                                            /*!< SCB SHCSR: BUSFAULTENA Position */
#define SCB_SHCSR_BUSFAULTENA_Msk          (1UL << SCB_SHCSR_BUSFAULTENA_Pos)             /*!< SCB SHCSR: BUSFAULTENA Mask */
#define SCB_SHCSR_MEMFAULTENA_Pos          16U                                            /*!< SCB SHCSR: MEMFAULTENA Position */
#define SCB_SHCSR_MEMFAULTENA_Msk          (1UL << SCB_SHCSR_MEMFAULTENA_Pos)             /*!< SCB SHCSR: MEMFAULTENA Mask */
#define SCB_SHCSR_SVCALLPENDED_Pos         15U                                            /*!< SCB SHCSR: SVCALLPENDED Position */
#define SCB_SHCSR_SVCALLPENDED_Msk         (1UL << SCB_SHCSR_SVCALLPENDED_Pos)            /*!< SCB SHCSR: SVCALLPENDED Mask */
#define SCB_SHCSR_BUSFAULTPENDED_Pos       14U                                            /*!< SCB SHCSR: BUSFAULTPENDED Position */
#define SCB_SHCSR_BUSFAULTPENDED_Msk       (1UL << SCB_SHCSR_BUSFAULTPENDED_Pos)          /*!< SCB SHCSR: BUSFAULTPENDED Mask */
#define SCB_SHCSR_MEMFAULTPENDED_Pos       13U                                            /*!< SCB SHCSR: MEMFAULTPENDED Position */
#define SCB_SHCSR_MEMFAULTPENDED_Msk       (1UL << SCB_SHCSR_MEMFAULTPENDED_Pos)          /*!< SCB SHCSR: MEMFAULTPENDED Mask */
#define SCB_SHCSR_USGFAULTPENDED_Pos       12U                                            /*!< SCB SHCSR: USGFAULTPENDED Position */
#define SCB_SHCSR_USGFAULTPENDED_Msk       (1UL << SCB_SHCSR_USGFAULTPENDED_Pos)          /*!< SCB SHCSR: USGFAULTPENDED Mask */
#define SCB_SHCSR_SYSTICKACT_Pos           11U                                            /*!< SCB SHCSR: SYSTICKACT Position */
#define SCB_SHCSR_SYSTICKACT_Msk           (1UL << SCB_SHCSR_SYSTICKACT_Pos)              /*!< SCB SHCSR: SYSTICKACT Mask */
#define SCB_SHCSR_PENDSVACT_Pos            10U                                            /*!< SCB SHCSR: PENDSVACT Position */
#define SCB_SHCSR_PENDSVACT_Msk            (1UL << SCB_SHCSR_PENDSVACT_Pos)               /*!< SCB SHCSR: PENDSVACT Mask */
#define SCB_SHCSR_MONITORACT_Pos            8U                                            /*!< SCB SHCSR: MONITORACT Position */
#define SCB_SHCSR_MONITORACT_Msk           (1UL << SCB_SHCSR_MONITORACT_Pos)              /*!< SCB SHCSR: MONITORACT Mask */
#define SCB_SHCSR_SVCALLACT_Pos             7U                                            /*!< SCB SHCSR: SVCALLACT Position */
#define SCB_SHCSR_SVCALLACT_Msk            (1UL << SCB_SHCSR_SVCALLACT_Pos)               /*!< SCB SHCSR: SVCALLACT Mask */
#define SCB_SHCSR_USGFAULTACT_Pos           3U                                            /*!< SCB SHCSR: USGFAULTACT Position */
#define SCB_SHCSR_USGFAULTACT_Msk          (1UL << SCB_SHCSR_USGFAULTACT_Pos)             /*!< SCB SHCSR: USGFAULTACT Mask */
#define SCB_SHCSR_BUSFAULTACT_Pos           1U                                            /*!< SCB SHCSR: BUSFAULTACT Position */
#define SCB_SHCSR_BUSFAULTACT_Msk          (1UL << SCB_SHCSR_BUSFAULTACT_Pos)             /*!< SCB SHCSR: BUSFAULTACT Mask */
#define SCB_SHCSR_MEMFAULTACT_Pos           0U                                            /*!< SCB SHCSR: MEMFAULTACT Position */
#define SCB_SHCSR_MEMFAULTACT_Msk          (1UL /*<< SCB_SHCSR_MEMFAULTACT_Pos*/)         /*!< SCB SHCSR: MEMFAULTACT Mask */

/* SCB Configurable Fault Status Register Definitions */
#define SCB_CFSR_USGFAULTSR_Pos            16U                                            /*!< SCB CFSR: Usage Fault Status Register Position */
#define SCB_CFSR_USGFAULTSR_Msk            (0xFFFFUL << SCB_CFSR_USGFAULTSR_Pos)          /*!< SCB CFSR: Usage Fault Status Register Mask */
#define SCB_CFSR_BUSFAULTSR_Pos             8U                                            /*!< SCB CFSR: Bus Fault Status Register Position */
#define SCB_CFSR_BUSFAULTSR_Msk            (0xFFUL << SCB_CFSR_BUSFAULTSR_Pos)            /*!< SCB CFSR: Bus Fault Status Register Mask */
#define SCB_CFSR_MEMFAULTSR_Pos             0U                                            /*!< SCB CFSR: Memory Manage Fault Status Register Position */
#define SCB_CFSR_MEMFAULTSR_Msk            (0xFFUL /*<< SCB_CFSR_MEMFAULTSR_Pos*/)        /*!< SCB CFSR: Memory Manage Fault Status Register Mask */

/* MemManage Fault Status Register (part of SCB Configurable Fault Status Register) */
#define SCB_CFSR_MMARVALID_Pos             (SCB_SHCSR_MEMFAULTACT_Pos + 7U)               /*!< SCB CFSR (MMFSR): MMARVALID Position */
#define SCB_CFSR_MMARVALID_Msk             (1UL << SCB_CFSR_MMARVALID_Pos)                /*!< SCB CFSR (MMFSR): MMARVALID Mask */
#define SCB_CFSR_MLSPERR_Pos               (SCB_SHCSR_MEMFAULTACT_Pos + 5U)               /*!< SCB CFSR (MMFSR): MLSPERR Position */
#define SCB_CFSR_MLSPERR_Msk               (1UL << SCB_CFSR_MLSPERR_Pos)                  /*!< SCB CFSR (MMFSR): MLSPERR Mask */
#define SCB_CFSR_MSTKERR_Pos               (SCB_SHCSR_MEMFAULTACT_Pos + 4U)               /*!< SCB CFSR (MMFSR): MSTKERR Position */
#define SCB_CFSR_MSTKERR_Msk               (1UL << SCB_CFSR_MSTKERR_Pos)                  /*!< SCB CFSR (MMFSR): MSTKERR Mask */
#define SCB_CFSR_MUNSTKERR_Pos             (SCB_SHCSR_MEMFAULTACT_Pos + 3U)               /*!< SCB CFSR (MMFSR): MUNSTKERR Position */
#define SCB_CFSR_MUNSTKERR_Msk             (1UL << SCB_CFSR_MUNSTKERR_Pos)                /*!< SCB CFSR (MMFSR): MUNSTKERR Mask */
#define SCB_CFSR_DACCVIOL_Pos              (SCB_SHCSR_MEMFAULTACT_Pos + 1U)               /*!< SCB CFSR (MMFSR): DACCVIOL Position */
#define SCB_CFSR_DACCVIOL_Msk              (1UL << SCB_CFSR_DACCVIOL_Pos)                 /*!< SCB CFSR (MMFSR): DACCVIOL Mask */
#define SCB_CFSR_IACCVIOL_Pos              (SCB_SHCSR_MEMFAULTACT_Pos + 0U)               /*!< SCB CFSR (MMFSR): IACCVIOL Position */
#define SCB_CFSR_IACCVIOL_Msk              (1UL /*<< SCB_CFSR_IACCVIOL_Pos*/)             /*!< SCB CFSR (MMFSR): IACCVIOL Mask */

/* BusFault Status Register (part of SCB Configurable Fault Status Register) */
#define SCB_CFSR_BFARVALID_Pos            (SCB_CFSR_BUSFAULTSR_Pos + 7U)                  /*!< SCB CFSR (BFSR): BFARVALID Position */
#define SCB_CFSR_BFARVALID_Msk            (1UL << SCB_CFSR_BFARVALID_Pos)                 /*!< SCB CFSR (BFSR): BFARVALID Mask */
#define SCB_CFSR_LSPERR_Pos               (SCB_CFSR_BUSFAULTSR_Pos + 5U)                  /*!< SCB CFSR (BFSR): LSPERR Position */
#define SCB_CFSR_LSPERR_Msk               (1UL << SCB_CFSR_LSPERR_Pos)                    /*!< SCB CFSR (BFSR): LSPERR Mask */
#define SCB_CFSR_STKERR_Pos               (SCB_CFSR_BUSFAULTSR_Pos + 4U)                  /*!< SCB CFSR (BFSR): STKERR Position */
#define SCB_CFSR_STKERR_Msk               (1UL << SCB_CFSR_STKERR_Pos)                    /*!< SCB CFSR (BFSR): STKERR Mask */
#define SCB_CFSR_UNSTKERR_Pos             (SCB_CFSR_BUSFAULTSR_Pos + 3U)                  /*!< SCB CFSR (BFSR): UNSTKERR Position */
#define SCB_CFSR_UNSTKERR_Msk             (1UL << SCB_CFSR_UNSTKERR_Pos)                  /*!< SCB CFSR (BFSR): UNSTKERR Mask */
#define SCB_CFSR_IMPRECISERR_Pos          (SCB_CFSR_BUSFAULTSR_Pos + 2U)                  /*!< SCB CFSR (BFSR): IMPRECISERR Position */
#define SCB_CFSR_IMPRECISERR_Msk          (1UL << SCB_CFSR_IMPRECISERR_Pos)               /*!< SCB CFSR (BFSR): IMPRECISERR Mask */
#define SCB_CFSR_PRECISERR_Pos            (SCB_CFSR_BUSFAULTSR_Pos + 1U)                  /*!< SCB CFSR (BFSR): PRECISERR Position */
#define SCB_CFSR_PRECISERR_Msk            (1UL << SCB_CFSR_PRECISERR_Pos)                 /*!< SCB CFSR (BFSR): PRECISERR Mask */
#define SCB_CFSR_IBUSERR_Pos              (SCB_CFSR_BUSFAULTSR_Pos + 0U)                  /*!< SCB CFSR (BFSR): IBUSERR Position */
#define SCB_CFSR_IBUSERR_Msk              (1UL << SCB_CFSR_IBUSERR_Pos)                   /*!< SCB CFSR (BFSR): IBUSERR Mask */

/* UsageFault Status Register (part of SCB Configurable Fault Status Register) */
#define SCB_CFSR_DIVBYZERO_Pos            (SCB_CFSR_USGFAULTSR_Pos + 9U)                  /*!< SCB CFSR (UFSR): DIVBYZERO Position */
#define SCB_CFSR_DIVBYZERO_Msk            (1UL << SCB_CFSR_DIVBYZERO_Pos)                 /*!< SCB CFSR (UFSR): DIVBYZERO Mask */
#define SCB_CFSR_UNALIGNED_Pos            (SCB_CFSR_USGFAULTSR_Pos + 8U)                  /*!< SCB CFSR (UFSR): UNALIGNED Position */
#define SCB_CFSR_UNALIGNED_Msk            (1UL << SCB_CFSR_UNALIGNED_Pos)                 /*!< SCB CFSR (UFSR): UNALIGNED Mask */
#define SCB_CFSR_NOCP_Pos                 (SCB_CFSR_USGFAULTSR_Pos + 3U)                  /*!< SCB CFSR (UFSR): NOCP Position */
#define SCB_CFSR_NOCP_Msk                 (1UL << SCB_CFSR_NOCP_Pos)                      /*!< SCB CFSR (UFSR): NOCP Mask */
#define SCB_CFSR_INVPC_Pos                (SCB_CFSR_USGFAULTSR_Pos + 2U)                  /*!< SCB CFSR (UFSR): INVPC Position */
#define SCB_CFSR_INVPC_Msk                (1UL << SCB_CFSR_INVPC_Pos)                     /*!< SCB CFSR (UFSR): INVPC Mask */
#define SCB_CFSR_INVSTATE_Pos             (SCB_CFSR_USGFAULTSR_Pos + 1U)                  /*!< SCB CFSR (UFSR): INVSTATE Position */
#define SCB_CFSR_INVSTATE_Msk             (1UL << SCB_CFSR_INVSTATE_Pos)                  /*!< SCB CFSR (UFSR): INVSTATE Mask */
#define SCB_CFSR_UNDEFINSTR_Pos           (SCB_CFSR_USGFAULTSR_Pos + 0U)                  /*!< SCB CFSR (UFSR): UNDEFINSTR Position */
#define SCB_CFSR_UNDEFINSTR_Msk           (1UL << SCB_CFSR_UNDEFINSTR_Pos)                /*!< SCB CFSR (UFSR): UNDEFINSTR Mask */

/* SCB Hard Fault Status Register Definitions */
#define SCB_HFSR_DEBUGEVT_Pos              31U                                            /*!< SCB HFSR: DEBUGEVT Position */
#define SCB_HFSR_DEBUGEVT_Msk              (1UL << SCB_HFSR_DEBUGEVT_Pos)                 /*!< SCB HFSR: DEBUGEVT Mask */
#define SCB_HFSR_FORCED_Pos                30U                                            /*!< SCB HFSR: FORCED Position */
#define SCB_HFSR_FORCED_Msk                (1UL << SCB_HFSR_FORCED_Pos)                   /*!< SCB HFSR: FORCED Mask */
#define SCB_HFSR_VECTTBL_Pos                1U                                            /*!< SCB HFSR: VECTTBL Position */
#define SCB_HFSR_VECTTBL_Msk               (1UL << SCB_HFSR_VECTTBL_Pos)                  /*!< SCB HFSR: VECTTBL Mask */

/* SCB Debug Fault Status Register Definitions */
#define SCB_DFSR_EXTERNAL_Pos               4U                                            /*!< SCB DFSR: EXTERNAL Position */
#define SCB_DFSR_EXTERNAL_Msk              (1UL << SCB_DFSR_EXTERNAL_Pos)                 /*!< SCB DFSR: EXTERNAL Mask */
#define SCB_DFSR_VCATCH_Pos                 3U                                            /*!< SCB DFSR: VCATCH Position */
#define SCB_DFSR_VCATCH_Msk                (1UL << SCB_DFSR_VCATCH_Pos)                   /*!< SCB DFSR: VCATCH Mask */
#define SCB_DFSR_DWTTRAP_Pos                2U                                            /*!< SCB DFSR: DWTTRAP Position */
#define SCB_DFSR_DWTTRAP_Msk               (1UL << SCB_DFSR_DWTTRAP_Pos)                  /*!< SCB DFSR: DWTTRAP Mask */
#define SCB_DFSR_BKPT_Pos                   1U                                            /*!< SCB DFSR: BKPT Position */
#define SCB_DFSR_BKPT_Msk                  (1UL << SCB_DFSR_BKPT_Pos)                     /*!< SCB DFSR: BKPT Mask */
#define SCB_DFSR_HALTED_Pos                 0U                                            /*!< SCB DFSR: HALTED Position */
#define SCB_DFSR_HALTED_Msk                (1UL /*<< SCB_DFSR_HALTED_Pos*/)               /*!< SCB DFSR: HALTED Mask */


/* SysTick Control / Status Register Definitions */
#define SysTick_CTRL_COUNTFLAG_Pos         16U                                            /*!< SysTick CTRL: COUNTFLAG Position */
#define SysTick_CTRL_COUNTFLAG_Msk         (1UL << SysTick_CTRL_COUNTFLAG_Pos)            /*!< SysTick CTRL: COUNTFLAG Mask */
#define SysTick_CTRL_CLKSOURCE_Pos          2U                                            /*!< SysTick CTRL: CLKSOURCE Position */
#define SysTick_CTRL_CLKSOURCE_Msk         (1UL << SysTick_CTRL_CLKSOURCE_Pos)            /*!< SysTick CTRL: CLKSOURCE Mask */
#define SysTick_CTRL_TICKINT_Pos            1U                                            /*!< SysTick CTRL: TICKINT Position */
#define SysTick_CTRL_TICKINT_Msk           (1UL << SysTick_CTRL_TICKINT_Pos)              /*!< SysTick CTRL: TICKINT Mask */
#define SysTick_CTRL_ENABLE_Pos             0U                                            /*!< SysTick CTRL: ENABLE Position */
#define SysTick_CTRL_ENABLE_Msk            (1UL /*<< SysTick_CTRL_ENABLE_Pos*/)           /*!< SysTick CTRL: ENABLE Mask */

/* SysTick Reload Register Definitions */
#define SysTick_LOAD_RELOAD_Pos             0U                                            /*!< SysTick LOAD: RELOAD Position */
#define SysTick_LOAD_RELOAD_Msk            (0xFFFFFFUL /*<< SysTick_LOAD_RELOAD_Pos*/)    /*!< SysTick LOAD: RELOAD Mask */

/* SysTick Current Register Definitions */
#define SysTick_VAL_CURRENT_Pos             0U                                            /*!< SysTick VAL: CURRENT Position */
#define SysTick_VAL_CURRENT_Msk            (0xFFFFFFUL /*<< SysTick_VAL_CURRENT_Pos*/)    /*!< SysTick VAL: CURRENT Mask */

/* SysTick Calibration Register Definitions */
#define SysTick_CALIB_NOREF_Pos            31U                                            /*!< SysTick CALIB: NOREF Position */
#define SysTick_CALIB_NOREF_Msk            (1UL << SysTick_CALIB_NOREF_Pos)               /*!< SysTick CALIB: NOREF Mask */
#define SysTick_CALIB_SKEW_Pos             30U                                            /*!< SysTick CALIB: SKEW Position */
#define SysTick_CALIB_SKEW_Msk             (1UL << SysTick_CALIB_SKEW_Pos)                /*!< SysTick CALIB: SKEW Mask */
#define SysTick_CALIB_TENMS_Pos             0U                                            /*!< SysTick CALIB: TENMS Position */
#define SysTick_CALIB_TENMS_Msk            (0xFFFFFFUL /*<< SysTick_CALIB_TENMS_Pos*/)    /*!< SysTick CALIB: TENMS Mask */


/******************************************************************************/
/*                                                                            */
/*                        Analog to Digital Converter                         */
/*                                                                            */
/******************************************************************************/
/*
 * @brief Specific device feature definitions (not present on all devices in the STM32F4 serie)
 */
#define ADC_MULTIMODE_SUPPORT                                                  /*!<ADC Multimode feature available on specific devices */

/********************  Bit definition for ADC_SR register  ********************/
#define ADC_SR_AWD_Pos            (0U)
#define ADC_SR_AWD_Msk            (0x1UL << ADC_SR_AWD_Pos)                     /*!< 0x00000001 */
#define ADC_SR_AWD                ADC_SR_AWD_Msk                               /*!<Analog watchdog flag */
#define ADC_SR_EOC_Pos            (1U)
#define ADC_SR_EOC_Msk            (0x1UL << ADC_SR_EOC_Pos)                     /*!< 0x00000002 */
#define ADC_SR_EOC                ADC_SR_EOC_Msk                               /*!<End of conversion */
#define ADC_SR_JEOC_Pos           (2U)
#define ADC_SR_JEOC_Msk           (0x1UL << ADC_SR_JEOC_Pos)                    /*!< 0x00000004 */
#define ADC_SR_JEOC               ADC_SR_JEOC_Msk                              /*!<Injected channel end of conversion */
#define ADC_SR_JSTRT_Pos          (3U)
#define ADC_SR_JSTRT_Msk          (0x1UL << ADC_SR_JSTRT_Pos)                   /*!< 0x00000008 */
#define ADC_SR_JSTRT              ADC_SR_JSTRT_Msk                             /*!<Injected channel Start flag */
#define ADC_SR_STRT_Pos           (4U)
#define ADC_SR_STRT_Msk           (0x1UL << ADC_SR_STRT_Pos)                    /*!< 0x00000010 */
#define ADC_SR_STRT               ADC_SR_STRT_Msk                              /*!<Regular channel Start flag */
#define ADC_SR_OVR_Pos            (5U)
#define ADC_SR_OVR_Msk            (0x1UL << ADC_SR_OVR_Pos)                     /*!< 0x00000020 */
#define ADC_SR_OVR                ADC_SR_OVR_Msk                               /*!<Overrun flag */

/*******************  Bit definition for ADC_CR1 register  ********************/
#define ADC_CR1_AWDCH_Pos         (0U)
#define ADC_CR1_AWDCH_Msk         (0x1FUL << ADC_CR1_AWDCH_Pos)                 /*!< 0x0000001F */
#define ADC_CR1_AWDCH             ADC_CR1_AWDCH_Msk                            /*!<AWDCH[4:0] bits (Analog watchdog channel select bits) */
#define ADC_CR1_AWDCH_0           (0x01UL << ADC_CR1_AWDCH_Pos)                 /*!< 0x00000001 */
#define ADC_CR1_AWDCH_1           (0x02UL << ADC_CR1_AWDCH_Pos)                 /*!< 0x00000002 */
#define ADC_CR1_AWDCH_2           (0x04UL << ADC_CR1_AWDCH_Pos)                 /*!< 0x00000004 */
#define ADC_CR1_AWDCH_3           (0x08UL << ADC_CR1_AWDCH_Pos)                 /*!< 0x00000008 */
#define ADC_CR1_AWDCH_4           (0x10UL << ADC_CR1_AWDCH_Pos)                 /*!< 0x00000010 */
#define ADC_CR1_EOCIE_Pos         (5U)
#define ADC_CR1_EOCIE_Msk         (0x1UL << ADC_CR1_EOCIE_Pos)                  /*!< 0x00000020 */
#define ADC_CR1_EOCIE             ADC_CR1_EOCIE_Msk                            /*!<Interrupt enable for EOC */
#define ADC_CR1_AWDIE_Pos         (6U)
#define ADC_CR1_AWDIE_Msk         (0x1UL << ADC_CR1_AWDIE_Pos)                  /*!< 0x00000040 */
#define ADC_CR1_AWDIE             ADC_CR1_AWDIE_Msk                            /*!<AAnalog Watchdog interrupt enable */
#define ADC_CR1_JEOCIE_Pos        (7U)
#define ADC_CR1_JEOCIE_Msk        (0x1UL << ADC_CR1_JEOCIE_Pos)                 /*!< 0x00000080 */
#define ADC_CR1_JEOCIE            ADC_CR1_JEOCIE_Msk                           /*!<Interrupt enable for injected channels */
#define ADC_CR1_SCAN_Pos          (8U)
#define ADC_CR1_SCAN_Msk          (0x1UL << ADC_CR1_SCAN_Pos)                   /*!< 0x00000100 */
#define ADC_CR1_SCAN              ADC_CR1_SCAN_Msk                             /*!<Scan mode */
#define ADC_CR1_AWDSGL_Pos        (9U)
#define ADC_CR1_AWDSGL_Msk        (0x1UL << ADC_CR1_AWDSGL_Pos)                 /*!< 0x00000200 */
#define ADC_CR1_AWDSGL            ADC_CR1_AWDSGL_Msk                           /*!<Enable the watchdog on a single channel in scan mode */
#define ADC_CR1_JAUTO_Pos         (10U)
#define ADC_CR1_JAUTO_Msk         (0x1UL << ADC_CR1_JAUTO_Pos)                  /*!< 0x00000400 */
#define ADC_CR1_JAUTO             ADC_CR1_JAUTO_Msk                            /*!<Automatic injected group conversion */
#define ADC_CR1_DISCEN_Pos        (11U)
#define ADC_CR1_DISCEN_Msk        (0x1UL << ADC_CR1_DISCEN_Pos)                 /*!< 0x00000800 */
#define ADC_CR1_DISCEN            ADC_CR1_DISCEN_Msk                           /*!<Discontinuous mode on regular channels */
#define ADC_CR1_JDISCEN_Pos       (12U)
#define ADC_CR1_JDISCEN_Msk       (0x1UL << ADC_CR1_JDISCEN_Pos)                /*!< 0x00001000 */
#define ADC_CR1_JDISCEN           ADC_CR1_JDISCEN_Msk                          /*!<Discontinuous mode on injected channels */
#define ADC_CR1_DISCNUM_Pos       (13U)
#define ADC_CR1_DISCNUM_Msk       (0x7UL << ADC_CR1_DISCNUM_Pos)                /*!< 0x0000E000 */
#define ADC_CR1_DISCNUM           ADC_CR1_DISCNUM_Msk                          /*!<DISCNUM[2:0] bits (Discontinuous mode channel count) */
#define ADC_CR1_DISCNUM_0         (0x1UL << ADC_CR1_DISCNUM_Pos)                /*!< 0x00002000 */
#define ADC_CR1_DISCNUM_1         (0x2UL << ADC_CR1_DISCNUM_Pos)                /*!< 0x00004000 */
#define ADC_CR1_DISCNUM_2         (0x4UL << ADC_CR1_DISCNUM_Pos)                /*!< 0x00008000 */
#define ADC_CR1_JAWDEN_Pos        (22U)
#define ADC_CR1_JAWDEN_Msk        (0x1UL << ADC_CR1_JAWDEN_Pos)                 /*!< 0x00400000 */
#define ADC_CR1_JAWDEN            ADC_CR1_JAWDEN_Msk                           /*!<Analog watchdog enable on injected channels */
#define ADC_CR1_AWDEN_Pos         (23U)
#define ADC_CR1_AWDEN_Msk         (0x1UL << ADC_CR1_AWDEN_Pos)                  /*!< 0x00800000 */
#define ADC_CR1_AWDEN             ADC_CR1_AWDEN_Msk                            /*!<Analog watchdog enable on regular channels */
#define ADC_CR1_RES_Pos           (24U)
#define ADC_CR1_RES_Msk           (0x3UL << ADC_CR1_RES_Pos)                    /*!< 0x03000000 */
#define ADC_CR1_RES               ADC_CR1_RES_Msk                              /*!<RES[2:0] bits (Resolution) */
#define ADC_CR1_RES_0             (0x1UL << ADC_CR1_RES_Pos)                    /*!< 0x01000000 */
#define ADC_CR1_RES_1             (0x2UL << ADC_CR1_RES_Pos)                    /*!< 0x02000000 */
#define ADC_CR1_OVRIE_Pos         (26U)
#define ADC_CR1_OVRIE_Msk         (0x1UL << ADC_CR1_OVRIE_Pos)                  /*!< 0x04000000 */
#define ADC_CR1_OVRIE             ADC_CR1_OVRIE_Msk                            /*!<overrun interrupt enable */

/*******************  Bit definition for ADC_CR2 register  ********************/
#define ADC_CR2_ADON_Pos          (0U)
#define ADC_CR2_ADON_Msk          (0x1UL << ADC_CR2_ADON_Pos)                   /*!< 0x00000001 */
#define ADC_CR2_ADON              ADC_CR2_ADON_Msk                             /*!<A/D Converter ON / OFF */
#define ADC_CR2_CONT_Pos          (1U)
#define ADC_CR2_CONT_Msk          (0x1UL << ADC_CR2_CONT_Pos)                   /*!< 0x00000002 */
#define ADC_CR2_CONT              ADC_CR2_CONT_Msk                             /*!<Continuous Conversion */
#define ADC_CR2_DMA_Pos           (8U)
#define ADC_CR2_DMA_Msk           (0x1UL << ADC_CR2_DMA_Pos)                    /*!< 0x00000100 */
#define ADC_CR2_DMA               ADC_CR2_DMA_Msk                              /*!<Direct Memory access mode */
#define ADC_CR2_DDS_Pos           (9U)
#define ADC_CR2_DDS_Msk           (0x1UL << ADC_CR2_DDS_Pos)                    /*!< 0x00000200 */
#define ADC_CR2_DDS               ADC_CR2_DDS_Msk                              /*!<DMA disable selection (Single ADC) */
#define ADC_CR2_EOCS_Pos          (10U)
#define ADC_CR2_EOCS_Msk          (0x1UL << ADC_CR2_EOCS_Pos)                   /*!< 0x00000400 */
#define ADC_CR2_EOCS              ADC_CR2_EOCS_Msk                             /*!<End of conversion selection */
#define ADC_CR2_ALIGN_Pos         (11U)
#define ADC_CR2_ALIGN_Msk         (0x1UL << ADC_CR2_ALIGN_Pos)                  /*!< 0x00000800 */
#define ADC_CR2_ALIGN             ADC_CR2_ALIGN_Msk                            /*!<Data Alignment */
#define ADC_CR2_JEXTSEL_Pos       (16U)
#define ADC_CR2_JEXTSEL_Msk       (0xFUL << ADC_CR2_JEXTSEL_Pos)                /*!< 0x000F0000 */
#define ADC_CR2_JEXTSEL           ADC_CR2_JEXTSEL_Msk                          /*!<JEXTSEL[3:0] bits (External event select for injected group) */
#define ADC_CR2_JEXTSEL_0         (0x1UL << ADC_CR2_JEXTSEL_Pos)                /*!< 0x00010000 */
#define ADC_CR2_JEXTSEL_1         (0x2UL << ADC_CR2_JEXTSEL_Pos)                /*!< 0x00020000 */
#define ADC_CR2_JEXTSEL_2         (0x4UL << ADC_CR2_JEXTSEL_Pos)                /*!< 0x00040000 */
#define ADC_CR2_JEXTSEL_3         (0x8UL << ADC_CR2_JEXTSEL_Pos)                /*!< 0x00080000 */
#define ADC_CR2_JEXTEN_Pos        (20U)
#define ADC_CR2_JEXTEN_Msk        (0x3UL << ADC_CR2_JEXTEN_Pos)                 /*!< 0x00300000 */
#define ADC_CR2_JEXTEN            ADC_CR2_JEXTEN_Msk                           /*!<JEXTEN[1:0] bits (External Trigger Conversion mode for injected channelsp) */
#define ADC_CR2_JEXTEN_0          (0x1UL << ADC_CR2_JEXTEN_Pos)                 /*!< 0x00100000 */
#define ADC_CR2_JEXTEN_1          (0x2UL << ADC_CR2_JEXTEN_Pos)                 /*!< 0x00200000 */
#define ADC_CR2_JSWSTART_Pos      (22U)
#define ADC_CR2_JSWSTART_Msk      (0x1UL << ADC_CR2_JSWSTART_Pos)               /*!< 0x00400000 */
#define ADC_CR2_JSWSTART          ADC_CR2_JSWSTART_Msk                         /*!<Start Conversion of injected channels */
#define ADC_CR2_EXTSEL_Pos        (24U)
#define ADC_CR2_EXTSEL_Msk        (0xFUL << ADC_CR2_EXTSEL_Pos)                 /*!< 0x0F000000 */
#define ADC_CR2_EXTSEL            ADC_CR2_EXTSEL_Msk                           /*!<EXTSEL[3:0] bits (External Event Select for regular group) */
#define ADC_CR2_EXTSEL_0          (0x1UL << ADC_CR2_EXTSEL_Pos)                 /*!< 0x01000000 */
#define ADC_CR2_EXTSEL_1          (0x2UL << ADC_CR2_EXTSEL_Pos)                 /*!< 0x02000000 */
#define ADC_CR2_EXTSEL_2          (0x4UL << ADC_CR2_EXTSEL_Pos)                 /*!< 0x04000000 */
#define ADC_CR2_EXTSEL_3          (0x8UL << ADC_CR2_EXTSEL_Pos)                 /*!< 0x08000000 */
#define ADC_CR2_EXTEN_Pos         (28U)
#define ADC_CR2_EXTEN_Msk         (0x3UL << ADC_CR2_EXTEN_Pos)                  /*!< 0x30000000 */
#define ADC_CR2_EXTEN             ADC_CR2_EXTEN_Msk                            /*!<EXTEN[1:0] bits (External Trigger Conversion mode for regular channelsp) */
#define ADC_CR2_EXTEN_0           (0x1UL << ADC_CR2_EXTEN_Pos)                  /*!< 0x10000000 */
#define ADC_CR2_EXTEN_1           (0x2UL << ADC_CR2_EXTEN_Pos)                  /*!< 0x20000000 */
#define ADC_CR2_SWSTART_Pos       (30U)
#define ADC_CR2_SWSTART_Msk       (0x1UL << ADC_CR2_SWSTART_Pos)                /*!< 0x40000000 */
#define ADC_CR2_SWSTART           ADC_CR2_SWSTART_Msk                          /*!<Start Conversion of regular channels */

/******************  Bit definition for ADC_SMPR1 register  *******************/
#define ADC_SMPR1_SMP10_Pos       (0U)
#define ADC_SMPR1_SMP10_Msk       (0x7UL << ADC_SMPR1_SMP10_Pos)                /*!< 0x00000007 */
#define ADC_SMPR1_SMP10           ADC_SMPR1_SMP10_Msk                          /*!<SMP10[2:0] bits (Channel 10 Sample time selection) */
#define ADC_SMPR1_SMP10_0         (0x1UL << ADC_SMPR1_SMP10_Pos)                /*!< 0x00000001 */
#define ADC_SMPR1_SMP10_1         (0x2UL << ADC_SMPR1_SMP10_Pos)                /*!< 0x00000002 */
#define ADC_SMPR1_SMP10_2         (0x4UL << ADC_SMPR1_SMP10_Pos)                /*!< 0x00000004 */
#define ADC_SMPR1_SMP11_Pos       (3U)
#define ADC_SMPR1_SMP11_Msk       (0x7UL << ADC_SMPR1_SMP11_Pos)                /*!< 0x00000038 */
#define ADC_SMPR1_SMP11           ADC_SMPR1_SMP11_Msk                          /*!<SMP11[2:0] bits (Channel 11 Sample time selection) */
#define ADC_SMPR1_SMP11_0         (0x1UL << ADC_SMPR1_SMP11_Pos)                /*!< 0x00000008 */
#define ADC_SMPR1_SMP11_1         (0x2UL << ADC_SMPR1_SMP11_Pos)                /*!< 0x00000010 */
#define ADC_SMPR1_SMP11_2         (0x4UL << ADC_SMPR1_SMP11_Pos)                /*!< 0x00000020 */
#define ADC_SMPR1_SMP12_Pos       (6U)
#define ADC_SMPR1_SMP12_Msk       (0x7UL << ADC_SMPR1_SMP12_Pos)                /*!< 0x000001C0 */
#define ADC_SMPR1_SMP12           ADC_SMPR1_SMP12_Msk                          /*!<SMP12[2:0] bits (Channel 12 Sample time selection) */
#define ADC_SMPR1_SMP12_0         (0x1UL << ADC_SMPR1_SMP12_Pos)                /*!< 0x00000040 */
#define ADC_SMPR1_SMP12_1         (0x2UL << ADC_SMPR1_SMP12_Pos)                /*!< 0x00000080 */
#define ADC_SMPR1_SMP12_2         (0x4UL << ADC_SMPR1_SMP12_Pos)                /*!< 0x00000100 */
#define ADC_SMPR1_SMP13_Pos       (9U)
#define ADC_SMPR1_SMP13_Msk       (0x7UL << ADC_SMPR1_SMP13_Pos)                /*!< 0x00000E00 */
#define ADC_SMPR1_SMP13           ADC_SMPR1_SMP13_Msk                          /*!<SMP13[2:0] bits (Channel 13 Sample time selection) */
#define ADC_SMPR1_SMP13_0         (0x1UL << ADC_SMPR1_SMP13_Pos)                /*!< 0x00000200 */
#define ADC_SMPR1_SMP13_1         (0x2UL << ADC_SMPR1_SMP13_Pos)                /*!< 0x00000400 */
#define ADC_SMPR1_SMP13_2         (0x4UL << ADC_SMPR1_SMP13_Pos)                /*!< 0x00000800 */
#define ADC_SMPR1_SMP14_Pos       (12U)
#define ADC_SMPR1_SMP14_Msk       (0x7UL << ADC_SMPR1_SMP14_Pos)                /*!< 0x00007000 */
#define ADC_SMPR1_SMP14           ADC_SMPR1_SMP14_Msk                          /*!<SMP14[2:0] bits (Channel 14 Sample time selection) */
#define ADC_SMPR1_SMP14_0         (0x1UL << ADC_SMPR1_SMP14_Pos)                /*!< 0x00001000 */
#define ADC_SMPR1_SMP14_1         (0x2UL << ADC_SMPR1_SMP14_Pos)                /*!< 0x00002000 */
#define ADC_SMPR1_SMP14_2         (0x4UL << ADC_SMPR1_SMP14_Pos)                /*!< 0x00004000 */
#define ADC_SMPR1_SMP15_Pos       (15U)
#define ADC_SMPR1_SMP15_Msk       (0x7UL << ADC_SMPR1_SMP15_Pos)                /*!< 0x00038000 */
#define ADC_SMPR1_SMP15           ADC_SMPR1_SMP15_Msk                          /*!<SMP15[2:0] bits (Channel 15 Sample time selection) */
#define ADC_SMPR1_SMP15_0         (0x1UL << ADC_SMPR1_SMP15_Pos)                /*!< 0x00008000 */
#define ADC_SMPR1_SMP15_1         (0x2UL << ADC_SMPR1_SMP15_Pos)                /*!< 0x00010000 */
#define ADC_SMPR1_SMP15_2         (0x4UL << ADC_SMPR1_SMP15_Pos)                /*!< 0x00020000 */
#define ADC_SMPR1_SMP16_Pos       (18U)
#define ADC_SMPR1_SMP16_Msk       (0x7UL << ADC_SMPR1_SMP16_Pos)                /*!< 0x001C0000 */
#define ADC_SMPR1_SMP16           ADC_SMPR1_SMP16_Msk                          /*!<SMP16[2:0] bits (Channel 16 Sample time selection) */
#define ADC_SMPR1_SMP16_0         (0x1UL << ADC_SMPR1_SMP16_Pos)                /*!< 0x00040000 */
#define ADC_SMPR1_SMP16_1         (0x2UL << ADC_SMPR1_SMP16_Pos)                /*!< 0x00080000 */
#define ADC_SMPR1_SMP16_2         (0x4UL << ADC_SMPR1_SMP16_Pos)                /*!< 0x00100000 */
#define ADC_SMPR1_SMP17_Pos       (21U)
#define ADC_SMPR1_SMP17_Msk       (0x7UL << ADC_SMPR1_SMP17_Pos)                /*!< 0x00E00000 */
#define ADC_SMPR1_SMP17           ADC_SMPR1_SMP17_Msk                          /*!<SMP17[2:0] bits (Channel 17 Sample time selection) */
#define ADC_SMPR1_SMP17_0         (0x1UL << ADC_SMPR1_SMP17_Pos)                /*!< 0x00200000 */
#define ADC_SMPR1_SMP17_1         (0x2UL << ADC_SMPR1_SMP17_Pos)                /*!< 0x00400000 */
#define ADC_SMPR1_SMP17_2         (0x4UL << ADC_SMPR1_SMP17_Pos)                /*!< 0x00800000 */
#define ADC_SMPR1_SMP18_Pos       (24U)
#define ADC_SMPR1_SMP18_Msk       (0x7UL << ADC_SMPR1_SMP18_Pos)                /*!< 0x07000000 */
#define ADC_SMPR1_SMP18           ADC_SMPR1_SMP18_Msk                          /*!<SMP18[2:0] bits (Channel 18 Sample time selection) */
#define ADC_SMPR1_SMP18_0         (0x1UL << ADC_SMPR1_SMP18_Pos)                /*!< 0x01000000 */
#define ADC_SMPR1_SMP18_1         (0x2UL << ADC_SMPR1_SMP18_Pos)                /*!< 0x02000000 */
#define ADC_SMPR1_SMP18_2         (0x4UL << ADC_SMPR1_SMP18_Pos)                /*!< 0x04000000 */

/******************  Bit definition for ADC_SMPR2 register  *******************/
#define ADC_SMPR2_SMP0_Pos        (0U)
#define ADC_SMPR2_SMP0_Msk        (0x7UL << ADC_SMPR2_SMP0_Pos)                 /*!< 0x00000007 */
#define ADC_SMPR2_SMP0            ADC_SMPR2_SMP0_Msk                           /*!<SMP0[2:0] bits (Channel 0 Sample time selection) */
#define ADC_SMPR2_SMP0_0          (0x1UL << ADC_SMPR2_SMP0_Pos)                 /*!< 0x00000001 */
#define ADC_SMPR2_SMP0_1          (0x2UL << ADC_SMPR2_SMP0_Pos)                 /*!< 0x00000002 */
#define ADC_SMPR2_SMP0_2          (0x4UL << ADC_SMPR2_SMP0_Pos)                 /*!< 0x00000004 */
#define ADC_SMPR2_SMP1_Pos        (3U)
#define ADC_SMPR2_SMP1_Msk        (0x7UL << ADC_SMPR2_SMP1_Pos)                 /*!< 0x00000038 */
#define ADC_SMPR2_SMP1            ADC_SMPR2_SMP1_Msk                           /*!<SMP1[2:0] bits (Channel 1 Sample time selection) */
#define ADC_SMPR2_SMP1_0          (0x1UL << ADC_SMPR2_SMP1_Pos)                 /*!< 0x00000008 */
#define ADC_SMPR2_SMP1_1          (0x2UL << ADC_SMPR2_SMP1_Pos)                 /*!< 0x00000010 */
#define ADC_SMPR2_SMP1_2          (0x4UL << ADC_SMPR2_SMP1_Pos)                 /*!< 0x00000020 */
#define ADC_SMPR2_SMP2_Pos        (6U)
#define ADC_SMPR2_SMP2_Msk        (0x7UL << ADC_SMPR2_SMP2_Pos)                 /*!< 0x000001C0 */
#define ADC_SMPR2_SMP2            ADC_SMPR2_SMP2_Msk                           /*!<SMP2[2:0] bits (Channel 2 Sample time selection) */
#define ADC_SMPR2_SMP2_0          (0x1UL << ADC_SMPR2_SMP2_Pos)                 /*!< 0x00000040 */
#define ADC_SMPR2_SMP2_1          (0x2UL << ADC_SMPR2_SMP2_Pos)                 /*!< 0x00000080 */
#define ADC_SMPR2_SMP2_2          (0x4UL << ADC_SMPR2_SMP2_Pos)                 /*!< 0x00000100 */
#define ADC_SMPR2_SMP3_Pos        (9U)
#define ADC_SMPR2_SMP3_Msk        (0x7UL << ADC_SMPR2_SMP3_Pos)                 /*!< 0x00000E00 */
#define ADC_SMPR2_SMP3            ADC_SMPR2_SMP3_Msk                           /*!<SMP3[2:0] bits (Channel 3 Sample time selection) */
#define ADC_SMPR2_SMP3_0          (0x1UL << ADC_SMPR2_SMP3_Pos)                 /*!< 0x00000200 */
#define ADC_SMPR2_SMP3_1          (0x2UL << ADC_SMPR2_SMP3_Pos)                 /*!< 0x00000400 */
#define ADC_SMPR2_SMP3_2          (0x4UL << ADC_SMPR2_SMP3_Pos)                 /*!< 0x00000800 */
#define ADC_SMPR2_SMP4_Pos        (12U)
#define ADC_SMPR2_SMP4_Msk        (0x7UL << ADC_SMPR2_SMP4_Pos)                 /*!< 0x00007000 */
#define ADC_SMPR2_SMP4            ADC_SMPR2_SMP4_Msk                           /*!<SMP4[2:0] bits (Channel 4 Sample time selection) */
#define ADC_SMPR2_SMP4_0          (0x1UL << ADC_SMPR2_SMP4_Pos)                 /*!< 0x00001000 */
#define ADC_SMPR2_SMP4_1          (0x2UL << ADC_SMPR2_SMP4_Pos)                 /*!< 0x00002000 */
#define ADC_SMPR2_SMP4_2          (0x4UL << ADC_SMPR2_SMP4_Pos)                 /*!< 0x00004000 */
#define ADC_SMPR2_SMP5_Pos        (15U)
#define ADC_SMPR2_SMP5_Msk        (0x7UL << ADC_SMPR2_SMP5_Pos)                 /*!< 0x00038000 */
#define ADC_SMPR2_SMP5            ADC_SMPR2_SMP5_Msk                           /*!<SMP5[2:0] bits (Channel 5 Sample time selection) */
#define ADC_SMPR2_SMP5_0          (0x1UL << ADC_SMPR2_SMP5_Pos)                 /*!< 0x00008000 */
#define ADC_SMPR2_SMP5_1          (0x2UL << ADC_SMPR2_SMP5_Pos)                 /*!< 0x00010000 */
#define ADC_SMPR2_SMP5_2          (0x4UL << ADC_SMPR2_SMP5_Pos)                 /*!< 0x00020000 */
#define ADC_SMPR2_SMP6_Pos        (18U)
#define ADC_SMPR2_SMP6_Msk        (0x7UL << ADC_SMPR2_SMP6_Pos)                 /*!< 0x001C0000 */
#define ADC_SMPR2_SMP6            ADC_SMPR2_SMP6_Msk                           /*!<SMP6[2:0] bits (Channel 6 Sample time selection) */
#define ADC_SMPR2_SMP6_0          (0x1UL << ADC_SMPR2_SMP6_Pos)                 /*!< 0x00040000 */
#define ADC_SMPR2_SMP6_1          (0x2UL << ADC_SMPR2_SMP6_Pos)                 /*!< 0x00080000 */
#define ADC_SMPR2_SMP6_2          (0x4UL << ADC_SMPR2_SMP6_Pos)                 /*!< 0x00100000 */
#define ADC_SMPR2_SMP7_Pos        (21U)
#define ADC_SMPR2_SMP7_Msk        (0x7UL << ADC_SMPR2_SMP7_Pos)                 /*!< 0x00E00000 */
#define ADC_SMPR2_SMP7            ADC_SMPR2_SMP7_Msk                           /*!<SMP7[2:0] bits (Channel 7 Sample time selection) */
#define ADC_SMPR2_SMP7_0          (0x1UL << ADC_SMPR2_SMP7_Pos)                 /*!< 0x00200000 */
#define ADC_SMPR2_SMP7_1          (0x2UL << ADC_SMPR2_SMP7_Pos)                 /*!< 0x00400000 */
#define ADC_SMPR2_SMP7_2          (0x4UL << ADC_SMPR2_SMP7_Pos)                 /*!< 0x00800000 */
#define ADC_SMPR2_SMP8_Pos        (24U)
#define ADC_SMPR2_SMP8_Msk        (0x7UL << ADC_SMPR2_SMP8_Pos)                 /*!< 0x07000000 */
#define ADC_SMPR2_SMP8            ADC_SMPR2_SMP8_Msk                           /*!<SMP8[2:0] bits (Channel 8 Sample time selection) */
#define ADC_SMPR2_SMP8_0          (0x1UL << ADC_SMPR2_SMP8_Pos)                 /*!< 0x01000000 */
#define ADC_SMPR2_SMP8_1          (0x2UL << ADC_SMPR2_SMP8_Pos)                 /*!< 0x02000000 */
#define ADC_SMPR2_SMP8_2          (0x4UL << ADC_SMPR2_SMP8_Pos)                 /*!< 0x04000000 */
#define ADC_SMPR2_SMP9_Pos        (27U)
#define ADC_SMPR2_SMP9_Msk        (0x7UL << ADC_SMPR2_SMP9_Pos)                 /*!< 0x38000000 */
#define ADC_SMPR2_SMP9            ADC_SMPR2_SMP9_Msk                           /*!<SMP9[2:0] bits (Channel 9 Sample time selection) */
#define ADC_SMPR2_SMP9_0          (0x1UL << ADC_SMPR2_SMP9_Pos)                 /*!< 0x08000000 */
#define ADC_SMPR2_SMP9_1          (0x2UL << ADC_SMPR2_SMP9_Pos)                 /*!< 0x10000000 */
#define ADC_SMPR2_SMP9_2          (0x4UL << ADC_SMPR2_SMP9_Pos)                 /*!< 0x20000000 */

/******************  Bit definition for ADC_JOFR1 register  *******************/
#define ADC_JOFR1_JOFFSET1_Pos    (0U)
#define ADC_JOFR1_JOFFSET1_Msk    (0xFFFUL << ADC_JOFR1_JOFFSET1_Pos)           /*!< 0x00000FFF */
#define ADC_JOFR1_JOFFSET1        ADC_JOFR1_JOFFSET1_Msk                       /*!<Data offset for injected channel 1 */

/******************  Bit definition for ADC_JOFR2 register  *******************/
#define ADC_JOFR2_JOFFSET2_Pos    (0U)
#define ADC_JOFR2_JOFFSET2_Msk    (0xFFFUL << ADC_JOFR2_JOFFSET2_Pos)           /*!< 0x00000FFF */
#define ADC_JOFR2_JOFFSET2        ADC_JOFR2_JOFFSET2_Msk                       /*!<Data offset for injected channel 2 */

/******************  Bit definition for ADC_JOFR3 register  *******************/
#define ADC_JOFR3_JOFFSET3_Pos    (0U)
#define ADC_JOFR3_JOFFSET3_Msk    (0xFFFUL << ADC_JOFR3_JOFFSET3_Pos)           /*!< 0x00000FFF */
#define ADC_JOFR3_JOFFSET3        ADC_JOFR3_JOFFSET3_Msk                       /*!<Data offset for injected channel 3 */

/******************  Bit definition for ADC_JOFR4 register  *******************/
#define ADC_JOFR4_JOFFSET4_Pos    (0U)
#define ADC_JOFR4_JOFFSET4_Msk    (0xFFFUL << ADC_JOFR4_JOFFSET4_Pos)           /*!< 0x00000FFF */
#define ADC_JOFR4_JOFFSET4        ADC_JOFR4_JOFFSET4_Msk                       /*!<Data offset for injected channel 4 */

/*******************  Bit definition for ADC_HTR register  ********************/
#define ADC_HTR_HT_Pos            (0U)
#define ADC_HTR_HT_Msk            (0xFFFUL << ADC_HTR_HT_Pos)                   /*!< 0x00000FFF */
#define ADC_HTR_HT                ADC_HTR_HT_Msk                               /*!<Analog watchdog high threshold */

/*******************  Bit definition for ADC_LTR register  ********************/
#define ADC_LTR_LT_Pos            (0U)
#define ADC_LTR_LT_Msk            (0xFFFUL << ADC_LTR_LT_Pos)                   /*!< 0x00000FFF */
#define ADC_LTR_LT                ADC_LTR_LT_Msk                               /*!<Analog watchdog low threshold */

/*******************  Bit definition for ADC_SQR1 register  *******************/
#define ADC_SQR1_SQ13_Pos         (0U)
#define ADC_SQR1_SQ13_Msk         (0x1FUL << ADC_SQR1_SQ13_Pos)                 /*!< 0x0000001F */
#define ADC_SQR1_SQ13             ADC_SQR1_SQ13_Msk                            /*!<SQ13[4:0] bits (13th conversion in regular sequence) */
#define ADC_SQR1_SQ13_0           (0x01UL << ADC_SQR1_SQ13_Pos)                 /*!< 0x00000001 */
#define ADC_SQR1_SQ13_1           (0x02UL << ADC_SQR1_SQ13_Pos)                 /*!< 0x00000002 */
#define ADC_SQR1_SQ13_2           (0x04UL << ADC_SQR1_SQ13_Pos)                 /*!< 0x00000004 */
#define ADC_SQR1_SQ13_3           (0x08UL << ADC_SQR1_SQ13_Pos)                 /*!< 0x00000008 */
#define ADC_SQR1_SQ13_4           (0x10UL << ADC_SQR1_SQ13_Pos)                 /*!< 0x00000010 */
#define ADC_SQR1_SQ14_Pos         (5U)
#define ADC_SQR1_SQ14_Msk         (0x1FUL << ADC_SQR1_SQ14_Pos)                 /*!< 0x000003E0 */
#define ADC_SQR1_SQ14             ADC_SQR1_SQ14_Msk                            /*!<SQ14[4:0] bits (14th conversion in regular sequence) */
#define ADC_SQR1_SQ14_0           (0x01UL << ADC_SQR1_SQ14_Pos)                 /*!< 0x00000020 */
#define ADC_SQR1_SQ14_1           (0x02UL << ADC_SQR1_SQ14_Pos)                 /*!< 0x00000040 */
#define ADC_SQR1_SQ14_2           (0x04UL << ADC_SQR1_SQ14_Pos)                 /*!< 0x00000080 */
#define ADC_SQR1_SQ14_3           (0x08UL << ADC_SQR1_SQ14_Pos)                 /*!< 0x00000100 */
#define ADC_SQR1_SQ14_4           (0x10UL << ADC_SQR1_SQ14_Pos)                 /*!< 0x00000200 */
#define ADC_SQR1_SQ15_Pos         (10U)
#define ADC_SQR1_SQ15_Msk         (0x1FUL << ADC_SQR1_SQ15_Pos)                 /*!< 0x00007C00 */
#define ADC_SQR1_SQ15             ADC_SQR1_SQ15_Msk                            /*!<SQ15[4:0] bits (15th conversion in regular sequence) */
#define ADC_SQR1_SQ15_0           (0x01UL << ADC_SQR1_SQ15_Pos)                 /*!< 0x00000400 */
#define ADC_SQR1_SQ15_1           (0x02UL << ADC_SQR1_SQ15_Pos)                 /*!< 0x00000800 */
#define ADC_SQR1_SQ15_2           (0x04UL << ADC_SQR1_SQ15_Pos)                 /*!< 0x00001000 */
#define ADC_SQR1_SQ15_3           (0x08UL << ADC_SQR1_SQ15_Pos)                 /*!< 0x00002000 */
#define ADC_SQR1_SQ15_4           (0x10UL << ADC_SQR1_SQ15_Pos)                 /*!< 0x00004000 */
#define ADC_SQR1_SQ16_Pos         (15U)
#define ADC_SQR1_SQ16_Msk         (0x1FUL << ADC_SQR1_SQ16_Pos)                 /*!< 0x000F8000 */
#define ADC_SQR1_SQ16             ADC_SQR1_SQ16_Msk                            /*!<SQ16[4:0] bits (16th conversion in regular sequence) */
#define ADC_SQR1_SQ16_0           (0x01UL << ADC_SQR1_SQ16_Pos)                 /*!< 0x00008000 */
#define ADC_SQR1_SQ16_1           (0x02UL << ADC_SQR1_SQ16_Pos)                 /*!< 0x00010000 */
#define ADC_SQR1_SQ16_2           (0x04UL << ADC_SQR1_SQ16_Pos)                 /*!< 0x00020000 */
#define ADC_SQR1_SQ16_3           (0x08UL << ADC_SQR1_SQ16_Pos)                 /*!< 0x00040000 */
#define ADC_SQR1_SQ16_4           (0x10UL << ADC_SQR1_SQ16_Pos)                 /*!< 0x00080000 */
#define ADC_SQR1_L_Pos            (20U)
#define ADC_SQR1_L_Msk            (0xFUL << ADC_SQR1_L_Pos)                     /*!< 0x00F00000 */
#define ADC_SQR1_L                ADC_SQR1_L_Msk                               /*!<L[3:0] bits (Regular channel sequence length) */
#define ADC_SQR1_L_0              (0x1UL << ADC_SQR1_L_Pos)                     /*!< 0x00100000 */
#define ADC_SQR1_L_1              (0x2UL << ADC_SQR1_L_Pos)                     /*!< 0x00200000 */
#define ADC_SQR1_L_2              (0x4UL << ADC_SQR1_L_Pos)                     /*!< 0x00400000 */
#define ADC_SQR1_L_3              (0x8UL << ADC_SQR1_L_Pos)                     /*!< 0x00800000 */

/*******************  Bit definition for ADC_SQR2 register  *******************/
#define ADC_SQR2_SQ7_Pos          (0U)
#define ADC_SQR2_SQ7_Msk          (0x1FUL << ADC_SQR2_SQ7_Pos)                  /*!< 0x0000001F */
#define ADC_SQR2_SQ7              ADC_SQR2_SQ7_Msk                             /*!<SQ7[4:0] bits (7th conversion in regular sequence) */
#define ADC_SQR2_SQ7_0            (0x01UL << ADC_SQR2_SQ7_Pos)                  /*!< 0x00000001 */
#define ADC_SQR2_SQ7_1            (0x02UL << ADC_SQR2_SQ7_Pos)                  /*!< 0x00000002 */
#define ADC_SQR2_SQ7_2            (0x04UL << ADC_SQR2_SQ7_Pos)                  /*!< 0x00000004 */
#define ADC_SQR2_SQ7_3            (0x08UL << ADC_SQR2_SQ7_Pos)                  /*!< 0x00000008 */
#define ADC_SQR2_SQ7_4            (0x10UL << ADC_SQR2_SQ7_Pos)                  /*!< 0x00000010 */
#define ADC_SQR2_SQ8_Pos          (5U)
#define ADC_SQR2_SQ8_Msk          (0x1FUL << ADC_SQR2_SQ8_Pos)                  /*!< 0x000003E0 */
#define ADC_SQR2_SQ8              ADC_SQR2_SQ8_Msk                             /*!<SQ8[4:0] bits (8th conversion in regular sequence) */
#define ADC_SQR2_SQ8_0            (0x01UL << ADC_SQR2_SQ8_Pos)                  /*!< 0x00000020 */
#define ADC_SQR2_SQ8_1            (0x02UL << ADC_SQR2_SQ8_Pos)                  /*!< 0x00000040 */
#define ADC_SQR2_SQ8_2            (0x04UL << ADC_SQR2_SQ8_Pos)                  /*!< 0x00000080 */
#define ADC_SQR2_SQ8_3            (0x08UL << ADC_SQR2_SQ8_Pos)                  /*!< 0x00000100 */
#define ADC_SQR2_SQ8_4            (0x10UL << ADC_SQR2_SQ8_Pos)                  /*!< 0x00000200 */
#define ADC_SQR2_SQ9_Pos          (10U)
#define ADC_SQR2_SQ9_Msk          (0x1FUL << ADC_SQR2_SQ9_Pos)                  /*!< 0x00007C00 */
#define ADC_SQR2_SQ9              ADC_SQR2_SQ9_Msk                             /*!<SQ9[4:0] bits (9th conversion in regular sequence) */
#define ADC_SQR2_SQ9_0            (0x01UL << ADC_SQR2_SQ9_Pos)                  /*!< 0x00000400 */
#define ADC_SQR2_SQ9_1            (0x02UL << ADC_SQR2_SQ9_Pos)                  /*!< 0x00000800 */
#define ADC_SQR2_SQ9_2            (0x04UL << ADC_SQR2_SQ9_Pos)                  /*!< 0x00001000 */
#define ADC_SQR2_SQ9_3            (0x08UL << ADC_SQR2_SQ9_Pos)                  /*!< 0x00002000 */
#define ADC_SQR2_SQ9_4            (0x10UL << ADC_SQR2_SQ9_Pos)                  /*!< 0x00004000 */
#define ADC_SQR2_SQ10_Pos         (15U)
#define ADC_SQR2_SQ10_Msk         (0x1FUL << ADC_SQR2_SQ10_Pos)                 /*!< 0x000F8000 */
#define ADC_SQR2_SQ10             ADC_SQR2_SQ10_Msk                            /*!<SQ10[4:0] bits (10th conversion in regular sequence) */
#define ADC_SQR2_SQ10_0           (0x01UL << ADC_SQR2_SQ10_Pos)                 /*!< 0x00008000 */
#define ADC_SQR2_SQ10_1           (0x02UL << ADC_SQR2_SQ10_Pos)                 /*!< 0x00010000 */
#define ADC_SQR2_SQ10_2           (0x04UL << ADC_SQR2_SQ10_Pos)                 /*!< 0x00020000 */
#define ADC_SQR2_SQ10_3           (0x08UL << ADC_SQR2_SQ10_Pos)                 /*!< 0x00040000 */
#define ADC_SQR2_SQ10_4           (0x10UL << ADC_SQR2_SQ10_Pos)                 /*!< 0x00080000 */
#define ADC_SQR2_SQ11_Pos         (20U)
#define ADC_SQR2_SQ11_Msk         (0x1FUL << ADC_SQR2_SQ11_Pos)                 /*!< 0x01F00000 */
#define ADC_SQR2_SQ11             ADC_SQR2_SQ11_Msk                            /*!<SQ11[4:0] bits (11th conversion in regular sequence) */
#define ADC_SQR2_SQ11_0           (0x01UL << ADC_SQR2_SQ11_Pos)                 /*!< 0x00100000 */
#define ADC_SQR2_SQ11_1           (0x02UL << ADC_SQR2_SQ11_Pos)                 /*!< 0x00200000 */
#define ADC_SQR2_SQ11_2           (0x04UL << ADC_SQR2_SQ11_Pos)                 /*!< 0x00400000 */
#define ADC_SQR2_SQ11_3           (0x08UL << ADC_SQR2_SQ11_Pos)                 /*!< 0x00800000 */
#define ADC_SQR2_SQ11_4           (0x10UL << ADC_SQR2_SQ11_Pos)                 /*!< 0x01000000 */
#define ADC_SQR2_SQ12_Pos         (25U)
#define ADC_SQR2_SQ12_Msk         (0x1FUL << ADC_SQR2_SQ12_Pos)                 /*!< 0x3E000000 */
#define ADC_SQR2_SQ12             ADC_SQR2_SQ12_Msk                            /*!<SQ12[4:0] bits (12th conversion in regular sequence) */
#define ADC_SQR2_SQ12_0           (0x01UL << ADC_SQR2_SQ12_Pos)                 /*!< 0x02000000 */
#define ADC_SQR2_SQ12_1           (0x02UL << ADC_SQR2_SQ12_Pos)                 /*!< 0x04000000 */
#define ADC_SQR2_SQ12_2           (0x04UL << ADC_SQR2_SQ12_Pos)                 /*!< 0x08000000 */
#define ADC_SQR2_SQ12_3           (0x08UL << ADC_SQR2_SQ12_Pos)                 /*!< 0x10000000 */
#define ADC_SQR2_SQ12_4           (0x10UL << ADC_SQR2_SQ12_Pos)                 /*!< 0x20000000 */

/*******************  Bit definition for ADC_SQR3 register  *******************/
#define ADC_SQR3_SQ1_Pos          (0U)
#define ADC_SQR3_SQ1_Msk          (0x1FUL << ADC_SQR3_SQ1_Pos)                  /*!< 0x0000001F */
#define ADC_SQR3_SQ1              ADC_SQR3_SQ1_Msk                             /*!<SQ1[4:0] bits (1st conversion in regular sequence) */
#define ADC_SQR3_SQ1_0            (0x01UL << ADC_SQR3_SQ1_Pos)                  /*!< 0x00000001 */
#define ADC_SQR3_SQ1_1            (0x02UL << ADC_SQR3_SQ1_Pos)                  /*!< 0x00000002 */
#define ADC_SQR3_SQ1_2            (0x04UL << ADC_SQR3_SQ1_Pos)                  /*!< 0x00000004 */
#define ADC_SQR3_SQ1_3            (0x08UL << ADC_SQR3_SQ1_Pos)                  /*!< 0x00000008 */
#define ADC_SQR3_SQ1_4            (0x10UL << ADC_SQR3_SQ1_Pos)                  /*!< 0x00000010 */
#define ADC_SQR3_SQ2_Pos          (5U)
#define ADC_SQR3_SQ2_Msk          (0x1FUL << ADC_SQR3_SQ2_Pos)                  /*!< 0x000003E0 */
#define ADC_SQR3_SQ2              ADC_SQR3_SQ2_Msk                             /*!<SQ2[4:0] bits (2nd conversion in regular sequence) */
#define ADC_SQR3_SQ2_0            (0x01UL << ADC_SQR3_SQ2_Pos)                  /*!< 0x00000020 */
#define ADC_SQR3_SQ2_1            (0x02UL << ADC_SQR3_SQ2_Pos)                  /*!< 0x00000040 */
#define ADC_SQR3_SQ2_2            (0x04UL << ADC_SQR3_SQ2_Pos)                  /*!< 0x00000080 */
#define ADC_SQR3_SQ2_3            (0x08UL << ADC_SQR3_SQ2_Pos)                  /*!< 0x00000100 */
#define ADC_SQR3_SQ2_4            (0x10UL << ADC_SQR3_SQ2_Pos)                  /*!< 0x00000200 */
#define ADC_SQR3_SQ3_Pos          (10U)
#define ADC_SQR3_SQ3_Msk          (0x1FUL << ADC_SQR3_SQ3_Pos)                  /*!< 0x00007C00 */
#define ADC_SQR3_SQ3              ADC_SQR3_SQ3_Msk                             /*!<SQ3[4:0] bits (3rd conversion in regular sequence) */
#define ADC_SQR3_SQ3_0            (0x01UL << ADC_SQR3_SQ3_Pos)                  /*!< 0x00000400 */
#define ADC_SQR3_SQ3_1            (0x02UL << ADC_SQR3_SQ3_Pos)                  /*!< 0x00000800 */
#define ADC_SQR3_SQ3_2            (0x04UL << ADC_SQR3_SQ3_Pos)                  /*!< 0x00001000 */
#define ADC_SQR3_SQ3_3            (0x08UL << ADC_SQR3_SQ3_Pos)                  /*!< 0x00002000 */
#define ADC_SQR3_SQ3_4            (0x10UL << ADC_SQR3_SQ3_Pos)                  /*!< 0x00004000 */
#define ADC_SQR3_SQ4_Pos          (15U)
#define ADC_SQR3_SQ4_Msk          (0x1FUL << ADC_SQR3_SQ4_Pos)                  /*!< 0x000F8000 */
#define ADC_SQR3_SQ4              ADC_SQR3_SQ4_Msk                             /*!<SQ4[4:0] bits (4th conversion in regular sequence) */
#define ADC_SQR3_SQ4_0            (0x01UL << ADC_SQR3_SQ4_Pos)                  /*!< 0x00008000 */
#define ADC_SQR3_SQ4_1            (0x02UL << ADC_SQR3_SQ4_Pos)                  /*!< 0x00010000 */
#define ADC_SQR3_SQ4_2            (0x04UL << ADC_SQR3_SQ4_Pos)                  /*!< 0x00020000 */
#define ADC_SQR3_SQ4_3            (0x08UL << ADC_SQR3_SQ4_Pos)                  /*!< 0x00040000 */
#define ADC_SQR3_SQ4_4            (0x10UL << ADC_SQR3_SQ4_Pos)                  /*!< 0x00080000 */
#define ADC_SQR3_SQ5_Pos          (20U)
#define ADC_SQR3_SQ5_Msk          (0x1FUL << ADC_SQR3_SQ5_Pos)                  /*!< 0x01F00000 */
#define ADC_SQR3_SQ5              ADC_SQR3_SQ5_Msk                             /*!<SQ5[4:0] bits (5th conversion in regular sequence) */
#define ADC_SQR3_SQ5_0            (0x01UL << ADC_SQR3_SQ5_Pos)                  /*!< 0x00100000 */
#define ADC_SQR3_SQ5_1            (0x02UL << ADC_SQR3_SQ5_Pos)                  /*!< 0x00200000 */
#define ADC_SQR3_SQ5_2            (0x04UL << ADC_SQR3_SQ5_Pos)                  /*!< 0x00400000 */
#define ADC_SQR3_SQ5_3            (0x08UL << ADC_SQR3_SQ5_Pos)                  /*!< 0x00800000 */
#define ADC_SQR3_SQ5_4            (0x10UL << ADC_SQR3_SQ5_Pos)                  /*!< 0x01000000 */
#define ADC_SQR3_SQ6_Pos          (25U)
#define ADC_SQR3_SQ6_Msk          (0x1FUL << ADC_SQR3_SQ6_Pos)                  /*!< 0x3E000000 */
#define ADC_SQR3_SQ6              ADC_SQR3_SQ6_Msk                             /*!<SQ6[4:0] bits (6th conversion in regular sequence) */
#define ADC_SQR3_SQ6_0            (0x01UL << ADC_SQR3_SQ6_Pos)                  /*!< 0x02000000 */
#define ADC_SQR3_SQ6_1            (0x02UL << ADC_SQR3_SQ6_Pos)                  /*!< 0x04000000 */
#define ADC_SQR3_SQ6_2            (0x04UL << ADC_SQR3_SQ6_Pos)                  /*!< 0x08000000 */
#define ADC_SQR3_SQ6_3            (0x08UL << ADC_SQR3_SQ6_Pos)                  /*!< 0x10000000 */
#define ADC_SQR3_SQ6_4            (0x10UL << ADC_SQR3_SQ6_Pos)                  /*!< 0x20000000 */

/*******************  Bit definition for ADC_JSQR register  *******************/
#define ADC_JSQR_JSQ1_Pos         (0U)
#define ADC_JSQR_JSQ1_Msk         (0x1FUL << ADC_JSQR_JSQ1_Pos)                 /*!< 0x0000001F */
#define ADC_JSQR_JSQ1             ADC_JSQR_JSQ1_Msk                            /*!<JSQ1[4:0] bits (1st conversion in injected sequence) */
#define ADC_JSQR_JSQ1_0           (0x01UL << ADC_JSQR_JSQ1_Pos)                 /*!< 0x00000001 */
#define ADC_JSQR_JSQ1_1           (0x02UL << ADC_JSQR_JSQ1_Pos)                 /*!< 0x00000002 */
#define ADC_JSQR_JSQ1_2           (0x04UL << ADC_JSQR_JSQ1_Pos)                 /*!< 0x00000004 */
#define ADC_JSQR_JSQ1_3           (0x08UL << ADC_JSQR_JSQ1_Pos)                 /*!< 0x00000008 */
#define ADC_JSQR_JSQ1_4           (0x10UL << ADC_JSQR_JSQ1_Pos)                 /*!< 0x00000010 */
#define ADC_JSQR_JSQ2_Pos         (5U)
#define ADC_JSQR_JSQ2_Msk         (0x1FUL << ADC_JSQR_JSQ2_Pos)                 /*!< 0x000003E0 */
#define ADC_JSQR_JSQ2             ADC_JSQR_JSQ2_Msk                            /*!<JSQ2[4:0] bits (2nd conversion in injected sequence) */
#define ADC_JSQR_JSQ2_0           (0x01UL << ADC_JSQR_JSQ2_Pos)                 /*!< 0x00000020 */
#define ADC_JSQR_JSQ2_1           (0x02UL << ADC_JSQR_JSQ2_Pos)                 /*!< 0x00000040 */
#define ADC_JSQR_JSQ2_2           (0x04UL << ADC_JSQR_JSQ2_Pos)                 /*!< 0x00000080 */
#define ADC_JSQR_JSQ2_3           (0x08UL << ADC_JSQR_JSQ2_Pos)                 /*!< 0x00000100 */
#define ADC_JSQR_JSQ2_4           (0x10UL << ADC_JSQR_JSQ2_Pos)                 /*!< 0x00000200 */
#define ADC_JSQR_JSQ3_Pos         (10U)
#define ADC_JSQR_JSQ3_Msk         (0x1FUL << ADC_JSQR_JSQ3_Pos)                 /*!< 0x00007C00 */
#define ADC_JSQR_JSQ3             ADC_JSQR_JSQ3_Msk                            /*!<JSQ3[4:0] bits (3rd conversion in injected sequence) */
#define ADC_JSQR_JSQ3_0           (0x01UL << ADC_JSQR_JSQ3_Pos)                 /*!< 0x00000400 */
#define ADC_JSQR_JSQ3_1           (0x02UL << ADC_JSQR_JSQ3_Pos)                 /*!< 0x00000800 */
#define ADC_JSQR_JSQ3_2           (0x04UL << ADC_JSQR_JSQ3_Pos)                 /*!< 0x00001000 */
#define ADC_JSQR_JSQ3_3           (0x08UL << ADC_JSQR_JSQ3_Pos)                 /*!< 0x00002000 */
#define ADC_JSQR_JSQ3_4           (0x10UL << ADC_JSQR_JSQ3_Pos)                 /*!< 0x00004000 */
#define ADC_JSQR_JSQ4_Pos         (15U)
#define ADC_JSQR_JSQ4_Msk         (0x1FUL << ADC_JSQR_JSQ4_Pos)                 /*!< 0x000F8000 */
#define ADC_JSQR_JSQ4             ADC_JSQR_JSQ4_Msk                            /*!<JSQ4[4:0] bits (4th conversion in injected sequence) */
#define ADC_JSQR_JSQ4_0           (0x01UL << ADC_JSQR_JSQ4_Pos)                 /*!< 0x00008000 */
#define ADC_JSQR_JSQ4_1           (0x02UL << ADC_JSQR_JSQ4_Pos)                 /*!< 0x00010000 */
#define ADC_JSQR_JSQ4_2           (0x04UL << ADC_JSQR_JSQ4_Pos)                 /*!< 0x00020000 */
#define ADC_JSQR_JSQ4_3           (0x08UL << ADC_JSQR_JSQ4_Pos)                 /*!< 0x00040000 */
#define ADC_JSQR_JSQ4_4           (0x10UL << ADC_JSQR_JSQ4_Pos)                 /*!< 0x00080000 */
#define ADC_JSQR_JL_Pos           (20U)
#define ADC_JSQR_JL_Msk           (0x3UL << ADC_JSQR_JL_Pos)                    /*!< 0x00300000 */
#define ADC_JSQR_JL               ADC_JSQR_JL_Msk                              /*!<JL[1:0] bits (Injected Sequence length) */
#define ADC_JSQR_JL_0             (0x1UL << ADC_JSQR_JL_Pos)                    /*!< 0x00100000 */
#define ADC_JSQR_JL_1             (0x2UL << ADC_JSQR_JL_Pos)                    /*!< 0x00200000 */

/*******************  Bit definition for ADC_JDR1 register  *******************/
#define ADC_JDR1_JDATA_Pos        (0U)
#define ADC_JDR1_JDATA_Msk        (0xFFFFUL << ADC_JDR1_JDATA_Pos)              /*!< 0x0000FFFF */
#define ADC_JDR1_JDATA            ADC_JDR1_JDATA_Msk                           /*!<Injected data */

/*******************  Bit definition for ADC_JDR2 register  *******************/
#define ADC_JDR2_JDATA_Pos        (0U)
#define ADC_JDR2_JDATA_Msk        (0xFFFFUL << ADC_JDR2_JDATA_Pos)              /*!< 0x0000FFFF */
#define ADC_JDR2_JDATA            ADC_JDR2_JDATA_Msk                           /*!<Injected data */

/*******************  Bit definition for ADC_JDR3 register  *******************/
#define ADC_JDR3_JDATA_Pos        (0U)
#define ADC_JDR3_JDATA_Msk        (0xFFFFUL << ADC_JDR3_JDATA_Pos)              /*!< 0x0000FFFF */
#define ADC_JDR3_JDATA            ADC_JDR3_JDATA_Msk                           /*!<Injected data */

/*******************  Bit definition for ADC_JDR4 register  *******************/
#define ADC_JDR4_JDATA_Pos        (0U)
#define ADC_JDR4_JDATA_Msk        (0xFFFFUL << ADC_JDR4_JDATA_Pos)              /*!< 0x0000FFFF */
#define ADC_JDR4_JDATA            ADC_JDR4_JDATA_Msk                           /*!<Injected data */

/********************  Bit definition for ADC_DR register  ********************/
#define ADC_DR_DATA_Pos           (0U)
#define ADC_DR_DATA_Msk           (0xFFFFUL << ADC_DR_DATA_Pos)                 /*!< 0x0000FFFF */
#define ADC_DR_DATA               ADC_DR_DATA_Msk                              /*!<Regular data */
#define ADC_DR_ADC2DATA_Pos       (16U)
#define ADC_DR_ADC2DATA_Msk       (0xFFFFUL << ADC_DR_ADC2DATA_Pos)             /*!< 0xFFFF0000 */
#define ADC_DR_ADC2DATA           ADC_DR_ADC2DATA_Msk                          /*!<ADC2 data */

/*******************  Bit definition for ADC_CSR register  ********************/
#define ADC_CSR_AWD1_Pos          (0U)
#define ADC_CSR_AWD1_Msk          (0x1UL << ADC_CSR_AWD1_Pos)                   /*!< 0x00000001 */
#define ADC_CSR_AWD1              ADC_CSR_AWD1_Msk                             /*!<ADC1 Analog watchdog flag */
#define ADC_CSR_EOC1_Pos          (1U)
#define ADC_CSR_EOC1_Msk          (0x1UL << ADC_CSR_EOC1_Pos)                   /*!< 0x00000002 */
#define ADC_CSR_EOC1              ADC_CSR_EOC1_Msk                             /*!<ADC1 End of conversion */
#define ADC_CSR_JEOC1_Pos         (2U)
#define ADC_CSR_JEOC1_Msk         (0x1UL << ADC_CSR_JEOC1_Pos)                  /*!< 0x00000004 */
#define ADC_CSR_JEOC1             ADC_CSR_JEOC1_Msk                            /*!<ADC1 Injected channel end of conversion */
#define ADC_CSR_JSTRT1_Pos        (3U)
#define ADC_CSR_JSTRT1_Msk        (0x1UL << ADC_CSR_JSTRT1_Pos)                 /*!< 0x00000008 */
#define ADC_CSR_JSTRT1            ADC_CSR_JSTRT1_Msk                           /*!<ADC1 Injected channel Start flag */
#define ADC_CSR_STRT1_Pos         (4U)
#define ADC_CSR_STRT1_Msk         (0x1UL << ADC_CSR_STRT1_Pos)                  /*!< 0x00000010 */
#define ADC_CSR_STRT1             ADC_CSR_STRT1_Msk                            /*!<ADC1 Regular channel Start flag */
#define ADC_CSR_OVR1_Pos          (5U)
#define ADC_CSR_OVR1_Msk          (0x1UL << ADC_CSR_OVR1_Pos)                   /*!< 0x00000020 */
#define ADC_CSR_OVR1              ADC_CSR_OVR1_Msk                             /*!<ADC1 DMA overrun  flag */
#define ADC_CSR_AWD2_Pos          (8U)
#define ADC_CSR_AWD2_Msk          (0x1UL << ADC_CSR_AWD2_Pos)                   /*!< 0x00000100 */
#define ADC_CSR_AWD2              ADC_CSR_AWD2_Msk                             /*!<ADC2 Analog watchdog flag */
#define ADC_CSR_EOC2_Pos          (9U)
#define ADC_CSR_EOC2_Msk          (0x1UL << ADC_CSR_EOC2_Pos)                   /*!< 0x00000200 */
#define ADC_CSR_EOC2              ADC_CSR_EOC2_Msk                             /*!<ADC2 End of conversion */
#define ADC_CSR_JEOC2_Pos         (10U)
#define ADC_CSR_JEOC2_Msk         (0x1UL << ADC_CSR_JEOC2_Pos)                  /*!< 0x00000400 */
#define ADC_CSR_JEOC2             ADC_CSR_JEOC2_Msk                            /*!<ADC2 Injected channel end of conversion */
#define ADC_CSR_JSTRT2_Pos        (11U)
#define ADC_CSR_JSTRT2_Msk        (0x1UL << ADC_CSR_JSTRT2_Pos)                 /*!< 0x00000800 */
#define ADC_CSR_JSTRT2            ADC_CSR_JSTRT2_Msk                           /*!<ADC2 Injected channel Start flag */
#define ADC_CSR_STRT2_Pos         (12U)
#define ADC_CSR_STRT2_Msk         (0x1UL << ADC_CSR_STRT2_Pos)                  /*!< 0x00001000 */
#define ADC_CSR_STRT2             ADC_CSR_STRT2_Msk                            /*!<ADC2 Regular channel Start flag */
#define ADC_CSR_OVR2_Pos          (13U)
#define ADC_CSR_OVR2_Msk          (0x1UL << ADC_CSR_OVR2_Pos)                   /*!< 0x00002000 */
#define ADC_CSR_OVR2              ADC_CSR_OVR2_Msk                             /*!<ADC2 DMA overrun  flag */
#define ADC_CSR_AWD3_Pos          (16U)
#define ADC_CSR_AWD3_Msk          (0x1UL << ADC_CSR_AWD3_Pos)                   /*!< 0x00010000 */
#define ADC_CSR_AWD3              ADC_CSR_AWD3_Msk                             /*!<ADC3 Analog watchdog flag */
#define ADC_CSR_EOC3_Pos          (17U)
#define ADC_CSR_EOC3_Msk          (0x1UL << ADC_CSR_EOC3_Pos)                   /*!< 0x00020000 */
#define ADC_CSR_EOC3              ADC_CSR_EOC3_Msk                             /*!<ADC3 End of conversion */
#define ADC_CSR_JEOC3_Pos         (18U)
#define ADC_CSR_JEOC3_Msk         (0x1UL << ADC_CSR_JEOC3_Pos)                  /*!< 0x00040000 */
#define ADC_CSR_JEOC3             ADC_CSR_JEOC3_Msk                            /*!<ADC3 Injected channel end of conversion */
#define ADC_CSR_JSTRT3_Pos        (19U)
#define ADC_CSR_JSTRT3_Msk        (0x1UL << ADC_CSR_JSTRT3_Pos)                 /*!< 0x00080000 */
#define ADC_CSR_JSTRT3            ADC_CSR_JSTRT3_Msk                           /*!<ADC3 Injected channel Start flag */
#define ADC_CSR_STRT3_Pos         (20U)
#define ADC_CSR_STRT3_Msk         (0x1UL << ADC_CSR_STRT3_Pos)                  /*!< 0x00100000 */
#define ADC_CSR_STRT3             ADC_CSR_STRT3_Msk                            /*!<ADC3 Regular channel Start flag */
#define ADC_CSR_OVR3_Pos          (21U)
#define ADC_CSR_OVR3_Msk          (0x1UL << ADC_CSR_OVR3_Pos)                   /*!< 0x00200000 */
#define ADC_CSR_OVR3              ADC_CSR_OVR3_Msk                             /*!<ADC3 DMA overrun  flag */

/* Legacy defines */
#define  ADC_CSR_DOVR1                        ADC_CSR_OVR1
#define  ADC_CSR_DOVR2                        ADC_CSR_OVR2
#define  ADC_CSR_DOVR3                        ADC_CSR_OVR3

/*******************  Bit definition for ADC_CCR register  ********************/
#define ADC_CCR_MULTI_Pos         (0U)
#define ADC_CCR_MULTI_Msk         (0x1FUL << ADC_CCR_MULTI_Pos)                 /*!< 0x0000001F */
#define ADC_CCR_MULTI             ADC_CCR_MULTI_Msk                            /*!<MULTI[4:0] bits (Multi-ADC mode selection) */
#define ADC_CCR_MULTI_0           (0x01UL << ADC_CCR_MULTI_Pos)                 /*!< 0x00000001 */
#define ADC_CCR_MULTI_1           (0x02UL << ADC_CCR_MULTI_Pos)                 /*!< 0x00000002 */
#define ADC_CCR_MULTI_2           (0x04UL << ADC_CCR_MULTI_Pos)                 /*!< 0x00000004 */
#define ADC_CCR_MULTI_3           (0x08UL << ADC_CCR_MULTI_Pos)                 /*!< 0x00000008 */
#define ADC_CCR_MULTI_4           (0x10UL << ADC_CCR_MULTI_Pos)                 /*!< 0x00000010 */
#define ADC_CCR_DELAY_Pos         (8U)
#define ADC_CCR_DELAY_Msk         (0xFUL << ADC_CCR_DELAY_Pos)                  /*!< 0x00000F00 */
#define ADC_CCR_DELAY             ADC_CCR_DELAY_Msk                            /*!<DELAY[3:0] bits (Delay between 2 sampling phases) */
#define ADC_CCR_DELAY_0           (0x1UL << ADC_CCR_DELAY_Pos)                  /*!< 0x00000100 */
#define ADC_CCR_DELAY_1           (0x2UL << ADC_CCR_DELAY_Pos)                  /*!< 0x00000200 */
#define ADC_CCR_DELAY_2           (0x4UL << ADC_CCR_DELAY_Pos)                  /*!< 0x00000400 */
#define ADC_CCR_DELAY_3           (0x8UL << ADC_CCR_DELAY_Pos)                  /*!< 0x00000800 */
#define ADC_CCR_DDS_Pos           (13U)
#define ADC_CCR_DDS_Msk           (0x1UL << ADC_CCR_DDS_Pos)                    /*!< 0x00002000 */
#define ADC_CCR_DDS               ADC_CCR_DDS_Msk                              /*!<DMA disable selection (Multi-ADC mode) */
#define ADC_CCR_DMA_Pos           (14U)
#define ADC_CCR_DMA_Msk           (0x3UL << ADC_CCR_DMA_Pos)                    /*!< 0x0000C000 */
#define ADC_CCR_DMA               ADC_CCR_DMA_Msk                              /*!<DMA[1:0] bits (Direct Memory Access mode for multimode) */
#define ADC_CCR_DMA_0             (0x1UL << ADC_CCR_DMA_Pos)                    /*!< 0x00004000 */
#define ADC_CCR_DMA_1             (0x2UL << ADC_CCR_DMA_Pos)                    /*!< 0x00008000 */
#define ADC_CCR_ADCPRE_Pos        (16U)
#define ADC_CCR_ADCPRE_Msk        (0x3UL << ADC_CCR_ADCPRE_Pos)                 /*!< 0x00030000 */
#define ADC_CCR_ADCPRE            ADC_CCR_ADCPRE_Msk                           /*!<ADCPRE[1:0] bits (ADC prescaler) */
#define ADC_CCR_ADCPRE_0          (0x1UL << ADC_CCR_ADCPRE_Pos)                 /*!< 0x00010000 */
#define ADC_CCR_ADCPRE_1          (0x2UL << ADC_CCR_ADCPRE_Pos)                 /*!< 0x00020000 */
#define ADC_CCR_VBATE_Pos         (22U)
#define ADC_CCR_VBATE_Msk         (0x1UL << ADC_CCR_VBATE_Pos)                  /*!< 0x00400000 */
#define ADC_CCR_VBATE             ADC_CCR_VBATE_Msk                            /*!<VBAT Enable */
#define ADC_CCR_TSVREFE_Pos       (23U)
#define ADC_CCR_TSVREFE_Msk       (0x1UL << ADC_CCR_TSVREFE_Pos)                /*!< 0x00800000 */
#define ADC_CCR_TSVREFE           ADC_CCR_TSVREFE_Msk                          /*!<Temperature Sensor and VREFINT Enable */

/*******************  Bit definition for ADC_CDR register  ********************/
#define ADC_CDR_DATA1_Pos         (0U)
#define ADC_CDR_DATA1_Msk         (0xFFFFUL << ADC_CDR_DATA1_Pos)               /*!< 0x0000FFFF */
#define ADC_CDR_DATA1             ADC_CDR_DATA1_Msk                            /*!<1st data of a pair of regular conversions */
#define ADC_CDR_DATA2_Pos         (16U)
#define ADC_CDR_DATA2_Msk         (0xFFFFUL << ADC_CDR_DATA2_Pos)               /*!< 0xFFFF0000 */
#define ADC_CDR_DATA2             ADC_CDR_DATA2_Msk                            /*!<2nd data of a pair of regular conversions */

/* Legacy defines */
#define ADC_CDR_RDATA_MST         ADC_CDR_DATA1
#define ADC_CDR_RDATA_SLV         ADC_CDR_DATA2



/******************************************************************************/
/*                                                                            */
/*                      Digital to Analog Converter                           */
/*                                                                            */
/******************************************************************************/
/*
 * @brief Specific device feature definitions (not present on all devices in the STM32F4 serie)
 */
#define DAC_CHANNEL2_SUPPORT                                    /*!< DAC feature available only on specific devices: availability of DAC channel 2 */
/********************  Bit definition for DAC_CR register  ********************/
#define DAC_CR_EN1_Pos              (0U)
#define DAC_CR_EN1_Msk              (0x1UL << DAC_CR_EN1_Pos)                   /*!< 0x00000001 */
#define DAC_CR_EN1                  DAC_CR_EN1_Msk                             /*!<DAC channel1 enable */
#define DAC_CR_BOFF1_Pos            (1U)
#define DAC_CR_BOFF1_Msk            (0x1UL << DAC_CR_BOFF1_Pos)                 /*!< 0x00000002 */
#define DAC_CR_BOFF1                DAC_CR_BOFF1_Msk                           /*!<DAC channel1 output buffer disable */
#define DAC_CR_TEN1_Pos             (2U)
#define DAC_CR_TEN1_Msk             (0x1UL << DAC_CR_TEN1_Pos)                  /*!< 0x00000004 */
#define DAC_CR_TEN1                 DAC_CR_TEN1_Msk                            /*!<DAC channel1 Trigger enable */

#define DAC_CR_TSEL1_Pos            (3U)
#define DAC_CR_TSEL1_Msk            (0x7UL << DAC_CR_TSEL1_Pos)                 /*!< 0x00000038 */
#define DAC_CR_TSEL1                DAC_CR_TSEL1_Msk                           /*!<TSEL1[2:0] (DAC channel1 Trigger selection) */
#define DAC_CR_TSEL1_0              (0x1UL << DAC_CR_TSEL1_Pos)                 /*!< 0x00000008 */
#define DAC_CR_TSEL1_1              (0x2UL << DAC_CR_TSEL1_Pos)                 /*!< 0x00000010 */
#define DAC_CR_TSEL1_2              (0x4UL << DAC_CR_TSEL1_Pos)                 /*!< 0x00000020 */

#define DAC_CR_WAVE1_Pos            (6U)
#define DAC_CR_WAVE1_Msk            (0x3UL << DAC_CR_WAVE1_Pos)                 /*!< 0x000000C0 */
#define DAC_CR_WAVE1                DAC_CR_WAVE1_Msk                           /*!<WAVE1[1:0] (DAC channel1 noise/triangle wave generation enable) */
#define DAC_CR_WAVE1_0              (0x1UL << DAC_CR_WAVE1_Pos)                 /*!< 0x00000040 */
#define DAC_CR_WAVE1_1              (0x2UL << DAC_CR_WAVE1_Pos)                 /*!< 0x00000080 */

#define DAC_CR_MAMP1_Pos            (8U)
#define DAC_CR_MAMP1_Msk            (0xFUL << DAC_CR_MAMP1_Pos)                 /*!< 0x00000F00 */
#define DAC_CR_MAMP1                DAC_CR_MAMP1_Msk                           /*!<MAMP1[3:0] (DAC channel1 Mask/Amplitude selector) */
#define DAC_CR_MAMP1_0              (0x1UL << DAC_CR_MAMP1_Pos)                 /*!< 0x00000100 */
#define DAC_CR_MAMP1_1              (0x2UL << DAC_CR_MAMP1_Pos)                 /*!< 0x00000200 */
#define DAC_CR_MAMP1_2              (0x4UL << DAC_CR_MAMP1_Pos)                 /*!< 0x00000400 */
#define DAC_CR_MAMP1_3              (0x8UL << DAC_CR_MAMP1_Pos)                 /*!< 0x00000800 */

#define DAC_CR_DMAEN1_Pos           (12U)
#define DAC_CR_DMAEN1_Msk           (0x1UL << DAC_CR_DMAEN1_Pos)                /*!< 0x00001000 */
#define DAC_CR_DMAEN1               DAC_CR_DMAEN1_Msk                          /*!<DAC channel1 DMA enable */
#define DAC_CR_DMAUDRIE1_Pos        (13U)
#define DAC_CR_DMAUDRIE1_Msk        (0x1UL << DAC_CR_DMAUDRIE1_Pos)             /*!< 0x00002000 */
#define DAC_CR_DMAUDRIE1            DAC_CR_DMAUDRIE1_Msk                       /*!<DAC channel1 DMA underrun interrupt enable*/
#define DAC_CR_EN2_Pos              (16U)
#define DAC_CR_EN2_Msk              (0x1UL << DAC_CR_EN2_Pos)                   /*!< 0x00010000 */
#define DAC_CR_EN2                  DAC_CR_EN2_Msk                             /*!<DAC channel2 enable */
#define DAC_CR_BOFF2_Pos            (17U)
#define DAC_CR_BOFF2_Msk            (0x1UL << DAC_CR_BOFF2_Pos)                 /*!< 0x00020000 */
#define DAC_CR_BOFF2                DAC_CR_BOFF2_Msk                           /*!<DAC channel2 output buffer disable */
#define DAC_CR_TEN2_Pos             (18U)
#define DAC_CR_TEN2_Msk             (0x1UL << DAC_CR_TEN2_Pos)                  /*!< 0x00040000 */
#define DAC_CR_TEN2                 DAC_CR_TEN2_Msk                            /*!<DAC channel2 Trigger enable */

#define DAC_CR_TSEL2_Pos            (19U)
#define DAC_CR_TSEL2_Msk            (0x7UL << DAC_CR_TSEL2_Pos)                 /*!< 0x00380000 */
#define DAC_CR_TSEL2                DAC_CR_TSEL2_Msk                           /*!<TSEL2[2:0] (DAC channel2 Trigger selection) */
#define DAC_CR_TSEL2_0              (0x1UL << DAC_CR_TSEL2_Pos)                 /*!< 0x00080000 */
#define DAC_CR_TSEL2_1              (0x2UL << DAC_CR_TSEL2_Pos)                 /*!< 0x00100000 */
#define DAC_CR_TSEL2_2              (0x4UL << DAC_CR_TSEL2_Pos)                 /*!< 0x00200000 */

#define DAC_CR_WAVE2_Pos            (22U)
#define DAC_CR_WAVE2_Msk            (0x3UL << DAC_CR_WAVE2_Pos)                 /*!< 0x00C00000 */
#define DAC_CR_WAVE2                DAC_CR_WAVE2_Msk                           /*!<WAVE2[1:0] (DAC channel2 noise/triangle wave generation enable) */
#define DAC_CR_WAVE2_0              (0x1UL << DAC_CR_WAVE2_Pos)                 /*!< 0x00400000 */
#define DAC_CR_WAVE2_1              (0x2UL << DAC_CR_WAVE2_Pos)                 /*!< 0x00800000 */

#define DAC_CR_MAMP2_Pos            (24U)
#define DAC_CR_MAMP2_Msk            (0xFUL << DAC_CR_MAMP2_Pos)                 /*!< 0x0F000000 */
#define DAC_CR_MAMP2                DAC_CR_MAMP2_Msk                           /*!<MAMP2[3:0] (DAC channel2 Mask/Amplitude selector) */
#define DAC_CR_MAMP2_0              (0x1UL << DAC_CR_MAMP2_Pos)                 /*!< 0x01000000 */
#define DAC_CR_MAMP2_1              (0x2UL << DAC_CR_MAMP2_Pos)                 /*!< 0x02000000 */
#define DAC_CR_MAMP2_2              (0x4UL << DAC_CR_MAMP2_Pos)                 /*!< 0x04000000 */
#define DAC_CR_MAMP2_3              (0x8UL << DAC_CR_MAMP2_Pos)                 /*!< 0x08000000 */

#define DAC_CR_DMAEN2_Pos           (28U)
#define DAC_CR_DMAEN2_Msk           (0x1UL << DAC_CR_DMAEN2_Pos)                /*!< 0x10000000 */
#define DAC_CR_DMAEN2               DAC_CR_DMAEN2_Msk                          /*!<DAC channel2 DMA enabled */
#define DAC_CR_DMAUDRIE2_Pos        (29U)
#define DAC_CR_DMAUDRIE2_Msk        (0x1UL << DAC_CR_DMAUDRIE2_Pos)             /*!< 0x20000000 */
#define DAC_CR_DMAUDRIE2            DAC_CR_DMAUDRIE2_Msk                       /*!<DAC channel2 DMA underrun interrupt enable*/

/*****************  Bit definition for DAC_SWTRIGR register  ******************/
#define DAC_SWTRIGR_SWTRIG1_Pos     (0U)
#define DAC_SWTRIGR_SWTRIG1_Msk     (0x1UL << DAC_SWTRIGR_SWTRIG1_Pos)          /*!< 0x00000001 */
#define DAC_SWTRIGR_SWTRIG1         DAC_SWTRIGR_SWTRIG1_Msk                    /*!<DAC channel1 software trigger */
#define DAC_SWTRIGR_SWTRIG2_Pos     (1U)
#define DAC_SWTRIGR_SWTRIG2_Msk     (0x1UL << DAC_SWTRIGR_SWTRIG2_Pos)          /*!< 0x00000002 */
#define DAC_SWTRIGR_SWTRIG2         DAC_SWTRIGR_SWTRIG2_Msk                    /*!<DAC channel2 software trigger */

/*****************  Bit definition for DAC_DHR12R1 register  ******************/
#define DAC_DHR12R1_DACC1DHR_Pos    (0U)
#define DAC_DHR12R1_DACC1DHR_Msk    (0xFFFUL << DAC_DHR12R1_DACC1DHR_Pos)       /*!< 0x00000FFF */
#define DAC_DHR12R1_DACC1DHR        DAC_DHR12R1_DACC1DHR_Msk                   /*!<DAC channel1 12-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12L1 register  ******************/
#define DAC_DHR12L1_DACC1DHR_Pos    (4U)
#define DAC_DHR12L1_DACC1DHR_Msk    (0xFFFUL << DAC_DHR12L1_DACC1DHR_Pos)       /*!< 0x0000FFF0 */
#define DAC_DHR12L1_DACC1DHR        DAC_DHR12L1_DACC1DHR_Msk                   /*!<DAC channel1 12-bit Left aligned data */

/******************  Bit definition for DAC_DHR8R1 register  ******************/
#define DAC_DHR8R1_DACC1DHR_Pos     (0U)
#define DAC_DHR8R1_DACC1DHR_Msk     (0xFFUL << DAC_DHR8R1_DACC1DHR_Pos)         /*!< 0x000000FF */
#define DAC_DHR8R1_DACC1DHR         DAC_DHR8R1_DACC1DHR_Msk                    /*!<DAC channel1 8-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12R2 register  ******************/
#define DAC_DHR12R2_DACC2DHR_Pos    (0U)
#define DAC_DHR12R2_DACC2DHR_Msk    (0xFFFUL << DAC_DHR12R2_DACC2DHR_Pos)       /*!< 0x00000FFF */
#define DAC_DHR12R2_DACC2DHR        DAC_DHR12R2_DACC2DHR_Msk                   /*!<DAC channel2 12-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12L2 register  ******************/
#define DAC_DHR12L2_DACC2DHR_Pos    (4U)
#define DAC_DHR12L2_DACC2DHR_Msk    (0xFFFUL << DAC_DHR12L2_DACC2DHR_Pos)       /*!< 0x0000FFF0 */
#define DAC_DHR12L2_DACC2DHR        DAC_DHR12L2_DACC2DHR_Msk                   /*!<DAC channel2 12-bit Left aligned data */

/******************  Bit definition for DAC_DHR8R2 register  ******************/
#define DAC_DHR8R2_DACC2DHR_Pos     (0U)
#define DAC_DHR8R2_DACC2DHR_Msk     (0xFFUL << DAC_DHR8R2_DACC2DHR_Pos)         /*!< 0x000000FF */
#define DAC_DHR8R2_DACC2DHR         DAC_DHR8R2_DACC2DHR_Msk                    /*!<DAC channel2 8-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12RD register  ******************/
#define DAC_DHR12RD_DACC1DHR_Pos    (0U)
#define DAC_DHR12RD_DACC1DHR_Msk    (0xFFFUL << DAC_DHR12RD_DACC1DHR_Pos)       /*!< 0x00000FFF */
#define DAC_DHR12RD_DACC1DHR        DAC_DHR12RD_DACC1DHR_Msk                   /*!<DAC channel1 12-bit Right aligned data */
#define DAC_DHR12RD_DACC2DHR_Pos    (16U)
#define DAC_DHR12RD_DACC2DHR_Msk    (0xFFFUL << DAC_DHR12RD_DACC2DHR_Pos)       /*!< 0x0FFF0000 */
#define DAC_DHR12RD_DACC2DHR        DAC_DHR12RD_DACC2DHR_Msk                   /*!<DAC channel2 12-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12LD register  ******************/
#define DAC_DHR12LD_DACC1DHR_Pos    (4U)
#define DAC_DHR12LD_DACC1DHR_Msk    (0xFFFUL << DAC_DHR12LD_DACC1DHR_Pos)       /*!< 0x0000FFF0 */
#define DAC_DHR12LD_DACC1DHR        DAC_DHR12LD_DACC1DHR_Msk                   /*!<DAC channel1 12-bit Left aligned data */
#define DAC_DHR12LD_DACC2DHR_Pos    (20U)
#define DAC_DHR12LD_DACC2DHR_Msk    (0xFFFUL << DAC_DHR12LD_DACC2DHR_Pos)       /*!< 0xFFF00000 */
#define DAC_DHR12LD_DACC2DHR        DAC_DHR12LD_DACC2DHR_Msk                   /*!<DAC channel2 12-bit Left aligned data */

/******************  Bit definition for DAC_DHR8RD register  ******************/
#define DAC_DHR8RD_DACC1DHR_Pos     (0U)
#define DAC_DHR8RD_DACC1DHR_Msk     (0xFFUL << DAC_DHR8RD_DACC1DHR_Pos)         /*!< 0x000000FF */
#define DAC_DHR8RD_DACC1DHR         DAC_DHR8RD_DACC1DHR_Msk                    /*!<DAC channel1 8-bit Right aligned data */
#define DAC_DHR8RD_DACC2DHR_Pos     (8U)
#define DAC_DHR8RD_DACC2DHR_Msk     (0xFFUL << DAC_DHR8RD_DACC2DHR_Pos)         /*!< 0x0000FF00 */
#define DAC_DHR8RD_DACC2DHR         DAC_DHR8RD_DACC2DHR_Msk                    /*!<DAC channel2 8-bit Right aligned data */

/*******************  Bit definition for DAC_DOR1 register  *******************/
#define DAC_DOR1_DACC1DOR_Pos       (0U)
#define DAC_DOR1_DACC1DOR_Msk       (0xFFFUL << DAC_DOR1_DACC1DOR_Pos)          /*!< 0x00000FFF */
#define DAC_DOR1_DACC1DOR           DAC_DOR1_DACC1DOR_Msk                      /*!<DAC channel1 data output */

/*******************  Bit definition for DAC_DOR2 register  *******************/
#define DAC_DOR2_DACC2DOR_Pos       (0U)
#define DAC_DOR2_DACC2DOR_Msk       (0xFFFUL << DAC_DOR2_DACC2DOR_Pos)          /*!< 0x00000FFF */
#define DAC_DOR2_DACC2DOR           DAC_DOR2_DACC2DOR_Msk                      /*!<DAC channel2 data output */

/********************  Bit definition for DAC_SR register  ********************/
#define DAC_SR_DMAUDR1_Pos          (13U)
#define DAC_SR_DMAUDR1_Msk          (0x1UL << DAC_SR_DMAUDR1_Pos)               /*!< 0x00002000 */
#define DAC_SR_DMAUDR1              DAC_SR_DMAUDR1_Msk                         /*!<DAC channel1 DMA underrun flag */
#define DAC_SR_DMAUDR2_Pos          (29U)
#define DAC_SR_DMAUDR2_Msk          (0x1UL << DAC_SR_DMAUDR2_Pos)               /*!< 0x20000000 */
#define DAC_SR_DMAUDR2              DAC_SR_DMAUDR2_Msk                         /*!<DAC channel2 DMA underrun flag */



/******************************************************************************/
/*                                                                            */
/*                             DMA Controller                                 */
/*                                                                            */
/******************************************************************************/
/********************  Bits definition for DMA_SxCR register  *****************/
#define DMA_SxCR_CHSEL_Pos       (25U)
#define DMA_SxCR_CHSEL_Msk       (0x7UL << DMA_SxCR_CHSEL_Pos)                  /*!< 0x0E000000 */
#define DMA_SxCR_CHSEL           DMA_SxCR_CHSEL_Msk
#define DMA_SxCR_CHSEL_0         0x02000000U
#define DMA_SxCR_CHSEL_1         0x04000000U
#define DMA_SxCR_CHSEL_2         0x08000000U
#define DMA_SxCR_MBURST_Pos      (23U)
#define DMA_SxCR_MBURST_Msk      (0x3UL << DMA_SxCR_MBURST_Pos)                 /*!< 0x01800000 */
#define DMA_SxCR_MBURST          DMA_SxCR_MBURST_Msk
#define DMA_SxCR_MBURST_0        (0x1UL << DMA_SxCR_MBURST_Pos)                 /*!< 0x00800000 */
#define DMA_SxCR_MBURST_1        (0x2UL << DMA_SxCR_MBURST_Pos)                 /*!< 0x01000000 */
#define DMA_SxCR_PBURST_Pos      (21U)
#define DMA_SxCR_PBURST_Msk      (0x3UL << DMA_SxCR_PBURST_Pos)                 /*!< 0x00600000 */
#define DMA_SxCR_PBURST          DMA_SxCR_PBURST_Msk
#define DMA_SxCR_PBURST_0        (0x1UL << DMA_SxCR_PBURST_Pos)                 /*!< 0x00200000 */
#define DMA_SxCR_PBURST_1        (0x2UL << DMA_SxCR_PBURST_Pos)                 /*!< 0x00400000 */
#define DMA_SxCR_CT_Pos          (19U)
#define DMA_SxCR_CT_Msk          (0x1UL << DMA_SxCR_CT_Pos)                     /*!< 0x00080000 */
#define DMA_SxCR_CT              DMA_SxCR_CT_Msk
#define DMA_SxCR_DBM_Pos         (18U)
#define DMA_SxCR_DBM_Msk         (0x1UL << DMA_SxCR_DBM_Pos)                    /*!< 0x00040000 */
#define DMA_SxCR_DBM             DMA_SxCR_DBM_Msk
#define DMA_SxCR_PL_Pos          (16U)
#define DMA_SxCR_PL_Msk          (0x3UL << DMA_SxCR_PL_Pos)                     /*!< 0x00030000 */
#define DMA_SxCR_PL              DMA_SxCR_PL_Msk
#define DMA_SxCR_PL_0            (0x1UL << DMA_SxCR_PL_Pos)                     /*!< 0x00010000 */
#define DMA_SxCR_PL_1            (0x2UL << DMA_SxCR_PL_Pos)                     /*!< 0x00020000 */
#define DMA_SxCR_PINCOS_Pos      (15U)
#define DMA_SxCR_PINCOS_Msk      (0x1UL << DMA_SxCR_PINCOS_Pos)                 /*!< 0x00008000 */
#define DMA_SxCR_PINCOS          DMA_SxCR_PINCOS_Msk
#define DMA_SxCR_MSIZE_Pos       (13U)
#define DMA_SxCR_MSIZE_Msk       (0x3UL << DMA_SxCR_MSIZE_Pos)                  /*!< 0x00006000 */
#define DMA_SxCR_MSIZE           DMA_SxCR_MSIZE_Msk
#define DMA_SxCR_MSIZE_0         (0x1UL << DMA_SxCR_MSIZE_Pos)                  /*!< 0x00002000 */
#define DMA_SxCR_MSIZE_1         (0x2UL << DMA_SxCR_MSIZE_Pos)                  /*!< 0x00004000 */
#define DMA_SxCR_PSIZE_Pos       (11U)
#define DMA_SxCR_PSIZE_Msk       (0x3UL << DMA_SxCR_PSIZE_Pos)                  /*!< 0x00001800 */
#define DMA_SxCR_PSIZE           DMA_SxCR_PSIZE_Msk
#define DMA_SxCR_PSIZE_0         (0x1UL << DMA_SxCR_PSIZE_Pos)                  /*!< 0x00000800 */
#define DMA_SxCR_PSIZE_1         (0x2UL << DMA_SxCR_PSIZE_Pos)                  /*!< 0x00001000 */
#define DMA_SxCR_MINC_Pos        (10U)
#define DMA_SxCR_MINC_Msk        (0x1UL << DMA_SxCR_MINC_Pos)                   /*!< 0x00000400 */
#define DMA_SxCR_MINC            DMA_SxCR_MINC_Msk
#define DMA_SxCR_PINC_Pos        (9U)
#define DMA_SxCR_PINC_Msk        (0x1UL << DMA_SxCR_PINC_Pos)                   /*!< 0x00000200 */
#define DMA_SxCR_PINC            DMA_SxCR_PINC_Msk
#define DMA_SxCR_CIRC_Pos        (8U)
#define DMA_SxCR_CIRC_Msk        (0x1UL << DMA_SxCR_CIRC_Pos)                   /*!< 0x00000100 */
#define DMA_SxCR_CIRC            DMA_SxCR_CIRC_Msk
#define DMA_SxCR_DIR_Pos         (6U)
#define DMA_SxCR_DIR_Msk         (0x3UL << DMA_SxCR_DIR_Pos)                    /*!< 0x000000C0 */
#define DMA_SxCR_DIR             DMA_SxCR_DIR_Msk
#define DMA_SxCR_DIR_0           (0x1UL << DMA_SxCR_DIR_Pos)                    /*!< 0x00000040 */
#define DMA_SxCR_DIR_1           (0x2UL << DMA_SxCR_DIR_Pos)                    /*!< 0x00000080 */
#define DMA_SxCR_PFCTRL_Pos      (5U)
#define DMA_SxCR_PFCTRL_Msk      (0x1UL << DMA_SxCR_PFCTRL_Pos)                 /*!< 0x00000020 */
#define DMA_SxCR_PFCTRL          DMA_SxCR_PFCTRL_Msk
#define DMA_SxCR_TCIE_Pos        (4U)
#define DMA_SxCR_TCIE_Msk        (0x1UL << DMA_SxCR_TCIE_Pos)                   /*!< 0x00000010 */
#define DMA_SxCR_TCIE            DMA_SxCR_TCIE_Msk
#define DMA_SxCR_HTIE_Pos        (3U)
#define DMA_SxCR_HTIE_Msk        (0x1UL << DMA_SxCR_HTIE_Pos)                   /*!< 0x00000008 */
#define DMA_SxCR_HTIE            DMA_SxCR_HTIE_Msk
#define DMA_SxCR_TEIE_Pos        (2U)
#define DMA_SxCR_TEIE_Msk        (0x1UL << DMA_SxCR_TEIE_Pos)                   /*!< 0x00000004 */
#define DMA_SxCR_TEIE            DMA_SxCR_TEIE_Msk
#define DMA_SxCR_DMEIE_Pos       (1U)
#define DMA_SxCR_DMEIE_Msk       (0x1UL << DMA_SxCR_DMEIE_Pos)                  /*!< 0x00000002 */
#define DMA_SxCR_DMEIE           DMA_SxCR_DMEIE_Msk
#define DMA_SxCR_EN_Pos          (0U)
#define DMA_SxCR_EN_Msk          (0x1UL << DMA_SxCR_EN_Pos)                     /*!< 0x00000001 */
#define DMA_SxCR_EN              DMA_SxCR_EN_Msk

/* Legacy defines */
#define DMA_SxCR_ACK_Pos         (20U)
#define DMA_SxCR_ACK_Msk         (0x1UL << DMA_SxCR_ACK_Pos)                    /*!< 0x00100000 */
#define DMA_SxCR_ACK             DMA_SxCR_ACK_Msk

/********************  Bits definition for DMA_SxCNDTR register  **************/
#define DMA_SxNDT_Pos            (0U)
#define DMA_SxNDT_Msk            (0xFFFFUL << DMA_SxNDT_Pos)                    /*!< 0x0000FFFF */
#define DMA_SxNDT                DMA_SxNDT_Msk
#define DMA_SxNDT_0              (0x0001UL << DMA_SxNDT_Pos)                    /*!< 0x00000001 */
#define DMA_SxNDT_1              (0x0002UL << DMA_SxNDT_Pos)                    /*!< 0x00000002 */
#define DMA_SxNDT_2              (0x0004UL << DMA_SxNDT_Pos)                    /*!< 0x00000004 */
#define DMA_SxNDT_3              (0x0008UL << DMA_SxNDT_Pos)                    /*!< 0x00000008 */
#define DMA_SxNDT_4              (0x0010UL << DMA_SxNDT_Pos)                    /*!< 0x00000010 */
#define DMA_SxNDT_5              (0x0020UL << DMA_SxNDT_Pos)                    /*!< 0x00000020 */
#define DMA_SxNDT_6              (0x0040UL << DMA_SxNDT_Pos)                    /*!< 0x00000040 */
#define DMA_SxNDT_7              (0x0080UL << DMA_SxNDT_Pos)                    /*!< 0x00000080 */
#define DMA_SxNDT_8              (0x0100UL << DMA_SxNDT_Pos)                    /*!< 0x00000100 */
#define DMA_SxNDT_9              (0x0200UL << DMA_SxNDT_Pos)                    /*!< 0x00000200 */
#define DMA_SxNDT_10             (0x0400UL << DMA_SxNDT_Pos)                    /*!< 0x00000400 */
#define DMA_SxNDT_11             (0x0800UL << DMA_SxNDT_Pos)                    /*!< 0x00000800 */
#define DMA_SxNDT_12             (0x1000UL << DMA_SxNDT_Pos)                    /*!< 0x00001000 */
#define DMA_SxNDT_13             (0x2000UL << DMA_SxNDT_Pos)                    /*!< 0x00002000 */
#define DMA_SxNDT_14             (0x4000UL << DMA_SxNDT_Pos)                    /*!< 0x00004000 */
#define DMA_SxNDT_15             (0x8000UL << DMA_SxNDT_Pos)                    /*!< 0x00008000 */

/********************  Bits definition for DMA_SxFCR register  ****************/
#define DMA_SxFCR_FEIE_Pos       (7U)
#define DMA_SxFCR_FEIE_Msk       (0x1UL << DMA_SxFCR_FEIE_Pos)                  /*!< 0x00000080 */
#define DMA_SxFCR_FEIE           DMA_SxFCR_FEIE_Msk
#define DMA_SxFCR_FS_Pos         (3U)
#define DMA_SxFCR_FS_Msk         (0x7UL << DMA_SxFCR_FS_Pos)                    /*!< 0x00000038 */
#define DMA_SxFCR_FS             DMA_SxFCR_FS_Msk
#define DMA_SxFCR_FS_0           (0x1UL << DMA_SxFCR_FS_Pos)                    /*!< 0x00000008 */
#define DMA_SxFCR_FS_1           (0x2UL << DMA_SxFCR_FS_Pos)                    /*!< 0x00000010 */
#define DMA_SxFCR_FS_2           (0x4UL << DMA_SxFCR_FS_Pos)                    /*!< 0x00000020 */
#define DMA_SxFCR_DMDIS_Pos      (2U)
#define DMA_SxFCR_DMDIS_Msk      (0x1UL << DMA_SxFCR_DMDIS_Pos)                 /*!< 0x00000004 */
#define DMA_SxFCR_DMDIS          DMA_SxFCR_DMDIS_Msk
#define DMA_SxFCR_FTH_Pos        (0U)
#define DMA_SxFCR_FTH_Msk        (0x3UL << DMA_SxFCR_FTH_Pos)                   /*!< 0x00000003 */
#define DMA_SxFCR_FTH            DMA_SxFCR_FTH_Msk
#define DMA_SxFCR_FTH_0          (0x1UL << DMA_SxFCR_FTH_Pos)                   /*!< 0x00000001 */
#define DMA_SxFCR_FTH_1          (0x2UL << DMA_SxFCR_FTH_Pos)                   /*!< 0x00000002 */

/********************  Bits definition for DMA_LISR register  *****************/
#define DMA_LISR_TCIF3_Pos       (27U)
#define DMA_LISR_TCIF3_Msk       (0x1UL << DMA_LISR_TCIF3_Pos)                  /*!< 0x08000000 */
#define DMA_LISR_TCIF3           DMA_LISR_TCIF3_Msk
#define DMA_LISR_HTIF3_Pos       (26U)
#define DMA_LISR_HTIF3_Msk       (0x1UL << DMA_LISR_HTIF3_Pos)                  /*!< 0x04000000 */
#define DMA_LISR_HTIF3           DMA_LISR_HTIF3_Msk
#define DMA_LISR_TEIF3_Pos       (25U)
#define DMA_LISR_TEIF3_Msk       (0x1UL << DMA_LISR_TEIF3_Pos)                  /*!< 0x02000000 */
#define DMA_LISR_TEIF3           DMA_LISR_TEIF3_Msk
#define DMA_LISR_DMEIF3_Pos      (24U)
#define DMA_LISR_DMEIF3_Msk      (0x1UL << DMA_LISR_DMEIF3_Pos)                 /*!< 0x01000000 */
#define DMA_LISR_DMEIF3          DMA_LISR_DMEIF3_Msk
#define DMA_LISR_FEIF3_Pos       (22U)
#define DMA_LISR_FEIF3_Msk       (0x1UL << DMA_LISR_FEIF3_Pos)                  /*!< 0x00400000 */
#define DMA_LISR_FEIF3           DMA_LISR_FEIF3_Msk
#define DMA_LISR_TCIF2_Pos       (21U)
#define DMA_LISR_TCIF2_Msk       (0x1UL << DMA_LISR_TCIF2_Pos)                  /*!< 0x00200000 */
#define DMA_LISR_TCIF2           DMA_LISR_TCIF2_Msk
#define DMA_LISR_HTIF2_Pos       (20U)
#define DMA_LISR_HTIF2_Msk       (0x1UL << DMA_LISR_HTIF2_Pos)                  /*!< 0x00100000 */
#define DMA_LISR_HTIF2           DMA_LISR_HTIF2_Msk
#define DMA_LISR_TEIF2_Pos       (19U)
#define DMA_LISR_TEIF2_Msk       (0x1UL << DMA_LISR_TEIF2_Pos)                  /*!< 0x00080000 */
#define DMA_LISR_TEIF2           DMA_LISR_TEIF2_Msk
#define DMA_LISR_DMEIF2_Pos      (18U)
#define DMA_LISR_DMEIF2_Msk      (0x1UL << DMA_LISR_DMEIF2_Pos)                 /*!< 0x00040000 */
#define DMA_LISR_DMEIF2          DMA_LISR_DMEIF2_Msk
#define DMA_LISR_FEIF2_Pos       (16U)
#define DMA_LISR_FEIF2_Msk       (0x1UL << DMA_LISR_FEIF2_Pos)                  /*!< 0x00010000 */
#define DMA_LISR_FEIF2           DMA_LISR_FEIF2_Msk
#define DMA_LISR_TCIF1_Pos       (11U)
#define DMA_LISR_TCIF1_Msk       (0x1UL << DMA_LISR_TCIF1_Pos)                  /*!< 0x00000800 */
#define DMA_LISR_TCIF1           DMA_LISR_TCIF1_Msk
#define DMA_LISR_HTIF1_Pos       (10U)
#define DMA_LISR_HTIF1_Msk       (0x1UL << DMA_LISR_HTIF1_Pos)                  /*!< 0x00000400 */
#define DMA_LISR_HTIF1           DMA_LISR_HTIF1_Msk
#define DMA_LISR_TEIF1_Pos       (9U)
#define DMA_LISR_TEIF1_Msk       (0x1UL << DMA_LISR_TEIF1_Pos)                  /*!< 0x00000200 */
#define DMA_LISR_TEIF1           DMA_LISR_TEIF1_Msk
#define DMA_LISR_DMEIF1_Pos      (8U)
#define DMA_LISR_DMEIF1_Msk      (0x1UL << DMA_LISR_DMEIF1_Pos)                 /*!< 0x00000100 */
#define DMA_LISR_DMEIF1          DMA_LISR_DMEIF1_Msk
#define DMA_LISR_FEIF1_Pos       (6U)
#define DMA_LISR_FEIF1_Msk       (0x1UL << DMA_LISR_FEIF1_Pos)                  /*!< 0x00000040 */
#define DMA_LISR_FEIF1           DMA_LISR_FEIF1_Msk
#define DMA_LISR_TCIF0_Pos       (5U)
#define DMA_LISR_TCIF0_Msk       (0x1UL << DMA_LISR_TCIF0_Pos)                  /*!< 0x00000020 */
#define DMA_LISR_TCIF0           DMA_LISR_TCIF0_Msk
#define DMA_LISR_HTIF0_Pos       (4U)
#define DMA_LISR_HTIF0_Msk       (0x1UL << DMA_LISR_HTIF0_Pos)                  /*!< 0x00000010 */
#define DMA_LISR_HTIF0           DMA_LISR_HTIF0_Msk
#define DMA_LISR_TEIF0_Pos       (3U)
#define DMA_LISR_TEIF0_Msk       (0x1UL << DMA_LISR_TEIF0_Pos)                  /*!< 0x00000008 */
#define DMA_LISR_TEIF0           DMA_LISR_TEIF0_Msk
#define DMA_LISR_DMEIF0_Pos      (2U)
#define DMA_LISR_DMEIF0_Msk      (0x1UL << DMA_LISR_DMEIF0_Pos)                 /*!< 0x00000004 */
#define DMA_LISR_DMEIF0          DMA_LISR_DMEIF0_Msk
#define DMA_LISR_FEIF0_Pos       (0U)
#define DMA_LISR_FEIF0_Msk       (0x1UL << DMA_LISR_FEIF0_Pos)                  /*!< 0x00000001 */
#define DMA_LISR_FEIF0           DMA_LISR_FEIF0_Msk

/********************  Bits definition for DMA_HISR register  *****************/
#define DMA_HISR_TCIF7_Pos       (27U)
#define DMA_HISR_TCIF7_Msk       (0x1UL << DMA_HISR_TCIF7_Pos)                  /*!< 0x08000000 */
#define DMA_HISR_TCIF7           DMA_HISR_TCIF7_Msk
#define DMA_HISR_HTIF7_Pos       (26U)
#define DMA_HISR_HTIF7_Msk       (0x1UL << DMA_HISR_HTIF7_Pos)                  /*!< 0x04000000 */
#define DMA_HISR_HTIF7           DMA_HISR_HTIF7_Msk
#define DMA_HISR_TEIF7_Pos       (25U)
#define DMA_HISR_TEIF7_Msk       (0x1UL << DMA_HISR_TEIF7_Pos)                  /*!< 0x02000000 */
#define DMA_HISR_TEIF7           DMA_HISR_TEIF7_Msk
#define DMA_HISR_DMEIF7_Pos      (24U)
#define DMA_HISR_DMEIF7_Msk      (0x1UL << DMA_HISR_DMEIF7_Pos)                 /*!< 0x01000000 */
#define DMA_HISR_DMEIF7          DMA_HISR_DMEIF7_Msk
#define DMA_HISR_FEIF7_Pos       (22U)
#define DMA_HISR_FEIF7_Msk       (0x1UL << DMA_HISR_FEIF7_Pos)                  /*!< 0x00400000 */
#define DMA_HISR_FEIF7           DMA_HISR_FEIF7_Msk
#define DMA_HISR_TCIF6_Pos       (21U)
#define DMA_HISR_TCIF6_Msk       (0x1UL << DMA_HISR_TCIF6_Pos)                  /*!< 0x00200000 */
#define DMA_HISR_TCIF6           DMA_HISR_TCIF6_Msk
#define DMA_HISR_HTIF6_Pos       (20U)
#define DMA_HISR_HTIF6_Msk       (0x1UL << DMA_HISR_HTIF6_Pos)                  /*!< 0x00100000 */
#define DMA_HISR_HTIF6           DMA_HISR_HTIF6_Msk
#define DMA_HISR_TEIF6_Pos       (19U)
#define DMA_HISR_TEIF6_Msk       (0x1UL << DMA_HISR_TEIF6_Pos)                  /*!< 0x00080000 */
#define DMA_HISR_TEIF6           DMA_HISR_TEIF6_Msk
#define DMA_HISR_DMEIF6_Pos      (18U)
#define DMA_HISR_DMEIF6_Msk      (0x1UL << DMA_HISR_DMEIF6_Pos)                 /*!< 0x00040000 */
#define DMA_HISR_DMEIF6          DMA_HISR_DMEIF6_Msk
#define DMA_HISR_FEIF6_Pos       (16U)
#define DMA_HISR_FEIF6_Msk       (0x1UL << DMA_HISR_FEIF6_Pos)                  /*!< 0x00010000 */
#define DMA_HISR_FEIF6           DMA_HISR_FEIF6_Msk
#define DMA_HISR_TCIF5_Pos       (11U)
#define DMA_HISR_TCIF5_Msk       (0x1UL << DMA_HISR_TCIF5_Pos)                  /*!< 0x00000800 */
#define DMA_HISR_TCIF5           DMA_HISR_TCIF5_Msk
#define DMA_HISR_HTIF5_Pos       (10U)
#define DMA_HISR_HTIF5_Msk       (0x1UL << DMA_HISR_HTIF5_Pos)                  /*!< 0x00000400 */
#define DMA_HISR_HTIF5           DMA_HISR_HTIF5_Msk
#define DMA_HISR_TEIF5_Pos       (9U)
#define DMA_HISR_TEIF5_Msk       (0x1UL << DMA_HISR_TEIF5_Pos)                  /*!< 0x00000200 */
#define DMA_HISR_TEIF5           DMA_HISR_TEIF5_Msk
#define DMA_HISR_DMEIF5_Pos      (8U)
#define DMA_HISR_DMEIF5_Msk      (0x1UL << DMA_HISR_DMEIF5_Pos)                 /*!< 0x00000100 */
#define DMA_HISR_DMEIF5          DMA_HISR_DMEIF5_Msk
#define DMA_HISR_FEIF5_Pos       (6U)
#define DMA_HISR_FEIF5_Msk       (0x1UL << DMA_HISR_FEIF5_Pos)                  /*!< 0x00000040 */
#define DMA_HISR_FEIF5           DMA_HISR_FEIF5_Msk
#define DMA_HISR_TCIF4_Pos       (5U)
#define DMA_HISR_TCIF4_Msk       (0x1UL << DMA_HISR_TCIF4_Pos)                  /*!< 0x00000020 */
#define DMA_HISR_TCIF4           DMA_HISR_TCIF4_Msk
#define DMA_HISR_HTIF4_Pos       (4U)
#define DMA_HISR_HTIF4_Msk       (0x1UL << DMA_HISR_HTIF4_Pos)                  /*!< 0x00000010 */
#define DMA_HISR_HTIF4           DMA_HISR_HTIF4_Msk
#define DMA_HISR_TEIF4_Pos       (3U)
#define DMA_HISR_TEIF4_Msk       (0x1UL << DMA_HISR_TEIF4_Pos)                  /*!< 0x00000008 */
#define DMA_HISR_TEIF4           DMA_HISR_TEIF4_Msk
#define DMA_HISR_DMEIF4_Pos      (2U)
#define DMA_HISR_DMEIF4_Msk      (0x1UL << DMA_HISR_DMEIF4_Pos)                 /*!< 0x00000004 */
#define DMA_HISR_DMEIF4          DMA_HISR_DMEIF4_Msk
#define DMA_HISR_FEIF4_Pos       (0U)
#define DMA_HISR_FEIF4_Msk       (0x1UL << DMA_HISR_FEIF4_Pos)                  /*!< 0x00000001 */
#define DMA_HISR_FEIF4           DMA_HISR_FEIF4_Msk

/********************  Bits definition for DMA_LIFCR register  ****************/
#define DMA_LIFCR_CTCIF3_Pos     (27U)
#define DMA_LIFCR_CTCIF3_Msk     (0x1UL << DMA_LIFCR_CTCIF3_Pos)                /*!< 0x08000000 */
#define DMA_LIFCR_CTCIF3         DMA_LIFCR_CTCIF3_Msk
#define DMA_LIFCR_CHTIF3_Pos     (26U)
#define DMA_LIFCR_CHTIF3_Msk     (0x1UL << DMA_LIFCR_CHTIF3_Pos)                /*!< 0x04000000 */
#define DMA_LIFCR_CHTIF3         DMA_LIFCR_CHTIF3_Msk
#define DMA_LIFCR_CTEIF3_Pos     (25U)
#define DMA_LIFCR_CTEIF3_Msk     (0x1UL << DMA_LIFCR_CTEIF3_Pos)                /*!< 0x02000000 */
#define DMA_LIFCR_CTEIF3         DMA_LIFCR_CTEIF3_Msk
#define DMA_LIFCR_CDMEIF3_Pos    (24U)
#define DMA_LIFCR_CDMEIF3_Msk    (0x1UL << DMA_LIFCR_CDMEIF3_Pos)               /*!< 0x01000000 */
#define DMA_LIFCR_CDMEIF3        DMA_LIFCR_CDMEIF3_Msk
#define DMA_LIFCR_CFEIF3_Pos     (22U)
#define DMA_LIFCR_CFEIF3_Msk     (0x1UL << DMA_LIFCR_CFEIF3_Pos)                /*!< 0x00400000 */
#define DMA_LIFCR_CFEIF3         DMA_LIFCR_CFEIF3_Msk
#define DMA_LIFCR_CTCIF2_Pos     (21U)
#define DMA_LIFCR_CTCIF2_Msk     (0x1UL << DMA_LIFCR_CTCIF2_Pos)                /*!< 0x00200000 */
#define DMA_LIFCR_CTCIF2         DMA_LIFCR_CTCIF2_Msk
#define DMA_LIFCR_CHTIF2_Pos     (20U)
#define DMA_LIFCR_CHTIF2_Msk     (0x1UL << DMA_LIFCR_CHTIF2_Pos)                /*!< 0x00100000 */
#define DMA_LIFCR_CHTIF2         DMA_LIFCR_CHTIF2_Msk
#define DMA_LIFCR_CTEIF2_Pos     (19U)
#define DMA_LIFCR_CTEIF2_Msk     (0x1UL << DMA_LIFCR_CTEIF2_Pos)                /*!< 0x00080000 */
#define DMA_LIFCR_CTEIF2         DMA_LIFCR_CTEIF2_Msk
#define DMA_LIFCR_CDMEIF2_Pos    (18U)
#define DMA_LIFCR_CDMEIF2_Msk    (0x1UL << DMA_LIFCR_CDMEIF2_Pos)               /*!< 0x00040000 */
#define DMA_LIFCR_CDMEIF2        DMA_LIFCR_CDMEIF2_Msk
#define DMA_LIFCR_CFEIF2_Pos     (16U)
#define DMA_LIFCR_CFEIF2_Msk     (0x1UL << DMA_LIFCR_CFEIF2_Pos)                /*!< 0x00010000 */
#define DMA_LIFCR_CFEIF2         DMA_LIFCR_CFEIF2_Msk
#define DMA_LIFCR_CTCIF1_Pos     (11U)
#define DMA_LIFCR_CTCIF1_Msk     (0x1UL << DMA_LIFCR_CTCIF1_Pos)                /*!< 0x00000800 */
#define DMA_LIFCR_CTCIF1         DMA_LIFCR_CTCIF1_Msk
#define DMA_LIFCR_CHTIF1_Pos     (10U)
#define DMA_LIFCR_CHTIF1_Msk     (0x1UL << DMA_LIFCR_CHTIF1_Pos)                /*!< 0x00000400 */
#define DMA_LIFCR_CHTIF1         DMA_LIFCR_CHTIF1_Msk
#define DMA_LIFCR_CTEIF1_Pos     (9U)
#define DMA_LIFCR_CTEIF1_Msk     (0x1UL << DMA_LIFCR_CTEIF1_Pos)                /*!< 0x00000200 */
#define DMA_LIFCR_CTEIF1         DMA_LIFCR_CTEIF1_Msk
#define DMA_LIFCR_CDMEIF1_Pos    (8U)
#define DMA_LIFCR_CDMEIF1_Msk    (0x1UL << DMA_LIFCR_CDMEIF1_Pos)               /*!< 0x00000100 */
#define DMA_LIFCR_CDMEIF1        DMA_LIFCR_CDMEIF1_Msk
#define DMA_LIFCR_CFEIF1_Pos     (6U)
#define DMA_LIFCR_CFEIF1_Msk     (0x1UL << DMA_LIFCR_CFEIF1_Pos)                /*!< 0x00000040 */
#define DMA_LIFCR_CFEIF1         DMA_LIFCR_CFEIF1_Msk
#define DMA_LIFCR_CTCIF0_Pos     (5U)
#define DMA_LIFCR_CTCIF0_Msk     (0x1UL << DMA_LIFCR_CTCIF0_Pos)                /*!< 0x00000020 */
#define DMA_LIFCR_CTCIF0         DMA_LIFCR_CTCIF0_Msk
#define DMA_LIFCR_CHTIF0_Pos     (4U)
#define DMA_LIFCR_CHTIF0_Msk     (0x1UL << DMA_LIFCR_CHTIF0_Pos)                /*!< 0x00000010 */
#define DMA_LIFCR_CHTIF0         DMA_LIFCR_CHTIF0_Msk
#define DMA_LIFCR_CTEIF0_Pos     (3U)
#define DMA_LIFCR_CTEIF0_Msk     (0x1UL << DMA_LIFCR_CTEIF0_Pos)                /*!< 0x00000008 */
#define DMA_LIFCR_CTEIF0         DMA_LIFCR_CTEIF0_Msk
#define DMA_LIFCR_CDMEIF0_Pos    (2U)
#define DMA_LIFCR_CDMEIF0_Msk    (0x1UL << DMA_LIFCR_CDMEIF0_Pos)               /*!< 0x00000004 */
#define DMA_LIFCR_CDMEIF0        DMA_LIFCR_CDMEIF0_Msk
#define DMA_LIFCR_CFEIF0_Pos     (0U)
#define DMA_LIFCR_CFEIF0_Msk     (0x1UL << DMA_LIFCR_CFEIF0_Pos)                /*!< 0x00000001 */
#define DMA_LIFCR_CFEIF0         DMA_LIFCR_CFEIF0_Msk

/********************  Bits definition for DMA_HIFCR  register  ****************/
#define DMA_HIFCR_CTCIF7_Pos     (27U)
#define DMA_HIFCR_CTCIF7_Msk     (0x1UL << DMA_HIFCR_CTCIF7_Pos)                /*!< 0x08000000 */
#define DMA_HIFCR_CTCIF7         DMA_HIFCR_CTCIF7_Msk
#define DMA_HIFCR_CHTIF7_Pos     (26U)
#define DMA_HIFCR_CHTIF7_Msk     (0x1UL << DMA_HIFCR_CHTIF7_Pos)                /*!< 0x04000000 */
#define DMA_HIFCR_CHTIF7         DMA_HIFCR_CHTIF7_Msk
#define DMA_HIFCR_CTEIF7_Pos     (25U)
#define DMA_HIFCR_CTEIF7_Msk     (0x1UL << DMA_HIFCR_CTEIF7_Pos)                /*!< 0x02000000 */
#define DMA_HIFCR_CTEIF7         DMA_HIFCR_CTEIF7_Msk
#define DMA_HIFCR_CDMEIF7_Pos    (24U)
#define DMA_HIFCR_CDMEIF7_Msk    (0x1UL << DMA_HIFCR_CDMEIF7_Pos)               /*!< 0x01000000 */
#define DMA_HIFCR_CDMEIF7        DMA_HIFCR_CDMEIF7_Msk
#define DMA_HIFCR_CFEIF7_Pos     (22U)
#define DMA_HIFCR_CFEIF7_Msk     (0x1UL << DMA_HIFCR_CFEIF7_Pos)                /*!< 0x00400000 */
#define DMA_HIFCR_CFEIF7         DMA_HIFCR_CFEIF7_Msk
#define DMA_HIFCR_CTCIF6_Pos     (21U)
#define DMA_HIFCR_CTCIF6_Msk     (0x1UL << DMA_HIFCR_CTCIF6_Pos)                /*!< 0x00200000 */
#define DMA_HIFCR_CTCIF6         DMA_HIFCR_CTCIF6_Msk
#define DMA_HIFCR_CHTIF6_Pos     (20U)
#define DMA_HIFCR_CHTIF6_Msk     (0x1UL << DMA_HIFCR_CHTIF6_Pos)                /*!< 0x00100000 */
#define DMA_HIFCR_CHTIF6         DMA_HIFCR_CHTIF6_Msk
#define DMA_HIFCR_CTEIF6_Pos     (19U)
#define DMA_HIFCR_CTEIF6_Msk     (0x1UL << DMA_HIFCR_CTEIF6_Pos)                /*!< 0x00080000 */
#define DMA_HIFCR_CTEIF6         DMA_HIFCR_CTEIF6_Msk
#define DMA_HIFCR_CDMEIF6_Pos    (18U)
#define DMA_HIFCR_CDMEIF6_Msk    (0x1UL << DMA_HIFCR_CDMEIF6_Pos)               /*!< 0x00040000 */
#define DMA_HIFCR_CDMEIF6        DMA_HIFCR_CDMEIF6_Msk
#define DMA_HIFCR_CFEIF6_Pos     (16U)
#define DMA_HIFCR_CFEIF6_Msk     (0x1UL << DMA_HIFCR_CFEIF6_Pos)                /*!< 0x00010000 */
#define DMA_HIFCR_CFEIF6         DMA_HIFCR_CFEIF6_Msk
#define DMA_HIFCR_CTCIF5_Pos     (11U)
#define DMA_HIFCR_CTCIF5_Msk     (0x1UL << DMA_HIFCR_CTCIF5_Pos)                /*!< 0x00000800 */
#define DMA_HIFCR_CTCIF5         DMA_HIFCR_CTCIF5_Msk
#define DMA_HIFCR_CHTIF5_Pos     (10U)
#define DMA_HIFCR_CHTIF5_Msk     (0x1UL << DMA_HIFCR_CHTIF5_Pos)                /*!< 0x00000400 */
#define DMA_HIFCR_CHTIF5         DMA_HIFCR_CHTIF5_Msk
#define DMA_HIFCR_CTEIF5_Pos     (9U)
#define DMA_HIFCR_CTEIF5_Msk     (0x1UL << DMA_HIFCR_CTEIF5_Pos)                /*!< 0x00000200 */
#define DMA_HIFCR_CTEIF5         DMA_HIFCR_CTEIF5_Msk
#define DMA_HIFCR_CDMEIF5_Pos    (8U)
#define DMA_HIFCR_CDMEIF5_Msk    (0x1UL << DMA_HIFCR_CDMEIF5_Pos)               /*!< 0x00000100 */
#define DMA_HIFCR_CDMEIF5        DMA_HIFCR_CDMEIF5_Msk
#define DMA_HIFCR_CFEIF5_Pos     (6U)
#define DMA_HIFCR_CFEIF5_Msk     (0x1UL << DMA_HIFCR_CFEIF5_Pos)                /*!< 0x00000040 */
#define DMA_HIFCR_CFEIF5         DMA_HIFCR_CFEIF5_Msk
#define DMA_HIFCR_CTCIF4_Pos     (5U)
#define DMA_HIFCR_CTCIF4_Msk     (0x1UL << DMA_HIFCR_CTCIF4_Pos)                /*!< 0x00000020 */
#define DMA_HIFCR_CTCIF4         DMA_HIFCR_CTCIF4_Msk
#define DMA_HIFCR_CHTIF4_Pos     (4U)
#define DMA_HIFCR_CHTIF4_Msk     (0x1UL << DMA_HIFCR_CHTIF4_Pos)                /*!< 0x00000010 */
#define DMA_HIFCR_CHTIF4         DMA_HIFCR_CHTIF4_Msk
#define DMA_HIFCR_CTEIF4_Pos     (3U)
#define DMA_HIFCR_CTEIF4_Msk     (0x1UL << DMA_HIFCR_CTEIF4_Pos)                /*!< 0x00000008 */
#define DMA_HIFCR_CTEIF4         DMA_HIFCR_CTEIF4_Msk
#define DMA_HIFCR_CDMEIF4_Pos    (2U)
#define DMA_HIFCR_CDMEIF4_Msk    (0x1UL << DMA_HIFCR_CDMEIF4_Pos)               /*!< 0x00000004 */
#define DMA_HIFCR_CDMEIF4        DMA_HIFCR_CDMEIF4_Msk
#define DMA_HIFCR_CFEIF4_Pos     (0U)
#define DMA_HIFCR_CFEIF4_Msk     (0x1UL << DMA_HIFCR_CFEIF4_Pos)                /*!< 0x00000001 */
#define DMA_HIFCR_CFEIF4         DMA_HIFCR_CFEIF4_Msk

/******************  Bit definition for DMA_SxPAR register  ********************/
#define DMA_SxPAR_PA_Pos         (0U)
#define DMA_SxPAR_PA_Msk         (0xFFFFFFFFUL << DMA_SxPAR_PA_Pos)             /*!< 0xFFFFFFFF */
#define DMA_SxPAR_PA             DMA_SxPAR_PA_Msk                              /*!< Peripheral Address */

/******************  Bit definition for DMA_SxM0AR register  ********************/
#define DMA_SxM0AR_M0A_Pos       (0U)
#define DMA_SxM0AR_M0A_Msk       (0xFFFFFFFFUL << DMA_SxM0AR_M0A_Pos)           /*!< 0xFFFFFFFF */
#define DMA_SxM0AR_M0A           DMA_SxM0AR_M0A_Msk                            /*!< Memory Address */

/******************  Bit definition for DMA_SxM1AR register  ********************/
#define DMA_SxM1AR_M1A_Pos       (0U)
#define DMA_SxM1AR_M1A_Msk       (0xFFFFFFFFUL << DMA_SxM1AR_M1A_Pos)           /*!< 0xFFFFFFFF */
#define DMA_SxM1AR_M1A           DMA_SxM1AR_M1A_Msk                            /*!< Memory Address */


/******************************************************************************/
/*                                                                            */
/*                    External Interrupt/Event Controller                     */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for EXTI_IMR register  *******************/
#define EXTI_IMR_MR0_Pos          (0U)
#define EXTI_IMR_MR0_Msk          (0x1UL << EXTI_IMR_MR0_Pos)                   /*!< 0x00000001 */
#define EXTI_IMR_MR0              EXTI_IMR_MR0_Msk                             /*!< Interrupt Mask on line 0 */
#define EXTI_IMR_MR1_Pos          (1U)
#define EXTI_IMR_MR1_Msk          (0x1UL << EXTI_IMR_MR1_Pos)                   /*!< 0x00000002 */
#define EXTI_IMR_MR1              EXTI_IMR_MR1_Msk                             /*!< Interrupt Mask on line 1 */
#define EXTI_IMR_MR2_Pos          (2U)
#define EXTI_IMR_MR2_Msk          (0x1UL << EXTI_IMR_MR2_Pos)                   /*!< 0x00000004 */
#define EXTI_IMR_MR2              EXTI_IMR_MR2_Msk                             /*!< Interrupt Mask on line 2 */
#define EXTI_IMR_MR3_Pos          (3U)
#define EXTI_IMR_MR3_Msk          (0x1UL << EXTI_IMR_MR3_Pos)                   /*!< 0x00000008 */
#define EXTI_IMR_MR3              EXTI_IMR_MR3_Msk                             /*!< Interrupt Mask on line 3 */
#define EXTI_IMR_MR4_Pos          (4U)
#define EXTI_IMR_MR4_Msk          (0x1UL << EXTI_IMR_MR4_Pos)                   /*!< 0x00000010 */
#define EXTI_IMR_MR4              EXTI_IMR_MR4_Msk                             /*!< Interrupt Mask on line 4 */
#define EXTI_IMR_MR5_Pos          (5U)
#define EXTI_IMR_MR5_Msk          (0x1UL << EXTI_IMR_MR5_Pos)                   /*!< 0x00000020 */
#define EXTI_IMR_MR5              EXTI_IMR_MR5_Msk                             /*!< Interrupt Mask on line 5 */
#define EXTI_IMR_MR6_Pos          (6U)
#define EXTI_IMR_MR6_Msk          (0x1UL << EXTI_IMR_MR6_Pos)                   /*!< 0x00000040 */
#define EXTI_IMR_MR6              EXTI_IMR_MR6_Msk                             /*!< Interrupt Mask on line 6 */
#define EXTI_IMR_MR7_Pos          (7U)
#define EXTI_IMR_MR7_Msk          (0x1UL << EXTI_IMR_MR7_Pos)                   /*!< 0x00000080 */
#define EXTI_IMR_MR7              EXTI_IMR_MR7_Msk                             /*!< Interrupt Mask on line 7 */
#define EXTI_IMR_MR8_Pos          (8U)
#define EXTI_IMR_MR8_Msk          (0x1UL << EXTI_IMR_MR8_Pos)                   /*!< 0x00000100 */
#define EXTI_IMR_MR8              EXTI_IMR_MR8_Msk                             /*!< Interrupt Mask on line 8 */
#define EXTI_IMR_MR9_Pos          (9U)
#define EXTI_IMR_MR9_Msk          (0x1UL << EXTI_IMR_MR9_Pos)                   /*!< 0x00000200 */
#define EXTI_IMR_MR9              EXTI_IMR_MR9_Msk                             /*!< Interrupt Mask on line 9 */
#define EXTI_IMR_MR10_Pos         (10U)
#define EXTI_IMR_MR10_Msk         (0x1UL << EXTI_IMR_MR10_Pos)                  /*!< 0x00000400 */
#define EXTI_IMR_MR10             EXTI_IMR_MR10_Msk                            /*!< Interrupt Mask on line 10 */
#define EXTI_IMR_MR11_Pos         (11U)
#define EXTI_IMR_MR11_Msk         (0x1UL << EXTI_IMR_MR11_Pos)                  /*!< 0x00000800 */
#define EXTI_IMR_MR11             EXTI_IMR_MR11_Msk                            /*!< Interrupt Mask on line 11 */
#define EXTI_IMR_MR12_Pos         (12U)
#define EXTI_IMR_MR12_Msk         (0x1UL << EXTI_IMR_MR12_Pos)                  /*!< 0x00001000 */
#define EXTI_IMR_MR12             EXTI_IMR_MR12_Msk                            /*!< Interrupt Mask on line 12 */
#define EXTI_IMR_MR13_Pos         (13U)
#define EXTI_IMR_MR13_Msk         (0x1UL << EXTI_IMR_MR13_Pos)                  /*!< 0x00002000 */
#define EXTI_IMR_MR13             EXTI_IMR_MR13_Msk                            /*!< Interrupt Mask on line 13 */
#define EXTI_IMR_MR14_Pos         (14U)
#define EXTI_IMR_MR14_Msk         (0x1UL << EXTI_IMR_MR14_Pos)                  /*!< 0x00004000 */
#define EXTI_IMR_MR14             EXTI_IMR_MR14_Msk                            /*!< Interrupt Mask on line 14 */
#define EXTI_IMR_MR15_Pos         (15U)
#define EXTI_IMR_MR15_Msk         (0x1UL << EXTI_IMR_MR15_Pos)                  /*!< 0x00008000 */
#define EXTI_IMR_MR15             EXTI_IMR_MR15_Msk                            /*!< Interrupt Mask on line 15 */
#define EXTI_IMR_MR16_Pos         (16U)
#define EXTI_IMR_MR16_Msk         (0x1UL << EXTI_IMR_MR16_Pos)                  /*!< 0x00010000 */
#define EXTI_IMR_MR16             EXTI_IMR_MR16_Msk                            /*!< Interrupt Mask on line 16 */
#define EXTI_IMR_MR17_Pos         (17U)
#define EXTI_IMR_MR17_Msk         (0x1UL << EXTI_IMR_MR17_Pos)                  /*!< 0x00020000 */
#define EXTI_IMR_MR17             EXTI_IMR_MR17_Msk                            /*!< Interrupt Mask on line 17 */
#define EXTI_IMR_MR18_Pos         (18U)
#define EXTI_IMR_MR18_Msk         (0x1UL << EXTI_IMR_MR18_Pos)                  /*!< 0x00040000 */
#define EXTI_IMR_MR18             EXTI_IMR_MR18_Msk                            /*!< Interrupt Mask on line 18 */
#define EXTI_IMR_MR19_Pos         (19U)
#define EXTI_IMR_MR19_Msk         (0x1UL << EXTI_IMR_MR19_Pos)                  /*!< 0x00080000 */
#define EXTI_IMR_MR19             EXTI_IMR_MR19_Msk                            /*!< Interrupt Mask on line 19 */
#define EXTI_IMR_MR20_Pos         (20U)
#define EXTI_IMR_MR20_Msk         (0x1UL << EXTI_IMR_MR20_Pos)                  /*!< 0x00100000 */
#define EXTI_IMR_MR20             EXTI_IMR_MR20_Msk                            /*!< Interrupt Mask on line 20 */
#define EXTI_IMR_MR21_Pos         (21U)
#define EXTI_IMR_MR21_Msk         (0x1UL << EXTI_IMR_MR21_Pos)                  /*!< 0x00200000 */
#define EXTI_IMR_MR21             EXTI_IMR_MR21_Msk                            /*!< Interrupt Mask on line 21 */
#define EXTI_IMR_MR22_Pos         (22U)
#define EXTI_IMR_MR22_Msk         (0x1UL << EXTI_IMR_MR22_Pos)                  /*!< 0x00400000 */
#define EXTI_IMR_MR22             EXTI_IMR_MR22_Msk                            /*!< Interrupt Mask on line 22 */

/* Reference Defines */
#define  EXTI_IMR_IM0                        EXTI_IMR_MR0
#define  EXTI_IMR_IM1                        EXTI_IMR_MR1
#define  EXTI_IMR_IM2                        EXTI_IMR_MR2
#define  EXTI_IMR_IM3                        EXTI_IMR_MR3
#define  EXTI_IMR_IM4                        EXTI_IMR_MR4
#define  EXTI_IMR_IM5                        EXTI_IMR_MR5
#define  EXTI_IMR_IM6                        EXTI_IMR_MR6
#define  EXTI_IMR_IM7                        EXTI_IMR_MR7
#define  EXTI_IMR_IM8                        EXTI_IMR_MR8
#define  EXTI_IMR_IM9                        EXTI_IMR_MR9
#define  EXTI_IMR_IM10                       EXTI_IMR_MR10
#define  EXTI_IMR_IM11                       EXTI_IMR_MR11
#define  EXTI_IMR_IM12                       EXTI_IMR_MR12
#define  EXTI_IMR_IM13                       EXTI_IMR_MR13
#define  EXTI_IMR_IM14                       EXTI_IMR_MR14
#define  EXTI_IMR_IM15                       EXTI_IMR_MR15
#define  EXTI_IMR_IM16                       EXTI_IMR_MR16
#define  EXTI_IMR_IM17                       EXTI_IMR_MR17
#define  EXTI_IMR_IM18                       EXTI_IMR_MR18
#define  EXTI_IMR_IM19                       EXTI_IMR_MR19
#define  EXTI_IMR_IM20                       EXTI_IMR_MR20
#define  EXTI_IMR_IM21                       EXTI_IMR_MR21
#define  EXTI_IMR_IM22                       EXTI_IMR_MR22
#define EXTI_IMR_IM_Pos           (0U)
#define EXTI_IMR_IM_Msk           (0x7FFFFFUL << EXTI_IMR_IM_Pos)               /*!< 0x007FFFFF */
#define EXTI_IMR_IM               EXTI_IMR_IM_Msk                              /*!< Interrupt Mask All */

/*******************  Bit definition for EXTI_EMR register  *******************/
#define EXTI_EMR_MR0_Pos          (0U)
#define EXTI_EMR_MR0_Msk          (0x1UL << EXTI_EMR_MR0_Pos)                   /*!< 0x00000001 */
#define EXTI_EMR_MR0              EXTI_EMR_MR0_Msk                             /*!< Event Mask on line 0 */
#define EXTI_EMR_MR1_Pos          (1U)
#define EXTI_EMR_MR1_Msk          (0x1UL << EXTI_EMR_MR1_Pos)                   /*!< 0x00000002 */
#define EXTI_EMR_MR1              EXTI_EMR_MR1_Msk                             /*!< Event Mask on line 1 */
#define EXTI_EMR_MR2_Pos          (2U)
#define EXTI_EMR_MR2_Msk          (0x1UL << EXTI_EMR_MR2_Pos)                   /*!< 0x00000004 */
#define EXTI_EMR_MR2              EXTI_EMR_MR2_Msk                             /*!< Event Mask on line 2 */
#define EXTI_EMR_MR3_Pos          (3U)
#define EXTI_EMR_MR3_Msk          (0x1UL << EXTI_EMR_MR3_Pos)                   /*!< 0x00000008 */
#define EXTI_EMR_MR3              EXTI_EMR_MR3_Msk                             /*!< Event Mask on line 3 */
#define EXTI_EMR_MR4_Pos          (4U)
#define EXTI_EMR_MR4_Msk          (0x1UL << EXTI_EMR_MR4_Pos)                   /*!< 0x00000010 */
#define EXTI_EMR_MR4              EXTI_EMR_MR4_Msk                             /*!< Event Mask on line 4 */
#define EXTI_EMR_MR5_Pos          (5U)
#define EXTI_EMR_MR5_Msk          (0x1UL << EXTI_EMR_MR5_Pos)                   /*!< 0x00000020 */
#define EXTI_EMR_MR5              EXTI_EMR_MR5_Msk                             /*!< Event Mask on line 5 */
#define EXTI_EMR_MR6_Pos          (6U)
#define EXTI_EMR_MR6_Msk          (0x1UL << EXTI_EMR_MR6_Pos)                   /*!< 0x00000040 */
#define EXTI_EMR_MR6              EXTI_EMR_MR6_Msk                             /*!< Event Mask on line 6 */
#define EXTI_EMR_MR7_Pos          (7U)
#define EXTI_EMR_MR7_Msk          (0x1UL << EXTI_EMR_MR7_Pos)                   /*!< 0x00000080 */
#define EXTI_EMR_MR7              EXTI_EMR_MR7_Msk                             /*!< Event Mask on line 7 */
#define EXTI_EMR_MR8_Pos          (8U)
#define EXTI_EMR_MR8_Msk          (0x1UL << EXTI_EMR_MR8_Pos)                   /*!< 0x00000100 */
#define EXTI_EMR_MR8              EXTI_EMR_MR8_Msk                             /*!< Event Mask on line 8 */
#define EXTI_EMR_MR9_Pos          (9U)
#define EXTI_EMR_MR9_Msk          (0x1UL << EXTI_EMR_MR9_Pos)                   /*!< 0x00000200 */
#define EXTI_EMR_MR9              EXTI_EMR_MR9_Msk                             /*!< Event Mask on line 9 */
#define EXTI_EMR_MR10_Pos         (10U)
#define EXTI_EMR_MR10_Msk         (0x1UL << EXTI_EMR_MR10_Pos)                  /*!< 0x00000400 */
#define EXTI_EMR_MR10             EXTI_EMR_MR10_Msk                            /*!< Event Mask on line 10 */
#define EXTI_EMR_MR11_Pos         (11U)
#define EXTI_EMR_MR11_Msk         (0x1UL << EXTI_EMR_MR11_Pos)                  /*!< 0x00000800 */
#define EXTI_EMR_MR11             EXTI_EMR_MR11_Msk                            /*!< Event Mask on line 11 */
#define EXTI_EMR_MR12_Pos         (12U)
#define EXTI_EMR_MR12_Msk         (0x1UL << EXTI_EMR_MR12_Pos)                  /*!< 0x00001000 */
#define EXTI_EMR_MR12             EXTI_EMR_MR12_Msk                            /*!< Event Mask on line 12 */
#define EXTI_EMR_MR13_Pos         (13U)
#define EXTI_EMR_MR13_Msk         (0x1UL << EXTI_EMR_MR13_Pos)                  /*!< 0x00002000 */
#define EXTI_EMR_MR13             EXTI_EMR_MR13_Msk                            /*!< Event Mask on line 13 */
#define EXTI_EMR_MR14_Pos         (14U)
#define EXTI_EMR_MR14_Msk         (0x1UL << EXTI_EMR_MR14_Pos)                  /*!< 0x00004000 */
#define EXTI_EMR_MR14             EXTI_EMR_MR14_Msk                            /*!< Event Mask on line 14 */
#define EXTI_EMR_MR15_Pos         (15U)
#define EXTI_EMR_MR15_Msk         (0x1UL << EXTI_EMR_MR15_Pos)                  /*!< 0x00008000 */
#define EXTI_EMR_MR15             EXTI_EMR_MR15_Msk                            /*!< Event Mask on line 15 */
#define EXTI_EMR_MR16_Pos         (16U)
#define EXTI_EMR_MR16_Msk         (0x1UL << EXTI_EMR_MR16_Pos)                  /*!< 0x00010000 */
#define EXTI_EMR_MR16             EXTI_EMR_MR16_Msk                            /*!< Event Mask on line 16 */
#define EXTI_EMR_MR17_Pos         (17U)
#define EXTI_EMR_MR17_Msk         (0x1UL << EXTI_EMR_MR17_Pos)                  /*!< 0x00020000 */
#define EXTI_EMR_MR17             EXTI_EMR_MR17_Msk                            /*!< Event Mask on line 17 */
#define EXTI_EMR_MR18_Pos         (18U)
#define EXTI_EMR_MR18_Msk         (0x1UL << EXTI_EMR_MR18_Pos)                  /*!< 0x00040000 */
#define EXTI_EMR_MR18             EXTI_EMR_MR18_Msk                            /*!< Event Mask on line 18 */
#define EXTI_EMR_MR19_Pos         (19U)
#define EXTI_EMR_MR19_Msk         (0x1UL << EXTI_EMR_MR19_Pos)                  /*!< 0x00080000 */
#define EXTI_EMR_MR19             EXTI_EMR_MR19_Msk                            /*!< Event Mask on line 19 */
#define EXTI_EMR_MR20_Pos         (20U)
#define EXTI_EMR_MR20_Msk         (0x1UL << EXTI_EMR_MR20_Pos)                  /*!< 0x00100000 */
#define EXTI_EMR_MR20             EXTI_EMR_MR20_Msk                            /*!< Event Mask on line 20 */
#define EXTI_EMR_MR21_Pos         (21U)
#define EXTI_EMR_MR21_Msk         (0x1UL << EXTI_EMR_MR21_Pos)                  /*!< 0x00200000 */
#define EXTI_EMR_MR21             EXTI_EMR_MR21_Msk                            /*!< Event Mask on line 21 */
#define EXTI_EMR_MR22_Pos         (22U)
#define EXTI_EMR_MR22_Msk         (0x1UL << EXTI_EMR_MR22_Pos)                  /*!< 0x00400000 */
#define EXTI_EMR_MR22             EXTI_EMR_MR22_Msk                            /*!< Event Mask on line 22 */

/* Reference Defines */
#define  EXTI_EMR_EM0                        EXTI_EMR_MR0
#define  EXTI_EMR_EM1                        EXTI_EMR_MR1
#define  EXTI_EMR_EM2                        EXTI_EMR_MR2
#define  EXTI_EMR_EM3                        EXTI_EMR_MR3
#define  EXTI_EMR_EM4                        EXTI_EMR_MR4
#define  EXTI_EMR_EM5                        EXTI_EMR_MR5
#define  EXTI_EMR_EM6                        EXTI_EMR_MR6
#define  EXTI_EMR_EM7                        EXTI_EMR_MR7
#define  EXTI_EMR_EM8                        EXTI_EMR_MR8
#define  EXTI_EMR_EM9                        EXTI_EMR_MR9
#define  EXTI_EMR_EM10                       EXTI_EMR_MR10
#define  EXTI_EMR_EM11                       EXTI_EMR_MR11
#define  EXTI_EMR_EM12                       EXTI_EMR_MR12
#define  EXTI_EMR_EM13                       EXTI_EMR_MR13
#define  EXTI_EMR_EM14                       EXTI_EMR_MR14
#define  EXTI_EMR_EM15                       EXTI_EMR_MR15
#define  EXTI_EMR_EM16                       EXTI_EMR_MR16
#define  EXTI_EMR_EM17                       EXTI_EMR_MR17
#define  EXTI_EMR_EM18                       EXTI_EMR_MR18
#define  EXTI_EMR_EM19                       EXTI_EMR_MR19
#define  EXTI_EMR_EM20                       EXTI_EMR_MR20
#define  EXTI_EMR_EM21                       EXTI_EMR_MR21
#define  EXTI_EMR_EM22                       EXTI_EMR_MR22

/******************  Bit definition for EXTI_RTSR register  *******************/
#define EXTI_RTSR_TR0_Pos         (0U)
#define EXTI_RTSR_TR0_Msk         (0x1UL << EXTI_RTSR_TR0_Pos)                  /*!< 0x00000001 */
#define EXTI_RTSR_TR0             EXTI_RTSR_TR0_Msk                            /*!< Rising trigger event configuration bit of line 0 */
#define EXTI_RTSR_TR1_Pos         (1U)
#define EXTI_RTSR_TR1_Msk         (0x1UL << EXTI_RTSR_TR1_Pos)                  /*!< 0x00000002 */
#define EXTI_RTSR_TR1             EXTI_RTSR_TR1_Msk                            /*!< Rising trigger event configuration bit of line 1 */
#define EXTI_RTSR_TR2_Pos         (2U)
#define EXTI_RTSR_TR2_Msk         (0x1UL << EXTI_RTSR_TR2_Pos)                  /*!< 0x00000004 */
#define EXTI_RTSR_TR2             EXTI_RTSR_TR2_Msk                            /*!< Rising trigger event configuration bit of line 2 */
#define EXTI_RTSR_TR3_Pos         (3U)
#define EXTI_RTSR_TR3_Msk         (0x1UL << EXTI_RTSR_TR3_Pos)                  /*!< 0x00000008 */
#define EXTI_RTSR_TR3             EXTI_RTSR_TR3_Msk                            /*!< Rising trigger event configuration bit of line 3 */
#define EXTI_RTSR_TR4_Pos         (4U)
#define EXTI_RTSR_TR4_Msk         (0x1UL << EXTI_RTSR_TR4_Pos)                  /*!< 0x00000010 */
#define EXTI_RTSR_TR4             EXTI_RTSR_TR4_Msk                            /*!< Rising trigger event configuration bit of line 4 */
#define EXTI_RTSR_TR5_Pos         (5U)
#define EXTI_RTSR_TR5_Msk         (0x1UL << EXTI_RTSR_TR5_Pos)                  /*!< 0x00000020 */
#define EXTI_RTSR_TR5             EXTI_RTSR_TR5_Msk                            /*!< Rising trigger event configuration bit of line 5 */
#define EXTI_RTSR_TR6_Pos         (6U)
#define EXTI_RTSR_TR6_Msk         (0x1UL << EXTI_RTSR_TR6_Pos)                  /*!< 0x00000040 */
#define EXTI_RTSR_TR6             EXTI_RTSR_TR6_Msk                            /*!< Rising trigger event configuration bit of line 6 */
#define EXTI_RTSR_TR7_Pos         (7U)
#define EXTI_RTSR_TR7_Msk         (0x1UL << EXTI_RTSR_TR7_Pos)                  /*!< 0x00000080 */
#define EXTI_RTSR_TR7             EXTI_RTSR_TR7_Msk                            /*!< Rising trigger event configuration bit of line 7 */
#define EXTI_RTSR_TR8_Pos         (8U)
#define EXTI_RTSR_TR8_Msk         (0x1UL << EXTI_RTSR_TR8_Pos)                  /*!< 0x00000100 */
#define EXTI_RTSR_TR8             EXTI_RTSR_TR8_Msk                            /*!< Rising trigger event configuration bit of line 8 */
#define EXTI_RTSR_TR9_Pos         (9U)
#define EXTI_RTSR_TR9_Msk         (0x1UL << EXTI_RTSR_TR9_Pos)                  /*!< 0x00000200 */
#define EXTI_RTSR_TR9             EXTI_RTSR_TR9_Msk                            /*!< Rising trigger event configuration bit of line 9 */
#define EXTI_RTSR_TR10_Pos        (10U)
#define EXTI_RTSR_TR10_Msk        (0x1UL << EXTI_RTSR_TR10_Pos)                 /*!< 0x00000400 */
#define EXTI_RTSR_TR10            EXTI_RTSR_TR10_Msk                           /*!< Rising trigger event configuration bit of line 10 */
#define EXTI_RTSR_TR11_Pos        (11U)
#define EXTI_RTSR_TR11_Msk        (0x1UL << EXTI_RTSR_TR11_Pos)                 /*!< 0x00000800 */
#define EXTI_RTSR_TR11            EXTI_RTSR_TR11_Msk                           /*!< Rising trigger event configuration bit of line 11 */
#define EXTI_RTSR_TR12_Pos        (12U)
#define EXTI_RTSR_TR12_Msk        (0x1UL << EXTI_RTSR_TR12_Pos)                 /*!< 0x00001000 */
#define EXTI_RTSR_TR12            EXTI_RTSR_TR12_Msk                           /*!< Rising trigger event configuration bit of line 12 */
#define EXTI_RTSR_TR13_Pos        (13U)
#define EXTI_RTSR_TR13_Msk        (0x1UL << EXTI_RTSR_TR13_Pos)                 /*!< 0x00002000 */
#define EXTI_RTSR_TR13            EXTI_RTSR_TR13_Msk                           /*!< Rising trigger event configuration bit of line 13 */
#define EXTI_RTSR_TR14_Pos        (14U)
#define EXTI_RTSR_TR14_Msk        (0x1UL << EXTI_RTSR_TR14_Pos)                 /*!< 0x00004000 */
#define EXTI_RTSR_TR14            EXTI_RTSR_TR14_Msk                           /*!< Rising trigger event configuration bit of line 14 */
#define EXTI_RTSR_TR15_Pos        (15U)
#define EXTI_RTSR_TR15_Msk        (0x1UL << EXTI_RTSR_TR15_Pos)                 /*!< 0x00008000 */
#define EXTI_RTSR_TR15            EXTI_RTSR_TR15_Msk                           /*!< Rising trigger event configuration bit of line 15 */
#define EXTI_RTSR_TR16_Pos        (16U)
#define EXTI_RTSR_TR16_Msk        (0x1UL << EXTI_RTSR_TR16_Pos)                 /*!< 0x00010000 */
#define EXTI_RTSR_TR16            EXTI_RTSR_TR16_Msk                           /*!< Rising trigger event configuration bit of line 16 */
#define EXTI_RTSR_TR17_Pos        (17U)
#define EXTI_RTSR_TR17_Msk        (0x1UL << EXTI_RTSR_TR17_Pos)                 /*!< 0x00020000 */
#define EXTI_RTSR_TR17            EXTI_RTSR_TR17_Msk                           /*!< Rising trigger event configuration bit of line 17 */
#define EXTI_RTSR_TR18_Pos        (18U)
#define EXTI_RTSR_TR18_Msk        (0x1UL << EXTI_RTSR_TR18_Pos)                 /*!< 0x00040000 */
#define EXTI_RTSR_TR18            EXTI_RTSR_TR18_Msk                           /*!< Rising trigger event configuration bit of line 18 */
#define EXTI_RTSR_TR19_Pos        (19U)
#define EXTI_RTSR_TR19_Msk        (0x1UL << EXTI_RTSR_TR19_Pos)                 /*!< 0x00080000 */
#define EXTI_RTSR_TR19            EXTI_RTSR_TR19_Msk                           /*!< Rising trigger event configuration bit of line 19 */
#define EXTI_RTSR_TR20_Pos        (20U)
#define EXTI_RTSR_TR20_Msk        (0x1UL << EXTI_RTSR_TR20_Pos)                 /*!< 0x00100000 */
#define EXTI_RTSR_TR20            EXTI_RTSR_TR20_Msk                           /*!< Rising trigger event configuration bit of line 20 */
#define EXTI_RTSR_TR21_Pos        (21U)
#define EXTI_RTSR_TR21_Msk        (0x1UL << EXTI_RTSR_TR21_Pos)                 /*!< 0x00200000 */
#define EXTI_RTSR_TR21            EXTI_RTSR_TR21_Msk                           /*!< Rising trigger event configuration bit of line 21 */
#define EXTI_RTSR_TR22_Pos        (22U)
#define EXTI_RTSR_TR22_Msk        (0x1UL << EXTI_RTSR_TR22_Pos)                 /*!< 0x00400000 */
#define EXTI_RTSR_TR22            EXTI_RTSR_TR22_Msk                           /*!< Rising trigger event configuration bit of line 22 */

/******************  Bit definition for EXTI_FTSR register  *******************/
#define EXTI_FTSR_TR0_Pos         (0U)
#define EXTI_FTSR_TR0_Msk         (0x1UL << EXTI_FTSR_TR0_Pos)                  /*!< 0x00000001 */
#define EXTI_FTSR_TR0             EXTI_FTSR_TR0_Msk                            /*!< Falling trigger event configuration bit of line 0 */
#define EXTI_FTSR_TR1_Pos         (1U)
#define EXTI_FTSR_TR1_Msk         (0x1UL << EXTI_FTSR_TR1_Pos)                  /*!< 0x00000002 */
#define EXTI_FTSR_TR1             EXTI_FTSR_TR1_Msk                            /*!< Falling trigger event configuration bit of line 1 */
#define EXTI_FTSR_TR2_Pos         (2U)
#define EXTI_FTSR_TR2_Msk         (0x1UL << EXTI_FTSR_TR2_Pos)                  /*!< 0x00000004 */
#define EXTI_FTSR_TR2             EXTI_FTSR_TR2_Msk                            /*!< Falling trigger event configuration bit of line 2 */
#define EXTI_FTSR_TR3_Pos         (3U)
#define EXTI_FTSR_TR3_Msk         (0x1UL << EXTI_FTSR_TR3_Pos)                  /*!< 0x00000008 */
#define EXTI_FTSR_TR3             EXTI_FTSR_TR3_Msk                            /*!< Falling trigger event configuration bit of line 3 */
#define EXTI_FTSR_TR4_Pos         (4U)
#define EXTI_FTSR_TR4_Msk         (0x1UL << EXTI_FTSR_TR4_Pos)                  /*!< 0x00000010 */
#define EXTI_FTSR_TR4             EXTI_FTSR_TR4_Msk                            /*!< Falling trigger event configuration bit of line 4 */
#define EXTI_FTSR_TR5_Pos         (5U)
#define EXTI_FTSR_TR5_Msk         (0x1UL << EXTI_FTSR_TR5_Pos)                  /*!< 0x00000020 */
#define EXTI_FTSR_TR5             EXTI_FTSR_TR5_Msk                            /*!< Falling trigger event configuration bit of line 5 */
#define EXTI_FTSR_TR6_Pos         (6U)
#define EXTI_FTSR_TR6_Msk         (0x1UL << EXTI_FTSR_TR6_Pos)                  /*!< 0x00000040 */
#define EXTI_FTSR_TR6             EXTI_FTSR_TR6_Msk                            /*!< Falling trigger event configuration bit of line 6 */
#define EXTI_FTSR_TR7_Pos         (7U)
#define EXTI_FTSR_TR7_Msk         (0x1UL << EXTI_FTSR_TR7_Pos)                  /*!< 0x00000080 */
#define EXTI_FTSR_TR7             EXTI_FTSR_TR7_Msk                            /*!< Falling trigger event configuration bit of line 7 */
#define EXTI_FTSR_TR8_Pos         (8U)
#define EXTI_FTSR_TR8_Msk         (0x1UL << EXTI_FTSR_TR8_Pos)                  /*!< 0x00000100 */
#define EXTI_FTSR_TR8             EXTI_FTSR_TR8_Msk                            /*!< Falling trigger event configuration bit of line 8 */
#define EXTI_FTSR_TR9_Pos         (9U)
#define EXTI_FTSR_TR9_Msk         (0x1UL << EXTI_FTSR_TR9_Pos)                  /*!< 0x00000200 */
#define EXTI_FTSR_TR9             EXTI_FTSR_TR9_Msk                            /*!< Falling trigger event configuration bit of line 9 */
#define EXTI_FTSR_TR10_Pos        (10U)
#define EXTI_FTSR_TR10_Msk        (0x1UL << EXTI_FTSR_TR10_Pos)                 /*!< 0x00000400 */
#define EXTI_FTSR_TR10            EXTI_FTSR_TR10_Msk                           /*!< Falling trigger event configuration bit of line 10 */
#define EXTI_FTSR_TR11_Pos        (11U)
#define EXTI_FTSR_TR11_Msk        (0x1UL << EXTI_FTSR_TR11_Pos)                 /*!< 0x00000800 */
#define EXTI_FTSR_TR11            EXTI_FTSR_TR11_Msk                           /*!< Falling trigger event configuration bit of line 11 */
#define EXTI_FTSR_TR12_Pos        (12U)
#define EXTI_FTSR_TR12_Msk        (0x1UL << EXTI_FTSR_TR12_Pos)                 /*!< 0x00001000 */
#define EXTI_FTSR_TR12            EXTI_FTSR_TR12_Msk                           /*!< Falling trigger event configuration bit of line 12 */
#define EXTI_FTSR_TR13_Pos        (13U)
#define EXTI_FTSR_TR13_Msk        (0x1UL << EXTI_FTSR_TR13_Pos)                 /*!< 0x00002000 */
#define EXTI_FTSR_TR13            EXTI_FTSR_TR13_Msk                           /*!< Falling trigger event configuration bit of line 13 */
#define EXTI_FTSR_TR14_Pos        (14U)
#define EXTI_FTSR_TR14_Msk        (0x1UL << EXTI_FTSR_TR14_Pos)                 /*!< 0x00004000 */
#define EXTI_FTSR_TR14            EXTI_FTSR_TR14_Msk                           /*!< Falling trigger event configuration bit of line 14 */
#define EXTI_FTSR_TR15_Pos        (15U)
#define EXTI_FTSR_TR15_Msk        (0x1UL << EXTI_FTSR_TR15_Pos)                 /*!< 0x00008000 */
#define EXTI_FTSR_TR15            EXTI_FTSR_TR15_Msk                           /*!< Falling trigger event configuration bit of line 15 */
#define EXTI_FTSR_TR16_Pos        (16U)
#define EXTI_FTSR_TR16_Msk        (0x1UL << EXTI_FTSR_TR16_Pos)                 /*!< 0x00010000 */
#define EXTI_FTSR_TR16            EXTI_FTSR_TR16_Msk                           /*!< Falling trigger event configuration bit of line 16 */
#define EXTI_FTSR_TR17_Pos        (17U)
#define EXTI_FTSR_TR17_Msk        (0x1UL << EXTI_FTSR_TR17_Pos)                 /*!< 0x00020000 */
#define EXTI_FTSR_TR17            EXTI_FTSR_TR17_Msk                           /*!< Falling trigger event configuration bit of line 17 */
#define EXTI_FTSR_TR18_Pos        (18U)
#define EXTI_FTSR_TR18_Msk        (0x1UL << EXTI_FTSR_TR18_Pos)                 /*!< 0x00040000 */
#define EXTI_FTSR_TR18            EXTI_FTSR_TR18_Msk                           /*!< Falling trigger event configuration bit of line 18 */
#define EXTI_FTSR_TR19_Pos        (19U)
#define EXTI_FTSR_TR19_Msk        (0x1UL << EXTI_FTSR_TR19_Pos)                 /*!< 0x00080000 */
#define EXTI_FTSR_TR19            EXTI_FTSR_TR19_Msk                           /*!< Falling trigger event configuration bit of line 19 */
#define EXTI_FTSR_TR20_Pos        (20U)
#define EXTI_FTSR_TR20_Msk        (0x1UL << EXTI_FTSR_TR20_Pos)                 /*!< 0x00100000 */
#define EXTI_FTSR_TR20            EXTI_FTSR_TR20_Msk                           /*!< Falling trigger event configuration bit of line 20 */
#define EXTI_FTSR_TR21_Pos        (21U)
#define EXTI_FTSR_TR21_Msk        (0x1UL << EXTI_FTSR_TR21_Pos)                 /*!< 0x00200000 */
#define EXTI_FTSR_TR21            EXTI_FTSR_TR21_Msk                           /*!< Falling trigger event configuration bit of line 21 */
#define EXTI_FTSR_TR22_Pos        (22U)
#define EXTI_FTSR_TR22_Msk        (0x1UL << EXTI_FTSR_TR22_Pos)                 /*!< 0x00400000 */
#define EXTI_FTSR_TR22            EXTI_FTSR_TR22_Msk                           /*!< Falling trigger event configuration bit of line 22 */

/******************  Bit definition for EXTI_SWIER register  ******************/
#define EXTI_SWIER_SWIER0_Pos     (0U)
#define EXTI_SWIER_SWIER0_Msk     (0x1UL << EXTI_SWIER_SWIER0_Pos)              /*!< 0x00000001 */
#define EXTI_SWIER_SWIER0         EXTI_SWIER_SWIER0_Msk                        /*!< Software Interrupt on line 0 */
#define EXTI_SWIER_SWIER1_Pos     (1U)
#define EXTI_SWIER_SWIER1_Msk     (0x1UL << EXTI_SWIER_SWIER1_Pos)              /*!< 0x00000002 */
#define EXTI_SWIER_SWIER1         EXTI_SWIER_SWIER1_Msk                        /*!< Software Interrupt on line 1 */
#define EXTI_SWIER_SWIER2_Pos     (2U)
#define EXTI_SWIER_SWIER2_Msk     (0x1UL << EXTI_SWIER_SWIER2_Pos)              /*!< 0x00000004 */
#define EXTI_SWIER_SWIER2         EXTI_SWIER_SWIER2_Msk                        /*!< Software Interrupt on line 2 */
#define EXTI_SWIER_SWIER3_Pos     (3U)
#define EXTI_SWIER_SWIER3_Msk     (0x1UL << EXTI_SWIER_SWIER3_Pos)              /*!< 0x00000008 */
#define EXTI_SWIER_SWIER3         EXTI_SWIER_SWIER3_Msk                        /*!< Software Interrupt on line 3 */
#define EXTI_SWIER_SWIER4_Pos     (4U)
#define EXTI_SWIER_SWIER4_Msk     (0x1UL << EXTI_SWIER_SWIER4_Pos)              /*!< 0x00000010 */
#define EXTI_SWIER_SWIER4         EXTI_SWIER_SWIER4_Msk                        /*!< Software Interrupt on line 4 */
#define EXTI_SWIER_SWIER5_Pos     (5U)
#define EXTI_SWIER_SWIER5_Msk     (0x1UL << EXTI_SWIER_SWIER5_Pos)              /*!< 0x00000020 */
#define EXTI_SWIER_SWIER5         EXTI_SWIER_SWIER5_Msk                        /*!< Software Interrupt on line 5 */
#define EXTI_SWIER_SWIER6_Pos     (6U)
#define EXTI_SWIER_SWIER6_Msk     (0x1UL << EXTI_SWIER_SWIER6_Pos)              /*!< 0x00000040 */
#define EXTI_SWIER_SWIER6         EXTI_SWIER_SWIER6_Msk                        /*!< Software Interrupt on line 6 */
#define EXTI_SWIER_SWIER7_Pos     (7U)
#define EXTI_SWIER_SWIER7_Msk     (0x1UL << EXTI_SWIER_SWIER7_Pos)              /*!< 0x00000080 */
#define EXTI_SWIER_SWIER7         EXTI_SWIER_SWIER7_Msk                        /*!< Software Interrupt on line 7 */
#define EXTI_SWIER_SWIER8_Pos     (8U)
#define EXTI_SWIER_SWIER8_Msk     (0x1UL << EXTI_SWIER_SWIER8_Pos)              /*!< 0x00000100 */
#define EXTI_SWIER_SWIER8         EXTI_SWIER_SWIER8_Msk                        /*!< Software Interrupt on line 8 */
#define EXTI_SWIER_SWIER9_Pos     (9U)
#define EXTI_SWIER_SWIER9_Msk     (0x1UL << EXTI_SWIER_SWIER9_Pos)              /*!< 0x00000200 */
#define EXTI_SWIER_SWIER9         EXTI_SWIER_SWIER9_Msk                        /*!< Software Interrupt on line 9 */
#define EXTI_SWIER_SWIER10_Pos    (10U)
#define EXTI_SWIER_SWIER10_Msk    (0x1UL << EXTI_SWIER_SWIER10_Pos)             /*!< 0x00000400 */
#define EXTI_SWIER_SWIER10        EXTI_SWIER_SWIER10_Msk                       /*!< Software Interrupt on line 10 */
#define EXTI_SWIER_SWIER11_Pos    (11U)
#define EXTI_SWIER_SWIER11_Msk    (0x1UL << EXTI_SWIER_SWIER11_Pos)             /*!< 0x00000800 */
#define EXTI_SWIER_SWIER11        EXTI_SWIER_SWIER11_Msk                       /*!< Software Interrupt on line 11 */
#define EXTI_SWIER_SWIER12_Pos    (12U)
#define EXTI_SWIER_SWIER12_Msk    (0x1UL << EXTI_SWIER_SWIER12_Pos)             /*!< 0x00001000 */
#define EXTI_SWIER_SWIER12        EXTI_SWIER_SWIER12_Msk                       /*!< Software Interrupt on line 12 */
#define EXTI_SWIER_SWIER13_Pos    (13U)
#define EXTI_SWIER_SWIER13_Msk    (0x1UL << EXTI_SWIER_SWIER13_Pos)             /*!< 0x00002000 */
#define EXTI_SWIER_SWIER13        EXTI_SWIER_SWIER13_Msk                       /*!< Software Interrupt on line 13 */
#define EXTI_SWIER_SWIER14_Pos    (14U)
#define EXTI_SWIER_SWIER14_Msk    (0x1UL << EXTI_SWIER_SWIER14_Pos)             /*!< 0x00004000 */
#define EXTI_SWIER_SWIER14        EXTI_SWIER_SWIER14_Msk                       /*!< Software Interrupt on line 14 */
#define EXTI_SWIER_SWIER15_Pos    (15U)
#define EXTI_SWIER_SWIER15_Msk    (0x1UL << EXTI_SWIER_SWIER15_Pos)             /*!< 0x00008000 */
#define EXTI_SWIER_SWIER15        EXTI_SWIER_SWIER15_Msk                       /*!< Software Interrupt on line 15 */
#define EXTI_SWIER_SWIER16_Pos    (16U)
#define EXTI_SWIER_SWIER16_Msk    (0x1UL << EXTI_SWIER_SWIER16_Pos)             /*!< 0x00010000 */
#define EXTI_SWIER_SWIER16        EXTI_SWIER_SWIER16_Msk                       /*!< Software Interrupt on line 16 */
#define EXTI_SWIER_SWIER17_Pos    (17U)
#define EXTI_SWIER_SWIER17_Msk    (0x1UL << EXTI_SWIER_SWIER17_Pos)             /*!< 0x00020000 */
#define EXTI_SWIER_SWIER17        EXTI_SWIER_SWIER17_Msk                       /*!< Software Interrupt on line 17 */
#define EXTI_SWIER_SWIER18_Pos    (18U)
#define EXTI_SWIER_SWIER18_Msk    (0x1UL << EXTI_SWIER_SWIER18_Pos)             /*!< 0x00040000 */
#define EXTI_SWIER_SWIER18        EXTI_SWIER_SWIER18_Msk                       /*!< Software Interrupt on line 18 */
#define EXTI_SWIER_SWIER19_Pos    (19U)
#define EXTI_SWIER_SWIER19_Msk    (0x1UL << EXTI_SWIER_SWIER19_Pos)             /*!< 0x00080000 */
#define EXTI_SWIER_SWIER19        EXTI_SWIER_SWIER19_Msk                       /*!< Software Interrupt on line 19 */
#define EXTI_SWIER_SWIER20_Pos    (20U)
#define EXTI_SWIER_SWIER20_Msk    (0x1UL << EXTI_SWIER_SWIER20_Pos)             /*!< 0x00100000 */
#define EXTI_SWIER_SWIER20        EXTI_SWIER_SWIER20_Msk                       /*!< Software Interrupt on line 20 */
#define EXTI_SWIER_SWIER21_Pos    (21U)
#define EXTI_SWIER_SWIER21_Msk    (0x1UL << EXTI_SWIER_SWIER21_Pos)             /*!< 0x00200000 */
#define EXTI_SWIER_SWIER21        EXTI_SWIER_SWIER21_Msk                       /*!< Software Interrupt on line 21 */
#define EXTI_SWIER_SWIER22_Pos    (22U)
#define EXTI_SWIER_SWIER22_Msk    (0x1UL << EXTI_SWIER_SWIER22_Pos)             /*!< 0x00400000 */
#define EXTI_SWIER_SWIER22        EXTI_SWIER_SWIER22_Msk                       /*!< Software Interrupt on line 22 */

/*******************  Bit definition for EXTI_PR register  ********************/
#define EXTI_PR_PR0_Pos           (0U)
#define EXTI_PR_PR0_Msk           (0x1UL << EXTI_PR_PR0_Pos)                    /*!< 0x00000001 */
#define EXTI_PR_PR0               EXTI_PR_PR0_Msk                              /*!< Pending bit for line 0 */
#define EXTI_PR_PR1_Pos           (1U)
#define EXTI_PR_PR1_Msk           (0x1UL << EXTI_PR_PR1_Pos)                    /*!< 0x00000002 */
#define EXTI_PR_PR1               EXTI_PR_PR1_Msk                              /*!< Pending bit for line 1 */
#define EXTI_PR_PR2_Pos           (2U)
#define EXTI_PR_PR2_Msk           (0x1UL << EXTI_PR_PR2_Pos)                    /*!< 0x00000004 */
#define EXTI_PR_PR2               EXTI_PR_PR2_Msk                              /*!< Pending bit for line 2 */
#define EXTI_PR_PR3_Pos           (3U)
#define EXTI_PR_PR3_Msk           (0x1UL << EXTI_PR_PR3_Pos)                    /*!< 0x00000008 */
#define EXTI_PR_PR3               EXTI_PR_PR3_Msk                              /*!< Pending bit for line 3 */
#define EXTI_PR_PR4_Pos           (4U)
#define EXTI_PR_PR4_Msk           (0x1UL << EXTI_PR_PR4_Pos)                    /*!< 0x00000010 */
#define EXTI_PR_PR4               EXTI_PR_PR4_Msk                              /*!< Pending bit for line 4 */
#define EXTI_PR_PR5_Pos           (5U)
#define EXTI_PR_PR5_Msk           (0x1UL << EXTI_PR_PR5_Pos)                    /*!< 0x00000020 */
#define EXTI_PR_PR5               EXTI_PR_PR5_Msk                              /*!< Pending bit for line 5 */
#define EXTI_PR_PR6_Pos           (6U)
#define EXTI_PR_PR6_Msk           (0x1UL << EXTI_PR_PR6_Pos)                    /*!< 0x00000040 */
#define EXTI_PR_PR6               EXTI_PR_PR6_Msk                              /*!< Pending bit for line 6 */
#define EXTI_PR_PR7_Pos           (7U)
#define EXTI_PR_PR7_Msk           (0x1UL << EXTI_PR_PR7_Pos)                    /*!< 0x00000080 */
#define EXTI_PR_PR7               EXTI_PR_PR7_Msk                              /*!< Pending bit for line 7 */
#define EXTI_PR_PR8_Pos           (8U)
#define EXTI_PR_PR8_Msk           (0x1UL << EXTI_PR_PR8_Pos)                    /*!< 0x00000100 */
#define EXTI_PR_PR8               EXTI_PR_PR8_Msk                              /*!< Pending bit for line 8 */
#define EXTI_PR_PR9_Pos           (9U)
#define EXTI_PR_PR9_Msk           (0x1UL << EXTI_PR_PR9_Pos)                    /*!< 0x00000200 */
#define EXTI_PR_PR9               EXTI_PR_PR9_Msk                              /*!< Pending bit for line 9 */
#define EXTI_PR_PR10_Pos          (10U)
#define EXTI_PR_PR10_Msk          (0x1UL << EXTI_PR_PR10_Pos)                   /*!< 0x00000400 */
#define EXTI_PR_PR10              EXTI_PR_PR10_Msk                             /*!< Pending bit for line 10 */
#define EXTI_PR_PR11_Pos          (11U)
#define EXTI_PR_PR11_Msk          (0x1UL << EXTI_PR_PR11_Pos)                   /*!< 0x00000800 */
#define EXTI_PR_PR11              EXTI_PR_PR11_Msk                             /*!< Pending bit for line 11 */
#define EXTI_PR_PR12_Pos          (12U)
#define EXTI_PR_PR12_Msk          (0x1UL << EXTI_PR_PR12_Pos)                   /*!< 0x00001000 */
#define EXTI_PR_PR12              EXTI_PR_PR12_Msk                             /*!< Pending bit for line 12 */
#define EXTI_PR_PR13_Pos          (13U)
#define EXTI_PR_PR13_Msk          (0x1UL << EXTI_PR_PR13_Pos)                   /*!< 0x00002000 */
#define EXTI_PR_PR13              EXTI_PR_PR13_Msk                             /*!< Pending bit for line 13 */
#define EXTI_PR_PR14_Pos          (14U)
#define EXTI_PR_PR14_Msk          (0x1UL << EXTI_PR_PR14_Pos)                   /*!< 0x00004000 */
#define EXTI_PR_PR14              EXTI_PR_PR14_Msk                             /*!< Pending bit for line 14 */
#define EXTI_PR_PR15_Pos          (15U)
#define EXTI_PR_PR15_Msk          (0x1UL << EXTI_PR_PR15_Pos)                   /*!< 0x00008000 */
#define EXTI_PR_PR15              EXTI_PR_PR15_Msk                             /*!< Pending bit for line 15 */
#define EXTI_PR_PR16_Pos          (16U)
#define EXTI_PR_PR16_Msk          (0x1UL << EXTI_PR_PR16_Pos)                   /*!< 0x00010000 */
#define EXTI_PR_PR16              EXTI_PR_PR16_Msk                             /*!< Pending bit for line 16 */
#define EXTI_PR_PR17_Pos          (17U)
#define EXTI_PR_PR17_Msk          (0x1UL << EXTI_PR_PR17_Pos)                   /*!< 0x00020000 */
#define EXTI_PR_PR17              EXTI_PR_PR17_Msk                             /*!< Pending bit for line 17 */
#define EXTI_PR_PR18_Pos          (18U)
#define EXTI_PR_PR18_Msk          (0x1UL << EXTI_PR_PR18_Pos)                   /*!< 0x00040000 */
#define EXTI_PR_PR18              EXTI_PR_PR18_Msk                             /*!< Pending bit for line 18 */
#define EXTI_PR_PR19_Pos          (19U)
#define EXTI_PR_PR19_Msk          (0x1UL << EXTI_PR_PR19_Pos)                   /*!< 0x00080000 */
#define EXTI_PR_PR19              EXTI_PR_PR19_Msk                             /*!< Pending bit for line 19 */
#define EXTI_PR_PR20_Pos          (20U)
#define EXTI_PR_PR20_Msk          (0x1UL << EXTI_PR_PR20_Pos)                   /*!< 0x00100000 */
#define EXTI_PR_PR20              EXTI_PR_PR20_Msk                             /*!< Pending bit for line 20 */
#define EXTI_PR_PR21_Pos          (21U)
#define EXTI_PR_PR21_Msk          (0x1UL << EXTI_PR_PR21_Pos)                   /*!< 0x00200000 */
#define EXTI_PR_PR21              EXTI_PR_PR21_Msk                             /*!< Pending bit for line 21 */
#define EXTI_PR_PR22_Pos          (22U)
#define EXTI_PR_PR22_Msk          (0x1UL << EXTI_PR_PR22_Pos)                   /*!< 0x00400000 */
#define EXTI_PR_PR22              EXTI_PR_PR22_Msk                             /*!< Pending bit for line 22 */

/******************************************************************************/
/*                                                                            */
/*                                    FLASH                                   */
/*                                                                            */
/******************************************************************************/
/*******************  Bits definition for FLASH_ACR register  *****************/
#define FLASH_ACR_LATENCY_Pos          (0U)
#define FLASH_ACR_LATENCY_Msk          (0xFUL << FLASH_ACR_LATENCY_Pos)         /*!< 0x0000000F */
#define FLASH_ACR_LATENCY              FLASH_ACR_LATENCY_Msk
#define FLASH_ACR_LATENCY_0WS          0x00000000U
#define FLASH_ACR_LATENCY_1WS          0x00000001U
#define FLASH_ACR_LATENCY_2WS          0x00000002U
#define FLASH_ACR_LATENCY_3WS          0x00000003U
#define FLASH_ACR_LATENCY_4WS          0x00000004U
#define FLASH_ACR_LATENCY_5WS          0x00000005U
#define FLASH_ACR_LATENCY_6WS          0x00000006U
#define FLASH_ACR_LATENCY_7WS          0x00000007U

#define FLASH_ACR_PRFTEN_Pos           (8U)
#define FLASH_ACR_PRFTEN_Msk           (0x1UL << FLASH_ACR_PRFTEN_Pos)          /*!< 0x00000100 */
#define FLASH_ACR_PRFTEN               FLASH_ACR_PRFTEN_Msk
#define FLASH_ACR_ICEN_Pos             (9U)
#define FLASH_ACR_ICEN_Msk             (0x1UL << FLASH_ACR_ICEN_Pos)            /*!< 0x00000200 */
#define FLASH_ACR_ICEN                 FLASH_ACR_ICEN_Msk
#define FLASH_ACR_DCEN_Pos             (10U)
#define FLASH_ACR_DCEN_Msk             (0x1UL << FLASH_ACR_DCEN_Pos)            /*!< 0x00000400 */
#define FLASH_ACR_DCEN                 FLASH_ACR_DCEN_Msk
#define FLASH_ACR_ICRST_Pos            (11U)
#define FLASH_ACR_ICRST_Msk            (0x1UL << FLASH_ACR_ICRST_Pos)           /*!< 0x00000800 */
#define FLASH_ACR_ICRST                FLASH_ACR_ICRST_Msk
#define FLASH_ACR_DCRST_Pos            (12U)
#define FLASH_ACR_DCRST_Msk            (0x1UL << FLASH_ACR_DCRST_Pos)           /*!< 0x00001000 */
#define FLASH_ACR_DCRST                FLASH_ACR_DCRST_Msk
#define FLASH_ACR_BYTE0_ADDRESS_Pos    (10U)
#define FLASH_ACR_BYTE0_ADDRESS_Msk    (0x10008FUL << FLASH_ACR_BYTE0_ADDRESS_Pos) /*!< 0x40023C00 */
#define FLASH_ACR_BYTE0_ADDRESS        FLASH_ACR_BYTE0_ADDRESS_Msk
#define FLASH_ACR_BYTE2_ADDRESS_Pos    (0U)
#define FLASH_ACR_BYTE2_ADDRESS_Msk    (0x40023C03UL << FLASH_ACR_BYTE2_ADDRESS_Pos) /*!< 0x40023C03 */
#define FLASH_ACR_BYTE2_ADDRESS        FLASH_ACR_BYTE2_ADDRESS_Msk

/*******************  Bits definition for FLASH_SR register  ******************/
#define FLASH_SR_EOP_Pos               (0U)
#define FLASH_SR_EOP_Msk               (0x1UL << FLASH_SR_EOP_Pos)              /*!< 0x00000001 */
#define FLASH_SR_EOP                   FLASH_SR_EOP_Msk
#define FLASH_SR_SOP_Pos               (1U)
#define FLASH_SR_SOP_Msk               (0x1UL << FLASH_SR_SOP_Pos)              /*!< 0x00000002 */
#define FLASH_SR_SOP                   FLASH_SR_SOP_Msk
#define FLASH_SR_WRPERR_Pos            (4U)
#define FLASH_SR_WRPERR_Msk            (0x1UL << FLASH_SR_WRPERR_Pos)           /*!< 0x00000010 */
#define FLASH_SR_WRPERR                FLASH_SR_WRPERR_Msk
#define FLASH_SR_PGAERR_Pos            (5U)
#define FLASH_SR_PGAERR_Msk            (0x1UL << FLASH_SR_PGAERR_Pos)           /*!< 0x00000020 */
#define FLASH_SR_PGAERR                FLASH_SR_PGAERR_Msk
#define FLASH_SR_PGPERR_Pos            (6U)
#define FLASH_SR_PGPERR_Msk            (0x1UL << FLASH_SR_PGPERR_Pos)           /*!< 0x00000040 */
#define FLASH_SR_PGPERR                FLASH_SR_PGPERR_Msk
#define FLASH_SR_PGSERR_Pos            (7U)
#define FLASH_SR_PGSERR_Msk            (0x1UL << FLASH_SR_PGSERR_Pos)           /*!< 0x00000080 */
#define FLASH_SR_PGSERR                FLASH_SR_PGSERR_Msk
#define FLASH_SR_BSY_Pos               (16U)
#define FLASH_SR_BSY_Msk               (0x1UL << FLASH_SR_BSY_Pos)              /*!< 0x00010000 */
#define FLASH_SR_BSY                   FLASH_SR_BSY_Msk

/*******************  Bits definition for FLASH_CR register  ******************/
#define FLASH_CR_PG_Pos                (0U)
#define FLASH_CR_PG_Msk                (0x1UL << FLASH_CR_PG_Pos)               /*!< 0x00000001 */
#define FLASH_CR_PG                    FLASH_CR_PG_Msk
#define FLASH_CR_SER_Pos               (1U)
#define FLASH_CR_SER_Msk               (0x1UL << FLASH_CR_SER_Pos)              /*!< 0x00000002 */
#define FLASH_CR_SER                   FLASH_CR_SER_Msk
#define FLASH_CR_MER_Pos               (2U)
#define FLASH_CR_MER_Msk               (0x1UL << FLASH_CR_MER_Pos)              /*!< 0x00000004 */
#define FLASH_CR_MER                   FLASH_CR_MER_Msk
#define FLASH_CR_SNB_Pos               (3U)
#define FLASH_CR_SNB_Msk               (0x1FUL << FLASH_CR_SNB_Pos)             /*!< 0x000000F8 */
#define FLASH_CR_SNB                   FLASH_CR_SNB_Msk
#define FLASH_CR_SNB_0                 (0x01UL << FLASH_CR_SNB_Pos)             /*!< 0x00000008 */
#define FLASH_CR_SNB_1                 (0x02UL << FLASH_CR_SNB_Pos)             /*!< 0x00000010 */
#define FLASH_CR_SNB_2                 (0x04UL << FLASH_CR_SNB_Pos)             /*!< 0x00000020 */
#define FLASH_CR_SNB_3                 (0x08UL << FLASH_CR_SNB_Pos)             /*!< 0x00000040 */
#define FLASH_CR_SNB_4                 (0x10UL << FLASH_CR_SNB_Pos)             /*!< 0x00000080 */
#define FLASH_CR_PSIZE_Pos             (8U)
#define FLASH_CR_PSIZE_Msk             (0x3UL << FLASH_CR_PSIZE_Pos)            /*!< 0x00000300 */
#define FLASH_CR_PSIZE                 FLASH_CR_PSIZE_Msk
#define FLASH_CR_PSIZE_0               (0x1UL << FLASH_CR_PSIZE_Pos)            /*!< 0x00000100 */
#define FLASH_CR_PSIZE_1               (0x2UL << FLASH_CR_PSIZE_Pos)            /*!< 0x00000200 */
#define FLASH_CR_STRT_Pos              (16U)
#define FLASH_CR_STRT_Msk              (0x1UL << FLASH_CR_STRT_Pos)             /*!< 0x00010000 */
#define FLASH_CR_STRT                  FLASH_CR_STRT_Msk
#define FLASH_CR_EOPIE_Pos             (24U)
#define FLASH_CR_EOPIE_Msk             (0x1UL << FLASH_CR_EOPIE_Pos)            /*!< 0x01000000 */
#define FLASH_CR_EOPIE                 FLASH_CR_EOPIE_Msk
#define FLASH_CR_LOCK_Pos              (31U)
#define FLASH_CR_LOCK_Msk              (0x1UL << FLASH_CR_LOCK_Pos)             /*!< 0x80000000 */
#define FLASH_CR_LOCK                  FLASH_CR_LOCK_Msk

/*******************  Bits definition for FLASH_OPTCR register  ***************/
#define FLASH_OPTCR_OPTLOCK_Pos        (0U)
#define FLASH_OPTCR_OPTLOCK_Msk        (0x1UL << FLASH_OPTCR_OPTLOCK_Pos)       /*!< 0x00000001 */
#define FLASH_OPTCR_OPTLOCK            FLASH_OPTCR_OPTLOCK_Msk
#define FLASH_OPTCR_OPTSTRT_Pos        (1U)
#define FLASH_OPTCR_OPTSTRT_Msk        (0x1UL << FLASH_OPTCR_OPTSTRT_Pos)       /*!< 0x00000002 */
#define FLASH_OPTCR_OPTSTRT            FLASH_OPTCR_OPTSTRT_Msk

#define FLASH_OPTCR_BOR_LEV_0          0x00000004U
#define FLASH_OPTCR_BOR_LEV_1          0x00000008U
#define FLASH_OPTCR_BOR_LEV_Pos        (2U)
#define FLASH_OPTCR_BOR_LEV_Msk        (0x3UL << FLASH_OPTCR_BOR_LEV_Pos)       /*!< 0x0000000C */
#define FLASH_OPTCR_BOR_LEV            FLASH_OPTCR_BOR_LEV_Msk
#define FLASH_OPTCR_WDG_SW_Pos         (5U)
#define FLASH_OPTCR_WDG_SW_Msk         (0x1UL << FLASH_OPTCR_WDG_SW_Pos)        /*!< 0x00000020 */
#define FLASH_OPTCR_WDG_SW             FLASH_OPTCR_WDG_SW_Msk
#define FLASH_OPTCR_nRST_STOP_Pos      (6U)
#define FLASH_OPTCR_nRST_STOP_Msk      (0x1UL << FLASH_OPTCR_nRST_STOP_Pos)     /*!< 0x00000040 */
#define FLASH_OPTCR_nRST_STOP          FLASH_OPTCR_nRST_STOP_Msk
#define FLASH_OPTCR_nRST_STDBY_Pos     (7U)
#define FLASH_OPTCR_nRST_STDBY_Msk     (0x1UL << FLASH_OPTCR_nRST_STDBY_Pos)    /*!< 0x00000080 */
#define FLASH_OPTCR_nRST_STDBY         FLASH_OPTCR_nRST_STDBY_Msk
#define FLASH_OPTCR_RDP_Pos            (8U)
#define FLASH_OPTCR_RDP_Msk            (0xFFUL << FLASH_OPTCR_RDP_Pos)          /*!< 0x0000FF00 */
#define FLASH_OPTCR_RDP                FLASH_OPTCR_RDP_Msk
#define FLASH_OPTCR_RDP_0              (0x01UL << FLASH_OPTCR_RDP_Pos)          /*!< 0x00000100 */
#define FLASH_OPTCR_RDP_1              (0x02UL << FLASH_OPTCR_RDP_Pos)          /*!< 0x00000200 */
#define FLASH_OPTCR_RDP_2              (0x04UL << FLASH_OPTCR_RDP_Pos)          /*!< 0x00000400 */
#define FLASH_OPTCR_RDP_3              (0x08UL << FLASH_OPTCR_RDP_Pos)          /*!< 0x00000800 */
#define FLASH_OPTCR_RDP_4              (0x10UL << FLASH_OPTCR_RDP_Pos)          /*!< 0x00001000 */
#define FLASH_OPTCR_RDP_5              (0x20UL << FLASH_OPTCR_RDP_Pos)          /*!< 0x00002000 */
#define FLASH_OPTCR_RDP_6              (0x40UL << FLASH_OPTCR_RDP_Pos)          /*!< 0x00004000 */
#define FLASH_OPTCR_RDP_7              (0x80UL << FLASH_OPTCR_RDP_Pos)          /*!< 0x00008000 */
#define FLASH_OPTCR_nWRP_Pos           (16U)
#define FLASH_OPTCR_nWRP_Msk           (0xFFFUL << FLASH_OPTCR_nWRP_Pos)        /*!< 0x0FFF0000 */
#define FLASH_OPTCR_nWRP               FLASH_OPTCR_nWRP_Msk
#define FLASH_OPTCR_nWRP_0             0x00010000U
#define FLASH_OPTCR_nWRP_1             0x00020000U
#define FLASH_OPTCR_nWRP_2             0x00040000U
#define FLASH_OPTCR_nWRP_3             0x00080000U
#define FLASH_OPTCR_nWRP_4             0x00100000U
#define FLASH_OPTCR_nWRP_5             0x00200000U
#define FLASH_OPTCR_nWRP_6             0x00400000U
#define FLASH_OPTCR_nWRP_7             0x00800000U
#define FLASH_OPTCR_nWRP_8             0x01000000U
#define FLASH_OPTCR_nWRP_9             0x02000000U
#define FLASH_OPTCR_nWRP_10            0x04000000U
#define FLASH_OPTCR_nWRP_11            0x08000000U

/******************  Bits definition for FLASH_OPTCR1 register  ***************/
#define FLASH_OPTCR1_nWRP_Pos          (16U)
#define FLASH_OPTCR1_nWRP_Msk          (0xFFFUL << FLASH_OPTCR1_nWRP_Pos)       /*!< 0x0FFF0000 */
#define FLASH_OPTCR1_nWRP              FLASH_OPTCR1_nWRP_Msk
#define FLASH_OPTCR1_nWRP_0            (0x001UL << FLASH_OPTCR1_nWRP_Pos)       /*!< 0x00010000 */
#define FLASH_OPTCR1_nWRP_1            (0x002UL << FLASH_OPTCR1_nWRP_Pos)       /*!< 0x00020000 */
#define FLASH_OPTCR1_nWRP_2            (0x004UL << FLASH_OPTCR1_nWRP_Pos)       /*!< 0x00040000 */
#define FLASH_OPTCR1_nWRP_3            (0x008UL << FLASH_OPTCR1_nWRP_Pos)       /*!< 0x00080000 */
#define FLASH_OPTCR1_nWRP_4            (0x010UL << FLASH_OPTCR1_nWRP_Pos)       /*!< 0x00100000 */
#define FLASH_OPTCR1_nWRP_5            (0x020UL << FLASH_OPTCR1_nWRP_Pos)       /*!< 0x00200000 */
#define FLASH_OPTCR1_nWRP_6            (0x040UL << FLASH_OPTCR1_nWRP_Pos)       /*!< 0x00400000 */
#define FLASH_OPTCR1_nWRP_7            (0x080UL << FLASH_OPTCR1_nWRP_Pos)       /*!< 0x00800000 */
#define FLASH_OPTCR1_nWRP_8            (0x100UL << FLASH_OPTCR1_nWRP_Pos)       /*!< 0x01000000 */
#define FLASH_OPTCR1_nWRP_9            (0x200UL << FLASH_OPTCR1_nWRP_Pos)       /*!< 0x02000000 */
#define FLASH_OPTCR1_nWRP_10           (0x400UL << FLASH_OPTCR1_nWRP_Pos)       /*!< 0x04000000 */
#define FLASH_OPTCR1_nWRP_11           (0x800UL << FLASH_OPTCR1_nWRP_Pos)       /*!< 0x08000000 */



/******************************************************************************/
/*                                                                            */
/*                            General Purpose I/O                             */
/*                                                                            */
/******************************************************************************/
/******************  Bits definition for GPIO_MODER register  *****************/
#define GPIO_MODER_MODER0_Pos            (0U)
#define GPIO_MODER_MODER0_Msk            (0x3UL << GPIO_MODER_MODER0_Pos)       /*!< 0x00000003 */
#define GPIO_MODER_MODER0                GPIO_MODER_MODER0_Msk
#define GPIO_MODER_MODER0_0              (0x1UL << GPIO_MODER_MODER0_Pos)       /*!< 0x00000001 */
#define GPIO_MODER_MODER0_1              (0x2UL << GPIO_MODER_MODER0_Pos)       /*!< 0x00000002 */
#define GPIO_MODER_MODER1_Pos            (2U)
#define GPIO_MODER_MODER1_Msk            (0x3UL << GPIO_MODER_MODER1_Pos)       /*!< 0x0000000C */
#define GPIO_MODER_MODER1                GPIO_MODER_MODER1_Msk
#define GPIO_MODER_MODER1_0              (0x1UL << GPIO_MODER_MODER1_Pos)       /*!< 0x00000004 */
#define GPIO_MODER_MODER1_1              (0x2UL << GPIO_MODER_MODER1_Pos)       /*!< 0x00000008 */
#define GPIO_MODER_MODER2_Pos            (4U)
#define GPIO_MODER_MODER2_Msk            (0x3UL << GPIO_MODER_MODER2_Pos)       /*!< 0x00000030 */
#define GPIO_MODER_MODER2                GPIO_MODER_MODER2_Msk
#define GPIO_MODER_MODER2_0              (0x1UL << GPIO_MODER_MODER2_Pos)       /*!< 0x00000010 */
#define GPIO_MODER_MODER2_1              (0x2UL << GPIO_MODER_MODER2_Pos)       /*!< 0x00000020 */
#define GPIO_MODER_MODER3_Pos            (6U)
#define GPIO_MODER_MODER3_Msk            (0x3UL << GPIO_MODER_MODER3_Pos)       /*!< 0x000000C0 */
#define GPIO_MODER_MODER3                GPIO_MODER_MODER3_Msk
#define GPIO_MODER_MODER3_0              (0x1UL << GPIO_MODER_MODER3_Pos)       /*!< 0x00000040 */
#define GPIO_MODER_MODER3_1              (0x2UL << GPIO_MODER_MODER3_Pos)       /*!< 0x00000080 */
#define GPIO_MODER_MODER4_Pos            (8U)
#define GPIO_MODER_MODER4_Msk            (0x3UL << GPIO_MODER_MODER4_Pos)       /*!< 0x00000300 */
#define GPIO_MODER_MODER4                GPIO_MODER_MODER4_Msk
#define GPIO_MODER_MODER4_0              (0x1UL << GPIO_MODER_MODER4_Pos)       /*!< 0x00000100 */
#define GPIO_MODER_MODER4_1              (0x2UL << GPIO_MODER_MODER4_Pos)       /*!< 0x00000200 */
#define GPIO_MODER_MODER5_Pos            (10U)
#define GPIO_MODER_MODER5_Msk            (0x3UL << GPIO_MODER_MODER5_Pos)       /*!< 0x00000C00 */
#define GPIO_MODER_MODER5                GPIO_MODER_MODER5_Msk
#define GPIO_MODER_MODER5_0              (0x1UL << GPIO_MODER_MODER5_Pos)       /*!< 0x00000400 */
#define GPIO_MODER_MODER5_1              (0x2UL << GPIO_MODER_MODER5_Pos)       /*!< 0x00000800 */
#define GPIO_MODER_MODER6_Pos            (12U)
#define GPIO_MODER_MODER6_Msk            (0x3UL << GPIO_MODER_MODER6_Pos)       /*!< 0x00003000 */
#define GPIO_MODER_MODER6                GPIO_MODER_MODER6_Msk
#define GPIO_MODER_MODER6_0              (0x1UL << GPIO_MODER_MODER6_Pos)       /*!< 0x00001000 */
#define GPIO_MODER_MODER6_1              (0x2UL << GPIO_MODER_MODER6_Pos)       /*!< 0x00002000 */
#define GPIO_MODER_MODER7_Pos            (14U)
#define GPIO_MODER_MODER7_Msk            (0x3UL << GPIO_MODER_MODER7_Pos)       /*!< 0x0000C000 */
#define GPIO_MODER_MODER7                GPIO_MODER_MODER7_Msk
#define GPIO_MODER_MODER7_0              (0x1UL << GPIO_MODER_MODER7_Pos)       /*!< 0x00004000 */
#define GPIO_MODER_MODER7_1              (0x2UL << GPIO_MODER_MODER7_Pos)       /*!< 0x00008000 */
#define GPIO_MODER_MODER8_Pos            (16U)
#define GPIO_MODER_MODER8_Msk            (0x3UL << GPIO_MODER_MODER8_Pos)       /*!< 0x00030000 */
#define GPIO_MODER_MODER8                GPIO_MODER_MODER8_Msk
#define GPIO_MODER_MODER8_0              (0x1UL << GPIO_MODER_MODER8_Pos)       /*!< 0x00010000 */
#define GPIO_MODER_MODER8_1              (0x2UL << GPIO_MODER_MODER8_Pos)       /*!< 0x00020000 */
#define GPIO_MODER_MODER9_Pos            (18U)
#define GPIO_MODER_MODER9_Msk            (0x3UL << GPIO_MODER_MODER9_Pos)       /*!< 0x000C0000 */
#define GPIO_MODER_MODER9                GPIO_MODER_MODER9_Msk
#define GPIO_MODER_MODER9_0              (0x1UL << GPIO_MODER_MODER9_Pos)       /*!< 0x00040000 */
#define GPIO_MODER_MODER9_1              (0x2UL << GPIO_MODER_MODER9_Pos)       /*!< 0x00080000 */
#define GPIO_MODER_MODER10_Pos           (20U)
#define GPIO_MODER_MODER10_Msk           (0x3UL << GPIO_MODER_MODER10_Pos)      /*!< 0x00300000 */
#define GPIO_MODER_MODER10               GPIO_MODER_MODER10_Msk
#define GPIO_MODER_MODER10_0             (0x1UL << GPIO_MODER_MODER10_Pos)      /*!< 0x00100000 */
#define GPIO_MODER_MODER10_1             (0x2UL << GPIO_MODER_MODER10_Pos)      /*!< 0x00200000 */
#define GPIO_MODER_MODER11_Pos           (22U)
#define GPIO_MODER_MODER11_Msk           (0x3UL << GPIO_MODER_MODER11_Pos)      /*!< 0x00C00000 */
#define GPIO_MODER_MODER11               GPIO_MODER_MODER11_Msk
#define GPIO_MODER_MODER11_0             (0x1UL << GPIO_MODER_MODER11_Pos)      /*!< 0x00400000 */
#define GPIO_MODER_MODER11_1             (0x2UL << GPIO_MODER_MODER11_Pos)      /*!< 0x00800000 */
#define GPIO_MODER_MODER12_Pos           (24U)
#define GPIO_MODER_MODER12_Msk           (0x3UL << GPIO_MODER_MODER12_Pos)      /*!< 0x03000000 */
#define GPIO_MODER_MODER12               GPIO_MODER_MODER12_Msk
#define GPIO_MODER_MODER12_0             (0x1UL << GPIO_MODER_MODER12_Pos)      /*!< 0x01000000 */
#define GPIO_MODER_MODER12_1             (0x2UL << GPIO_MODER_MODER12_Pos)      /*!< 0x02000000 */
#define GPIO_MODER_MODER13_Pos           (26U)
#define GPIO_MODER_MODER13_Msk           (0x3UL << GPIO_MODER_MODER13_Pos)      /*!< 0x0C000000 */
#define GPIO_MODER_MODER13               GPIO_MODER_MODER13_Msk
#define GPIO_MODER_MODER13_0             (0x1UL << GPIO_MODER_MODER13_Pos)      /*!< 0x04000000 */
#define GPIO_MODER_MODER13_1             (0x2UL << GPIO_MODER_MODER13_Pos)      /*!< 0x08000000 */
#define GPIO_MODER_MODER14_Pos           (28U)
#define GPIO_MODER_MODER14_Msk           (0x3UL << GPIO_MODER_MODER14_Pos)      /*!< 0x30000000 */
#define GPIO_MODER_MODER14               GPIO_MODER_MODER14_Msk
#define GPIO_MODER_MODER14_0             (0x1UL << GPIO_MODER_MODER14_Pos)      /*!< 0x10000000 */
#define GPIO_MODER_MODER14_1             (0x2UL << GPIO_MODER_MODER14_Pos)      /*!< 0x20000000 */
#define GPIO_MODER_MODER15_Pos           (30U)
#define GPIO_MODER_MODER15_Msk           (0x3UL << GPIO_MODER_MODER15_Pos)      /*!< 0xC0000000 */
#define GPIO_MODER_MODER15               GPIO_MODER_MODER15_Msk
#define GPIO_MODER_MODER15_0             (0x1UL << GPIO_MODER_MODER15_Pos)      /*!< 0x40000000 */
#define GPIO_MODER_MODER15_1             (0x2UL << GPIO_MODER_MODER15_Pos)      /*!< 0x80000000 */

/* Legacy defines */
#define GPIO_MODER_MODE0_Pos             GPIO_MODER_MODER0_Pos
#define GPIO_MODER_MODE0_Msk             GPIO_MODER_MODER0_Msk
#define GPIO_MODER_MODE0                 GPIO_MODER_MODER0
#define GPIO_MODER_MODE0_0               GPIO_MODER_MODER0_0
#define GPIO_MODER_MODE0_1               GPIO_MODER_MODER0_1
#define GPIO_MODER_MODE1_Pos             GPIO_MODER_MODER1_Pos
#define GPIO_MODER_MODE1_Msk             GPIO_MODER_MODER1_Msk
#define GPIO_MODER_MODE1                 GPIO_MODER_MODER1
#define GPIO_MODER_MODE1_0               GPIO_MODER_MODER1_0
#define GPIO_MODER_MODE1_1               GPIO_MODER_MODER1_1
#define GPIO_MODER_MODE2_Pos             GPIO_MODER_MODER2_PoS
#define GPIO_MODER_MODE2_Msk             GPIO_MODER_MODER2_Msk
#define GPIO_MODER_MODE2                 GPIO_MODER_MODER2
#define GPIO_MODER_MODE2_0               GPIO_MODER_MODER2_0
#define GPIO_MODER_MODE2_1               GPIO_MODER_MODER2_1
#define GPIO_MODER_MODE3_Pos             GPIO_MODER_MODER3_Pos
#define GPIO_MODER_MODE3_Msk             GPIO_MODER_MODER3_Msk
#define GPIO_MODER_MODE3                 GPIO_MODER_MODER3
#define GPIO_MODER_MODE3_0               GPIO_MODER_MODER3_0
#define GPIO_MODER_MODE3_1               GPIO_MODER_MODER3_1
#define GPIO_MODER_MODE4_Pos             GPIO_MODER_MODER4_Pos
#define GPIO_MODER_MODE4_Msk             GPIO_MODER_MODER4_Msk
#define GPIO_MODER_MODE4                 GPIO_MODER_MODER4
#define GPIO_MODER_MODE4_0               GPIO_MODER_MODER4_0
#define GPIO_MODER_MODE4_1               GPIO_MODER_MODER4_1
#define GPIO_MODER_MODE5_Pos             GPIO_MODER_MODER5_Pos
#define GPIO_MODER_MODE5_Msk             GPIO_MODER_MODER5_Msk
#define GPIO_MODER_MODE5                 GPIO_MODER_MODER5
#define GPIO_MODER_MODE5_0               GPIO_MODER_MODER5_0
#define GPIO_MODER_MODE5_1               GPIO_MODER_MODER5_1
#define GPIO_MODER_MODE6_Pos             GPIO_MODER_MODER6_Pos
#define GPIO_MODER_MODE6_Msk             GPIO_MODER_MODER6_Msk
#define GPIO_MODER_MODE6                 GPIO_MODER_MODER6
#define GPIO_MODER_MODE6_0               GPIO_MODER_MODER6_0
#define GPIO_MODER_MODE6_1               GPIO_MODER_MODER6_1
#define GPIO_MODER_MODE7_Pos             GPIO_MODER_MODER7_Pos
#define GPIO_MODER_MODE7_Msk             GPIO_MODER_MODER7_Msk
#define GPIO_MODER_MODE7                 GPIO_MODER_MODER7
#define GPIO_MODER_MODE7_0               GPIO_MODER_MODER7_0
#define GPIO_MODER_MODE7_1               GPIO_MODER_MODER7_1
#define GPIO_MODER_MODE8_Pos             GPIO_MODER_MODER8_Pos
#define GPIO_MODER_MODE8_Msk             GPIO_MODER_MODER2_Msk
#define GPIO_MODER_MODE8                 GPIO_MODER_MODER8
#define GPIO_MODER_MODE8_0               GPIO_MODER_MODER8_0
#define GPIO_MODER_MODE8_1               GPIO_MODER_MODER8_1
#define GPIO_MODER_MODE9_Pos             GPIO_MODER_MODER9_Pos
#define GPIO_MODER_MODE9_Msk             GPIO_MODER_MODER9_Msk
#define GPIO_MODER_MODE9                 GPIO_MODER_MODER9
#define GPIO_MODER_MODE9_0               GPIO_MODER_MODER9_0
#define GPIO_MODER_MODE9_1               GPIO_MODER_MODER9_1
#define GPIO_MODER_MODE10_Pos            GPIO_MODER_MODER10_Pos
#define GPIO_MODER_MODE10_Msk            GPIO_MODER_MODER10_Msk
#define GPIO_MODER_MODE10                GPIO_MODER_MODER10
#define GPIO_MODER_MODE10_0              GPIO_MODER_MODER10_0
#define GPIO_MODER_MODE10_1              GPIO_MODER_MODER10_1
#define GPIO_MODER_MODE11_Pos            GPIO_MODER_MODER11_Pos
#define GPIO_MODER_MODE11_Msk            GPIO_MODER_MODER11_Msk
#define GPIO_MODER_MODE11                GPIO_MODER_MODER11
#define GPIO_MODER_MODE11_0              GPIO_MODER_MODER11_0
#define GPIO_MODER_MODE11_1              GPIO_MODER_MODER11_1
#define GPIO_MODER_MODE12_Pos            GPIO_MODER_MODER12_Pos
#define GPIO_MODER_MODE12_Msk            GPIO_MODER_MODER12_Msk
#define GPIO_MODER_MODE12                GPIO_MODER_MODER12
#define GPIO_MODER_MODE12_0              GPIO_MODER_MODER12_0
#define GPIO_MODER_MODE12_1              GPIO_MODER_MODER12_1
#define GPIO_MODER_MODE13_Pos            GPIO_MODER_MODER13_Pos
#define GPIO_MODER_MODE13_Msk            GPIO_MODER_MODER13_Msk
#define GPIO_MODER_MODE13                GPIO_MODER_MODER13
#define GPIO_MODER_MODE13_0              GPIO_MODER_MODER13_0
#define GPIO_MODER_MODE13_1              GPIO_MODER_MODER13_1
#define GPIO_MODER_MODE14_Pos            GPIO_MODER_MODER14_Pos
#define GPIO_MODER_MODE14_Msk            GPIO_MODER_MODER14_Msk
#define GPIO_MODER_MODE14                GPIO_MODER_MODER14
#define GPIO_MODER_MODE14_0              GPIO_MODER_MODER14_0
#define GPIO_MODER_MODE14_1              GPIO_MODER_MODER14_1
#define GPIO_MODER_MODE15_Pos            GPIO_MODER_MODER15_Pos
#define GPIO_MODER_MODE15_Msk            GPIO_MODER_MODER15_Msk
#define GPIO_MODER_MODE15                GPIO_MODER_MODER15
#define GPIO_MODER_MODE15_0              GPIO_MODER_MODER15_0
#define GPIO_MODER_MODE15_1              GPIO_MODER_MODER15_1

/******************  Bits definition for GPIO_OTYPER register  ****************/
#define GPIO_OTYPER_OT0_Pos              (0U)
#define GPIO_OTYPER_OT0_Msk              (0x1UL << GPIO_OTYPER_OT0_Pos)         /*!< 0x00000001 */
#define GPIO_OTYPER_OT0                  GPIO_OTYPER_OT0_Msk
#define GPIO_OTYPER_OT1_Pos              (1U)
#define GPIO_OTYPER_OT1_Msk              (0x1UL << GPIO_OTYPER_OT1_Pos)         /*!< 0x00000002 */
#define GPIO_OTYPER_OT1                  GPIO_OTYPER_OT1_Msk
#define GPIO_OTYPER_OT2_Pos              (2U)
#define GPIO_OTYPER_OT2_Msk              (0x1UL << GPIO_OTYPER_OT2_Pos)         /*!< 0x00000004 */
#define GPIO_OTYPER_OT2                  GPIO_OTYPER_OT2_Msk
#define GPIO_OTYPER_OT3_Pos              (3U)
#define GPIO_OTYPER_OT3_Msk              (0x1UL << GPIO_OTYPER_OT3_Pos)         /*!< 0x00000008 */
#define GPIO_OTYPER_OT3                  GPIO_OTYPER_OT3_Msk
#define GPIO_OTYPER_OT4_Pos              (4U)
#define GPIO_OTYPER_OT4_Msk              (0x1UL << GPIO_OTYPER_OT4_Pos)         /*!< 0x00000010 */
#define GPIO_OTYPER_OT4                  GPIO_OTYPER_OT4_Msk
#define GPIO_OTYPER_OT5_Pos              (5U)
#define GPIO_OTYPER_OT5_Msk              (0x1UL << GPIO_OTYPER_OT5_Pos)         /*!< 0x00000020 */
#define GPIO_OTYPER_OT5                  GPIO_OTYPER_OT5_Msk
#define GPIO_OTYPER_OT6_Pos              (6U)
#define GPIO_OTYPER_OT6_Msk              (0x1UL << GPIO_OTYPER_OT6_Pos)         /*!< 0x00000040 */
#define GPIO_OTYPER_OT6                  GPIO_OTYPER_OT6_Msk
#define GPIO_OTYPER_OT7_Pos              (7U)
#define GPIO_OTYPER_OT7_Msk              (0x1UL << GPIO_OTYPER_OT7_Pos)         /*!< 0x00000080 */
#define GPIO_OTYPER_OT7                  GPIO_OTYPER_OT7_Msk
#define GPIO_OTYPER_OT8_Pos              (8U)
#define GPIO_OTYPER_OT8_Msk              (0x1UL << GPIO_OTYPER_OT8_Pos)         /*!< 0x00000100 */
#define GPIO_OTYPER_OT8                  GPIO_OTYPER_OT8_Msk
#define GPIO_OTYPER_OT9_Pos              (9U)
#define GPIO_OTYPER_OT9_Msk              (0x1UL << GPIO_OTYPER_OT9_Pos)         /*!< 0x00000200 */
#define GPIO_OTYPER_OT9                  GPIO_OTYPER_OT9_Msk
#define GPIO_OTYPER_OT10_Pos             (10U)
#define GPIO_OTYPER_OT10_Msk             (0x1UL << GPIO_OTYPER_OT10_Pos)        /*!< 0x00000400 */
#define GPIO_OTYPER_OT10                 GPIO_OTYPER_OT10_Msk
#define GPIO_OTYPER_OT11_Pos             (11U)
#define GPIO_OTYPER_OT11_Msk             (0x1UL << GPIO_OTYPER_OT11_Pos)        /*!< 0x00000800 */
#define GPIO_OTYPER_OT11                 GPIO_OTYPER_OT11_Msk
#define GPIO_OTYPER_OT12_Pos             (12U)
#define GPIO_OTYPER_OT12_Msk             (0x1UL << GPIO_OTYPER_OT12_Pos)        /*!< 0x00001000 */
#define GPIO_OTYPER_OT12                 GPIO_OTYPER_OT12_Msk
#define GPIO_OTYPER_OT13_Pos             (13U)
#define GPIO_OTYPER_OT13_Msk             (0x1UL << GPIO_OTYPER_OT13_Pos)        /*!< 0x00002000 */
#define GPIO_OTYPER_OT13                 GPIO_OTYPER_OT13_Msk
#define GPIO_OTYPER_OT14_Pos             (14U)
#define GPIO_OTYPER_OT14_Msk             (0x1UL << GPIO_OTYPER_OT14_Pos)        /*!< 0x00004000 */
#define GPIO_OTYPER_OT14                 GPIO_OTYPER_OT14_Msk
#define GPIO_OTYPER_OT15_Pos             (15U)
#define GPIO_OTYPER_OT15_Msk             (0x1UL << GPIO_OTYPER_OT15_Pos)        /*!< 0x00008000 */
#define GPIO_OTYPER_OT15                 GPIO_OTYPER_OT15_Msk

/* Legacy defines */
#define GPIO_OTYPER_OT_0                 GPIO_OTYPER_OT0
#define GPIO_OTYPER_OT_1                 GPIO_OTYPER_OT1
#define GPIO_OTYPER_OT_2                 GPIO_OTYPER_OT2
#define GPIO_OTYPER_OT_3                 GPIO_OTYPER_OT3
#define GPIO_OTYPER_OT_4                 GPIO_OTYPER_OT4
#define GPIO_OTYPER_OT_5                 GPIO_OTYPER_OT5
#define GPIO_OTYPER_OT_6                 GPIO_OTYPER_OT6
#define GPIO_OTYPER_OT_7                 GPIO_OTYPER_OT7
#define GPIO_OTYPER_OT_8                 GPIO_OTYPER_OT8
#define GPIO_OTYPER_OT_9                 GPIO_OTYPER_OT9
#define GPIO_OTYPER_OT_10                GPIO_OTYPER_OT10
#define GPIO_OTYPER_OT_11                GPIO_OTYPER_OT11
#define GPIO_OTYPER_OT_12                GPIO_OTYPER_OT12
#define GPIO_OTYPER_OT_13                GPIO_OTYPER_OT13
#define GPIO_OTYPER_OT_14                GPIO_OTYPER_OT14
#define GPIO_OTYPER_OT_15                GPIO_OTYPER_OT15

/******************  Bits definition for GPIO_OSPEEDR register  ***************/
#define GPIO_OSPEEDR_OSPEED0_Pos         (0U)
#define GPIO_OSPEEDR_OSPEED0_Msk         (0x3UL << GPIO_OSPEEDR_OSPEED0_Pos)    /*!< 0x00000003 */
#define GPIO_OSPEEDR_OSPEED0             GPIO_OSPEEDR_OSPEED0_Msk
#define GPIO_OSPEEDR_OSPEED0_0           (0x1UL << GPIO_OSPEEDR_OSPEED0_Pos)    /*!< 0x00000001 */
#define GPIO_OSPEEDR_OSPEED0_1           (0x2UL << GPIO_OSPEEDR_OSPEED0_Pos)    /*!< 0x00000002 */
#define GPIO_OSPEEDR_OSPEED1_Pos         (2U)
#define GPIO_OSPEEDR_OSPEED1_Msk         (0x3UL << GPIO_OSPEEDR_OSPEED1_Pos)    /*!< 0x0000000C */
#define GPIO_OSPEEDR_OSPEED1             GPIO_OSPEEDR_OSPEED1_Msk
#define GPIO_OSPEEDR_OSPEED1_0           (0x1UL << GPIO_OSPEEDR_OSPEED1_Pos)    /*!< 0x00000004 */
#define GPIO_OSPEEDR_OSPEED1_1           (0x2UL << GPIO_OSPEEDR_OSPEED1_Pos)    /*!< 0x00000008 */
#define GPIO_OSPEEDR_OSPEED2_Pos         (4U)
#define GPIO_OSPEEDR_OSPEED2_Msk         (0x3UL << GPIO_OSPEEDR_OSPEED2_Pos)    /*!< 0x00000030 */
#define GPIO_OSPEEDR_OSPEED2             GPIO_OSPEEDR_OSPEED2_Msk
#define GPIO_OSPEEDR_OSPEED2_0           (0x1UL << GPIO_OSPEEDR_OSPEED2_Pos)    /*!< 0x00000010 */
#define GPIO_OSPEEDR_OSPEED2_1           (0x2UL << GPIO_OSPEEDR_OSPEED2_Pos)    /*!< 0x00000020 */
#define GPIO_OSPEEDR_OSPEED3_Pos         (6U)
#define GPIO_OSPEEDR_OSPEED3_Msk         (0x3UL << GPIO_OSPEEDR_OSPEED3_Pos)    /*!< 0x000000C0 */
#define GPIO_OSPEEDR_OSPEED3             GPIO_OSPEEDR_OSPEED3_Msk
#define GPIO_OSPEEDR_OSPEED3_0           (0x1UL << GPIO_OSPEEDR_OSPEED3_Pos)    /*!< 0x00000040 */
#define GPIO_OSPEEDR_OSPEED3_1           (0x2UL << GPIO_OSPEEDR_OSPEED3_Pos)    /*!< 0x00000080 */
#define GPIO_OSPEEDR_OSPEED4_Pos         (8U)
#define GPIO_OSPEEDR_OSPEED4_Msk         (0x3UL << GPIO_OSPEEDR_OSPEED4_Pos)    /*!< 0x00000300 */
#define GPIO_OSPEEDR_OSPEED4             GPIO_OSPEEDR_OSPEED4_Msk
#define GPIO_OSPEEDR_OSPEED4_0           (0x1UL << GPIO_OSPEEDR_OSPEED4_Pos)    /*!< 0x00000100 */
#define GPIO_OSPEEDR_OSPEED4_1           (0x2UL << GPIO_OSPEEDR_OSPEED4_Pos)    /*!< 0x00000200 */
#define GPIO_OSPEEDR_OSPEED5_Pos         (10U)
#define GPIO_OSPEEDR_OSPEED5_Msk         (0x3UL << GPIO_OSPEEDR_OSPEED5_Pos)    /*!< 0x00000C00 */
#define GPIO_OSPEEDR_OSPEED5             GPIO_OSPEEDR_OSPEED5_Msk
#define GPIO_OSPEEDR_OSPEED5_0           (0x1UL << GPIO_OSPEEDR_OSPEED5_Pos)    /*!< 0x00000400 */
#define GPIO_OSPEEDR_OSPEED5_1           (0x2UL << GPIO_OSPEEDR_OSPEED5_Pos)    /*!< 0x00000800 */
#define GPIO_OSPEEDR_OSPEED6_Pos         (12U)
#define GPIO_OSPEEDR_OSPEED6_Msk         (0x3UL << GPIO_OSPEEDR_OSPEED6_Pos)    /*!< 0x00003000 */
#define GPIO_OSPEEDR_OSPEED6             GPIO_OSPEEDR_OSPEED6_Msk
#define GPIO_OSPEEDR_OSPEED6_0           (0x1UL << GPIO_OSPEEDR_OSPEED6_Pos)    /*!< 0x00001000 */
#define GPIO_OSPEEDR_OSPEED6_1           (0x2UL << GPIO_OSPEEDR_OSPEED6_Pos)    /*!< 0x00002000 */
#define GPIO_OSPEEDR_OSPEED7_Pos         (14U)
#define GPIO_OSPEEDR_OSPEED7_Msk         (0x3UL << GPIO_OSPEEDR_OSPEED7_Pos)    /*!< 0x0000C000 */
#define GPIO_OSPEEDR_OSPEED7             GPIO_OSPEEDR_OSPEED7_Msk
#define GPIO_OSPEEDR_OSPEED7_0           (0x1UL << GPIO_OSPEEDR_OSPEED7_Pos)    /*!< 0x00004000 */
#define GPIO_OSPEEDR_OSPEED7_1           (0x2UL << GPIO_OSPEEDR_OSPEED7_Pos)    /*!< 0x00008000 */
#define GPIO_OSPEEDR_OSPEED8_Pos         (16U)
#define GPIO_OSPEEDR_OSPEED8_Msk         (0x3UL << GPIO_OSPEEDR_OSPEED8_Pos)    /*!< 0x00030000 */
#define GPIO_OSPEEDR_OSPEED8             GPIO_OSPEEDR_OSPEED8_Msk
#define GPIO_OSPEEDR_OSPEED8_0           (0x1UL << GPIO_OSPEEDR_OSPEED8_Pos)    /*!< 0x00010000 */
#define GPIO_OSPEEDR_OSPEED8_1           (0x2UL << GPIO_OSPEEDR_OSPEED8_Pos)    /*!< 0x00020000 */
#define GPIO_OSPEEDR_OSPEED9_Pos         (18U)
#define GPIO_OSPEEDR_OSPEED9_Msk         (0x3UL << GPIO_OSPEEDR_OSPEED9_Pos)    /*!< 0x000C0000 */
#define GPIO_OSPEEDR_OSPEED9             GPIO_OSPEEDR_OSPEED9_Msk
#define GPIO_OSPEEDR_OSPEED9_0           (0x1UL << GPIO_OSPEEDR_OSPEED9_Pos)    /*!< 0x00040000 */
#define GPIO_OSPEEDR_OSPEED9_1           (0x2UL << GPIO_OSPEEDR_OSPEED9_Pos)    /*!< 0x00080000 */
#define GPIO_OSPEEDR_OSPEED10_Pos        (20U)
#define GPIO_OSPEEDR_OSPEED10_Msk        (0x3UL << GPIO_OSPEEDR_OSPEED10_Pos)   /*!< 0x00300000 */
#define GPIO_OSPEEDR_OSPEED10            GPIO_OSPEEDR_OSPEED10_Msk
#define GPIO_OSPEEDR_OSPEED10_0          (0x1UL << GPIO_OSPEEDR_OSPEED10_Pos)   /*!< 0x00100000 */
#define GPIO_OSPEEDR_OSPEED10_1          (0x2UL << GPIO_OSPEEDR_OSPEED10_Pos)   /*!< 0x00200000 */
#define GPIO_OSPEEDR_OSPEED11_Pos        (22U)
#define GPIO_OSPEEDR_OSPEED11_Msk        (0x3UL << GPIO_OSPEEDR_OSPEED11_Pos)   /*!< 0x00C00000 */
#define GPIO_OSPEEDR_OSPEED11            GPIO_OSPEEDR_OSPEED11_Msk
#define GPIO_OSPEEDR_OSPEED11_0          (0x1UL << GPIO_OSPEEDR_OSPEED11_Pos)   /*!< 0x00400000 */
#define GPIO_OSPEEDR_OSPEED11_1          (0x2UL << GPIO_OSPEEDR_OSPEED11_Pos)   /*!< 0x00800000 */
#define GPIO_OSPEEDR_OSPEED12_Pos        (24U)
#define GPIO_OSPEEDR_OSPEED12_Msk        (0x3UL << GPIO_OSPEEDR_OSPEED12_Pos)   /*!< 0x03000000 */
#define GPIO_OSPEEDR_OSPEED12            GPIO_OSPEEDR_OSPEED12_Msk
#define GPIO_OSPEEDR_OSPEED12_0          (0x1UL << GPIO_OSPEEDR_OSPEED12_Pos)   /*!< 0x01000000 */
#define GPIO_OSPEEDR_OSPEED12_1          (0x2UL << GPIO_OSPEEDR_OSPEED12_Pos)   /*!< 0x02000000 */
#define GPIO_OSPEEDR_OSPEED13_Pos        (26U)
#define GPIO_OSPEEDR_OSPEED13_Msk        (0x3UL << GPIO_OSPEEDR_OSPEED13_Pos)   /*!< 0x0C000000 */
#define GPIO_OSPEEDR_OSPEED13            GPIO_OSPEEDR_OSPEED13_Msk
#define GPIO_OSPEEDR_OSPEED13_0          (0x1UL << GPIO_OSPEEDR_OSPEED13_Pos)   /*!< 0x04000000 */
#define GPIO_OSPEEDR_OSPEED13_1          (0x2UL << GPIO_OSPEEDR_OSPEED13_Pos)   /*!< 0x08000000 */
#define GPIO_OSPEEDR_OSPEED14_Pos        (28U)
#define GPIO_OSPEEDR_OSPEED14_Msk        (0x3UL << GPIO_OSPEEDR_OSPEED14_Pos)   /*!< 0x30000000 */
#define GPIO_OSPEEDR_OSPEED14            GPIO_OSPEEDR_OSPEED14_Msk
#define GPIO_OSPEEDR_OSPEED14_0          (0x1UL << GPIO_OSPEEDR_OSPEED14_Pos)   /*!< 0x10000000 */
#define GPIO_OSPEEDR_OSPEED14_1          (0x2UL << GPIO_OSPEEDR_OSPEED14_Pos)   /*!< 0x20000000 */
#define GPIO_OSPEEDR_OSPEED15_Pos        (30U)
#define GPIO_OSPEEDR_OSPEED15_Msk        (0x3UL << GPIO_OSPEEDR_OSPEED15_Pos)   /*!< 0xC0000000 */
#define GPIO_OSPEEDR_OSPEED15            GPIO_OSPEEDR_OSPEED15_Msk
#define GPIO_OSPEEDR_OSPEED15_0          (0x1UL << GPIO_OSPEEDR_OSPEED15_Pos)   /*!< 0x40000000 */
#define GPIO_OSPEEDR_OSPEED15_1          (0x2UL << GPIO_OSPEEDR_OSPEED15_Pos)   /*!< 0x80000000 */

/* Legacy defines */
#define GPIO_OSPEEDER_OSPEEDR0           GPIO_OSPEEDR_OSPEED0
#define GPIO_OSPEEDER_OSPEEDR0_0         GPIO_OSPEEDR_OSPEED0_0
#define GPIO_OSPEEDER_OSPEEDR0_1         GPIO_OSPEEDR_OSPEED0_1
#define GPIO_OSPEEDER_OSPEEDR1           GPIO_OSPEEDR_OSPEED1
#define GPIO_OSPEEDER_OSPEEDR1_0         GPIO_OSPEEDR_OSPEED1_0
#define GPIO_OSPEEDER_OSPEEDR1_1         GPIO_OSPEEDR_OSPEED1_1
#define GPIO_OSPEEDER_OSPEEDR2           GPIO_OSPEEDR_OSPEED2
#define GPIO_OSPEEDER_OSPEEDR2_0         GPIO_OSPEEDR_OSPEED2_0
#define GPIO_OSPEEDER_OSPEEDR2_1         GPIO_OSPEEDR_OSPEED2_1
#define GPIO_OSPEEDER_OSPEEDR3           GPIO_OSPEEDR_OSPEED3
#define GPIO_OSPEEDER_OSPEEDR3_0         GPIO_OSPEEDR_OSPEED3_0
#define GPIO_OSPEEDER_OSPEEDR3_1         GPIO_OSPEEDR_OSPEED3_1
#define GPIO_OSPEEDER_OSPEEDR4           GPIO_OSPEEDR_OSPEED4
#define GPIO_OSPEEDER_OSPEEDR4_0         GPIO_OSPEEDR_OSPEED4_0
#define GPIO_OSPEEDER_OSPEEDR4_1         GPIO_OSPEEDR_OSPEED4_1
#define GPIO_OSPEEDER_OSPEEDR5           GPIO_OSPEEDR_OSPEED5
#define GPIO_OSPEEDER_OSPEEDR5_0         GPIO_OSPEEDR_OSPEED5_0
#define GPIO_OSPEEDER_OSPEEDR5_1         GPIO_OSPEEDR_OSPEED5_1
#define GPIO_OSPEEDER_OSPEEDR6           GPIO_OSPEEDR_OSPEED6
#define GPIO_OSPEEDER_OSPEEDR6_0         GPIO_OSPEEDR_OSPEED6_0
#define GPIO_OSPEEDER_OSPEEDR6_1         GPIO_OSPEEDR_OSPEED6_1
#define GPIO_OSPEEDER_OSPEEDR7           GPIO_OSPEEDR_OSPEED7
#define GPIO_OSPEEDER_OSPEEDR7_0         GPIO_OSPEEDR_OSPEED7_0
#define GPIO_OSPEEDER_OSPEEDR7_1         GPIO_OSPEEDR_OSPEED7_1
#define GPIO_OSPEEDER_OSPEEDR8           GPIO_OSPEEDR_OSPEED8
#define GPIO_OSPEEDER_OSPEEDR8_0         GPIO_OSPEEDR_OSPEED8_0
#define GPIO_OSPEEDER_OSPEEDR8_1         GPIO_OSPEEDR_OSPEED8_1
#define GPIO_OSPEEDER_OSPEEDR9           GPIO_OSPEEDR_OSPEED9
#define GPIO_OSPEEDER_OSPEEDR9_0         GPIO_OSPEEDR_OSPEED9_0
#define GPIO_OSPEEDER_OSPEEDR9_1         GPIO_OSPEEDR_OSPEED9_1
#define GPIO_OSPEEDER_OSPEEDR10          GPIO_OSPEEDR_OSPEED10
#define GPIO_OSPEEDER_OSPEEDR10_0        GPIO_OSPEEDR_OSPEED10_0
#define GPIO_OSPEEDER_OSPEEDR10_1        GPIO_OSPEEDR_OSPEED10_1
#define GPIO_OSPEEDER_OSPEEDR11          GPIO_OSPEEDR_OSPEED11
#define GPIO_OSPEEDER_OSPEEDR11_0        GPIO_OSPEEDR_OSPEED11_0
#define GPIO_OSPEEDER_OSPEEDR11_1        GPIO_OSPEEDR_OSPEED11_1
#define GPIO_OSPEEDER_OSPEEDR12          GPIO_OSPEEDR_OSPEED12
#define GPIO_OSPEEDER_OSPEEDR12_0        GPIO_OSPEEDR_OSPEED12_0
#define GPIO_OSPEEDER_OSPEEDR12_1        GPIO_OSPEEDR_OSPEED12_1
#define GPIO_OSPEEDER_OSPEEDR13          GPIO_OSPEEDR_OSPEED13
#define GPIO_OSPEEDER_OSPEEDR13_0        GPIO_OSPEEDR_OSPEED13_0
#define GPIO_OSPEEDER_OSPEEDR13_1        GPIO_OSPEEDR_OSPEED13_1
#define GPIO_OSPEEDER_OSPEEDR14          GPIO_OSPEEDR_OSPEED14
#define GPIO_OSPEEDER_OSPEEDR14_0        GPIO_OSPEEDR_OSPEED14_0
#define GPIO_OSPEEDER_OSPEEDR14_1        GPIO_OSPEEDR_OSPEED14_1
#define GPIO_OSPEEDER_OSPEEDR15          GPIO_OSPEEDR_OSPEED15
#define GPIO_OSPEEDER_OSPEEDR15_0        GPIO_OSPEEDR_OSPEED15_0
#define GPIO_OSPEEDER_OSPEEDR15_1        GPIO_OSPEEDR_OSPEED15_1

/******************  Bits definition for GPIO_PUPDR register  *****************/
#define GPIO_PUPDR_PUPD0_Pos             (0U)
#define GPIO_PUPDR_PUPD0_Msk             (0x3UL << GPIO_PUPDR_PUPD0_Pos)        /*!< 0x00000003 */
#define GPIO_PUPDR_PUPD0                 GPIO_PUPDR_PUPD0_Msk
#define GPIO_PUPDR_PUPD0_0               (0x1UL << GPIO_PUPDR_PUPD0_Pos)        /*!< 0x00000001 */
#define GPIO_PUPDR_PUPD0_1               (0x2UL << GPIO_PUPDR_PUPD0_Pos)        /*!< 0x00000002 */
#define GPIO_PUPDR_PUPD1_Pos             (2U)
#define GPIO_PUPDR_PUPD1_Msk             (0x3UL << GPIO_PUPDR_PUPD1_Pos)        /*!< 0x0000000C */
#define GPIO_PUPDR_PUPD1                 GPIO_PUPDR_PUPD1_Msk
#define GPIO_PUPDR_PUPD1_0               (0x1UL << GPIO_PUPDR_PUPD1_Pos)        /*!< 0x00000004 */
#define GPIO_PUPDR_PUPD1_1               (0x2UL << GPIO_PUPDR_PUPD1_Pos)        /*!< 0x00000008 */
#define GPIO_PUPDR_PUPD2_Pos             (4U)
#define GPIO_PUPDR_PUPD2_Msk             (0x3UL << GPIO_PUPDR_PUPD2_Pos)        /*!< 0x00000030 */
#define GPIO_PUPDR_PUPD2                 GPIO_PUPDR_PUPD2_Msk
#define GPIO_PUPDR_PUPD2_0               (0x1UL << GPIO_PUPDR_PUPD2_Pos)        /*!< 0x00000010 */
#define GPIO_PUPDR_PUPD2_1               (0x2UL << GPIO_PUPDR_PUPD2_Pos)        /*!< 0x00000020 */
#define GPIO_PUPDR_PUPD3_Pos             (6U)
#define GPIO_PUPDR_PUPD3_Msk             (0x3UL << GPIO_PUPDR_PUPD3_Pos)        /*!< 0x000000C0 */
#define GPIO_PUPDR_PUPD3                 GPIO_PUPDR_PUPD3_Msk
#define GPIO_PUPDR_PUPD3_0               (0x1UL << GPIO_PUPDR_PUPD3_Pos)        /*!< 0x00000040 */
#define GPIO_PUPDR_PUPD3_1               (0x2UL << GPIO_PUPDR_PUPD3_Pos)        /*!< 0x00000080 */
#define GPIO_PUPDR_PUPD4_Pos             (8U)
#define GPIO_PUPDR_PUPD4_Msk             (0x3UL << GPIO_PUPDR_PUPD4_Pos)        /*!< 0x00000300 */
#define GPIO_PUPDR_PUPD4                 GPIO_PUPDR_PUPD4_Msk
#define GPIO_PUPDR_PUPD4_0               (0x1UL << GPIO_PUPDR_PUPD4_Pos)        /*!< 0x00000100 */
#define GPIO_PUPDR_PUPD4_1               (0x2UL << GPIO_PUPDR_PUPD4_Pos)        /*!< 0x00000200 */
#define GPIO_PUPDR_PUPD5_Pos             (10U)
#define GPIO_PUPDR_PUPD5_Msk             (0x3UL << GPIO_PUPDR_PUPD5_Pos)        /*!< 0x00000C00 */
#define GPIO_PUPDR_PUPD5                 GPIO_PUPDR_PUPD5_Msk
#define GPIO_PUPDR_PUPD5_0               (0x1UL << GPIO_PUPDR_PUPD5_Pos)        /*!< 0x00000400 */
#define GPIO_PUPDR_PUPD5_1               (0x2UL << GPIO_PUPDR_PUPD5_Pos)        /*!< 0x00000800 */
#define GPIO_PUPDR_PUPD6_Pos             (12U)
#define GPIO_PUPDR_PUPD6_Msk             (0x3UL << GPIO_PUPDR_PUPD6_Pos)        /*!< 0x00003000 */
#define GPIO_PUPDR_PUPD6                 GPIO_PUPDR_PUPD6_Msk
#define GPIO_PUPDR_PUPD6_0               (0x1UL << GPIO_PUPDR_PUPD6_Pos)        /*!< 0x00001000 */
#define GPIO_PUPDR_PUPD6_1               (0x2UL << GPIO_PUPDR_PUPD6_Pos)        /*!< 0x00002000 */
#define GPIO_PUPDR_PUPD7_Pos             (14U)
#define GPIO_PUPDR_PUPD7_Msk             (0x3UL << GPIO_PUPDR_PUPD7_Pos)        /*!< 0x0000C000 */
#define GPIO_PUPDR_PUPD7                 GPIO_PUPDR_PUPD7_Msk
#define GPIO_PUPDR_PUPD7_0               (0x1UL << GPIO_PUPDR_PUPD7_Pos)        /*!< 0x00004000 */
#define GPIO_PUPDR_PUPD7_1               (0x2UL << GPIO_PUPDR_PUPD7_Pos)        /*!< 0x00008000 */
#define GPIO_PUPDR_PUPD8_Pos             (16U)
#define GPIO_PUPDR_PUPD8_Msk             (0x3UL << GPIO_PUPDR_PUPD8_Pos)        /*!< 0x00030000 */
#define GPIO_PUPDR_PUPD8                 GPIO_PUPDR_PUPD8_Msk
#define GPIO_PUPDR_PUPD8_0               (0x1UL << GPIO_PUPDR_PUPD8_Pos)        /*!< 0x00010000 */
#define GPIO_PUPDR_PUPD8_1               (0x2UL << GPIO_PUPDR_PUPD8_Pos)        /*!< 0x00020000 */
#define GPIO_PUPDR_PUPD9_Pos             (18U)
#define GPIO_PUPDR_PUPD9_Msk             (0x3UL << GPIO_PUPDR_PUPD9_Pos)        /*!< 0x000C0000 */
#define GPIO_PUPDR_PUPD9                 GPIO_PUPDR_PUPD9_Msk
#define GPIO_PUPDR_PUPD9_0               (0x1UL << GPIO_PUPDR_PUPD9_Pos)        /*!< 0x00040000 */
#define GPIO_PUPDR_PUPD9_1               (0x2UL << GPIO_PUPDR_PUPD9_Pos)        /*!< 0x00080000 */
#define GPIO_PUPDR_PUPD10_Pos            (20U)
#define GPIO_PUPDR_PUPD10_Msk            (0x3UL << GPIO_PUPDR_PUPD10_Pos)       /*!< 0x00300000 */
#define GPIO_PUPDR_PUPD10                GPIO_PUPDR_PUPD10_Msk
#define GPIO_PUPDR_PUPD10_0              (0x1UL << GPIO_PUPDR_PUPD10_Pos)       /*!< 0x00100000 */
#define GPIO_PUPDR_PUPD10_1              (0x2UL << GPIO_PUPDR_PUPD10_Pos)       /*!< 0x00200000 */
#define GPIO_PUPDR_PUPD11_Pos            (22U)
#define GPIO_PUPDR_PUPD11_Msk            (0x3UL << GPIO_PUPDR_PUPD11_Pos)       /*!< 0x00C00000 */
#define GPIO_PUPDR_PUPD11                GPIO_PUPDR_PUPD11_Msk
#define GPIO_PUPDR_PUPD11_0              (0x1UL << GPIO_PUPDR_PUPD11_Pos)       /*!< 0x00400000 */
#define GPIO_PUPDR_PUPD11_1              (0x2UL << GPIO_PUPDR_PUPD11_Pos)       /*!< 0x00800000 */
#define GPIO_PUPDR_PUPD12_Pos            (24U)
#define GPIO_PUPDR_PUPD12_Msk            (0x3UL << GPIO_PUPDR_PUPD12_Pos)       /*!< 0x03000000 */
#define GPIO_PUPDR_PUPD12                GPIO_PUPDR_PUPD12_Msk
#define GPIO_PUPDR_PUPD12_0              (0x1UL << GPIO_PUPDR_PUPD12_Pos)       /*!< 0x01000000 */
#define GPIO_PUPDR_PUPD12_1              (0x2UL << GPIO_PUPDR_PUPD12_Pos)       /*!< 0x02000000 */
#define GPIO_PUPDR_PUPD13_Pos            (26U)
#define GPIO_PUPDR_PUPD13_Msk            (0x3UL << GPIO_PUPDR_PUPD13_Pos)       /*!< 0x0C000000 */
#define GPIO_PUPDR_PUPD13                GPIO_PUPDR_PUPD13_Msk
#define GPIO_PUPDR_PUPD13_0              (0x1UL << GPIO_PUPDR_PUPD13_Pos)       /*!< 0x04000000 */
#define GPIO_PUPDR_PUPD13_1              (0x2UL << GPIO_PUPDR_PUPD13_Pos)       /*!< 0x08000000 */
#define GPIO_PUPDR_PUPD14_Pos            (28U)
#define GPIO_PUPDR_PUPD14_Msk            (0x3UL << GPIO_PUPDR_PUPD14_Pos)       /*!< 0x30000000 */
#define GPIO_PUPDR_PUPD14                GPIO_PUPDR_PUPD14_Msk
#define GPIO_PUPDR_PUPD14_0              (0x1UL << GPIO_PUPDR_PUPD14_Pos)       /*!< 0x10000000 */
#define GPIO_PUPDR_PUPD14_1              (0x2UL << GPIO_PUPDR_PUPD14_Pos)       /*!< 0x20000000 */
#define GPIO_PUPDR_PUPD15_Pos            (30U)
#define GPIO_PUPDR_PUPD15_Msk            (0x3UL << GPIO_PUPDR_PUPD15_Pos)       /*!< 0xC0000000 */
#define GPIO_PUPDR_PUPD15                GPIO_PUPDR_PUPD15_Msk
#define GPIO_PUPDR_PUPD15_0              (0x1UL << GPIO_PUPDR_PUPD15_Pos)       /*!< 0x40000000 */
#define GPIO_PUPDR_PUPD15_1              (0x2UL << GPIO_PUPDR_PUPD15_Pos)       /*!< 0x80000000 */

/* Legacy defines */
#define GPIO_PUPDR_PUPDR0                GPIO_PUPDR_PUPD0
#define GPIO_PUPDR_PUPDR0_0              GPIO_PUPDR_PUPD0_0
#define GPIO_PUPDR_PUPDR0_1              GPIO_PUPDR_PUPD0_1
#define GPIO_PUPDR_PUPDR1                GPIO_PUPDR_PUPD1
#define GPIO_PUPDR_PUPDR1_0              GPIO_PUPDR_PUPD1_0
#define GPIO_PUPDR_PUPDR1_1              GPIO_PUPDR_PUPD1_1
#define GPIO_PUPDR_PUPDR2                GPIO_PUPDR_PUPD2
#define GPIO_PUPDR_PUPDR2_0              GPIO_PUPDR_PUPD2_0
#define GPIO_PUPDR_PUPDR2_1              GPIO_PUPDR_PUPD2_1
#define GPIO_PUPDR_PUPDR3                GPIO_PUPDR_PUPD3
#define GPIO_PUPDR_PUPDR3_0              GPIO_PUPDR_PUPD3_0
#define GPIO_PUPDR_PUPDR3_1              GPIO_PUPDR_PUPD3_1
#define GPIO_PUPDR_PUPDR4                GPIO_PUPDR_PUPD4
#define GPIO_PUPDR_PUPDR4_0              GPIO_PUPDR_PUPD4_0
#define GPIO_PUPDR_PUPDR4_1              GPIO_PUPDR_PUPD4_1
#define GPIO_PUPDR_PUPDR5                GPIO_PUPDR_PUPD5
#define GPIO_PUPDR_PUPDR5_0              GPIO_PUPDR_PUPD5_0
#define GPIO_PUPDR_PUPDR5_1              GPIO_PUPDR_PUPD5_1
#define GPIO_PUPDR_PUPDR6                GPIO_PUPDR_PUPD6
#define GPIO_PUPDR_PUPDR6_0              GPIO_PUPDR_PUPD6_0
#define GPIO_PUPDR_PUPDR6_1              GPIO_PUPDR_PUPD6_1
#define GPIO_PUPDR_PUPDR7                GPIO_PUPDR_PUPD7
#define GPIO_PUPDR_PUPDR7_0              GPIO_PUPDR_PUPD7_0
#define GPIO_PUPDR_PUPDR7_1              GPIO_PUPDR_PUPD7_1
#define GPIO_PUPDR_PUPDR8                GPIO_PUPDR_PUPD8
#define GPIO_PUPDR_PUPDR8_0              GPIO_PUPDR_PUPD8_0
#define GPIO_PUPDR_PUPDR8_1              GPIO_PUPDR_PUPD8_1
#define GPIO_PUPDR_PUPDR9                GPIO_PUPDR_PUPD9
#define GPIO_PUPDR_PUPDR9_0              GPIO_PUPDR_PUPD9_0
#define GPIO_PUPDR_PUPDR9_1              GPIO_PUPDR_PUPD9_1
#define GPIO_PUPDR_PUPDR10               GPIO_PUPDR_PUPD10
#define GPIO_PUPDR_PUPDR10_0             GPIO_PUPDR_PUPD10_0
#define GPIO_PUPDR_PUPDR10_1             GPIO_PUPDR_PUPD10_1
#define GPIO_PUPDR_PUPDR11               GPIO_PUPDR_PUPD11
#define GPIO_PUPDR_PUPDR11_0             GPIO_PUPDR_PUPD11_0
#define GPIO_PUPDR_PUPDR11_1             GPIO_PUPDR_PUPD11_1
#define GPIO_PUPDR_PUPDR12               GPIO_PUPDR_PUPD12
#define GPIO_PUPDR_PUPDR12_0             GPIO_PUPDR_PUPD12_0
#define GPIO_PUPDR_PUPDR12_1             GPIO_PUPDR_PUPD12_1
#define GPIO_PUPDR_PUPDR13               GPIO_PUPDR_PUPD13
#define GPIO_PUPDR_PUPDR13_0             GPIO_PUPDR_PUPD13_0
#define GPIO_PUPDR_PUPDR13_1             GPIO_PUPDR_PUPD13_1
#define GPIO_PUPDR_PUPDR14               GPIO_PUPDR_PUPD14
#define GPIO_PUPDR_PUPDR14_0             GPIO_PUPDR_PUPD14_0
#define GPIO_PUPDR_PUPDR14_1             GPIO_PUPDR_PUPD14_1
#define GPIO_PUPDR_PUPDR15               GPIO_PUPDR_PUPD15
#define GPIO_PUPDR_PUPDR15_0             GPIO_PUPDR_PUPD15_0
#define GPIO_PUPDR_PUPDR15_1             GPIO_PUPDR_PUPD15_1

/******************  Bits definition for GPIO_IDR register  *******************/
#define GPIO_IDR_ID0_Pos                 (0U)
#define GPIO_IDR_ID0_Msk                 (0x1UL << GPIO_IDR_ID0_Pos)            /*!< 0x00000001 */
#define GPIO_IDR_ID0                     GPIO_IDR_ID0_Msk
#define GPIO_IDR_ID1_Pos                 (1U)
#define GPIO_IDR_ID1_Msk                 (0x1UL << GPIO_IDR_ID1_Pos)            /*!< 0x00000002 */
#define GPIO_IDR_ID1                     GPIO_IDR_ID1_Msk
#define GPIO_IDR_ID2_Pos                 (2U)
#define GPIO_IDR_ID2_Msk                 (0x1UL << GPIO_IDR_ID2_Pos)            /*!< 0x00000004 */
#define GPIO_IDR_ID2                     GPIO_IDR_ID2_Msk
#define GPIO_IDR_ID3_Pos                 (3U)
#define GPIO_IDR_ID3_Msk                 (0x1UL << GPIO_IDR_ID3_Pos)            /*!< 0x00000008 */
#define GPIO_IDR_ID3                     GPIO_IDR_ID3_Msk
#define GPIO_IDR_ID4_Pos                 (4U)
#define GPIO_IDR_ID4_Msk                 (0x1UL << GPIO_IDR_ID4_Pos)            /*!< 0x00000010 */
#define GPIO_IDR_ID4                     GPIO_IDR_ID4_Msk
#define GPIO_IDR_ID5_Pos                 (5U)
#define GPIO_IDR_ID5_Msk                 (0x1UL << GPIO_IDR_ID5_Pos)            /*!< 0x00000020 */
#define GPIO_IDR_ID5                     GPIO_IDR_ID5_Msk
#define GPIO_IDR_ID6_Pos                 (6U)
#define GPIO_IDR_ID6_Msk                 (0x1UL << GPIO_IDR_ID6_Pos)            /*!< 0x00000040 */
#define GPIO_IDR_ID6                     GPIO_IDR_ID6_Msk
#define GPIO_IDR_ID7_Pos                 (7U)
#define GPIO_IDR_ID7_Msk                 (0x1UL << GPIO_IDR_ID7_Pos)            /*!< 0x00000080 */
#define GPIO_IDR_ID7                     GPIO_IDR_ID7_Msk
#define GPIO_IDR_ID8_Pos                 (8U)
#define GPIO_IDR_ID8_Msk                 (0x1UL << GPIO_IDR_ID8_Pos)            /*!< 0x00000100 */
#define GPIO_IDR_ID8                     GPIO_IDR_ID8_Msk
#define GPIO_IDR_ID9_Pos                 (9U)
#define GPIO_IDR_ID9_Msk                 (0x1UL << GPIO_IDR_ID9_Pos)            /*!< 0x00000200 */
#define GPIO_IDR_ID9                     GPIO_IDR_ID9_Msk
#define GPIO_IDR_ID10_Pos                (10U)
#define GPIO_IDR_ID10_Msk                (0x1UL << GPIO_IDR_ID10_Pos)           /*!< 0x00000400 */
#define GPIO_IDR_ID10                    GPIO_IDR_ID10_Msk
#define GPIO_IDR_ID11_Pos                (11U)
#define GPIO_IDR_ID11_Msk                (0x1UL << GPIO_IDR_ID11_Pos)           /*!< 0x00000800 */
#define GPIO_IDR_ID11                    GPIO_IDR_ID11_Msk
#define GPIO_IDR_ID12_Pos                (12U)
#define GPIO_IDR_ID12_Msk                (0x1UL << GPIO_IDR_ID12_Pos)           /*!< 0x00001000 */
#define GPIO_IDR_ID12                    GPIO_IDR_ID12_Msk
#define GPIO_IDR_ID13_Pos                (13U)
#define GPIO_IDR_ID13_Msk                (0x1UL << GPIO_IDR_ID13_Pos)           /*!< 0x00002000 */
#define GPIO_IDR_ID13                    GPIO_IDR_ID13_Msk
#define GPIO_IDR_ID14_Pos                (14U)
#define GPIO_IDR_ID14_Msk                (0x1UL << GPIO_IDR_ID14_Pos)           /*!< 0x00004000 */
#define GPIO_IDR_ID14                    GPIO_IDR_ID14_Msk
#define GPIO_IDR_ID15_Pos                (15U)
#define GPIO_IDR_ID15_Msk                (0x1UL << GPIO_IDR_ID15_Pos)           /*!< 0x00008000 */
#define GPIO_IDR_ID15                    GPIO_IDR_ID15_Msk

/* Legacy defines */
#define GPIO_IDR_IDR_0                   GPIO_IDR_ID0
#define GPIO_IDR_IDR_1                   GPIO_IDR_ID1
#define GPIO_IDR_IDR_2                   GPIO_IDR_ID2
#define GPIO_IDR_IDR_3                   GPIO_IDR_ID3
#define GPIO_IDR_IDR_4                   GPIO_IDR_ID4
#define GPIO_IDR_IDR_5                   GPIO_IDR_ID5
#define GPIO_IDR_IDR_6                   GPIO_IDR_ID6
#define GPIO_IDR_IDR_7                   GPIO_IDR_ID7
#define GPIO_IDR_IDR_8                   GPIO_IDR_ID8
#define GPIO_IDR_IDR_9                   GPIO_IDR_ID9
#define GPIO_IDR_IDR_10                  GPIO_IDR_ID10
#define GPIO_IDR_IDR_11                  GPIO_IDR_ID11
#define GPIO_IDR_IDR_12                  GPIO_IDR_ID12
#define GPIO_IDR_IDR_13                  GPIO_IDR_ID13
#define GPIO_IDR_IDR_14                  GPIO_IDR_ID14
#define GPIO_IDR_IDR_15                  GPIO_IDR_ID15

/******************  Bits definition for GPIO_ODR register  *******************/
#define GPIO_ODR_OD0_Pos                 (0U)
#define GPIO_ODR_OD0_Msk                 (0x1UL << GPIO_ODR_OD0_Pos)            /*!< 0x00000001 */
#define GPIO_ODR_OD0                     GPIO_ODR_OD0_Msk
#define GPIO_ODR_OD1_Pos                 (1U)
#define GPIO_ODR_OD1_Msk                 (0x1UL << GPIO_ODR_OD1_Pos)            /*!< 0x00000002 */
#define GPIO_ODR_OD1                     GPIO_ODR_OD1_Msk
#define GPIO_ODR_OD2_Pos                 (2U)
#define GPIO_ODR_OD2_Msk                 (0x1UL << GPIO_ODR_OD2_Pos)            /*!< 0x00000004 */
#define GPIO_ODR_OD2                     GPIO_ODR_OD2_Msk
#define GPIO_ODR_OD3_Pos                 (3U)
#define GPIO_ODR_OD3_Msk                 (0x1UL << GPIO_ODR_OD3_Pos)            /*!< 0x00000008 */
#define GPIO_ODR_OD3                     GPIO_ODR_OD3_Msk
#define GPIO_ODR_OD4_Pos                 (4U)
#define GPIO_ODR_OD4_Msk                 (0x1UL << GPIO_ODR_OD4_Pos)            /*!< 0x00000010 */
#define GPIO_ODR_OD4                     GPIO_ODR_OD4_Msk
#define GPIO_ODR_OD5_Pos                 (5U)
#define GPIO_ODR_OD5_Msk                 (0x1UL << GPIO_ODR_OD5_Pos)            /*!< 0x00000020 */
#define GPIO_ODR_OD5                     GPIO_ODR_OD5_Msk
#define GPIO_ODR_OD6_Pos                 (6U)
#define GPIO_ODR_OD6_Msk                 (0x1UL << GPIO_ODR_OD6_Pos)            /*!< 0x00000040 */
#define GPIO_ODR_OD6                     GPIO_ODR_OD6_Msk
#define GPIO_ODR_OD7_Pos                 (7U)
#define GPIO_ODR_OD7_Msk                 (0x1UL << GPIO_ODR_OD7_Pos)            /*!< 0x00000080 */
#define GPIO_ODR_OD7                     GPIO_ODR_OD7_Msk
#define GPIO_ODR_OD8_Pos                 (8U)
#define GPIO_ODR_OD8_Msk                 (0x1UL << GPIO_ODR_OD8_Pos)            /*!< 0x00000100 */
#define GPIO_ODR_OD8                     GPIO_ODR_OD8_Msk
#define GPIO_ODR_OD9_Pos                 (9U)
#define GPIO_ODR_OD9_Msk                 (0x1UL << GPIO_ODR_OD9_Pos)            /*!< 0x00000200 */
#define GPIO_ODR_OD9                     GPIO_ODR_OD9_Msk
#define GPIO_ODR_OD10_Pos                (10U)
#define GPIO_ODR_OD10_Msk                (0x1UL << GPIO_ODR_OD10_Pos)           /*!< 0x00000400 */
#define GPIO_ODR_OD10                    GPIO_ODR_OD10_Msk
#define GPIO_ODR_OD11_Pos                (11U)
#define GPIO_ODR_OD11_Msk                (0x1UL << GPIO_ODR_OD11_Pos)           /*!< 0x00000800 */
#define GPIO_ODR_OD11                    GPIO_ODR_OD11_Msk
#define GPIO_ODR_OD12_Pos                (12U)
#define GPIO_ODR_OD12_Msk                (0x1UL << GPIO_ODR_OD12_Pos)           /*!< 0x00001000 */
#define GPIO_ODR_OD12                    GPIO_ODR_OD12_Msk
#define GPIO_ODR_OD13_Pos                (13U)
#define GPIO_ODR_OD13_Msk                (0x1UL << GPIO_ODR_OD13_Pos)           /*!< 0x00002000 */
#define GPIO_ODR_OD13                    GPIO_ODR_OD13_Msk
#define GPIO_ODR_OD14_Pos                (14U)
#define GPIO_ODR_OD14_Msk                (0x1UL << GPIO_ODR_OD14_Pos)           /*!< 0x00004000 */
#define GPIO_ODR_OD14                    GPIO_ODR_OD14_Msk
#define GPIO_ODR_OD15_Pos                (15U)
#define GPIO_ODR_OD15_Msk                (0x1UL << GPIO_ODR_OD15_Pos)           /*!< 0x00008000 */
#define GPIO_ODR_OD15                    GPIO_ODR_OD15_Msk
/* Legacy defines */
#define GPIO_ODR_ODR_0                   GPIO_ODR_OD0
#define GPIO_ODR_ODR_1                   GPIO_ODR_OD1
#define GPIO_ODR_ODR_2                   GPIO_ODR_OD2
#define GPIO_ODR_ODR_3                   GPIO_ODR_OD3
#define GPIO_ODR_ODR_4                   GPIO_ODR_OD4
#define GPIO_ODR_ODR_5                   GPIO_ODR_OD5
#define GPIO_ODR_ODR_6                   GPIO_ODR_OD6
#define GPIO_ODR_ODR_7                   GPIO_ODR_OD7
#define GPIO_ODR_ODR_8                   GPIO_ODR_OD8
#define GPIO_ODR_ODR_9                   GPIO_ODR_OD9
#define GPIO_ODR_ODR_10                  GPIO_ODR_OD10
#define GPIO_ODR_ODR_11                  GPIO_ODR_OD11
#define GPIO_ODR_ODR_12                  GPIO_ODR_OD12
#define GPIO_ODR_ODR_13                  GPIO_ODR_OD13
#define GPIO_ODR_ODR_14                  GPIO_ODR_OD14
#define GPIO_ODR_ODR_15                  GPIO_ODR_OD15

/******************  Bits definition for GPIO_BSRR register  ******************/
#define GPIO_BSRR_BS0_Pos                (0U)
#define GPIO_BSRR_BS0_Msk                (0x1UL << GPIO_BSRR_BS0_Pos)           /*!< 0x00000001 */
#define GPIO_BSRR_BS0                    GPIO_BSRR_BS0_Msk
#define GPIO_BSRR_BS1_Pos                (1U)
#define GPIO_BSRR_BS1_Msk                (0x1UL << GPIO_BSRR_BS1_Pos)           /*!< 0x00000002 */
#define GPIO_BSRR_BS1                    GPIO_BSRR_BS1_Msk
#define GPIO_BSRR_BS2_Pos                (2U)
#define GPIO_BSRR_BS2_Msk                (0x1UL << GPIO_BSRR_BS2_Pos)           /*!< 0x00000004 */
#define GPIO_BSRR_BS2                    GPIO_BSRR_BS2_Msk
#define GPIO_BSRR_BS3_Pos                (3U)
#define GPIO_BSRR_BS3_Msk                (0x1UL << GPIO_BSRR_BS3_Pos)           /*!< 0x00000008 */
#define GPIO_BSRR_BS3                    GPIO_BSRR_BS3_Msk
#define GPIO_BSRR_BS4_Pos                (4U)
#define GPIO_BSRR_BS4_Msk                (0x1UL << GPIO_BSRR_BS4_Pos)           /*!< 0x00000010 */
#define GPIO_BSRR_BS4                    GPIO_BSRR_BS4_Msk
#define GPIO_BSRR_BS5_Pos                (5U)
#define GPIO_BSRR_BS5_Msk                (0x1UL << GPIO_BSRR_BS5_Pos)           /*!< 0x00000020 */
#define GPIO_BSRR_BS5                    GPIO_BSRR_BS5_Msk
#define GPIO_BSRR_BS6_Pos                (6U)
#define GPIO_BSRR_BS6_Msk                (0x1UL << GPIO_BSRR_BS6_Pos)           /*!< 0x00000040 */
#define GPIO_BSRR_BS6                    GPIO_BSRR_BS6_Msk
#define GPIO_BSRR_BS7_Pos                (7U)
#define GPIO_BSRR_BS7_Msk                (0x1UL << GPIO_BSRR_BS7_Pos)           /*!< 0x00000080 */
#define GPIO_BSRR_BS7                    GPIO_BSRR_BS7_Msk
#define GPIO_BSRR_BS8_Pos                (8U)
#define GPIO_BSRR_BS8_Msk                (0x1UL << GPIO_BSRR_BS8_Pos)           /*!< 0x00000100 */
#define GPIO_BSRR_BS8                    GPIO_BSRR_BS8_Msk
#define GPIO_BSRR_BS9_Pos                (9U)
#define GPIO_BSRR_BS9_Msk                (0x1UL << GPIO_BSRR_BS9_Pos)           /*!< 0x00000200 */
#define GPIO_BSRR_BS9                    GPIO_BSRR_BS9_Msk
#define GPIO_BSRR_BS10_Pos               (10U)
#define GPIO_BSRR_BS10_Msk               (0x1UL << GPIO_BSRR_BS10_Pos)          /*!< 0x00000400 */
#define GPIO_BSRR_BS10                   GPIO_BSRR_BS10_Msk
#define GPIO_BSRR_BS11_Pos               (11U)
#define GPIO_BSRR_BS11_Msk               (0x1UL << GPIO_BSRR_BS11_Pos)          /*!< 0x00000800 */
#define GPIO_BSRR_BS11                   GPIO_BSRR_BS11_Msk
#define GPIO_BSRR_BS12_Pos               (12U)
#define GPIO_BSRR_BS12_Msk               (0x1UL << GPIO_BSRR_BS12_Pos)          /*!< 0x00001000 */
#define GPIO_BSRR_BS12                   GPIO_BSRR_BS12_Msk
#define GPIO_BSRR_BS13_Pos               (13U)
#define GPIO_BSRR_BS13_Msk               (0x1UL << GPIO_BSRR_BS13_Pos)          /*!< 0x00002000 */
#define GPIO_BSRR_BS13                   GPIO_BSRR_BS13_Msk
#define GPIO_BSRR_BS14_Pos               (14U)
#define GPIO_BSRR_BS14_Msk               (0x1UL << GPIO_BSRR_BS14_Pos)          /*!< 0x00004000 */
#define GPIO_BSRR_BS14                   GPIO_BSRR_BS14_Msk
#define GPIO_BSRR_BS15_Pos               (15U)
#define GPIO_BSRR_BS15_Msk               (0x1UL << GPIO_BSRR_BS15_Pos)          /*!< 0x00008000 */
#define GPIO_BSRR_BS15                   GPIO_BSRR_BS15_Msk
#define GPIO_BSRR_BR0_Pos                (16U)
#define GPIO_BSRR_BR0_Msk                (0x1UL << GPIO_BSRR_BR0_Pos)           /*!< 0x00010000 */
#define GPIO_BSRR_BR0                    GPIO_BSRR_BR0_Msk
#define GPIO_BSRR_BR1_Pos                (17U)
#define GPIO_BSRR_BR1_Msk                (0x1UL << GPIO_BSRR_BR1_Pos)           /*!< 0x00020000 */
#define GPIO_BSRR_BR1                    GPIO_BSRR_BR1_Msk
#define GPIO_BSRR_BR2_Pos                (18U)
#define GPIO_BSRR_BR2_Msk                (0x1UL << GPIO_BSRR_BR2_Pos)           /*!< 0x00040000 */
#define GPIO_BSRR_BR2                    GPIO_BSRR_BR2_Msk
#define GPIO_BSRR_BR3_Pos                (19U)
#define GPIO_BSRR_BR3_Msk                (0x1UL << GPIO_BSRR_BR3_Pos)           /*!< 0x00080000 */
#define GPIO_BSRR_BR3                    GPIO_BSRR_BR3_Msk
#define GPIO_BSRR_BR4_Pos                (20U)
#define GPIO_BSRR_BR4_Msk                (0x1UL << GPIO_BSRR_BR4_Pos)           /*!< 0x00100000 */
#define GPIO_BSRR_BR4                    GPIO_BSRR_BR4_Msk
#define GPIO_BSRR_BR5_Pos                (21U)
#define GPIO_BSRR_BR5_Msk                (0x1UL << GPIO_BSRR_BR5_Pos)           /*!< 0x00200000 */
#define GPIO_BSRR_BR5                    GPIO_BSRR_BR5_Msk
#define GPIO_BSRR_BR6_Pos                (22U)
#define GPIO_BSRR_BR6_Msk                (0x1UL << GPIO_BSRR_BR6_Pos)           /*!< 0x00400000 */
#define GPIO_BSRR_BR6                    GPIO_BSRR_BR6_Msk
#define GPIO_BSRR_BR7_Pos                (23U)
#define GPIO_BSRR_BR7_Msk                (0x1UL << GPIO_BSRR_BR7_Pos)           /*!< 0x00800000 */
#define GPIO_BSRR_BR7                    GPIO_BSRR_BR7_Msk
#define GPIO_BSRR_BR8_Pos                (24U)
#define GPIO_BSRR_BR8_Msk                (0x1UL << GPIO_BSRR_BR8_Pos)           /*!< 0x01000000 */
#define GPIO_BSRR_BR8                    GPIO_BSRR_BR8_Msk
#define GPIO_BSRR_BR9_Pos                (25U)
#define GPIO_BSRR_BR9_Msk                (0x1UL << GPIO_BSRR_BR9_Pos)           /*!< 0x02000000 */
#define GPIO_BSRR_BR9                    GPIO_BSRR_BR9_Msk
#define GPIO_BSRR_BR10_Pos               (26U)
#define GPIO_BSRR_BR10_Msk               (0x1UL << GPIO_BSRR_BR10_Pos)          /*!< 0x04000000 */
#define GPIO_BSRR_BR10                   GPIO_BSRR_BR10_Msk
#define GPIO_BSRR_BR11_Pos               (27U)
#define GPIO_BSRR_BR11_Msk               (0x1UL << GPIO_BSRR_BR11_Pos)          /*!< 0x08000000 */
#define GPIO_BSRR_BR11                   GPIO_BSRR_BR11_Msk
#define GPIO_BSRR_BR12_Pos               (28U)
#define GPIO_BSRR_BR12_Msk               (0x1UL << GPIO_BSRR_BR12_Pos)          /*!< 0x10000000 */
#define GPIO_BSRR_BR12                   GPIO_BSRR_BR12_Msk
#define GPIO_BSRR_BR13_Pos               (29U)
#define GPIO_BSRR_BR13_Msk               (0x1UL << GPIO_BSRR_BR13_Pos)          /*!< 0x20000000 */
#define GPIO_BSRR_BR13                   GPIO_BSRR_BR13_Msk
#define GPIO_BSRR_BR14_Pos               (30U)
#define GPIO_BSRR_BR14_Msk               (0x1UL << GPIO_BSRR_BR14_Pos)          /*!< 0x40000000 */
#define GPIO_BSRR_BR14                   GPIO_BSRR_BR14_Msk
#define GPIO_BSRR_BR15_Pos               (31U)
#define GPIO_BSRR_BR15_Msk               (0x1UL << GPIO_BSRR_BR15_Pos)          /*!< 0x80000000 */
#define GPIO_BSRR_BR15                   GPIO_BSRR_BR15_Msk

/* Legacy defines */
#define GPIO_BSRR_BS_0                   GPIO_BSRR_BS0
#define GPIO_BSRR_BS_1                   GPIO_BSRR_BS1
#define GPIO_BSRR_BS_2                   GPIO_BSRR_BS2
#define GPIO_BSRR_BS_3                   GPIO_BSRR_BS3
#define GPIO_BSRR_BS_4                   GPIO_BSRR_BS4
#define GPIO_BSRR_BS_5                   GPIO_BSRR_BS5
#define GPIO_BSRR_BS_6                   GPIO_BSRR_BS6
#define GPIO_BSRR_BS_7                   GPIO_BSRR_BS7
#define GPIO_BSRR_BS_8                   GPIO_BSRR_BS8
#define GPIO_BSRR_BS_9                   GPIO_BSRR_BS9
#define GPIO_BSRR_BS_10                  GPIO_BSRR_BS10
#define GPIO_BSRR_BS_11                  GPIO_BSRR_BS11
#define GPIO_BSRR_BS_12                  GPIO_BSRR_BS12
#define GPIO_BSRR_BS_13                  GPIO_BSRR_BS13
#define GPIO_BSRR_BS_14                  GPIO_BSRR_BS14
#define GPIO_BSRR_BS_15                  GPIO_BSRR_BS15
#define GPIO_BSRR_BR_0                   GPIO_BSRR_BR0
#define GPIO_BSRR_BR_1                   GPIO_BSRR_BR1
#define GPIO_BSRR_BR_2                   GPIO_BSRR_BR2
#define GPIO_BSRR_BR_3                   GPIO_BSRR_BR3
#define GPIO_BSRR_BR_4                   GPIO_BSRR_BR4
#define GPIO_BSRR_BR_5                   GPIO_BSRR_BR5
#define GPIO_BSRR_BR_6                   GPIO_BSRR_BR6
#define GPIO_BSRR_BR_7                   GPIO_BSRR_BR7
#define GPIO_BSRR_BR_8                   GPIO_BSRR_BR8
#define GPIO_BSRR_BR_9                   GPIO_BSRR_BR9
#define GPIO_BSRR_BR_10                  GPIO_BSRR_BR10
#define GPIO_BSRR_BR_11                  GPIO_BSRR_BR11
#define GPIO_BSRR_BR_12                  GPIO_BSRR_BR12
#define GPIO_BSRR_BR_13                  GPIO_BSRR_BR13
#define GPIO_BSRR_BR_14                  GPIO_BSRR_BR14
#define GPIO_BSRR_BR_15                  GPIO_BSRR_BR15
#define GPIO_BRR_BR0                     GPIO_BSRR_BR0
#define GPIO_BRR_BR0_Pos                 GPIO_BSRR_BR0_Pos
#define GPIO_BRR_BR0_Msk                 GPIO_BSRR_BR0_Msk
#define GPIO_BRR_BR1                     GPIO_BSRR_BR1
#define GPIO_BRR_BR1_Pos                 GPIO_BSRR_BR1_Pos
#define GPIO_BRR_BR1_Msk                 GPIO_BSRR_BR1_Msk
#define GPIO_BRR_BR2                     GPIO_BSRR_BR2
#define GPIO_BRR_BR2_Pos                 GPIO_BSRR_BR2_Pos
#define GPIO_BRR_BR2_Msk                 GPIO_BSRR_BR2_Msk
#define GPIO_BRR_BR3                     GPIO_BSRR_BR3
#define GPIO_BRR_BR3_Pos                 GPIO_BSRR_BR3_Pos
#define GPIO_BRR_BR3_Msk                 GPIO_BSRR_BR3_Msk
#define GPIO_BRR_BR4                     GPIO_BSRR_BR4
#define GPIO_BRR_BR4_Pos                 GPIO_BSRR_BR4_Pos
#define GPIO_BRR_BR4_Msk                 GPIO_BSRR_BR4_Msk
#define GPIO_BRR_BR5                     GPIO_BSRR_BR5
#define GPIO_BRR_BR5_Pos                 GPIO_BSRR_BR5_Pos
#define GPIO_BRR_BR5_Msk                 GPIO_BSRR_BR5_Msk
#define GPIO_BRR_BR6                     GPIO_BSRR_BR6
#define GPIO_BRR_BR6_Pos                 GPIO_BSRR_BR6_Pos
#define GPIO_BRR_BR6_Msk                 GPIO_BSRR_BR6_Msk
#define GPIO_BRR_BR7                     GPIO_BSRR_BR7
#define GPIO_BRR_BR7_Pos                 GPIO_BSRR_BR7_Pos
#define GPIO_BRR_BR7_Msk                 GPIO_BSRR_BR7_Msk
#define GPIO_BRR_BR8                     GPIO_BSRR_BR8
#define GPIO_BRR_BR8_Pos                 GPIO_BSRR_BR8_Pos
#define GPIO_BRR_BR8_Msk                 GPIO_BSRR_BR8_Msk
#define GPIO_BRR_BR9                     GPIO_BSRR_BR9
#define GPIO_BRR_BR9_Pos                 GPIO_BSRR_BR9_Pos
#define GPIO_BRR_BR9_Msk                 GPIO_BSRR_BR9_Msk
#define GPIO_BRR_BR10                    GPIO_BSRR_BR10
#define GPIO_BRR_BR10_Pos                GPIO_BSRR_BR10_Pos
#define GPIO_BRR_BR10_Msk                GPIO_BSRR_BR10_Msk
#define GPIO_BRR_BR11                    GPIO_BSRR_BR11
#define GPIO_BRR_BR11_Pos                GPIO_BSRR_BR11_Pos
#define GPIO_BRR_BR11_Msk                GPIO_BSRR_BR11_Msk
#define GPIO_BRR_BR12                    GPIO_BSRR_BR12
#define GPIO_BRR_BR12_Pos                GPIO_BSRR_BR12_Pos
#define GPIO_BRR_BR12_Msk                GPIO_BSRR_BR12_Msk
#define GPIO_BRR_BR13                    GPIO_BSRR_BR13
#define GPIO_BRR_BR13_Pos                GPIO_BSRR_BR13_Pos
#define GPIO_BRR_BR13_Msk                GPIO_BSRR_BR13_Msk
#define GPIO_BRR_BR14                    GPIO_BSRR_BR14
#define GPIO_BRR_BR14_Pos                GPIO_BSRR_BR14_Pos
#define GPIO_BRR_BR14_Msk                GPIO_BSRR_BR14_Msk
#define GPIO_BRR_BR15                    GPIO_BSRR_BR15
#define GPIO_BRR_BR15_Pos                GPIO_BSRR_BR15_Pos
#define GPIO_BRR_BR15_Msk                GPIO_BSRR_BR15_Msk
/****************** Bit definition for GPIO_LCKR register *********************/
#define GPIO_LCKR_LCK0_Pos               (0U)
#define GPIO_LCKR_LCK0_Msk               (0x1UL << GPIO_LCKR_LCK0_Pos)          /*!< 0x00000001 */
#define GPIO_LCKR_LCK0                   GPIO_LCKR_LCK0_Msk
#define GPIO_LCKR_LCK1_Pos               (1U)
#define GPIO_LCKR_LCK1_Msk               (0x1UL << GPIO_LCKR_LCK1_Pos)          /*!< 0x00000002 */
#define GPIO_LCKR_LCK1                   GPIO_LCKR_LCK1_Msk
#define GPIO_LCKR_LCK2_Pos               (2U)
#define GPIO_LCKR_LCK2_Msk               (0x1UL << GPIO_LCKR_LCK2_Pos)          /*!< 0x00000004 */
#define GPIO_LCKR_LCK2                   GPIO_LCKR_LCK2_Msk
#define GPIO_LCKR_LCK3_Pos               (3U)
#define GPIO_LCKR_LCK3_Msk               (0x1UL << GPIO_LCKR_LCK3_Pos)          /*!< 0x00000008 */
#define GPIO_LCKR_LCK3                   GPIO_LCKR_LCK3_Msk
#define GPIO_LCKR_LCK4_Pos               (4U)
#define GPIO_LCKR_LCK4_Msk               (0x1UL << GPIO_LCKR_LCK4_Pos)          /*!< 0x00000010 */
#define GPIO_LCKR_LCK4                   GPIO_LCKR_LCK4_Msk
#define GPIO_LCKR_LCK5_Pos               (5U)
#define GPIO_LCKR_LCK5_Msk               (0x1UL << GPIO_LCKR_LCK5_Pos)          /*!< 0x00000020 */
#define GPIO_LCKR_LCK5                   GPIO_LCKR_LCK5_Msk
#define GPIO_LCKR_LCK6_Pos               (6U)
#define GPIO_LCKR_LCK6_Msk               (0x1UL << GPIO_LCKR_LCK6_Pos)          /*!< 0x00000040 */
#define GPIO_LCKR_LCK6                   GPIO_LCKR_LCK6_Msk
#define GPIO_LCKR_LCK7_Pos               (7U)
#define GPIO_LCKR_LCK7_Msk               (0x1UL << GPIO_LCKR_LCK7_Pos)          /*!< 0x00000080 */
#define GPIO_LCKR_LCK7                   GPIO_LCKR_LCK7_Msk
#define GPIO_LCKR_LCK8_Pos               (8U)
#define GPIO_LCKR_LCK8_Msk               (0x1UL << GPIO_LCKR_LCK8_Pos)          /*!< 0x00000100 */
#define GPIO_LCKR_LCK8                   GPIO_LCKR_LCK8_Msk
#define GPIO_LCKR_LCK9_Pos               (9U)
#define GPIO_LCKR_LCK9_Msk               (0x1UL << GPIO_LCKR_LCK9_Pos)          /*!< 0x00000200 */
#define GPIO_LCKR_LCK9                   GPIO_LCKR_LCK9_Msk
#define GPIO_LCKR_LCK10_Pos              (10U)
#define GPIO_LCKR_LCK10_Msk              (0x1UL << GPIO_LCKR_LCK10_Pos)         /*!< 0x00000400 */
#define GPIO_LCKR_LCK10                  GPIO_LCKR_LCK10_Msk
#define GPIO_LCKR_LCK11_Pos              (11U)
#define GPIO_LCKR_LCK11_Msk              (0x1UL << GPIO_LCKR_LCK11_Pos)         /*!< 0x00000800 */
#define GPIO_LCKR_LCK11                  GPIO_LCKR_LCK11_Msk
#define GPIO_LCKR_LCK12_Pos              (12U)
#define GPIO_LCKR_LCK12_Msk              (0x1UL << GPIO_LCKR_LCK12_Pos)         /*!< 0x00001000 */
#define GPIO_LCKR_LCK12                  GPIO_LCKR_LCK12_Msk
#define GPIO_LCKR_LCK13_Pos              (13U)
#define GPIO_LCKR_LCK13_Msk              (0x1UL << GPIO_LCKR_LCK13_Pos)         /*!< 0x00002000 */
#define GPIO_LCKR_LCK13                  GPIO_LCKR_LCK13_Msk
#define GPIO_LCKR_LCK14_Pos              (14U)
#define GPIO_LCKR_LCK14_Msk              (0x1UL << GPIO_LCKR_LCK14_Pos)         /*!< 0x00004000 */
#define GPIO_LCKR_LCK14                  GPIO_LCKR_LCK14_Msk
#define GPIO_LCKR_LCK15_Pos              (15U)
#define GPIO_LCKR_LCK15_Msk              (0x1UL << GPIO_LCKR_LCK15_Pos)         /*!< 0x00008000 */
#define GPIO_LCKR_LCK15                  GPIO_LCKR_LCK15_Msk
#define GPIO_LCKR_LCKK_Pos               (16U)
#define GPIO_LCKR_LCKK_Msk               (0x1UL << GPIO_LCKR_LCKK_Pos)          /*!< 0x00010000 */
#define GPIO_LCKR_LCKK                   GPIO_LCKR_LCKK_Msk
/****************** Bit definition for GPIO_AFRL register *********************/
#define GPIO_AFRL_AFSEL0_Pos             (0U)
#define GPIO_AFRL_AFSEL0_Msk             (0xFUL << GPIO_AFRL_AFSEL0_Pos)        /*!< 0x0000000F */
#define GPIO_AFRL_AFSEL0                 GPIO_AFRL_AFSEL0_Msk
#define GPIO_AFRL_AFSEL0_0               (0x1UL << GPIO_AFRL_AFSEL0_Pos)        /*!< 0x00000001 */
#define GPIO_AFRL_AFSEL0_1               (0x2UL << GPIO_AFRL_AFSEL0_Pos)        /*!< 0x00000002 */
#define GPIO_AFRL_AFSEL0_2               (0x4UL << GPIO_AFRL_AFSEL0_Pos)        /*!< 0x00000004 */
#define GPIO_AFRL_AFSEL0_3               (0x8UL << GPIO_AFRL_AFSEL0_Pos)        /*!< 0x00000008 */
#define GPIO_AFRL_AFSEL1_Pos             (4U)
#define GPIO_AFRL_AFSEL1_Msk             (0xFUL << GPIO_AFRL_AFSEL1_Pos)        /*!< 0x000000F0 */
#define GPIO_AFRL_AFSEL1                 GPIO_AFRL_AFSEL1_Msk
#define GPIO_AFRL_AFSEL1_0               (0x1UL << GPIO_AFRL_AFSEL1_Pos)        /*!< 0x00000010 */
#define GPIO_AFRL_AFSEL1_1               (0x2UL << GPIO_AFRL_AFSEL1_Pos)        /*!< 0x00000020 */
#define GPIO_AFRL_AFSEL1_2               (0x4UL << GPIO_AFRL_AFSEL1_Pos)        /*!< 0x00000040 */
#define GPIO_AFRL_AFSEL1_3               (0x8UL << GPIO_AFRL_AFSEL1_Pos)        /*!< 0x00000080 */
#define GPIO_AFRL_AFSEL2_Pos             (8U)
#define GPIO_AFRL_AFSEL2_Msk             (0xFUL << GPIO_AFRL_AFSEL2_Pos)        /*!< 0x00000F00 */
#define GPIO_AFRL_AFSEL2                 GPIO_AFRL_AFSEL2_Msk
#define GPIO_AFRL_AFSEL2_0               (0x1UL << GPIO_AFRL_AFSEL2_Pos)        /*!< 0x00000100 */
#define GPIO_AFRL_AFSEL2_1               (0x2UL << GPIO_AFRL_AFSEL2_Pos)        /*!< 0x00000200 */
#define GPIO_AFRL_AFSEL2_2               (0x4UL << GPIO_AFRL_AFSEL2_Pos)        /*!< 0x00000400 */
#define GPIO_AFRL_AFSEL2_3               (0x8UL << GPIO_AFRL_AFSEL2_Pos)        /*!< 0x00000800 */
#define GPIO_AFRL_AFSEL3_Pos             (12U)
#define GPIO_AFRL_AFSEL3_Msk             (0xFUL << GPIO_AFRL_AFSEL3_Pos)        /*!< 0x0000F000 */
#define GPIO_AFRL_AFSEL3                 GPIO_AFRL_AFSEL3_Msk
#define GPIO_AFRL_AFSEL3_0               (0x1UL << GPIO_AFRL_AFSEL3_Pos)        /*!< 0x00001000 */
#define GPIO_AFRL_AFSEL3_1               (0x2UL << GPIO_AFRL_AFSEL3_Pos)        /*!< 0x00002000 */
#define GPIO_AFRL_AFSEL3_2               (0x4UL << GPIO_AFRL_AFSEL3_Pos)        /*!< 0x00004000 */
#define GPIO_AFRL_AFSEL3_3               (0x8UL << GPIO_AFRL_AFSEL3_Pos)        /*!< 0x00008000 */
#define GPIO_AFRL_AFSEL4_Pos             (16U)
#define GPIO_AFRL_AFSEL4_Msk             (0xFUL << GPIO_AFRL_AFSEL4_Pos)        /*!< 0x000F0000 */
#define GPIO_AFRL_AFSEL4                 GPIO_AFRL_AFSEL4_Msk
#define GPIO_AFRL_AFSEL4_0               (0x1UL << GPIO_AFRL_AFSEL4_Pos)        /*!< 0x00010000 */
#define GPIO_AFRL_AFSEL4_1               (0x2UL << GPIO_AFRL_AFSEL4_Pos)        /*!< 0x00020000 */
#define GPIO_AFRL_AFSEL4_2               (0x4UL << GPIO_AFRL_AFSEL4_Pos)        /*!< 0x00040000 */
#define GPIO_AFRL_AFSEL4_3               (0x8UL << GPIO_AFRL_AFSEL4_Pos)        /*!< 0x00080000 */
#define GPIO_AFRL_AFSEL5_Pos             (20U)
#define GPIO_AFRL_AFSEL5_Msk             (0xFUL << GPIO_AFRL_AFSEL5_Pos)        /*!< 0x00F00000 */
#define GPIO_AFRL_AFSEL5                 GPIO_AFRL_AFSEL5_Msk
#define GPIO_AFRL_AFSEL5_0               (0x1UL << GPIO_AFRL_AFSEL5_Pos)        /*!< 0x00100000 */
#define GPIO_AFRL_AFSEL5_1               (0x2UL << GPIO_AFRL_AFSEL5_Pos)        /*!< 0x00200000 */
#define GPIO_AFRL_AFSEL5_2               (0x4UL << GPIO_AFRL_AFSEL5_Pos)        /*!< 0x00400000 */
#define GPIO_AFRL_AFSEL5_3               (0x8UL << GPIO_AFRL_AFSEL5_Pos)        /*!< 0x00800000 */
#define GPIO_AFRL_AFSEL6_Pos             (24U)
#define GPIO_AFRL_AFSEL6_Msk             (0xFUL << GPIO_AFRL_AFSEL6_Pos)        /*!< 0x0F000000 */
#define GPIO_AFRL_AFSEL6                 GPIO_AFRL_AFSEL6_Msk
#define GPIO_AFRL_AFSEL6_0               (0x1UL << GPIO_AFRL_AFSEL6_Pos)        /*!< 0x01000000 */
#define GPIO_AFRL_AFSEL6_1               (0x2UL << GPIO_AFRL_AFSEL6_Pos)        /*!< 0x02000000 */
#define GPIO_AFRL_AFSEL6_2               (0x4UL << GPIO_AFRL_AFSEL6_Pos)        /*!< 0x04000000 */
#define GPIO_AFRL_AFSEL6_3               (0x8UL << GPIO_AFRL_AFSEL6_Pos)        /*!< 0x08000000 */
#define GPIO_AFRL_AFSEL7_Pos             (28U)
#define GPIO_AFRL_AFSEL7_Msk             (0xFUL << GPIO_AFRL_AFSEL7_Pos)        /*!< 0xF0000000 */
#define GPIO_AFRL_AFSEL7                 GPIO_AFRL_AFSEL7_Msk
#define GPIO_AFRL_AFSEL7_0               (0x1UL << GPIO_AFRL_AFSEL7_Pos)        /*!< 0x10000000 */
#define GPIO_AFRL_AFSEL7_1               (0x2UL << GPIO_AFRL_AFSEL7_Pos)        /*!< 0x20000000 */
#define GPIO_AFRL_AFSEL7_2               (0x4UL << GPIO_AFRL_AFSEL7_Pos)        /*!< 0x40000000 */
#define GPIO_AFRL_AFSEL7_3               (0x8UL << GPIO_AFRL_AFSEL7_Pos)        /*!< 0x80000000 */

/* Legacy defines */
#define GPIO_AFRL_AFRL0                  GPIO_AFRL_AFSEL0
#define GPIO_AFRL_AFRL0_0                GPIO_AFRL_AFSEL0_0
#define GPIO_AFRL_AFRL0_1                GPIO_AFRL_AFSEL0_1
#define GPIO_AFRL_AFRL0_2                GPIO_AFRL_AFSEL0_2
#define GPIO_AFRL_AFRL0_3                GPIO_AFRL_AFSEL0_3
#define GPIO_AFRL_AFRL1                  GPIO_AFRL_AFSEL1
#define GPIO_AFRL_AFRL1_0                GPIO_AFRL_AFSEL1_0
#define GPIO_AFRL_AFRL1_1                GPIO_AFRL_AFSEL1_1
#define GPIO_AFRL_AFRL1_2                GPIO_AFRL_AFSEL1_2
#define GPIO_AFRL_AFRL1_3                GPIO_AFRL_AFSEL1_3
#define GPIO_AFRL_AFRL2                  GPIO_AFRL_AFSEL2
#define GPIO_AFRL_AFRL2_0                GPIO_AFRL_AFSEL2_0
#define GPIO_AFRL_AFRL2_1                GPIO_AFRL_AFSEL2_1
#define GPIO_AFRL_AFRL2_2                GPIO_AFRL_AFSEL2_2
#define GPIO_AFRL_AFRL2_3                GPIO_AFRL_AFSEL2_3
#define GPIO_AFRL_AFRL3                  GPIO_AFRL_AFSEL3
#define GPIO_AFRL_AFRL3_0                GPIO_AFRL_AFSEL3_0
#define GPIO_AFRL_AFRL3_1                GPIO_AFRL_AFSEL3_1
#define GPIO_AFRL_AFRL3_2                GPIO_AFRL_AFSEL3_2
#define GPIO_AFRL_AFRL3_3                GPIO_AFRL_AFSEL3_3
#define GPIO_AFRL_AFRL4                  GPIO_AFRL_AFSEL4
#define GPIO_AFRL_AFRL4_0                GPIO_AFRL_AFSEL4_0
#define GPIO_AFRL_AFRL4_1                GPIO_AFRL_AFSEL4_1
#define GPIO_AFRL_AFRL4_2                GPIO_AFRL_AFSEL4_2
#define GPIO_AFRL_AFRL4_3                GPIO_AFRL_AFSEL4_3
#define GPIO_AFRL_AFRL5                  GPIO_AFRL_AFSEL5
#define GPIO_AFRL_AFRL5_0                GPIO_AFRL_AFSEL5_0
#define GPIO_AFRL_AFRL5_1                GPIO_AFRL_AFSEL5_1
#define GPIO_AFRL_AFRL5_2                GPIO_AFRL_AFSEL5_2
#define GPIO_AFRL_AFRL5_3                GPIO_AFRL_AFSEL5_3
#define GPIO_AFRL_AFRL6                  GPIO_AFRL_AFSEL6
#define GPIO_AFRL_AFRL6_0                GPIO_AFRL_AFSEL6_0
#define GPIO_AFRL_AFRL6_1                GPIO_AFRL_AFSEL6_1
#define GPIO_AFRL_AFRL6_2                GPIO_AFRL_AFSEL6_2
#define GPIO_AFRL_AFRL6_3                GPIO_AFRL_AFSEL6_3
#define GPIO_AFRL_AFRL7                  GPIO_AFRL_AFSEL7
#define GPIO_AFRL_AFRL7_0                GPIO_AFRL_AFSEL7_0
#define GPIO_AFRL_AFRL7_1                GPIO_AFRL_AFSEL7_1
#define GPIO_AFRL_AFRL7_2                GPIO_AFRL_AFSEL7_2
#define GPIO_AFRL_AFRL7_3                GPIO_AFRL_AFSEL7_3

/****************** Bit definition for GPIO_AFRH register *********************/
#define GPIO_AFRH_AFSEL8_Pos             (0U)
#define GPIO_AFRH_AFSEL8_Msk             (0xFUL << GPIO_AFRH_AFSEL8_Pos)        /*!< 0x0000000F */
#define GPIO_AFRH_AFSEL8                 GPIO_AFRH_AFSEL8_Msk
#define GPIO_AFRH_AFSEL8_0               (0x1UL << GPIO_AFRH_AFSEL8_Pos)        /*!< 0x00000001 */
#define GPIO_AFRH_AFSEL8_1               (0x2UL << GPIO_AFRH_AFSEL8_Pos)        /*!< 0x00000002 */
#define GPIO_AFRH_AFSEL8_2               (0x4UL << GPIO_AFRH_AFSEL8_Pos)        /*!< 0x00000004 */
#define GPIO_AFRH_AFSEL8_3               (0x8UL << GPIO_AFRH_AFSEL8_Pos)        /*!< 0x00000008 */
#define GPIO_AFRH_AFSEL9_Pos             (4U)
#define GPIO_AFRH_AFSEL9_Msk             (0xFUL << GPIO_AFRH_AFSEL9_Pos)        /*!< 0x000000F0 */
#define GPIO_AFRH_AFSEL9                 GPIO_AFRH_AFSEL9_Msk
#define GPIO_AFRH_AFSEL9_0               (0x1UL << GPIO_AFRH_AFSEL9_Pos)        /*!< 0x00000010 */
#define GPIO_AFRH_AFSEL9_1               (0x2UL << GPIO_AFRH_AFSEL9_Pos)        /*!< 0x00000020 */
#define GPIO_AFRH_AFSEL9_2               (0x4UL << GPIO_AFRH_AFSEL9_Pos)        /*!< 0x00000040 */
#define GPIO_AFRH_AFSEL9_3               (0x8UL << GPIO_AFRH_AFSEL9_Pos)        /*!< 0x00000080 */
#define GPIO_AFRH_AFSEL10_Pos            (8U)
#define GPIO_AFRH_AFSEL10_Msk            (0xFUL << GPIO_AFRH_AFSEL10_Pos)       /*!< 0x00000F00 */
#define GPIO_AFRH_AFSEL10                GPIO_AFRH_AFSEL10_Msk
#define GPIO_AFRH_AFSEL10_0              (0x1UL << GPIO_AFRH_AFSEL10_Pos)       /*!< 0x00000100 */
#define GPIO_AFRH_AFSEL10_1              (0x2UL << GPIO_AFRH_AFSEL10_Pos)       /*!< 0x00000200 */
#define GPIO_AFRH_AFSEL10_2              (0x4UL << GPIO_AFRH_AFSEL10_Pos)       /*!< 0x00000400 */
#define GPIO_AFRH_AFSEL10_3              (0x8UL << GPIO_AFRH_AFSEL10_Pos)       /*!< 0x00000800 */
#define GPIO_AFRH_AFSEL11_Pos            (12U)
#define GPIO_AFRH_AFSEL11_Msk            (0xFUL << GPIO_AFRH_AFSEL11_Pos)       /*!< 0x0000F000 */
#define GPIO_AFRH_AFSEL11                GPIO_AFRH_AFSEL11_Msk
#define GPIO_AFRH_AFSEL11_0              (0x1UL << GPIO_AFRH_AFSEL11_Pos)       /*!< 0x00001000 */
#define GPIO_AFRH_AFSEL11_1              (0x2UL << GPIO_AFRH_AFSEL11_Pos)       /*!< 0x00002000 */
#define GPIO_AFRH_AFSEL11_2              (0x4UL << GPIO_AFRH_AFSEL11_Pos)       /*!< 0x00004000 */
#define GPIO_AFRH_AFSEL11_3              (0x8UL << GPIO_AFRH_AFSEL11_Pos)       /*!< 0x00008000 */
#define GPIO_AFRH_AFSEL12_Pos            (16U)
#define GPIO_AFRH_AFSEL12_Msk            (0xFUL << GPIO_AFRH_AFSEL12_Pos)       /*!< 0x000F0000 */
#define GPIO_AFRH_AFSEL12                GPIO_AFRH_AFSEL12_Msk
#define GPIO_AFRH_AFSEL12_0              (0x1UL << GPIO_AFRH_AFSEL12_Pos)       /*!< 0x00010000 */
#define GPIO_AFRH_AFSEL12_1              (0x2UL << GPIO_AFRH_AFSEL12_Pos)       /*!< 0x00020000 */
#define GPIO_AFRH_AFSEL12_2              (0x4UL << GPIO_AFRH_AFSEL12_Pos)       /*!< 0x00040000 */
#define GPIO_AFRH_AFSEL12_3              (0x8UL << GPIO_AFRH_AFSEL12_Pos)       /*!< 0x00080000 */
#define GPIO_AFRH_AFSEL13_Pos            (20U)
#define GPIO_AFRH_AFSEL13_Msk            (0xFUL << GPIO_AFRH_AFSEL13_Pos)       /*!< 0x00F00000 */
#define GPIO_AFRH_AFSEL13                GPIO_AFRH_AFSEL13_Msk
#define GPIO_AFRH_AFSEL13_0              (0x1UL << GPIO_AFRH_AFSEL13_Pos)       /*!< 0x00100000 */
#define GPIO_AFRH_AFSEL13_1              (0x2UL << GPIO_AFRH_AFSEL13_Pos)       /*!< 0x00200000 */
#define GPIO_AFRH_AFSEL13_2              (0x4UL << GPIO_AFRH_AFSEL13_Pos)       /*!< 0x00400000 */
#define GPIO_AFRH_AFSEL13_3              (0x8UL << GPIO_AFRH_AFSEL13_Pos)       /*!< 0x00800000 */
#define GPIO_AFRH_AFSEL14_Pos            (24U)
#define GPIO_AFRH_AFSEL14_Msk            (0xFUL << GPIO_AFRH_AFSEL14_Pos)       /*!< 0x0F000000 */
#define GPIO_AFRH_AFSEL14                GPIO_AFRH_AFSEL14_Msk
#define GPIO_AFRH_AFSEL14_0              (0x1UL << GPIO_AFRH_AFSEL14_Pos)       /*!< 0x01000000 */
#define GPIO_AFRH_AFSEL14_1              (0x2UL << GPIO_AFRH_AFSEL14_Pos)       /*!< 0x02000000 */
#define GPIO_AFRH_AFSEL14_2              (0x4UL << GPIO_AFRH_AFSEL14_Pos)       /*!< 0x04000000 */
#define GPIO_AFRH_AFSEL14_3              (0x8UL << GPIO_AFRH_AFSEL14_Pos)       /*!< 0x08000000 */
#define GPIO_AFRH_AFSEL15_Pos            (28U)
#define GPIO_AFRH_AFSEL15_Msk            (0xFUL << GPIO_AFRH_AFSEL15_Pos)       /*!< 0xF0000000 */
#define GPIO_AFRH_AFSEL15                GPIO_AFRH_AFSEL15_Msk
#define GPIO_AFRH_AFSEL15_0              (0x1UL << GPIO_AFRH_AFSEL15_Pos)       /*!< 0x10000000 */
#define GPIO_AFRH_AFSEL15_1              (0x2UL << GPIO_AFRH_AFSEL15_Pos)       /*!< 0x20000000 */
#define GPIO_AFRH_AFSEL15_2              (0x4UL << GPIO_AFRH_AFSEL15_Pos)       /*!< 0x40000000 */
#define GPIO_AFRH_AFSEL15_3              (0x8UL << GPIO_AFRH_AFSEL15_Pos)       /*!< 0x80000000 */

/* Legacy defines */
#define GPIO_AFRH_AFRH0                  GPIO_AFRH_AFSEL8
#define GPIO_AFRH_AFRH0_0                GPIO_AFRH_AFSEL8_0
#define GPIO_AFRH_AFRH0_1                GPIO_AFRH_AFSEL8_1
#define GPIO_AFRH_AFRH0_2                GPIO_AFRH_AFSEL8_2
#define GPIO_AFRH_AFRH0_3                GPIO_AFRH_AFSEL8_3
#define GPIO_AFRH_AFRH1                  GPIO_AFRH_AFSEL9
#define GPIO_AFRH_AFRH1_0                GPIO_AFRH_AFSEL9_0
#define GPIO_AFRH_AFRH1_1                GPIO_AFRH_AFSEL9_1
#define GPIO_AFRH_AFRH1_2                GPIO_AFRH_AFSEL9_2
#define GPIO_AFRH_AFRH1_3                GPIO_AFRH_AFSEL9_3
#define GPIO_AFRH_AFRH2                  GPIO_AFRH_AFSEL10
#define GPIO_AFRH_AFRH2_0                GPIO_AFRH_AFSEL10_0
#define GPIO_AFRH_AFRH2_1                GPIO_AFRH_AFSEL10_1
#define GPIO_AFRH_AFRH2_2                GPIO_AFRH_AFSEL10_2
#define GPIO_AFRH_AFRH2_3                GPIO_AFRH_AFSEL10_3
#define GPIO_AFRH_AFRH3                  GPIO_AFRH_AFSEL11
#define GPIO_AFRH_AFRH3_0                GPIO_AFRH_AFSEL11_0
#define GPIO_AFRH_AFRH3_1                GPIO_AFRH_AFSEL11_1
#define GPIO_AFRH_AFRH3_2                GPIO_AFRH_AFSEL11_2
#define GPIO_AFRH_AFRH3_3                GPIO_AFRH_AFSEL11_3
#define GPIO_AFRH_AFRH4                  GPIO_AFRH_AFSEL12
#define GPIO_AFRH_AFRH4_0                GPIO_AFRH_AFSEL12_0
#define GPIO_AFRH_AFRH4_1                GPIO_AFRH_AFSEL12_1
#define GPIO_AFRH_AFRH4_2                GPIO_AFRH_AFSEL12_2
#define GPIO_AFRH_AFRH4_3                GPIO_AFRH_AFSEL12_3
#define GPIO_AFRH_AFRH5                  GPIO_AFRH_AFSEL13
#define GPIO_AFRH_AFRH5_0                GPIO_AFRH_AFSEL13_0
#define GPIO_AFRH_AFRH5_1                GPIO_AFRH_AFSEL13_1
#define GPIO_AFRH_AFRH5_2                GPIO_AFRH_AFSEL13_2
#define GPIO_AFRH_AFRH5_3                GPIO_AFRH_AFSEL13_3
#define GPIO_AFRH_AFRH6                  GPIO_AFRH_AFSEL14
#define GPIO_AFRH_AFRH6_0                GPIO_AFRH_AFSEL14_0
#define GPIO_AFRH_AFRH6_1                GPIO_AFRH_AFSEL14_1
#define GPIO_AFRH_AFRH6_2                GPIO_AFRH_AFSEL14_2
#define GPIO_AFRH_AFRH6_3                GPIO_AFRH_AFSEL14_3
#define GPIO_AFRH_AFRH7                  GPIO_AFRH_AFSEL15
#define GPIO_AFRH_AFRH7_0                GPIO_AFRH_AFSEL15_0
#define GPIO_AFRH_AFRH7_1                GPIO_AFRH_AFSEL15_1
#define GPIO_AFRH_AFRH7_2                GPIO_AFRH_AFSEL15_2
#define GPIO_AFRH_AFRH7_3                GPIO_AFRH_AFSEL15_3


/******************************************************************************/
/*                                                                            */
/*                      Inter-integrated Circuit Interface                    */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for I2C_CR1 register  ********************/
#define I2C_CR1_PE_Pos            (0U)
#define I2C_CR1_PE_Msk            (0x1UL << I2C_CR1_PE_Pos)                     /*!< 0x00000001 */
#define I2C_CR1_PE                I2C_CR1_PE_Msk                               /*!<Peripheral Enable                             */
#define I2C_CR1_SMBUS_Pos         (1U)
#define I2C_CR1_SMBUS_Msk         (0x1UL << I2C_CR1_SMBUS_Pos)                  /*!< 0x00000002 */
#define I2C_CR1_SMBUS             I2C_CR1_SMBUS_Msk                            /*!<SMBus Mode                                    */
#define I2C_CR1_SMBTYPE_Pos       (3U)
#define I2C_CR1_SMBTYPE_Msk       (0x1UL << I2C_CR1_SMBTYPE_Pos)                /*!< 0x00000008 */
#define I2C_CR1_SMBTYPE           I2C_CR1_SMBTYPE_Msk                          /*!<SMBus Type                                    */
#define I2C_CR1_ENARP_Pos         (4U)
#define I2C_CR1_ENARP_Msk         (0x1UL << I2C_CR1_ENARP_Pos)                  /*!< 0x00000010 */
#define I2C_CR1_ENARP             I2C_CR1_ENARP_Msk                            /*!<ARP Enable                                    */
#define I2C_CR1_ENPEC_Pos         (5U)
#define I2C_CR1_ENPEC_Msk         (0x1UL << I2C_CR1_ENPEC_Pos)                  /*!< 0x00000020 */
#define I2C_CR1_ENPEC             I2C_CR1_ENPEC_Msk                            /*!<PEC Enable                                    */
#define I2C_CR1_ENGC_Pos          (6U)
#define I2C_CR1_ENGC_Msk          (0x1UL << I2C_CR1_ENGC_Pos)                   /*!< 0x00000040 */
#define I2C_CR1_ENGC              I2C_CR1_ENGC_Msk                             /*!<General Call Enable                           */
#define I2C_CR1_NOSTRETCH_Pos     (7U)
#define I2C_CR1_NOSTRETCH_Msk     (0x1UL << I2C_CR1_NOSTRETCH_Pos)              /*!< 0x00000080 */
#define I2C_CR1_NOSTRETCH         I2C_CR1_NOSTRETCH_Msk                        /*!<Clock Stretching Disable (Slave mode)         */
#define I2C_CR1_START_Pos         (8U)
#define I2C_CR1_START_Msk         (0x1UL << I2C_CR1_START_Pos)                  /*!< 0x00000100 */
#define I2C_CR1_START             I2C_CR1_START_Msk                            /*!<Start Generation                              */
#define I2C_CR1_STOP_Pos          (9U)
#define I2C_CR1_STOP_Msk          (0x1UL << I2C_CR1_STOP_Pos)                   /*!< 0x00000200 */
#define I2C_CR1_STOP              I2C_CR1_STOP_Msk                             /*!<Stop Generation                               */
#define I2C_CR1_ACK_Pos           (10U)
#define I2C_CR1_ACK_Msk           (0x1UL << I2C_CR1_ACK_Pos)                    /*!< 0x00000400 */
#define I2C_CR1_ACK               I2C_CR1_ACK_Msk                              /*!<Acknowledge Enable                            */
#define I2C_CR1_POS_Pos           (11U)
#define I2C_CR1_POS_Msk           (0x1UL << I2C_CR1_POS_Pos)                    /*!< 0x00000800 */
#define I2C_CR1_POS               I2C_CR1_POS_Msk                              /*!<Acknowledge/PEC Position (for data reception) */
#define I2C_CR1_PEC_Pos           (12U)
#define I2C_CR1_PEC_Msk           (0x1UL << I2C_CR1_PEC_Pos)                    /*!< 0x00001000 */
#define I2C_CR1_PEC               I2C_CR1_PEC_Msk                              /*!<Packet Error Checking                         */
#define I2C_CR1_ALERT_Pos         (13U)
#define I2C_CR1_ALERT_Msk         (0x1UL << I2C_CR1_ALERT_Pos)                  /*!< 0x00002000 */
#define I2C_CR1_ALERT             I2C_CR1_ALERT_Msk                            /*!<SMBus Alert                                   */
#define I2C_CR1_SWRST_Pos         (15U)
#define I2C_CR1_SWRST_Msk         (0x1UL << I2C_CR1_SWRST_Pos)                  /*!< 0x00008000 */
#define I2C_CR1_SWRST             I2C_CR1_SWRST_Msk                            /*!<Software Reset                                */

/*******************  Bit definition for I2C_CR2 register  ********************/
#define I2C_CR2_FREQ_Pos          (0U)
#define I2C_CR2_FREQ_Msk          (0x3FUL << I2C_CR2_FREQ_Pos)                  /*!< 0x0000003F */
#define I2C_CR2_FREQ              I2C_CR2_FREQ_Msk                             /*!<FREQ[5:0] bits (Peripheral Clock Frequency)   */
#define I2C_CR2_FREQ_0            (0x01UL << I2C_CR2_FREQ_Pos)                  /*!< 0x00000001 */
#define I2C_CR2_FREQ_1            (0x02UL << I2C_CR2_FREQ_Pos)                  /*!< 0x00000002 */
#define I2C_CR2_FREQ_2            (0x04UL << I2C_CR2_FREQ_Pos)                  /*!< 0x00000004 */
#define I2C_CR2_FREQ_3            (0x08UL << I2C_CR2_FREQ_Pos)                  /*!< 0x00000008 */
#define I2C_CR2_FREQ_4            (0x10UL << I2C_CR2_FREQ_Pos)                  /*!< 0x00000010 */
#define I2C_CR2_FREQ_5            (0x20UL << I2C_CR2_FREQ_Pos)                  /*!< 0x00000020 */

#define I2C_CR2_ITERREN_Pos       (8U)
#define I2C_CR2_ITERREN_Msk       (0x1UL << I2C_CR2_ITERREN_Pos)                /*!< 0x00000100 */
#define I2C_CR2_ITERREN           I2C_CR2_ITERREN_Msk                          /*!<Error Interrupt Enable  */
#define I2C_CR2_ITEVTEN_Pos       (9U)
#define I2C_CR2_ITEVTEN_Msk       (0x1UL << I2C_CR2_ITEVTEN_Pos)                /*!< 0x00000200 */
#define I2C_CR2_ITEVTEN           I2C_CR2_ITEVTEN_Msk                          /*!<Event Interrupt Enable  */
#define I2C_CR2_ITBUFEN_Pos       (10U)
#define I2C_CR2_ITBUFEN_Msk       (0x1UL << I2C_CR2_ITBUFEN_Pos)                /*!< 0x00000400 */
#define I2C_CR2_ITBUFEN           I2C_CR2_ITBUFEN_Msk                          /*!<Buffer Interrupt Enable */
#define I2C_CR2_DMAEN_Pos         (11U)
#define I2C_CR2_DMAEN_Msk         (0x1UL << I2C_CR2_DMAEN_Pos)                  /*!< 0x00000800 */
#define I2C_CR2_DMAEN             I2C_CR2_DMAEN_Msk                            /*!<DMA Requests Enable     */
#define I2C_CR2_LAST_Pos          (12U)
#define I2C_CR2_LAST_Msk          (0x1UL << I2C_CR2_LAST_Pos)                   /*!< 0x00001000 */
#define I2C_CR2_LAST              I2C_CR2_LAST_Msk                             /*!<DMA Last Transfer       */

/*******************  Bit definition for I2C_OAR1 register  *******************/
#define I2C_OAR1_ADD1_7           0x000000FEU                                  /*!<Interface Address */
#define I2C_OAR1_ADD8_9           0x00000300U                                  /*!<Interface Address */

#define I2C_OAR1_ADD0_Pos         (0U)
#define I2C_OAR1_ADD0_Msk         (0x1UL << I2C_OAR1_ADD0_Pos)                  /*!< 0x00000001 */
#define I2C_OAR1_ADD0             I2C_OAR1_ADD0_Msk                            /*!<Bit 0 */
#define I2C_OAR1_ADD1_Pos         (1U)
#define I2C_OAR1_ADD1_Msk         (0x1UL << I2C_OAR1_ADD1_Pos)                  /*!< 0x00000002 */
#define I2C_OAR1_ADD1             I2C_OAR1_ADD1_Msk                            /*!<Bit 1 */
#define I2C_OAR1_ADD2_Pos         (2U)
#define I2C_OAR1_ADD2_Msk         (0x1UL << I2C_OAR1_ADD2_Pos)                  /*!< 0x00000004 */
#define I2C_OAR1_ADD2             I2C_OAR1_ADD2_Msk                            /*!<Bit 2 */
#define I2C_OAR1_ADD3_Pos         (3U)
#define I2C_OAR1_ADD3_Msk         (0x1UL << I2C_OAR1_ADD3_Pos)                  /*!< 0x00000008 */
#define I2C_OAR1_ADD3             I2C_OAR1_ADD3_Msk                            /*!<Bit 3 */
#define I2C_OAR1_ADD4_Pos         (4U)
#define I2C_OAR1_ADD4_Msk         (0x1UL << I2C_OAR1_ADD4_Pos)                  /*!< 0x00000010 */
#define I2C_OAR1_ADD4             I2C_OAR1_ADD4_Msk                            /*!<Bit 4 */
#define I2C_OAR1_ADD5_Pos         (5U)
#define I2C_OAR1_ADD5_Msk         (0x1UL << I2C_OAR1_ADD5_Pos)                  /*!< 0x00000020 */
#define I2C_OAR1_ADD5             I2C_OAR1_ADD5_Msk                            /*!<Bit 5 */
#define I2C_OAR1_ADD6_Pos         (6U)
#define I2C_OAR1_ADD6_Msk         (0x1UL << I2C_OAR1_ADD6_Pos)                  /*!< 0x00000040 */
#define I2C_OAR1_ADD6             I2C_OAR1_ADD6_Msk                            /*!<Bit 6 */
#define I2C_OAR1_ADD7_Pos         (7U)
#define I2C_OAR1_ADD7_Msk         (0x1UL << I2C_OAR1_ADD7_Pos)                  /*!< 0x00000080 */
#define I2C_OAR1_ADD7             I2C_OAR1_ADD7_Msk                            /*!<Bit 7 */
#define I2C_OAR1_ADD8_Pos         (8U)
#define I2C_OAR1_ADD8_Msk         (0x1UL << I2C_OAR1_ADD8_Pos)                  /*!< 0x00000100 */
#define I2C_OAR1_ADD8             I2C_OAR1_ADD8_Msk                            /*!<Bit 8 */
#define I2C_OAR1_ADD9_Pos         (9U)
#define I2C_OAR1_ADD9_Msk         (0x1UL << I2C_OAR1_ADD9_Pos)                  /*!< 0x00000200 */
#define I2C_OAR1_ADD9             I2C_OAR1_ADD9_Msk                            /*!<Bit 9 */

#define I2C_OAR1_ADDMODE_Pos      (15U)
#define I2C_OAR1_ADDMODE_Msk      (0x1UL << I2C_OAR1_ADDMODE_Pos)               /*!< 0x00008000 */
#define I2C_OAR1_ADDMODE          I2C_OAR1_ADDMODE_Msk                         /*!<Addressing Mode (Slave mode) */

/*******************  Bit definition for I2C_OAR2 register  *******************/
#define I2C_OAR2_ENDUAL_Pos       (0U)
#define I2C_OAR2_ENDUAL_Msk       (0x1UL << I2C_OAR2_ENDUAL_Pos)                /*!< 0x00000001 */
#define I2C_OAR2_ENDUAL           I2C_OAR2_ENDUAL_Msk                          /*!<Dual addressing mode enable */
#define I2C_OAR2_ADD2_Pos         (1U)
#define I2C_OAR2_ADD2_Msk         (0x7FUL << I2C_OAR2_ADD2_Pos)                 /*!< 0x000000FE */
#define I2C_OAR2_ADD2             I2C_OAR2_ADD2_Msk                            /*!<Interface address           */

/********************  Bit definition for I2C_DR register  ********************/
#define I2C_DR_DR_Pos             (0U)
#define I2C_DR_DR_Msk             (0xFFUL << I2C_DR_DR_Pos)                     /*!< 0x000000FF */
#define I2C_DR_DR                 I2C_DR_DR_Msk                                /*!<8-bit Data Register         */

/*******************  Bit definition for I2C_SR1 register  ********************/
#define I2C_SR1_SB_Pos            (0U)
#define I2C_SR1_SB_Msk            (0x1UL << I2C_SR1_SB_Pos)                     /*!< 0x00000001 */
#define I2C_SR1_SB                I2C_SR1_SB_Msk                               /*!<Start Bit (Master mode)                         */
#define I2C_SR1_ADDR_Pos          (1U)
#define I2C_SR1_ADDR_Msk          (0x1UL << I2C_SR1_ADDR_Pos)                   /*!< 0x00000002 */
#define I2C_SR1_ADDR              I2C_SR1_ADDR_Msk                             /*!<Address sent (master mode)/matched (slave mode) */
#define I2C_SR1_BTF_Pos           (2U)
#define I2C_SR1_BTF_Msk           (0x1UL << I2C_SR1_BTF_Pos)                    /*!< 0x00000004 */
#define I2C_SR1_BTF               I2C_SR1_BTF_Msk                              /*!<Byte Transfer Finished                          */
#define I2C_SR1_ADD10_Pos         (3U)
#define I2C_SR1_ADD10_Msk         (0x1UL << I2C_SR1_ADD10_Pos)                  /*!< 0x00000008 */
#define I2C_SR1_ADD10             I2C_SR1_ADD10_Msk                            /*!<10-bit header sent (Master mode)                */
#define I2C_SR1_STOPF_Pos         (4U)
#define I2C_SR1_STOPF_Msk         (0x1UL << I2C_SR1_STOPF_Pos)                  /*!< 0x00000010 */
#define I2C_SR1_STOPF             I2C_SR1_STOPF_Msk                            /*!<Stop detection (Slave mode)                     */
#define I2C_SR1_RXNE_Pos          (6U)
#define I2C_SR1_RXNE_Msk          (0x1UL << I2C_SR1_RXNE_Pos)                   /*!< 0x00000040 */
#define I2C_SR1_RXNE              I2C_SR1_RXNE_Msk                             /*!<Data Register not Empty (receivers)             */
#define I2C_SR1_TXE_Pos           (7U)
#define I2C_SR1_TXE_Msk           (0x1UL << I2C_SR1_TXE_Pos)                    /*!< 0x00000080 */
#define I2C_SR1_TXE               I2C_SR1_TXE_Msk                              /*!<Data Register Empty (transmitters)              */
#define I2C_SR1_BERR_Pos          (8U)
#define I2C_SR1_BERR_Msk          (0x1UL << I2C_SR1_BERR_Pos)                   /*!< 0x00000100 */
#define I2C_SR1_BERR              I2C_SR1_BERR_Msk                             /*!<Bus Error                                       */
#define I2C_SR1_ARLO_Pos          (9U)
#define I2C_SR1_ARLO_Msk          (0x1UL << I2C_SR1_ARLO_Pos)                   /*!< 0x00000200 */
#define I2C_SR1_ARLO              I2C_SR1_ARLO_Msk                             /*!<Arbitration Lost (master mode)                  */
#define I2C_SR1_AF_Pos            (10U)
#define I2C_SR1_AF_Msk            (0x1UL << I2C_SR1_AF_Pos)                     /*!< 0x00000400 */
#define I2C_SR1_AF                I2C_SR1_AF_Msk                               /*!<Acknowledge Failure                             */
#define I2C_SR1_OVR_Pos           (11U)
#define I2C_SR1_OVR_Msk           (0x1UL << I2C_SR1_OVR_Pos)                    /*!< 0x00000800 */
#define I2C_SR1_OVR               I2C_SR1_OVR_Msk                              /*!<Overrun/Underrun                                */
#define I2C_SR1_PECERR_Pos        (12U)
#define I2C_SR1_PECERR_Msk        (0x1UL << I2C_SR1_PECERR_Pos)                 /*!< 0x00001000 */
#define I2C_SR1_PECERR            I2C_SR1_PECERR_Msk                           /*!<PEC Error in reception                          */
#define I2C_SR1_TIMEOUT_Pos       (14U)
#define I2C_SR1_TIMEOUT_Msk       (0x1UL << I2C_SR1_TIMEOUT_Pos)                /*!< 0x00004000 */
#define I2C_SR1_TIMEOUT           I2C_SR1_TIMEOUT_Msk                          /*!<Timeout or Tlow Error                           */
#define I2C_SR1_SMBALERT_Pos      (15U)
#define I2C_SR1_SMBALERT_Msk      (0x1UL << I2C_SR1_SMBALERT_Pos)               /*!< 0x00008000 */
#define I2C_SR1_SMBALERT          I2C_SR1_SMBALERT_Msk                         /*!<SMBus Alert                                     */

/*******************  Bit definition for I2C_SR2 register  ********************/
#define I2C_SR2_MSL_Pos           (0U)
#define I2C_SR2_MSL_Msk           (0x1UL << I2C_SR2_MSL_Pos)                    /*!< 0x00000001 */
#define I2C_SR2_MSL               I2C_SR2_MSL_Msk                              /*!<Master/Slave                                    */
#define I2C_SR2_BUSY_Pos          (1U)
#define I2C_SR2_BUSY_Msk          (0x1UL << I2C_SR2_BUSY_Pos)                   /*!< 0x00000002 */
#define I2C_SR2_BUSY              I2C_SR2_BUSY_Msk                             /*!<Bus Busy                                        */
#define I2C_SR2_TRA_Pos           (2U)
#define I2C_SR2_TRA_Msk           (0x1UL << I2C_SR2_TRA_Pos)                    /*!< 0x00000004 */
#define I2C_SR2_TRA               I2C_SR2_TRA_Msk                              /*!<Transmitter/Receiver                            */
#define I2C_SR2_GENCALL_Pos       (4U)
#define I2C_SR2_GENCALL_Msk       (0x1UL << I2C_SR2_GENCALL_Pos)                /*!< 0x00000010 */
#define I2C_SR2_GENCALL           I2C_SR2_GENCALL_Msk                          /*!<General Call Address (Slave mode)               */
#define I2C_SR2_SMBDEFAULT_Pos    (5U)
#define I2C_SR2_SMBDEFAULT_Msk    (0x1UL << I2C_SR2_SMBDEFAULT_Pos)             /*!< 0x00000020 */
#define I2C_SR2_SMBDEFAULT        I2C_SR2_SMBDEFAULT_Msk                       /*!<SMBus Device Default Address (Slave mode)       */
#define I2C_SR2_SMBHOST_Pos       (6U)
#define I2C_SR2_SMBHOST_Msk       (0x1UL << I2C_SR2_SMBHOST_Pos)                /*!< 0x00000040 */
#define I2C_SR2_SMBHOST           I2C_SR2_SMBHOST_Msk                          /*!<SMBus Host Header (Slave mode)                  */
#define I2C_SR2_DUALF_Pos         (7U)
#define I2C_SR2_DUALF_Msk         (0x1UL << I2C_SR2_DUALF_Pos)                  /*!< 0x00000080 */
#define I2C_SR2_DUALF             I2C_SR2_DUALF_Msk                            /*!<Dual Flag (Slave mode)                          */
#define I2C_SR2_PEC_Pos           (8U)
#define I2C_SR2_PEC_Msk           (0xFFUL << I2C_SR2_PEC_Pos)                   /*!< 0x0000FF00 */
#define I2C_SR2_PEC               I2C_SR2_PEC_Msk                              /*!<Packet Error Checking Register                  */

/*******************  Bit definition for I2C_CCR register  ********************/
#define I2C_CCR_CCR_Pos           (0U)
#define I2C_CCR_CCR_Msk           (0xFFFUL << I2C_CCR_CCR_Pos)                  /*!< 0x00000FFF */
#define I2C_CCR_CCR               I2C_CCR_CCR_Msk                              /*!<Clock Control Register in Fast/Standard mode (Master mode) */
#define I2C_CCR_DUTY_Pos          (14U)
#define I2C_CCR_DUTY_Msk          (0x1UL << I2C_CCR_DUTY_Pos)                   /*!< 0x00004000 */
#define I2C_CCR_DUTY              I2C_CCR_DUTY_Msk                             /*!<Fast Mode Duty Cycle                                       */
#define I2C_CCR_FS_Pos            (15U)
#define I2C_CCR_FS_Msk            (0x1UL << I2C_CCR_FS_Pos)                     /*!< 0x00008000 */
#define I2C_CCR_FS                I2C_CCR_FS_Msk                               /*!<I2C Master Mode Selection                                  */

/******************  Bit definition for I2C_TRISE register  *******************/
#define I2C_TRISE_TRISE_Pos       (0U)
#define I2C_TRISE_TRISE_Msk       (0x3FUL << I2C_TRISE_TRISE_Pos)               /*!< 0x0000003F */
#define I2C_TRISE_TRISE           I2C_TRISE_TRISE_Msk                          /*!<Maximum Rise Time in Fast/Standard mode (Master mode) */



/******************************************************************************/
/*                                                                            */
/*                             Power Control                                  */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for PWR_CR register  ********************/
#define PWR_CR_LPDS_Pos        (0U)
#define PWR_CR_LPDS_Msk        (0x1UL << PWR_CR_LPDS_Pos)                       /*!< 0x00000001 */
#define PWR_CR_LPDS            PWR_CR_LPDS_Msk                                 /*!< Low-Power Deepsleep                 */
#define PWR_CR_PDDS_Pos        (1U)
#define PWR_CR_PDDS_Msk        (0x1UL << PWR_CR_PDDS_Pos)                       /*!< 0x00000002 */
#define PWR_CR_PDDS            PWR_CR_PDDS_Msk                                 /*!< Power Down Deepsleep                */
#define PWR_CR_CWUF_Pos        (2U)
#define PWR_CR_CWUF_Msk        (0x1UL << PWR_CR_CWUF_Pos)                       /*!< 0x00000004 */
#define PWR_CR_CWUF            PWR_CR_CWUF_Msk                                 /*!< Clear Wakeup Flag                   */
#define PWR_CR_CSBF_Pos        (3U)
#define PWR_CR_CSBF_Msk        (0x1UL << PWR_CR_CSBF_Pos)                       /*!< 0x00000008 */
#define PWR_CR_CSBF            PWR_CR_CSBF_Msk                                 /*!< Clear Standby Flag                  */
#define PWR_CR_PVDE_Pos        (4U)
#define PWR_CR_PVDE_Msk        (0x1UL << PWR_CR_PVDE_Pos)                       /*!< 0x00000010 */
#define PWR_CR_PVDE            PWR_CR_PVDE_Msk                                 /*!< Power Voltage Detector Enable       */

#define PWR_CR_PLS_Pos         (5U)
#define PWR_CR_PLS_Msk         (0x7UL << PWR_CR_PLS_Pos)                        /*!< 0x000000E0 */
#define PWR_CR_PLS             PWR_CR_PLS_Msk                                  /*!< PLS[2:0] bits (PVD Level Selection) */
#define PWR_CR_PLS_0           (0x1UL << PWR_CR_PLS_Pos)                        /*!< 0x00000020 */
#define PWR_CR_PLS_1           (0x2UL << PWR_CR_PLS_Pos)                        /*!< 0x00000040 */
#define PWR_CR_PLS_2           (0x4UL << PWR_CR_PLS_Pos)                        /*!< 0x00000080 */

/*!< PVD level configuration */
#define PWR_CR_PLS_LEV0        0x00000000U                                     /*!< PVD level 0 */
#define PWR_CR_PLS_LEV1        0x00000020U                                     /*!< PVD level 1 */
#define PWR_CR_PLS_LEV2        0x00000040U                                     /*!< PVD level 2 */
#define PWR_CR_PLS_LEV3        0x00000060U                                     /*!< PVD level 3 */
#define PWR_CR_PLS_LEV4        0x00000080U                                     /*!< PVD level 4 */
#define PWR_CR_PLS_LEV5        0x000000A0U                                     /*!< PVD level 5 */
#define PWR_CR_PLS_LEV6        0x000000C0U                                     /*!< PVD level 6 */
#define PWR_CR_PLS_LEV7        0x000000E0U                                     /*!< PVD level 7 */
#define PWR_CR_DBP_Pos         (8U)
#define PWR_CR_DBP_Msk         (0x1UL << PWR_CR_DBP_Pos)                        /*!< 0x00000100 */
#define PWR_CR_DBP             PWR_CR_DBP_Msk                                  /*!< Disable Backup Domain write protection                     */
#define PWR_CR_FPDS_Pos        (9U)
#define PWR_CR_FPDS_Msk        (0x1UL << PWR_CR_FPDS_Pos)                       /*!< 0x00000200 */
#define PWR_CR_FPDS            PWR_CR_FPDS_Msk                                 /*!< Flash power down in Stop mode                              */
#define PWR_CR_VOS_Pos         (14U)
#define PWR_CR_VOS_Msk         (0x1UL << PWR_CR_VOS_Pos)                        /*!< 0x00004000 */
#define PWR_CR_VOS             PWR_CR_VOS_Msk                                  /*!< VOS bit (Regulator voltage scaling output selection) */

/* Legacy define */
#define  PWR_CR_PMODE                        PWR_CR_VOS

/*******************  Bit definition for PWR_CSR register  ********************/
#define PWR_CSR_WUF_Pos        (0U)
#define PWR_CSR_WUF_Msk        (0x1UL << PWR_CSR_WUF_Pos)                       /*!< 0x00000001 */
#define PWR_CSR_WUF            PWR_CSR_WUF_Msk                                 /*!< Wakeup Flag                                      */
#define PWR_CSR_SBF_Pos        (1U)
#define PWR_CSR_SBF_Msk        (0x1UL << PWR_CSR_SBF_Pos)                       /*!< 0x00000002 */
#define PWR_CSR_SBF            PWR_CSR_SBF_Msk                                 /*!< Standby Flag                                     */
#define PWR_CSR_PVDO_Pos       (2U)
#define PWR_CSR_PVDO_Msk       (0x1UL << PWR_CSR_PVDO_Pos)                      /*!< 0x00000004 */
#define PWR_CSR_PVDO           PWR_CSR_PVDO_Msk                                /*!< PVD Output                                       */
#define PWR_CSR_BRR_Pos        (3U)
#define PWR_CSR_BRR_Msk        (0x1UL << PWR_CSR_BRR_Pos)                       /*!< 0x00000008 */
#define PWR_CSR_BRR            PWR_CSR_BRR_Msk                                 /*!< Backup regulator ready                           */
#define PWR_CSR_EWUP_Pos       (8U)
#define PWR_CSR_EWUP_Msk       (0x1UL << PWR_CSR_EWUP_Pos)                      /*!< 0x00000100 */
#define PWR_CSR_EWUP           PWR_CSR_EWUP_Msk                                /*!< Enable WKUP pin                                  */
#define PWR_CSR_BRE_Pos        (9U)
#define PWR_CSR_BRE_Msk        (0x1UL << PWR_CSR_BRE_Pos)                       /*!< 0x00000200 */
#define PWR_CSR_BRE            PWR_CSR_BRE_Msk                                 /*!< Backup regulator enable                          */
#define PWR_CSR_VOSRDY_Pos     (14U)
#define PWR_CSR_VOSRDY_Msk     (0x1UL << PWR_CSR_VOSRDY_Pos)                    /*!< 0x00004000 */
#define PWR_CSR_VOSRDY         PWR_CSR_VOSRDY_Msk                              /*!< Regulator voltage scaling output selection ready */

/* Legacy define */
#define  PWR_CSR_REGRDY                      PWR_CSR_VOSRDY

/******************************************************************************/
/*                                                                            */
/*                         Reset and Clock Control                            */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for RCC_CR register  ********************/
#define RCC_CR_HSION_Pos                   (0U)
#define RCC_CR_HSION_Msk                   (0x1UL << RCC_CR_HSION_Pos)          /*!< 0x00000001 */
#define RCC_CR_HSION                       RCC_CR_HSION_Msk
#define RCC_CR_HSIRDY_Pos                  (1U)
#define RCC_CR_HSIRDY_Msk                  (0x1UL << RCC_CR_HSIRDY_Pos)         /*!< 0x00000002 */
#define RCC_CR_HSIRDY                      RCC_CR_HSIRDY_Msk

#define RCC_CR_HSITRIM_Pos                 (3U)
#define RCC_CR_HSITRIM_Msk                 (0x1FUL << RCC_CR_HSITRIM_Pos)       /*!< 0x000000F8 */
#define RCC_CR_HSITRIM                     RCC_CR_HSITRIM_Msk
#define RCC_CR_HSITRIM_0                   (0x01UL << RCC_CR_HSITRIM_Pos)       /*!< 0x00000008 */
#define RCC_CR_HSITRIM_1                   (0x02UL << RCC_CR_HSITRIM_Pos)       /*!< 0x00000010 */
#define RCC_CR_HSITRIM_2                   (0x04UL << RCC_CR_HSITRIM_Pos)       /*!< 0x00000020 */
#define RCC_CR_HSITRIM_3                   (0x08UL << RCC_CR_HSITRIM_Pos)       /*!< 0x00000040 */
#define RCC_CR_HSITRIM_4                   (0x10UL << RCC_CR_HSITRIM_Pos)       /*!< 0x00000080 */

#define RCC_CR_HSICAL_Pos                  (8U)
#define RCC_CR_HSICAL_Msk                  (0xFFUL << RCC_CR_HSICAL_Pos)        /*!< 0x0000FF00 */
#define RCC_CR_HSICAL                      RCC_CR_HSICAL_Msk
#define RCC_CR_HSICAL_0                    (0x01UL << RCC_CR_HSICAL_Pos)        /*!< 0x00000100 */
#define RCC_CR_HSICAL_1                    (0x02UL << RCC_CR_HSICAL_Pos)        /*!< 0x00000200 */
#define RCC_CR_HSICAL_2                    (0x04UL << RCC_CR_HSICAL_Pos)        /*!< 0x00000400 */
#define RCC_CR_HSICAL_3                    (0x08UL << RCC_CR_HSICAL_Pos)        /*!< 0x00000800 */
#define RCC_CR_HSICAL_4                    (0x10UL << RCC_CR_HSICAL_Pos)        /*!< 0x00001000 */
#define RCC_CR_HSICAL_5                    (0x20UL << RCC_CR_HSICAL_Pos)        /*!< 0x00002000 */
#define RCC_CR_HSICAL_6                    (0x40UL << RCC_CR_HSICAL_Pos)        /*!< 0x00004000 */
#define RCC_CR_HSICAL_7                    (0x80UL << RCC_CR_HSICAL_Pos)        /*!< 0x00008000 */

#define RCC_CR_HSEON_Pos                   (16U)
#define RCC_CR_HSEON_Msk                   (0x1UL << RCC_CR_HSEON_Pos)          /*!< 0x00010000 */
#define RCC_CR_HSEON                       RCC_CR_HSEON_Msk
#define RCC_CR_HSERDY_Pos                  (17U)
#define RCC_CR_HSERDY_Msk                  (0x1UL << RCC_CR_HSERDY_Pos)         /*!< 0x00020000 */
#define RCC_CR_HSERDY                      RCC_CR_HSERDY_Msk
#define RCC_CR_HSEBYP_Pos                  (18U)
#define RCC_CR_HSEBYP_Msk                  (0x1UL << RCC_CR_HSEBYP_Pos)         /*!< 0x00040000 */
#define RCC_CR_HSEBYP                      RCC_CR_HSEBYP_Msk
#define RCC_CR_CSSON_Pos                   (19U)
#define RCC_CR_CSSON_Msk                   (0x1UL << RCC_CR_CSSON_Pos)          /*!< 0x00080000 */
#define RCC_CR_CSSON                       RCC_CR_CSSON_Msk
#define RCC_CR_PLLON_Pos                   (24U)
#define RCC_CR_PLLON_Msk                   (0x1UL << RCC_CR_PLLON_Pos)          /*!< 0x01000000 */
#define RCC_CR_PLLON                       RCC_CR_PLLON_Msk
#define RCC_CR_PLLRDY_Pos                  (25U)
#define RCC_CR_PLLRDY_Msk                  (0x1UL << RCC_CR_PLLRDY_Pos)         /*!< 0x02000000 */
#define RCC_CR_PLLRDY                      RCC_CR_PLLRDY_Msk
/*
 * @brief Specific device feature definitions (not present on all devices in the STM32F4 serie)
 */
#define RCC_PLLI2S_SUPPORT                                                     /*!< Support PLLI2S oscillator */

#define RCC_CR_PLLI2SON_Pos                (26U)
#define RCC_CR_PLLI2SON_Msk                (0x1UL << RCC_CR_PLLI2SON_Pos)       /*!< 0x04000000 */
#define RCC_CR_PLLI2SON                    RCC_CR_PLLI2SON_Msk
#define RCC_CR_PLLI2SRDY_Pos               (27U)
#define RCC_CR_PLLI2SRDY_Msk               (0x1UL << RCC_CR_PLLI2SRDY_Pos)      /*!< 0x08000000 */
#define RCC_CR_PLLI2SRDY                   RCC_CR_PLLI2SRDY_Msk

/********************  Bit definition for RCC_PLLCFGR register  ***************/
#define RCC_PLLCFGR_PLLM_Pos               (0U)
#define RCC_PLLCFGR_PLLM_Msk               (0x3FUL << RCC_PLLCFGR_PLLM_Pos)     /*!< 0x0000003F */
#define RCC_PLLCFGR_PLLM                   RCC_PLLCFGR_PLLM_Msk
#define RCC_PLLCFGR_PLLM_0                 (0x01UL << RCC_PLLCFGR_PLLM_Pos)     /*!< 0x00000001 */
#define RCC_PLLCFGR_PLLM_1                 (0x02UL << RCC_PLLCFGR_PLLM_Pos)     /*!< 0x00000002 */
#define RCC_PLLCFGR_PLLM_2                 (0x04UL << RCC_PLLCFGR_PLLM_Pos)     /*!< 0x00000004 */
#define RCC_PLLCFGR_PLLM_3                 (0x08UL << RCC_PLLCFGR_PLLM_Pos)     /*!< 0x00000008 */
#define RCC_PLLCFGR_PLLM_4                 (0x10UL << RCC_PLLCFGR_PLLM_Pos)     /*!< 0x00000010 */
#define RCC_PLLCFGR_PLLM_5                 (0x20UL << RCC_PLLCFGR_PLLM_Pos)     /*!< 0x00000020 */

#define RCC_PLLCFGR_PLLN_Pos               (6U)
#define RCC_PLLCFGR_PLLN_Msk               (0x1FFUL << RCC_PLLCFGR_PLLN_Pos)    /*!< 0x00007FC0 */
#define RCC_PLLCFGR_PLLN                   RCC_PLLCFGR_PLLN_Msk
#define RCC_PLLCFGR_PLLN_0                 (0x001UL << RCC_PLLCFGR_PLLN_Pos)    /*!< 0x00000040 */
#define RCC_PLLCFGR_PLLN_1                 (0x002UL << RCC_PLLCFGR_PLLN_Pos)    /*!< 0x00000080 */
#define RCC_PLLCFGR_PLLN_2                 (0x004UL << RCC_PLLCFGR_PLLN_Pos)    /*!< 0x00000100 */
#define RCC_PLLCFGR_PLLN_3                 (0x008UL << RCC_PLLCFGR_PLLN_Pos)    /*!< 0x00000200 */
#define RCC_PLLCFGR_PLLN_4                 (0x010UL << RCC_PLLCFGR_PLLN_Pos)    /*!< 0x00000400 */
#define RCC_PLLCFGR_PLLN_5                 (0x020UL << RCC_PLLCFGR_PLLN_Pos)    /*!< 0x00000800 */
#define RCC_PLLCFGR_PLLN_6                 (0x040UL << RCC_PLLCFGR_PLLN_Pos)    /*!< 0x00001000 */
#define RCC_PLLCFGR_PLLN_7                 (0x080UL << RCC_PLLCFGR_PLLN_Pos)    /*!< 0x00002000 */
#define RCC_PLLCFGR_PLLN_8                 (0x100UL << RCC_PLLCFGR_PLLN_Pos)    /*!< 0x00004000 */

#define RCC_PLLCFGR_PLLP_Pos               (16U)
#define RCC_PLLCFGR_PLLP_Msk               (0x3UL << RCC_PLLCFGR_PLLP_Pos)      /*!< 0x00030000 */
#define RCC_PLLCFGR_PLLP                   RCC_PLLCFGR_PLLP_Msk
#define RCC_PLLCFGR_PLLP_0                 (0x1UL << RCC_PLLCFGR_PLLP_Pos)      /*!< 0x00010000 */
#define RCC_PLLCFGR_PLLP_1                 (0x2UL << RCC_PLLCFGR_PLLP_Pos)      /*!< 0x00020000 */

#define RCC_PLLCFGR_PLLSRC_Pos             (22U)
#define RCC_PLLCFGR_PLLSRC_Msk             (0x1UL << RCC_PLLCFGR_PLLSRC_Pos)    /*!< 0x00400000 */
#define RCC_PLLCFGR_PLLSRC                 RCC_PLLCFGR_PLLSRC_Msk
#define RCC_PLLCFGR_PLLSRC_HSE_Pos         (22U)
#define RCC_PLLCFGR_PLLSRC_HSE_Msk         (0x1UL << RCC_PLLCFGR_PLLSRC_HSE_Pos) /*!< 0x00400000 */
#define RCC_PLLCFGR_PLLSRC_HSE             RCC_PLLCFGR_PLLSRC_HSE_Msk
#define RCC_PLLCFGR_PLLSRC_HSI             0x00000000U

#define RCC_PLLCFGR_PLLQ_Pos               (24U)
#define RCC_PLLCFGR_PLLQ_Msk               (0xFUL << RCC_PLLCFGR_PLLQ_Pos)      /*!< 0x0F000000 */
#define RCC_PLLCFGR_PLLQ                   RCC_PLLCFGR_PLLQ_Msk
#define RCC_PLLCFGR_PLLQ_0                 (0x1UL << RCC_PLLCFGR_PLLQ_Pos)      /*!< 0x01000000 */
#define RCC_PLLCFGR_PLLQ_1                 (0x2UL << RCC_PLLCFGR_PLLQ_Pos)      /*!< 0x02000000 */
#define RCC_PLLCFGR_PLLQ_2                 (0x4UL << RCC_PLLCFGR_PLLQ_Pos)      /*!< 0x04000000 */
#define RCC_PLLCFGR_PLLQ_3                 (0x8UL << RCC_PLLCFGR_PLLQ_Pos)      /*!< 0x08000000 */


/********************  Bit definition for RCC_CFGR register  ******************/
/*!< SW configuration */
#define RCC_CFGR_SW_Pos                    (0U)
#define RCC_CFGR_SW_Msk                    (0x3UL << RCC_CFGR_SW_Pos)           /*!< 0x00000003 */
#define RCC_CFGR_SW                        RCC_CFGR_SW_Msk                     /*!< SW[1:0] bits (System clock Switch) */
#define RCC_CFGR_SW_0                      (0x1UL << RCC_CFGR_SW_Pos)           /*!< 0x00000001 */
#define RCC_CFGR_SW_1                      (0x2UL << RCC_CFGR_SW_Pos)           /*!< 0x00000002 */

#define RCC_CFGR_SW_HSI                    0x00000000U                         /*!< HSI selected as system clock */
#define RCC_CFGR_SW_HSE                    0x00000001U                         /*!< HSE selected as system clock */
#define RCC_CFGR_SW_PLL                    0x00000002U                         /*!< PLL selected as system clock */

/*!< SWS configuration */
#define RCC_CFGR_SWS_Pos                   (2U)
#define RCC_CFGR_SWS_Msk                   (0x3UL << RCC_CFGR_SWS_Pos)          /*!< 0x0000000C */
#define RCC_CFGR_SWS                       RCC_CFGR_SWS_Msk                    /*!< SWS[1:0] bits (System Clock Switch Status) */
#define RCC_CFGR_SWS_0                     (0x1UL << RCC_CFGR_SWS_Pos)          /*!< 0x00000004 */
#define RCC_CFGR_SWS_1                     (0x2UL << RCC_CFGR_SWS_Pos)          /*!< 0x00000008 */

#define RCC_CFGR_SWS_HSI                   0x00000000U                         /*!< HSI oscillator used as system clock        */
#define RCC_CFGR_SWS_HSE                   0x00000004U                         /*!< HSE oscillator used as system clock        */
#define RCC_CFGR_SWS_PLL                   0x00000008U                         /*!< PLL used as system clock                   */

/*!< HPRE configuration */
#define RCC_CFGR_HPRE_Pos                  (4U)
#define RCC_CFGR_HPRE_Msk                  (0xFUL << RCC_CFGR_HPRE_Pos)         /*!< 0x000000F0 */
#define RCC_CFGR_HPRE                      RCC_CFGR_HPRE_Msk                   /*!< HPRE[3:0] bits (AHB prescaler) */
#define RCC_CFGR_HPRE_0                    (0x1UL << RCC_CFGR_HPRE_Pos)         /*!< 0x00000010 */
#define RCC_CFGR_HPRE_1                    (0x2UL << RCC_CFGR_HPRE_Pos)         /*!< 0x00000020 */
#define RCC_CFGR_HPRE_2                    (0x4UL << RCC_CFGR_HPRE_Pos)         /*!< 0x00000040 */
#define RCC_CFGR_HPRE_3                    (0x8UL << RCC_CFGR_HPRE_Pos)         /*!< 0x00000080 */

#define RCC_CFGR_HPRE_DIV1                 0x00000000U                         /*!< SYSCLK not divided    */
#define RCC_CFGR_HPRE_DIV2                 0x00000080U                         /*!< SYSCLK divided by 2   */
#define RCC_CFGR_HPRE_DIV4                 0x00000090U                         /*!< SYSCLK divided by 4   */
#define RCC_CFGR_HPRE_DIV8                 0x000000A0U                         /*!< SYSCLK divided by 8   */
#define RCC_CFGR_HPRE_DIV16                0x000000B0U                         /*!< SYSCLK divided by 16  */
#define RCC_CFGR_HPRE_DIV64                0x000000C0U                         /*!< SYSCLK divided by 64  */
#define RCC_CFGR_HPRE_DIV128               0x000000D0U                         /*!< SYSCLK divided by 128 */
#define RCC_CFGR_HPRE_DIV256               0x000000E0U                         /*!< SYSCLK divided by 256 */
#define RCC_CFGR_HPRE_DIV512               0x000000F0U                         /*!< SYSCLK divided by 512 */

/*!< PPRE1 configuration */
#define RCC_CFGR_PPRE1_Pos                 (10U)
#define RCC_CFGR_PPRE1_Msk                 (0x7UL << RCC_CFGR_PPRE1_Pos)        /*!< 0x00001C00 */
#define RCC_CFGR_PPRE1                     RCC_CFGR_PPRE1_Msk                  /*!< PRE1[2:0] bits (APB1 prescaler) */
#define RCC_CFGR_PPRE1_0                   (0x1UL << RCC_CFGR_PPRE1_Pos)        /*!< 0x00000400 */
#define RCC_CFGR_PPRE1_1                   (0x2UL << RCC_CFGR_PPRE1_Pos)        /*!< 0x00000800 */
#define RCC_CFGR_PPRE1_2                   (0x4UL << RCC_CFGR_PPRE1_Pos)        /*!< 0x00001000 */

#define RCC_CFGR_PPRE1_DIV1                0x00000000U                         /*!< HCLK not divided   */
#define RCC_CFGR_PPRE1_DIV2                0x00001000U                         /*!< HCLK divided by 2  */
#define RCC_CFGR_PPRE1_DIV4                0x00001400U                         /*!< HCLK divided by 4  */
#define RCC_CFGR_PPRE1_DIV8                0x00001800U                         /*!< HCLK divided by 8  */
#define RCC_CFGR_PPRE1_DIV16               0x00001C00U                         /*!< HCLK divided by 16 */

/*!< PPRE2 configuration */
#define RCC_CFGR_PPRE2_Pos                 (13U)
#define RCC_CFGR_PPRE2_Msk                 (0x7UL << RCC_CFGR_PPRE2_Pos)        /*!< 0x0000E000 */
#define RCC_CFGR_PPRE2                     RCC_CFGR_PPRE2_Msk                  /*!< PRE2[2:0] bits (APB2 prescaler) */
#define RCC_CFGR_PPRE2_0                   (0x1UL << RCC_CFGR_PPRE2_Pos)        /*!< 0x00002000 */
#define RCC_CFGR_PPRE2_1                   (0x2UL << RCC_CFGR_PPRE2_Pos)        /*!< 0x00004000 */
#define RCC_CFGR_PPRE2_2                   (0x4UL << RCC_CFGR_PPRE2_Pos)        /*!< 0x00008000 */

#define RCC_CFGR_PPRE2_DIV1                0x00000000U                         /*!< HCLK not divided   */
#define RCC_CFGR_PPRE2_DIV2                0x00008000U                         /*!< HCLK divided by 2  */
#define RCC_CFGR_PPRE2_DIV4                0x0000A000U                         /*!< HCLK divided by 4  */
#define RCC_CFGR_PPRE2_DIV8                0x0000C000U                         /*!< HCLK divided by 8  */
#define RCC_CFGR_PPRE2_DIV16               0x0000E000U                         /*!< HCLK divided by 16 */

/*!< RTCPRE configuration */
#define RCC_CFGR_RTCPRE_Pos                (16U)
#define RCC_CFGR_RTCPRE_Msk                (0x1FUL << RCC_CFGR_RTCPRE_Pos)      /*!< 0x001F0000 */
#define RCC_CFGR_RTCPRE                    RCC_CFGR_RTCPRE_Msk
#define RCC_CFGR_RTCPRE_0                  (0x01UL << RCC_CFGR_RTCPRE_Pos)      /*!< 0x00010000 */
#define RCC_CFGR_RTCPRE_1                  (0x02UL << RCC_CFGR_RTCPRE_Pos)      /*!< 0x00020000 */
#define RCC_CFGR_RTCPRE_2                  (0x04UL << RCC_CFGR_RTCPRE_Pos)      /*!< 0x00040000 */
#define RCC_CFGR_RTCPRE_3                  (0x08UL << RCC_CFGR_RTCPRE_Pos)      /*!< 0x00080000 */
#define RCC_CFGR_RTCPRE_4                  (0x10UL << RCC_CFGR_RTCPRE_Pos)      /*!< 0x00100000 */

/*!< MCO1 configuration */
#define RCC_CFGR_MCO1_Pos                  (21U)
#define RCC_CFGR_MCO1_Msk                  (0x3UL << RCC_CFGR_MCO1_Pos)         /*!< 0x00600000 */
#define RCC_CFGR_MCO1                      RCC_CFGR_MCO1_Msk
#define RCC_CFGR_MCO1_0                    (0x1UL << RCC_CFGR_MCO1_Pos)         /*!< 0x00200000 */
#define RCC_CFGR_MCO1_1                    (0x2UL << RCC_CFGR_MCO1_Pos)         /*!< 0x00400000 */

#define RCC_CFGR_I2SSRC_Pos                (23U)
#define RCC_CFGR_I2SSRC_Msk                (0x1UL << RCC_CFGR_I2SSRC_Pos)       /*!< 0x00800000 */
#define RCC_CFGR_I2SSRC                    RCC_CFGR_I2SSRC_Msk

#define RCC_CFGR_MCO1PRE_Pos               (24U)
#define RCC_CFGR_MCO1PRE_Msk               (0x7UL << RCC_CFGR_MCO1PRE_Pos)      /*!< 0x07000000 */
#define RCC_CFGR_MCO1PRE                   RCC_CFGR_MCO1PRE_Msk
#define RCC_CFGR_MCO1PRE_0                 (0x1UL << RCC_CFGR_MCO1PRE_Pos)      /*!< 0x01000000 */
#define RCC_CFGR_MCO1PRE_1                 (0x2UL << RCC_CFGR_MCO1PRE_Pos)      /*!< 0x02000000 */
#define RCC_CFGR_MCO1PRE_2                 (0x4UL << RCC_CFGR_MCO1PRE_Pos)      /*!< 0x04000000 */

#define RCC_CFGR_MCO2PRE_Pos               (27U)
#define RCC_CFGR_MCO2PRE_Msk               (0x7UL << RCC_CFGR_MCO2PRE_Pos)      /*!< 0x38000000 */
#define RCC_CFGR_MCO2PRE                   RCC_CFGR_MCO2PRE_Msk
#define RCC_CFGR_MCO2PRE_0                 (0x1UL << RCC_CFGR_MCO2PRE_Pos)      /*!< 0x08000000 */
#define RCC_CFGR_MCO2PRE_1                 (0x2UL << RCC_CFGR_MCO2PRE_Pos)      /*!< 0x10000000 */
#define RCC_CFGR_MCO2PRE_2                 (0x4UL << RCC_CFGR_MCO2PRE_Pos)      /*!< 0x20000000 */

#define RCC_CFGR_MCO2_Pos                  (30U)
#define RCC_CFGR_MCO2_Msk                  (0x3UL << RCC_CFGR_MCO2_Pos)         /*!< 0xC0000000 */
#define RCC_CFGR_MCO2                      RCC_CFGR_MCO2_Msk
#define RCC_CFGR_MCO2_0                    (0x1UL << RCC_CFGR_MCO2_Pos)         /*!< 0x40000000 */
#define RCC_CFGR_MCO2_1                    (0x2UL << RCC_CFGR_MCO2_Pos)         /*!< 0x80000000 */

/********************  Bit definition for RCC_CIR register  *******************/
#define RCC_CIR_LSIRDYF_Pos                (0U)
#define RCC_CIR_LSIRDYF_Msk                (0x1UL << RCC_CIR_LSIRDYF_Pos)       /*!< 0x00000001 */
#define RCC_CIR_LSIRDYF                    RCC_CIR_LSIRDYF_Msk
#define RCC_CIR_LSERDYF_Pos                (1U)
#define RCC_CIR_LSERDYF_Msk                (0x1UL << RCC_CIR_LSERDYF_Pos)       /*!< 0x00000002 */
#define RCC_CIR_LSERDYF                    RCC_CIR_LSERDYF_Msk
#define RCC_CIR_HSIRDYF_Pos                (2U)
#define RCC_CIR_HSIRDYF_Msk                (0x1UL << RCC_CIR_HSIRDYF_Pos)       /*!< 0x00000004 */
#define RCC_CIR_HSIRDYF                    RCC_CIR_HSIRDYF_Msk
#define RCC_CIR_HSERDYF_Pos                (3U)
#define RCC_CIR_HSERDYF_Msk                (0x1UL << RCC_CIR_HSERDYF_Pos)       /*!< 0x00000008 */
#define RCC_CIR_HSERDYF                    RCC_CIR_HSERDYF_Msk
#define RCC_CIR_PLLRDYF_Pos                (4U)
#define RCC_CIR_PLLRDYF_Msk                (0x1UL << RCC_CIR_PLLRDYF_Pos)       /*!< 0x00000010 */
#define RCC_CIR_PLLRDYF                    RCC_CIR_PLLRDYF_Msk
#define RCC_CIR_PLLI2SRDYF_Pos             (5U)
#define RCC_CIR_PLLI2SRDYF_Msk             (0x1UL << RCC_CIR_PLLI2SRDYF_Pos)    /*!< 0x00000020 */
#define RCC_CIR_PLLI2SRDYF                 RCC_CIR_PLLI2SRDYF_Msk

#define RCC_CIR_CSSF_Pos                   (7U)
#define RCC_CIR_CSSF_Msk                   (0x1UL << RCC_CIR_CSSF_Pos)          /*!< 0x00000080 */
#define RCC_CIR_CSSF                       RCC_CIR_CSSF_Msk
#define RCC_CIR_LSIRDYIE_Pos               (8U)
#define RCC_CIR_LSIRDYIE_Msk               (0x1UL << RCC_CIR_LSIRDYIE_Pos)      /*!< 0x00000100 */
#define RCC_CIR_LSIRDYIE                   RCC_CIR_LSIRDYIE_Msk
#define RCC_CIR_LSERDYIE_Pos               (9U)
#define RCC_CIR_LSERDYIE_Msk               (0x1UL << RCC_CIR_LSERDYIE_Pos)      /*!< 0x00000200 */
#define RCC_CIR_LSERDYIE                   RCC_CIR_LSERDYIE_Msk
#define RCC_CIR_HSIRDYIE_Pos               (10U)
#define RCC_CIR_HSIRDYIE_Msk               (0x1UL << RCC_CIR_HSIRDYIE_Pos)      /*!< 0x00000400 */
#define RCC_CIR_HSIRDYIE                   RCC_CIR_HSIRDYIE_Msk
#define RCC_CIR_HSERDYIE_Pos               (11U)
#define RCC_CIR_HSERDYIE_Msk               (0x1UL << RCC_CIR_HSERDYIE_Pos)      /*!< 0x00000800 */
#define RCC_CIR_HSERDYIE                   RCC_CIR_HSERDYIE_Msk
#define RCC_CIR_PLLRDYIE_Pos               (12U)
#define RCC_CIR_PLLRDYIE_Msk               (0x1UL << RCC_CIR_PLLRDYIE_Pos)      /*!< 0x00001000 */
#define RCC_CIR_PLLRDYIE                   RCC_CIR_PLLRDYIE_Msk
#define RCC_CIR_PLLI2SRDYIE_Pos            (13U)
#define RCC_CIR_PLLI2SRDYIE_Msk            (0x1UL << RCC_CIR_PLLI2SRDYIE_Pos)   /*!< 0x00002000 */
#define RCC_CIR_PLLI2SRDYIE                RCC_CIR_PLLI2SRDYIE_Msk

#define RCC_CIR_LSIRDYC_Pos                (16U)
#define RCC_CIR_LSIRDYC_Msk                (0x1UL << RCC_CIR_LSIRDYC_Pos)       /*!< 0x00010000 */
#define RCC_CIR_LSIRDYC                    RCC_CIR_LSIRDYC_Msk
#define RCC_CIR_LSERDYC_Pos                (17U)
#define RCC_CIR_LSERDYC_Msk                (0x1UL << RCC_CIR_LSERDYC_Pos)       /*!< 0x00020000 */
#define RCC_CIR_LSERDYC                    RCC_CIR_LSERDYC_Msk
#define RCC_CIR_HSIRDYC_Pos                (18U)
#define RCC_CIR_HSIRDYC_Msk                (0x1UL << RCC_CIR_HSIRDYC_Pos)       /*!< 0x00040000 */
#define RCC_CIR_HSIRDYC                    RCC_CIR_HSIRDYC_Msk
#define RCC_CIR_HSERDYC_Pos                (19U)
#define RCC_CIR_HSERDYC_Msk                (0x1UL << RCC_CIR_HSERDYC_Pos)       /*!< 0x00080000 */
#define RCC_CIR_HSERDYC                    RCC_CIR_HSERDYC_Msk
#define RCC_CIR_PLLRDYC_Pos                (20U)
#define RCC_CIR_PLLRDYC_Msk                (0x1UL << RCC_CIR_PLLRDYC_Pos)       /*!< 0x00100000 */
#define RCC_CIR_PLLRDYC                    RCC_CIR_PLLRDYC_Msk
#define RCC_CIR_PLLI2SRDYC_Pos             (21U)
#define RCC_CIR_PLLI2SRDYC_Msk             (0x1UL << RCC_CIR_PLLI2SRDYC_Pos)    /*!< 0x00200000 */
#define RCC_CIR_PLLI2SRDYC                 RCC_CIR_PLLI2SRDYC_Msk

#define RCC_CIR_CSSC_Pos                   (23U)
#define RCC_CIR_CSSC_Msk                   (0x1UL << RCC_CIR_CSSC_Pos)          /*!< 0x00800000 */
#define RCC_CIR_CSSC                       RCC_CIR_CSSC_Msk

/********************  Bit definition for RCC_AHB1RSTR register  **************/
#define RCC_AHB1RSTR_GPIOARST_Pos          (0U)
#define RCC_AHB1RSTR_GPIOARST_Msk          (0x1UL << RCC_AHB1RSTR_GPIOARST_Pos) /*!< 0x00000001 */
#define RCC_AHB1RSTR_GPIOARST              RCC_AHB1RSTR_GPIOARST_Msk
#define RCC_AHB1RSTR_GPIOBRST_Pos          (1U)
#define RCC_AHB1RSTR_GPIOBRST_Msk          (0x1UL << RCC_AHB1RSTR_GPIOBRST_Pos) /*!< 0x00000002 */
#define RCC_AHB1RSTR_GPIOBRST              RCC_AHB1RSTR_GPIOBRST_Msk
#define RCC_AHB1RSTR_GPIOCRST_Pos          (2U)
#define RCC_AHB1RSTR_GPIOCRST_Msk          (0x1UL << RCC_AHB1RSTR_GPIOCRST_Pos) /*!< 0x00000004 */
#define RCC_AHB1RSTR_GPIOCRST              RCC_AHB1RSTR_GPIOCRST_Msk
#define RCC_AHB1RSTR_GPIODRST_Pos          (3U)
#define RCC_AHB1RSTR_GPIODRST_Msk          (0x1UL << RCC_AHB1RSTR_GPIODRST_Pos) /*!< 0x00000008 */
#define RCC_AHB1RSTR_GPIODRST              RCC_AHB1RSTR_GPIODRST_Msk
#define RCC_AHB1RSTR_GPIOERST_Pos          (4U)
#define RCC_AHB1RSTR_GPIOERST_Msk          (0x1UL << RCC_AHB1RSTR_GPIOERST_Pos) /*!< 0x00000010 */
#define RCC_AHB1RSTR_GPIOERST              RCC_AHB1RSTR_GPIOERST_Msk
#define RCC_AHB1RSTR_GPIOFRST_Pos          (5U)
#define RCC_AHB1RSTR_GPIOFRST_Msk          (0x1UL << RCC_AHB1RSTR_GPIOFRST_Pos) /*!< 0x00000020 */
#define RCC_AHB1RSTR_GPIOFRST              RCC_AHB1RSTR_GPIOFRST_Msk
#define RCC_AHB1RSTR_GPIOGRST_Pos          (6U)
#define RCC_AHB1RSTR_GPIOGRST_Msk          (0x1UL << RCC_AHB1RSTR_GPIOGRST_Pos) /*!< 0x00000040 */
#define RCC_AHB1RSTR_GPIOGRST              RCC_AHB1RSTR_GPIOGRST_Msk
#define RCC_AHB1RSTR_GPIOHRST_Pos          (7U)
#define RCC_AHB1RSTR_GPIOHRST_Msk          (0x1UL << RCC_AHB1RSTR_GPIOHRST_Pos) /*!< 0x00000080 */
#define RCC_AHB1RSTR_GPIOHRST              RCC_AHB1RSTR_GPIOHRST_Msk
#define RCC_AHB1RSTR_GPIOIRST_Pos          (8U)
#define RCC_AHB1RSTR_GPIOIRST_Msk          (0x1UL << RCC_AHB1RSTR_GPIOIRST_Pos) /*!< 0x00000100 */
#define RCC_AHB1RSTR_GPIOIRST              RCC_AHB1RSTR_GPIOIRST_Msk
#define RCC_AHB1RSTR_CRCRST_Pos            (12U)
#define RCC_AHB1RSTR_CRCRST_Msk            (0x1UL << RCC_AHB1RSTR_CRCRST_Pos)   /*!< 0x00001000 */
#define RCC_AHB1RSTR_CRCRST                RCC_AHB1RSTR_CRCRST_Msk
#define RCC_AHB1RSTR_DMA1RST_Pos           (21U)
#define RCC_AHB1RSTR_DMA1RST_Msk           (0x1UL << RCC_AHB1RSTR_DMA1RST_Pos)  /*!< 0x00200000 */
#define RCC_AHB1RSTR_DMA1RST               RCC_AHB1RSTR_DMA1RST_Msk
#define RCC_AHB1RSTR_DMA2RST_Pos           (22U)
#define RCC_AHB1RSTR_DMA2RST_Msk           (0x1UL << RCC_AHB1RSTR_DMA2RST_Pos)  /*!< 0x00400000 */
#define RCC_AHB1RSTR_DMA2RST               RCC_AHB1RSTR_DMA2RST_Msk
#define RCC_AHB1RSTR_ETHMACRST_Pos         (25U)
#define RCC_AHB1RSTR_ETHMACRST_Msk         (0x1UL << RCC_AHB1RSTR_ETHMACRST_Pos) /*!< 0x02000000 */
#define RCC_AHB1RSTR_ETHMACRST             RCC_AHB1RSTR_ETHMACRST_Msk
#define RCC_AHB1RSTR_OTGHRST_Pos           (29U)
#define RCC_AHB1RSTR_OTGHRST_Msk           (0x1UL << RCC_AHB1RSTR_OTGHRST_Pos)  /*!< 0x20000000 */
#define RCC_AHB1RSTR_OTGHRST               RCC_AHB1RSTR_OTGHRST_Msk

/********************  Bit definition for RCC_AHB2RSTR register  **************/
#define RCC_AHB2RSTR_DCMIRST_Pos           (0U)
#define RCC_AHB2RSTR_DCMIRST_Msk           (0x1UL << RCC_AHB2RSTR_DCMIRST_Pos)  /*!< 0x00000001 */
#define RCC_AHB2RSTR_DCMIRST               RCC_AHB2RSTR_DCMIRST_Msk
#define RCC_AHB2RSTR_RNGRST_Pos            (6U)
#define RCC_AHB2RSTR_RNGRST_Msk            (0x1UL << RCC_AHB2RSTR_RNGRST_Pos)   /*!< 0x00000040 */
#define RCC_AHB2RSTR_RNGRST                RCC_AHB2RSTR_RNGRST_Msk
#define RCC_AHB2RSTR_OTGFSRST_Pos          (7U)
#define RCC_AHB2RSTR_OTGFSRST_Msk          (0x1UL << RCC_AHB2RSTR_OTGFSRST_Pos) /*!< 0x00000080 */
#define RCC_AHB2RSTR_OTGFSRST              RCC_AHB2RSTR_OTGFSRST_Msk
/********************  Bit definition for RCC_AHB3RSTR register  **************/
#define RCC_AHB3RSTR_FSMCRST_Pos           (0U)
#define RCC_AHB3RSTR_FSMCRST_Msk           (0x1UL << RCC_AHB3RSTR_FSMCRST_Pos)  /*!< 0x00000001 */
#define RCC_AHB3RSTR_FSMCRST               RCC_AHB3RSTR_FSMCRST_Msk


/********************  Bit definition for RCC_APB1RSTR register  **************/
#define RCC_APB1RSTR_TIM2RST_Pos           (0U)
#define RCC_APB1RSTR_TIM2RST_Msk           (0x1UL << RCC_APB1RSTR_TIM2RST_Pos)  /*!< 0x00000001 */
#define RCC_APB1RSTR_TIM2RST               RCC_APB1RSTR_TIM2RST_Msk
#define RCC_APB1RSTR_TIM3RST_Pos           (1U)
#define RCC_APB1RSTR_TIM3RST_Msk           (0x1UL << RCC_APB1RSTR_TIM3RST_Pos)  /*!< 0x00000002 */
#define RCC_APB1RSTR_TIM3RST               RCC_APB1RSTR_TIM3RST_Msk
#define RCC_APB1RSTR_TIM4RST_Pos           (2U)
#define RCC_APB1RSTR_TIM4RST_Msk           (0x1UL << RCC_APB1RSTR_TIM4RST_Pos)  /*!< 0x00000004 */
#define RCC_APB1RSTR_TIM4RST               RCC_APB1RSTR_TIM4RST_Msk
#define RCC_APB1RSTR_TIM5RST_Pos           (3U)
#define RCC_APB1RSTR_TIM5RST_Msk           (0x1UL << RCC_APB1RSTR_TIM5RST_Pos)  /*!< 0x00000008 */
#define RCC_APB1RSTR_TIM5RST               RCC_APB1RSTR_TIM5RST_Msk
#define RCC_APB1RSTR_TIM6RST_Pos           (4U)
#define RCC_APB1RSTR_TIM6RST_Msk           (0x1UL << RCC_APB1RSTR_TIM6RST_Pos)  /*!< 0x00000010 */
#define RCC_APB1RSTR_TIM6RST               RCC_APB1RSTR_TIM6RST_Msk
#define RCC_APB1RSTR_TIM7RST_Pos           (5U)
#define RCC_APB1RSTR_TIM7RST_Msk           (0x1UL << RCC_APB1RSTR_TIM7RST_Pos)  /*!< 0x00000020 */
#define RCC_APB1RSTR_TIM7RST               RCC_APB1RSTR_TIM7RST_Msk
#define RCC_APB1RSTR_TIM12RST_Pos          (6U)
#define RCC_APB1RSTR_TIM12RST_Msk          (0x1UL << RCC_APB1RSTR_TIM12RST_Pos) /*!< 0x00000040 */
#define RCC_APB1RSTR_TIM12RST              RCC_APB1RSTR_TIM12RST_Msk
#define RCC_APB1RSTR_TIM13RST_Pos          (7U)
#define RCC_APB1RSTR_TIM13RST_Msk          (0x1UL << RCC_APB1RSTR_TIM13RST_Pos) /*!< 0x00000080 */
#define RCC_APB1RSTR_TIM13RST              RCC_APB1RSTR_TIM13RST_Msk
#define RCC_APB1RSTR_TIM14RST_Pos          (8U)
#define RCC_APB1RSTR_TIM14RST_Msk          (0x1UL << RCC_APB1RSTR_TIM14RST_Pos) /*!< 0x00000100 */
#define RCC_APB1RSTR_TIM14RST              RCC_APB1RSTR_TIM14RST_Msk
#define RCC_APB1RSTR_WWDGRST_Pos           (11U)
#define RCC_APB1RSTR_WWDGRST_Msk           (0x1UL << RCC_APB1RSTR_WWDGRST_Pos)  /*!< 0x00000800 */
#define RCC_APB1RSTR_WWDGRST               RCC_APB1RSTR_WWDGRST_Msk
#define RCC_APB1RSTR_SPI2RST_Pos           (14U)
#define RCC_APB1RSTR_SPI2RST_Msk           (0x1UL << RCC_APB1RSTR_SPI2RST_Pos)  /*!< 0x00004000 */
#define RCC_APB1RSTR_SPI2RST               RCC_APB1RSTR_SPI2RST_Msk
#define RCC_APB1RSTR_SPI3RST_Pos           (15U)
#define RCC_APB1RSTR_SPI3RST_Msk           (0x1UL << RCC_APB1RSTR_SPI3RST_Pos)  /*!< 0x00008000 */
#define RCC_APB1RSTR_SPI3RST               RCC_APB1RSTR_SPI3RST_Msk
#define RCC_APB1RSTR_USART2RST_Pos         (17U)
#define RCC_APB1RSTR_USART2RST_Msk         (0x1UL << RCC_APB1RSTR_USART2RST_Pos) /*!< 0x00020000 */
#define RCC_APB1RSTR_USART2RST             RCC_APB1RSTR_USART2RST_Msk
#define RCC_APB1RSTR_USART3RST_Pos         (18U)
#define RCC_APB1RSTR_USART3RST_Msk         (0x1UL << RCC_APB1RSTR_USART3RST_Pos) /*!< 0x00040000 */
#define RCC_APB1RSTR_USART3RST             RCC_APB1RSTR_USART3RST_Msk
#define RCC_APB1RSTR_UART4RST_Pos          (19U)
#define RCC_APB1RSTR_UART4RST_Msk          (0x1UL << RCC_APB1RSTR_UART4RST_Pos) /*!< 0x00080000 */
#define RCC_APB1RSTR_UART4RST              RCC_APB1RSTR_UART4RST_Msk
#define RCC_APB1RSTR_UART5RST_Pos          (20U)
#define RCC_APB1RSTR_UART5RST_Msk          (0x1UL << RCC_APB1RSTR_UART5RST_Pos) /*!< 0x00100000 */
#define RCC_APB1RSTR_UART5RST              RCC_APB1RSTR_UART5RST_Msk
#define RCC_APB1RSTR_I2C1RST_Pos           (21U)
#define RCC_APB1RSTR_I2C1RST_Msk           (0x1UL << RCC_APB1RSTR_I2C1RST_Pos)  /*!< 0x00200000 */
#define RCC_APB1RSTR_I2C1RST               RCC_APB1RSTR_I2C1RST_Msk
#define RCC_APB1RSTR_I2C2RST_Pos           (22U)
#define RCC_APB1RSTR_I2C2RST_Msk           (0x1UL << RCC_APB1RSTR_I2C2RST_Pos)  /*!< 0x00400000 */
#define RCC_APB1RSTR_I2C2RST               RCC_APB1RSTR_I2C2RST_Msk
#define RCC_APB1RSTR_I2C3RST_Pos           (23U)
#define RCC_APB1RSTR_I2C3RST_Msk           (0x1UL << RCC_APB1RSTR_I2C3RST_Pos)  /*!< 0x00800000 */
#define RCC_APB1RSTR_I2C3RST               RCC_APB1RSTR_I2C3RST_Msk
#define RCC_APB1RSTR_CAN1RST_Pos           (25U)
#define RCC_APB1RSTR_CAN1RST_Msk           (0x1UL << RCC_APB1RSTR_CAN1RST_Pos)  /*!< 0x02000000 */
#define RCC_APB1RSTR_CAN1RST               RCC_APB1RSTR_CAN1RST_Msk
#define RCC_APB1RSTR_CAN2RST_Pos           (26U)
#define RCC_APB1RSTR_CAN2RST_Msk           (0x1UL << RCC_APB1RSTR_CAN2RST_Pos)  /*!< 0x04000000 */
#define RCC_APB1RSTR_CAN2RST               RCC_APB1RSTR_CAN2RST_Msk
#define RCC_APB1RSTR_PWRRST_Pos            (28U)
#define RCC_APB1RSTR_PWRRST_Msk            (0x1UL << RCC_APB1RSTR_PWRRST_Pos)   /*!< 0x10000000 */
#define RCC_APB1RSTR_PWRRST                RCC_APB1RSTR_PWRRST_Msk
#define RCC_APB1RSTR_DACRST_Pos            (29U)
#define RCC_APB1RSTR_DACRST_Msk            (0x1UL << RCC_APB1RSTR_DACRST_Pos)   /*!< 0x20000000 */
#define RCC_APB1RSTR_DACRST                RCC_APB1RSTR_DACRST_Msk

/********************  Bit definition for RCC_APB2RSTR register  **************/
#define RCC_APB2RSTR_TIM1RST_Pos           (0U)
#define RCC_APB2RSTR_TIM1RST_Msk           (0x1UL << RCC_APB2RSTR_TIM1RST_Pos)  /*!< 0x00000001 */
#define RCC_APB2RSTR_TIM1RST               RCC_APB2RSTR_TIM1RST_Msk
#define RCC_APB2RSTR_TIM8RST_Pos           (1U)
#define RCC_APB2RSTR_TIM8RST_Msk           (0x1UL << RCC_APB2RSTR_TIM8RST_Pos)  /*!< 0x00000002 */
#define RCC_APB2RSTR_TIM8RST               RCC_APB2RSTR_TIM8RST_Msk
#define RCC_APB2RSTR_USART1RST_Pos         (4U)
#define RCC_APB2RSTR_USART1RST_Msk         (0x1UL << RCC_APB2RSTR_USART1RST_Pos) /*!< 0x00000010 */
#define RCC_APB2RSTR_USART1RST             RCC_APB2RSTR_USART1RST_Msk
#define RCC_APB2RSTR_USART6RST_Pos         (5U)
#define RCC_APB2RSTR_USART6RST_Msk         (0x1UL << RCC_APB2RSTR_USART6RST_Pos) /*!< 0x00000020 */
#define RCC_APB2RSTR_USART6RST             RCC_APB2RSTR_USART6RST_Msk
#define RCC_APB2RSTR_ADCRST_Pos            (8U)
#define RCC_APB2RSTR_ADCRST_Msk            (0x1UL << RCC_APB2RSTR_ADCRST_Pos)   /*!< 0x00000100 */
#define RCC_APB2RSTR_ADCRST                RCC_APB2RSTR_ADCRST_Msk
#define RCC_APB2RSTR_SDIORST_Pos           (11U)
#define RCC_APB2RSTR_SDIORST_Msk           (0x1UL << RCC_APB2RSTR_SDIORST_Pos)  /*!< 0x00000800 */
#define RCC_APB2RSTR_SDIORST               RCC_APB2RSTR_SDIORST_Msk
#define RCC_APB2RSTR_SPI1RST_Pos           (12U)
#define RCC_APB2RSTR_SPI1RST_Msk           (0x1UL << RCC_APB2RSTR_SPI1RST_Pos)  /*!< 0x00001000 */
#define RCC_APB2RSTR_SPI1RST               RCC_APB2RSTR_SPI1RST_Msk
#define RCC_APB2RSTR_SYSCFGRST_Pos         (14U)
#define RCC_APB2RSTR_SYSCFGRST_Msk         (0x1UL << RCC_APB2RSTR_SYSCFGRST_Pos) /*!< 0x00004000 */
#define RCC_APB2RSTR_SYSCFGRST             RCC_APB2RSTR_SYSCFGRST_Msk
#define RCC_APB2RSTR_TIM9RST_Pos           (16U)
#define RCC_APB2RSTR_TIM9RST_Msk           (0x1UL << RCC_APB2RSTR_TIM9RST_Pos)  /*!< 0x00010000 */
#define RCC_APB2RSTR_TIM9RST               RCC_APB2RSTR_TIM9RST_Msk
#define RCC_APB2RSTR_TIM10RST_Pos          (17U)
#define RCC_APB2RSTR_TIM10RST_Msk          (0x1UL << RCC_APB2RSTR_TIM10RST_Pos) /*!< 0x00020000 */
#define RCC_APB2RSTR_TIM10RST              RCC_APB2RSTR_TIM10RST_Msk
#define RCC_APB2RSTR_TIM11RST_Pos          (18U)
#define RCC_APB2RSTR_TIM11RST_Msk          (0x1UL << RCC_APB2RSTR_TIM11RST_Pos) /*!< 0x00040000 */
#define RCC_APB2RSTR_TIM11RST              RCC_APB2RSTR_TIM11RST_Msk

/* Old SPI1RST bit definition, maintained for legacy purpose */
#define  RCC_APB2RSTR_SPI1                   RCC_APB2RSTR_SPI1RST

/********************  Bit definition for RCC_AHB1ENR register  ***************/
#define RCC_AHB1ENR_GPIOAEN_Pos            (0U)
#define RCC_AHB1ENR_GPIOAEN_Msk            (0x1UL << RCC_AHB1ENR_GPIOAEN_Pos)   /*!< 0x00000001 */
#define RCC_AHB1ENR_GPIOAEN                RCC_AHB1ENR_GPIOAEN_Msk
#define RCC_AHB1ENR_GPIOBEN_Pos            (1U)
#define RCC_AHB1ENR_GPIOBEN_Msk            (0x1UL << RCC_AHB1ENR_GPIOBEN_Pos)   /*!< 0x00000002 */
#define RCC_AHB1ENR_GPIOBEN                RCC_AHB1ENR_GPIOBEN_Msk
#define RCC_AHB1ENR_GPIOCEN_Pos            (2U)
#define RCC_AHB1ENR_GPIOCEN_Msk            (0x1UL << RCC_AHB1ENR_GPIOCEN_Pos)   /*!< 0x00000004 */
#define RCC_AHB1ENR_GPIOCEN                RCC_AHB1ENR_GPIOCEN_Msk
#define RCC_AHB1ENR_GPIODEN_Pos            (3U)
#define RCC_AHB1ENR_GPIODEN_Msk            (0x1UL << RCC_AHB1ENR_GPIODEN_Pos)   /*!< 0x00000008 */
#define RCC_AHB1ENR_GPIODEN                RCC_AHB1ENR_GPIODEN_Msk
#define RCC_AHB1ENR_GPIOEEN_Pos            (4U)
#define RCC_AHB1ENR_GPIOEEN_Msk            (0x1UL << RCC_AHB1ENR_GPIOEEN_Pos)   /*!< 0x00000010 */
#define RCC_AHB1ENR_GPIOEEN                RCC_AHB1ENR_GPIOEEN_Msk
#define RCC_AHB1ENR_GPIOFEN_Pos            (5U)
#define RCC_AHB1ENR_GPIOFEN_Msk            (0x1UL << RCC_AHB1ENR_GPIOFEN_Pos)   /*!< 0x00000020 */
#define RCC_AHB1ENR_GPIOFEN                RCC_AHB1ENR_GPIOFEN_Msk
#define RCC_AHB1ENR_GPIOGEN_Pos            (6U)
#define RCC_AHB1ENR_GPIOGEN_Msk            (0x1UL << RCC_AHB1ENR_GPIOGEN_Pos)   /*!< 0x00000040 */
#define RCC_AHB1ENR_GPIOGEN                RCC_AHB1ENR_GPIOGEN_Msk
#define RCC_AHB1ENR_GPIOHEN_Pos            (7U)
#define RCC_AHB1ENR_GPIOHEN_Msk            (0x1UL << RCC_AHB1ENR_GPIOHEN_Pos)   /*!< 0x00000080 */
#define RCC_AHB1ENR_GPIOHEN                RCC_AHB1ENR_GPIOHEN_Msk
#define RCC_AHB1ENR_GPIOIEN_Pos            (8U)
#define RCC_AHB1ENR_GPIOIEN_Msk            (0x1UL << RCC_AHB1ENR_GPIOIEN_Pos)   /*!< 0x00000100 */
#define RCC_AHB1ENR_GPIOIEN                RCC_AHB1ENR_GPIOIEN_Msk
#define RCC_AHB1ENR_CRCEN_Pos              (12U)
#define RCC_AHB1ENR_CRCEN_Msk              (0x1UL << RCC_AHB1ENR_CRCEN_Pos)     /*!< 0x00001000 */
#define RCC_AHB1ENR_CRCEN                  RCC_AHB1ENR_CRCEN_Msk
#define RCC_AHB1ENR_BKPSRAMEN_Pos          (18U)
#define RCC_AHB1ENR_BKPSRAMEN_Msk          (0x1UL << RCC_AHB1ENR_BKPSRAMEN_Pos) /*!< 0x00040000 */
#define RCC_AHB1ENR_BKPSRAMEN              RCC_AHB1ENR_BKPSRAMEN_Msk
#define RCC_AHB1ENR_CCMDATARAMEN_Pos       (20U)
#define RCC_AHB1ENR_CCMDATARAMEN_Msk       (0x1UL << RCC_AHB1ENR_CCMDATARAMEN_Pos) /*!< 0x00100000 */
#define RCC_AHB1ENR_CCMDATARAMEN           RCC_AHB1ENR_CCMDATARAMEN_Msk
#define RCC_AHB1ENR_DMA1EN_Pos             (21U)
#define RCC_AHB1ENR_DMA1EN_Msk             (0x1UL << RCC_AHB1ENR_DMA1EN_Pos)    /*!< 0x00200000 */
#define RCC_AHB1ENR_DMA1EN                 RCC_AHB1ENR_DMA1EN_Msk
#define RCC_AHB1ENR_DMA2EN_Pos             (22U)
#define RCC_AHB1ENR_DMA2EN_Msk             (0x1UL << RCC_AHB1ENR_DMA2EN_Pos)    /*!< 0x00400000 */
#define RCC_AHB1ENR_DMA2EN                 RCC_AHB1ENR_DMA2EN_Msk
#define RCC_AHB1ENR_ETHMACEN_Pos           (25U)
#define RCC_AHB1ENR_ETHMACEN_Msk           (0x1UL << RCC_AHB1ENR_ETHMACEN_Pos)  /*!< 0x02000000 */
#define RCC_AHB1ENR_ETHMACEN               RCC_AHB1ENR_ETHMACEN_Msk
#define RCC_AHB1ENR_ETHMACTXEN_Pos         (26U)
#define RCC_AHB1ENR_ETHMACTXEN_Msk         (0x1UL << RCC_AHB1ENR_ETHMACTXEN_Pos) /*!< 0x04000000 */
#define RCC_AHB1ENR_ETHMACTXEN             RCC_AHB1ENR_ETHMACTXEN_Msk
#define RCC_AHB1ENR_ETHMACRXEN_Pos         (27U)
#define RCC_AHB1ENR_ETHMACRXEN_Msk         (0x1UL << RCC_AHB1ENR_ETHMACRXEN_Pos) /*!< 0x08000000 */
#define RCC_AHB1ENR_ETHMACRXEN             RCC_AHB1ENR_ETHMACRXEN_Msk
#define RCC_AHB1ENR_ETHMACPTPEN_Pos        (28U)
#define RCC_AHB1ENR_ETHMACPTPEN_Msk        (0x1UL << RCC_AHB1ENR_ETHMACPTPEN_Pos) /*!< 0x10000000 */
#define RCC_AHB1ENR_ETHMACPTPEN            RCC_AHB1ENR_ETHMACPTPEN_Msk
#define RCC_AHB1ENR_OTGHSEN_Pos            (29U)
#define RCC_AHB1ENR_OTGHSEN_Msk            (0x1UL << RCC_AHB1ENR_OTGHSEN_Pos)   /*!< 0x20000000 */
#define RCC_AHB1ENR_OTGHSEN                RCC_AHB1ENR_OTGHSEN_Msk
#define RCC_AHB1ENR_OTGHSULPIEN_Pos        (30U)
#define RCC_AHB1ENR_OTGHSULPIEN_Msk        (0x1UL << RCC_AHB1ENR_OTGHSULPIEN_Pos) /*!< 0x40000000 */
#define RCC_AHB1ENR_OTGHSULPIEN            RCC_AHB1ENR_OTGHSULPIEN_Msk
/********************  Bit definition for RCC_AHB2ENR register  ***************/
/*
 * @brief Specific device feature definitions (not present on all devices in the STM32F4 serie)
 */
#define RCC_AHB2_SUPPORT                   /*!< AHB2 Bus is supported */

#define RCC_AHB2ENR_DCMIEN_Pos             (0U)
#define RCC_AHB2ENR_DCMIEN_Msk             (0x1UL << RCC_AHB2ENR_DCMIEN_Pos)    /*!< 0x00000001 */
#define RCC_AHB2ENR_DCMIEN                 RCC_AHB2ENR_DCMIEN_Msk
#define RCC_AHB2ENR_RNGEN_Pos              (6U)
#define RCC_AHB2ENR_RNGEN_Msk              (0x1UL << RCC_AHB2ENR_RNGEN_Pos)     /*!< 0x00000040 */
#define RCC_AHB2ENR_RNGEN                  RCC_AHB2ENR_RNGEN_Msk
#define RCC_AHB2ENR_OTGFSEN_Pos            (7U)
#define RCC_AHB2ENR_OTGFSEN_Msk            (0x1UL << RCC_AHB2ENR_OTGFSEN_Pos)   /*!< 0x00000080 */
#define RCC_AHB2ENR_OTGFSEN                RCC_AHB2ENR_OTGFSEN_Msk

/********************  Bit definition for RCC_AHB3ENR register  ***************/
/*
 * @brief Specific device feature definitions (not present on all devices in the STM32F4 serie)
 */
#define RCC_AHB3_SUPPORT                   /*!< AHB3 Bus is supported */

#define RCC_AHB3ENR_FSMCEN_Pos             (0U)
#define RCC_AHB3ENR_FSMCEN_Msk             (0x1UL << RCC_AHB3ENR_FSMCEN_Pos)    /*!< 0x00000001 */
#define RCC_AHB3ENR_FSMCEN                 RCC_AHB3ENR_FSMCEN_Msk

/********************  Bit definition for RCC_APB1ENR register  ***************/
#define RCC_APB1ENR_TIM2EN_Pos             (0U)
#define RCC_APB1ENR_TIM2EN_Msk             (0x1UL << RCC_APB1ENR_TIM2EN_Pos)    /*!< 0x00000001 */
#define RCC_APB1ENR_TIM2EN                 RCC_APB1ENR_TIM2EN_Msk
#define RCC_APB1ENR_TIM3EN_Pos             (1U)
#define RCC_APB1ENR_TIM3EN_Msk             (0x1UL << RCC_APB1ENR_TIM3EN_Pos)    /*!< 0x00000002 */
#define RCC_APB1ENR_TIM3EN                 RCC_APB1ENR_TIM3EN_Msk
#define RCC_APB1ENR_TIM4EN_Pos             (2U)
#define RCC_APB1ENR_TIM4EN_Msk             (0x1UL << RCC_APB1ENR_TIM4EN_Pos)    /*!< 0x00000004 */
#define RCC_APB1ENR_TIM4EN                 RCC_APB1ENR_TIM4EN_Msk
#define RCC_APB1ENR_TIM5EN_Pos             (3U)
#define RCC_APB1ENR_TIM5EN_Msk             (0x1UL << RCC_APB1ENR_TIM5EN_Pos)    /*!< 0x00000008 */
#define RCC_APB1ENR_TIM5EN                 RCC_APB1ENR_TIM5EN_Msk
#define RCC_APB1ENR_TIM6EN_Pos             (4U)
#define RCC_APB1ENR_TIM6EN_Msk             (0x1UL << RCC_APB1ENR_TIM6EN_Pos)    /*!< 0x00000010 */
#define RCC_APB1ENR_TIM6EN                 RCC_APB1ENR_TIM6EN_Msk
#define RCC_APB1ENR_TIM7EN_Pos             (5U)
#define RCC_APB1ENR_TIM7EN_Msk             (0x1UL << RCC_APB1ENR_TIM7EN_Pos)    /*!< 0x00000020 */
#define RCC_APB1ENR_TIM7EN                 RCC_APB1ENR_TIM7EN_Msk
#define RCC_APB1ENR_TIM12EN_Pos            (6U)
#define RCC_APB1ENR_TIM12EN_Msk            (0x1UL << RCC_APB1ENR_TIM12EN_Pos)   /*!< 0x00000040 */
#define RCC_APB1ENR_TIM12EN                RCC_APB1ENR_TIM12EN_Msk
#define RCC_APB1ENR_TIM13EN_Pos            (7U)
#define RCC_APB1ENR_TIM13EN_Msk            (0x1UL << RCC_APB1ENR_TIM13EN_Pos)   /*!< 0x00000080 */
#define RCC_APB1ENR_TIM13EN                RCC_APB1ENR_TIM13EN_Msk
#define RCC_APB1ENR_TIM14EN_Pos            (8U)
#define RCC_APB1ENR_TIM14EN_Msk            (0x1UL << RCC_APB1ENR_TIM14EN_Pos)   /*!< 0x00000100 */
#define RCC_APB1ENR_TIM14EN                RCC_APB1ENR_TIM14EN_Msk
#define RCC_APB1ENR_WWDGEN_Pos             (11U)
#define RCC_APB1ENR_WWDGEN_Msk             (0x1UL << RCC_APB1ENR_WWDGEN_Pos)    /*!< 0x00000800 */
#define RCC_APB1ENR_WWDGEN                 RCC_APB1ENR_WWDGEN_Msk
#define RCC_APB1ENR_SPI2EN_Pos             (14U)
#define RCC_APB1ENR_SPI2EN_Msk             (0x1UL << RCC_APB1ENR_SPI2EN_Pos)    /*!< 0x00004000 */
#define RCC_APB1ENR_SPI2EN                 RCC_APB1ENR_SPI2EN_Msk
#define RCC_APB1ENR_SPI3EN_Pos             (15U)
#define RCC_APB1ENR_SPI3EN_Msk             (0x1UL << RCC_APB1ENR_SPI3EN_Pos)    /*!< 0x00008000 */
#define RCC_APB1ENR_SPI3EN                 RCC_APB1ENR_SPI3EN_Msk
#define RCC_APB1ENR_USART2EN_Pos           (17U)
#define RCC_APB1ENR_USART2EN_Msk           (0x1UL << RCC_APB1ENR_USART2EN_Pos)  /*!< 0x00020000 */
#define RCC_APB1ENR_USART2EN               RCC_APB1ENR_USART2EN_Msk
#define RCC_APB1ENR_USART3EN_Pos           (18U)
#define RCC_APB1ENR_USART3EN_Msk           (0x1UL << RCC_APB1ENR_USART3EN_Pos)  /*!< 0x00040000 */
#define RCC_APB1ENR_USART3EN               RCC_APB1ENR_USART3EN_Msk
#define RCC_APB1ENR_UART4EN_Pos            (19U)
#define RCC_APB1ENR_UART4EN_Msk            (0x1UL << RCC_APB1ENR_UART4EN_Pos)   /*!< 0x00080000 */
#define RCC_APB1ENR_UART4EN                RCC_APB1ENR_UART4EN_Msk
#define RCC_APB1ENR_UART5EN_Pos            (20U)
#define RCC_APB1ENR_UART5EN_Msk            (0x1UL << RCC_APB1ENR_UART5EN_Pos)   /*!< 0x00100000 */
#define RCC_APB1ENR_UART5EN                RCC_APB1ENR_UART5EN_Msk
#define RCC_APB1ENR_I2C1EN_Pos             (21U)
#define RCC_APB1ENR_I2C1EN_Msk             (0x1UL << RCC_APB1ENR_I2C1EN_Pos)    /*!< 0x00200000 */
#define RCC_APB1ENR_I2C1EN                 RCC_APB1ENR_I2C1EN_Msk
#define RCC_APB1ENR_I2C2EN_Pos             (22U)
#define RCC_APB1ENR_I2C2EN_Msk             (0x1UL << RCC_APB1ENR_I2C2EN_Pos)    /*!< 0x00400000 */
#define RCC_APB1ENR_I2C2EN                 RCC_APB1ENR_I2C2EN_Msk
#define RCC_APB1ENR_I2C3EN_Pos             (23U)
#define RCC_APB1ENR_I2C3EN_Msk             (0x1UL << RCC_APB1ENR_I2C3EN_Pos)    /*!< 0x00800000 */
#define RCC_APB1ENR_I2C3EN                 RCC_APB1ENR_I2C3EN_Msk
#define RCC_APB1ENR_CAN1EN_Pos             (25U)
#define RCC_APB1ENR_CAN1EN_Msk             (0x1UL << RCC_APB1ENR_CAN1EN_Pos)    /*!< 0x02000000 */
#define RCC_APB1ENR_CAN1EN                 RCC_APB1ENR_CAN1EN_Msk
#define RCC_APB1ENR_CAN2EN_Pos             (26U)
#define RCC_APB1ENR_CAN2EN_Msk             (0x1UL << RCC_APB1ENR_CAN2EN_Pos)    /*!< 0x04000000 */
#define RCC_APB1ENR_CAN2EN                 RCC_APB1ENR_CAN2EN_Msk
#define RCC_APB1ENR_PWREN_Pos              (28U)
#define RCC_APB1ENR_PWREN_Msk              (0x1UL << RCC_APB1ENR_PWREN_Pos)     /*!< 0x10000000 */
#define RCC_APB1ENR_PWREN                  RCC_APB1ENR_PWREN_Msk
#define RCC_APB1ENR_DACEN_Pos              (29U)
#define RCC_APB1ENR_DACEN_Msk              (0x1UL << RCC_APB1ENR_DACEN_Pos)     /*!< 0x20000000 */
#define RCC_APB1ENR_DACEN                  RCC_APB1ENR_DACEN_Msk

/********************  Bit definition for RCC_APB2ENR register  ***************/
#define RCC_APB2ENR_TIM1EN_Pos             (0U)
#define RCC_APB2ENR_TIM1EN_Msk             (0x1UL << RCC_APB2ENR_TIM1EN_Pos)    /*!< 0x00000001 */
#define RCC_APB2ENR_TIM1EN                 RCC_APB2ENR_TIM1EN_Msk
#define RCC_APB2ENR_TIM8EN_Pos             (1U)
#define RCC_APB2ENR_TIM8EN_Msk             (0x1UL << RCC_APB2ENR_TIM8EN_Pos)    /*!< 0x00000002 */
#define RCC_APB2ENR_TIM8EN                 RCC_APB2ENR_TIM8EN_Msk
#define RCC_APB2ENR_USART1EN_Pos           (4U)
#define RCC_APB2ENR_USART1EN_Msk           (0x1UL << RCC_APB2ENR_USART1EN_Pos)  /*!< 0x00000010 */
#define RCC_APB2ENR_USART1EN               RCC_APB2ENR_USART1EN_Msk
#define RCC_APB2ENR_USART6EN_Pos           (5U)
#define RCC_APB2ENR_USART6EN_Msk           (0x1UL << RCC_APB2ENR_USART6EN_Pos)  /*!< 0x00000020 */
#define RCC_APB2ENR_USART6EN               RCC_APB2ENR_USART6EN_Msk
#define RCC_APB2ENR_ADC1EN_Pos             (8U)
#define RCC_APB2ENR_ADC1EN_Msk             (0x1UL << RCC_APB2ENR_ADC1EN_Pos)    /*!< 0x00000100 */
#define RCC_APB2ENR_ADC1EN                 RCC_APB2ENR_ADC1EN_Msk
#define RCC_APB2ENR_ADC2EN_Pos             (9U)
#define RCC_APB2ENR_ADC2EN_Msk             (0x1UL << RCC_APB2ENR_ADC2EN_Pos)    /*!< 0x00000200 */
#define RCC_APB2ENR_ADC2EN                 RCC_APB2ENR_ADC2EN_Msk
#define RCC_APB2ENR_ADC3EN_Pos             (10U)
#define RCC_APB2ENR_ADC3EN_Msk             (0x1UL << RCC_APB2ENR_ADC3EN_Pos)    /*!< 0x00000400 */
#define RCC_APB2ENR_ADC3EN                 RCC_APB2ENR_ADC3EN_Msk
#define RCC_APB2ENR_SDIOEN_Pos             (11U)
#define RCC_APB2ENR_SDIOEN_Msk             (0x1UL << RCC_APB2ENR_SDIOEN_Pos)    /*!< 0x00000800 */
#define RCC_APB2ENR_SDIOEN                 RCC_APB2ENR_SDIOEN_Msk
#define RCC_APB2ENR_SPI1EN_Pos             (12U)
#define RCC_APB2ENR_SPI1EN_Msk             (0x1UL << RCC_APB2ENR_SPI1EN_Pos)    /*!< 0x00001000 */
#define RCC_APB2ENR_SPI1EN                 RCC_APB2ENR_SPI1EN_Msk
#define RCC_APB2ENR_SYSCFGEN_Pos           (14U)
#define RCC_APB2ENR_SYSCFGEN_Msk           (0x1UL << RCC_APB2ENR_SYSCFGEN_Pos)  /*!< 0x00004000 */
#define RCC_APB2ENR_SYSCFGEN               RCC_APB2ENR_SYSCFGEN_Msk
#define RCC_APB2ENR_TIM9EN_Pos             (16U)
#define RCC_APB2ENR_TIM9EN_Msk             (0x1UL << RCC_APB2ENR_TIM9EN_Pos)    /*!< 0x00010000 */
#define RCC_APB2ENR_TIM9EN                 RCC_APB2ENR_TIM9EN_Msk
#define RCC_APB2ENR_TIM10EN_Pos            (17U)
#define RCC_APB2ENR_TIM10EN_Msk            (0x1UL << RCC_APB2ENR_TIM10EN_Pos)   /*!< 0x00020000 */
#define RCC_APB2ENR_TIM10EN                RCC_APB2ENR_TIM10EN_Msk
#define RCC_APB2ENR_TIM11EN_Pos            (18U)
#define RCC_APB2ENR_TIM11EN_Msk            (0x1UL << RCC_APB2ENR_TIM11EN_Pos)   /*!< 0x00040000 */
#define RCC_APB2ENR_TIM11EN                RCC_APB2ENR_TIM11EN_Msk

/********************  Bit definition for RCC_AHB1LPENR register  *************/
#define RCC_AHB1LPENR_GPIOALPEN_Pos        (0U)
#define RCC_AHB1LPENR_GPIOALPEN_Msk        (0x1UL << RCC_AHB1LPENR_GPIOALPEN_Pos) /*!< 0x00000001 */
#define RCC_AHB1LPENR_GPIOALPEN            RCC_AHB1LPENR_GPIOALPEN_Msk
#define RCC_AHB1LPENR_GPIOBLPEN_Pos        (1U)
#define RCC_AHB1LPENR_GPIOBLPEN_Msk        (0x1UL << RCC_AHB1LPENR_GPIOBLPEN_Pos) /*!< 0x00000002 */
#define RCC_AHB1LPENR_GPIOBLPEN            RCC_AHB1LPENR_GPIOBLPEN_Msk
#define RCC_AHB1LPENR_GPIOCLPEN_Pos        (2U)
#define RCC_AHB1LPENR_GPIOCLPEN_Msk        (0x1UL << RCC_AHB1LPENR_GPIOCLPEN_Pos) /*!< 0x00000004 */
#define RCC_AHB1LPENR_GPIOCLPEN            RCC_AHB1LPENR_GPIOCLPEN_Msk
#define RCC_AHB1LPENR_GPIODLPEN_Pos        (3U)
#define RCC_AHB1LPENR_GPIODLPEN_Msk        (0x1UL << RCC_AHB1LPENR_GPIODLPEN_Pos) /*!< 0x00000008 */
#define RCC_AHB1LPENR_GPIODLPEN            RCC_AHB1LPENR_GPIODLPEN_Msk
#define RCC_AHB1LPENR_GPIOELPEN_Pos        (4U)
#define RCC_AHB1LPENR_GPIOELPEN_Msk        (0x1UL << RCC_AHB1LPENR_GPIOELPEN_Pos) /*!< 0x00000010 */
#define RCC_AHB1LPENR_GPIOELPEN            RCC_AHB1LPENR_GPIOELPEN_Msk
#define RCC_AHB1LPENR_GPIOFLPEN_Pos        (5U)
#define RCC_AHB1LPENR_GPIOFLPEN_Msk        (0x1UL << RCC_AHB1LPENR_GPIOFLPEN_Pos) /*!< 0x00000020 */
#define RCC_AHB1LPENR_GPIOFLPEN            RCC_AHB1LPENR_GPIOFLPEN_Msk
#define RCC_AHB1LPENR_GPIOGLPEN_Pos        (6U)
#define RCC_AHB1LPENR_GPIOGLPEN_Msk        (0x1UL << RCC_AHB1LPENR_GPIOGLPEN_Pos) /*!< 0x00000040 */
#define RCC_AHB1LPENR_GPIOGLPEN            RCC_AHB1LPENR_GPIOGLPEN_Msk
#define RCC_AHB1LPENR_GPIOHLPEN_Pos        (7U)
#define RCC_AHB1LPENR_GPIOHLPEN_Msk        (0x1UL << RCC_AHB1LPENR_GPIOHLPEN_Pos) /*!< 0x00000080 */
#define RCC_AHB1LPENR_GPIOHLPEN            RCC_AHB1LPENR_GPIOHLPEN_Msk
#define RCC_AHB1LPENR_GPIOILPEN_Pos        (8U)
#define RCC_AHB1LPENR_GPIOILPEN_Msk        (0x1UL << RCC_AHB1LPENR_GPIOILPEN_Pos) /*!< 0x00000100 */
#define RCC_AHB1LPENR_GPIOILPEN            RCC_AHB1LPENR_GPIOILPEN_Msk
#define RCC_AHB1LPENR_CRCLPEN_Pos          (12U)
#define RCC_AHB1LPENR_CRCLPEN_Msk          (0x1UL << RCC_AHB1LPENR_CRCLPEN_Pos) /*!< 0x00001000 */
#define RCC_AHB1LPENR_CRCLPEN              RCC_AHB1LPENR_CRCLPEN_Msk
#define RCC_AHB1LPENR_FLITFLPEN_Pos        (15U)
#define RCC_AHB1LPENR_FLITFLPEN_Msk        (0x1UL << RCC_AHB1LPENR_FLITFLPEN_Pos) /*!< 0x00008000 */
#define RCC_AHB1LPENR_FLITFLPEN            RCC_AHB1LPENR_FLITFLPEN_Msk
#define RCC_AHB1LPENR_SRAM1LPEN_Pos        (16U)
#define RCC_AHB1LPENR_SRAM1LPEN_Msk        (0x1UL << RCC_AHB1LPENR_SRAM1LPEN_Pos) /*!< 0x00010000 */
#define RCC_AHB1LPENR_SRAM1LPEN            RCC_AHB1LPENR_SRAM1LPEN_Msk
#define RCC_AHB1LPENR_SRAM2LPEN_Pos        (17U)
#define RCC_AHB1LPENR_SRAM2LPEN_Msk        (0x1UL << RCC_AHB1LPENR_SRAM2LPEN_Pos) /*!< 0x00020000 */
#define RCC_AHB1LPENR_SRAM2LPEN            RCC_AHB1LPENR_SRAM2LPEN_Msk
#define RCC_AHB1LPENR_BKPSRAMLPEN_Pos      (18U)
#define RCC_AHB1LPENR_BKPSRAMLPEN_Msk      (0x1UL << RCC_AHB1LPENR_BKPSRAMLPEN_Pos) /*!< 0x00040000 */
#define RCC_AHB1LPENR_BKPSRAMLPEN          RCC_AHB1LPENR_BKPSRAMLPEN_Msk
#define RCC_AHB1LPENR_DMA1LPEN_Pos         (21U)
#define RCC_AHB1LPENR_DMA1LPEN_Msk         (0x1UL << RCC_AHB1LPENR_DMA1LPEN_Pos) /*!< 0x00200000 */
#define RCC_AHB1LPENR_DMA1LPEN             RCC_AHB1LPENR_DMA1LPEN_Msk
#define RCC_AHB1LPENR_DMA2LPEN_Pos         (22U)
#define RCC_AHB1LPENR_DMA2LPEN_Msk         (0x1UL << RCC_AHB1LPENR_DMA2LPEN_Pos) /*!< 0x00400000 */
#define RCC_AHB1LPENR_DMA2LPEN             RCC_AHB1LPENR_DMA2LPEN_Msk

#define RCC_AHB1LPENR_ETHMACLPEN_Pos       (25U)
#define RCC_AHB1LPENR_ETHMACLPEN_Msk       (0x1UL << RCC_AHB1LPENR_ETHMACLPEN_Pos) /*!< 0x02000000 */
#define RCC_AHB1LPENR_ETHMACLPEN           RCC_AHB1LPENR_ETHMACLPEN_Msk
#define RCC_AHB1LPENR_ETHMACTXLPEN_Pos     (26U)
#define RCC_AHB1LPENR_ETHMACTXLPEN_Msk     (0x1UL << RCC_AHB1LPENR_ETHMACTXLPEN_Pos) /*!< 0x04000000 */
#define RCC_AHB1LPENR_ETHMACTXLPEN         RCC_AHB1LPENR_ETHMACTXLPEN_Msk
#define RCC_AHB1LPENR_ETHMACRXLPEN_Pos     (27U)
#define RCC_AHB1LPENR_ETHMACRXLPEN_Msk     (0x1UL << RCC_AHB1LPENR_ETHMACRXLPEN_Pos) /*!< 0x08000000 */
#define RCC_AHB1LPENR_ETHMACRXLPEN         RCC_AHB1LPENR_ETHMACRXLPEN_Msk
#define RCC_AHB1LPENR_ETHMACPTPLPEN_Pos    (28U)
#define RCC_AHB1LPENR_ETHMACPTPLPEN_Msk    (0x1UL << RCC_AHB1LPENR_ETHMACPTPLPEN_Pos) /*!< 0x10000000 */
#define RCC_AHB1LPENR_ETHMACPTPLPEN        RCC_AHB1LPENR_ETHMACPTPLPEN_Msk
#define RCC_AHB1LPENR_OTGHSLPEN_Pos        (29U)
#define RCC_AHB1LPENR_OTGHSLPEN_Msk        (0x1UL << RCC_AHB1LPENR_OTGHSLPEN_Pos) /*!< 0x20000000 */
#define RCC_AHB1LPENR_OTGHSLPEN            RCC_AHB1LPENR_OTGHSLPEN_Msk
#define RCC_AHB1LPENR_OTGHSULPILPEN_Pos    (30U)
#define RCC_AHB1LPENR_OTGHSULPILPEN_Msk    (0x1UL << RCC_AHB1LPENR_OTGHSULPILPEN_Pos) /*!< 0x40000000 */
#define RCC_AHB1LPENR_OTGHSULPILPEN        RCC_AHB1LPENR_OTGHSULPILPEN_Msk

/********************  Bit definition for RCC_AHB2LPENR register  *************/
#define RCC_AHB2LPENR_DCMILPEN_Pos         (0U)
#define RCC_AHB2LPENR_DCMILPEN_Msk         (0x1UL << RCC_AHB2LPENR_DCMILPEN_Pos) /*!< 0x00000001 */
#define RCC_AHB2LPENR_DCMILPEN             RCC_AHB2LPENR_DCMILPEN_Msk
#define RCC_AHB2LPENR_RNGLPEN_Pos          (6U)
#define RCC_AHB2LPENR_RNGLPEN_Msk          (0x1UL << RCC_AHB2LPENR_RNGLPEN_Pos) /*!< 0x00000040 */
#define RCC_AHB2LPENR_RNGLPEN              RCC_AHB2LPENR_RNGLPEN_Msk
#define RCC_AHB2LPENR_OTGFSLPEN_Pos        (7U)
#define RCC_AHB2LPENR_OTGFSLPEN_Msk        (0x1UL << RCC_AHB2LPENR_OTGFSLPEN_Pos) /*!< 0x00000080 */
#define RCC_AHB2LPENR_OTGFSLPEN            RCC_AHB2LPENR_OTGFSLPEN_Msk

/********************  Bit definition for RCC_AHB3LPENR register  *************/
#define RCC_AHB3LPENR_FSMCLPEN_Pos         (0U)
#define RCC_AHB3LPENR_FSMCLPEN_Msk         (0x1UL << RCC_AHB3LPENR_FSMCLPEN_Pos) /*!< 0x00000001 */
#define RCC_AHB3LPENR_FSMCLPEN             RCC_AHB3LPENR_FSMCLPEN_Msk

/********************  Bit definition for RCC_APB1LPENR register  *************/
#define RCC_APB1LPENR_TIM2LPEN_Pos         (0U)
#define RCC_APB1LPENR_TIM2LPEN_Msk         (0x1UL << RCC_APB1LPENR_TIM2LPEN_Pos) /*!< 0x00000001 */
#define RCC_APB1LPENR_TIM2LPEN             RCC_APB1LPENR_TIM2LPEN_Msk
#define RCC_APB1LPENR_TIM3LPEN_Pos         (1U)
#define RCC_APB1LPENR_TIM3LPEN_Msk         (0x1UL << RCC_APB1LPENR_TIM3LPEN_Pos) /*!< 0x00000002 */
#define RCC_APB1LPENR_TIM3LPEN             RCC_APB1LPENR_TIM3LPEN_Msk
#define RCC_APB1LPENR_TIM4LPEN_Pos         (2U)
#define RCC_APB1LPENR_TIM4LPEN_Msk         (0x1UL << RCC_APB1LPENR_TIM4LPEN_Pos) /*!< 0x00000004 */
#define RCC_APB1LPENR_TIM4LPEN             RCC_APB1LPENR_TIM4LPEN_Msk
#define RCC_APB1LPENR_TIM5LPEN_Pos         (3U)
#define RCC_APB1LPENR_TIM5LPEN_Msk         (0x1UL << RCC_APB1LPENR_TIM5LPEN_Pos) /*!< 0x00000008 */
#define RCC_APB1LPENR_TIM5LPEN             RCC_APB1LPENR_TIM5LPEN_Msk
#define RCC_APB1LPENR_TIM6LPEN_Pos         (4U)
#define RCC_APB1LPENR_TIM6LPEN_Msk         (0x1UL << RCC_APB1LPENR_TIM6LPEN_Pos) /*!< 0x00000010 */
#define RCC_APB1LPENR_TIM6LPEN             RCC_APB1LPENR_TIM6LPEN_Msk
#define RCC_APB1LPENR_TIM7LPEN_Pos         (5U)
#define RCC_APB1LPENR_TIM7LPEN_Msk         (0x1UL << RCC_APB1LPENR_TIM7LPEN_Pos) /*!< 0x00000020 */
#define RCC_APB1LPENR_TIM7LPEN             RCC_APB1LPENR_TIM7LPEN_Msk
#define RCC_APB1LPENR_TIM12LPEN_Pos        (6U)
#define RCC_APB1LPENR_TIM12LPEN_Msk        (0x1UL << RCC_APB1LPENR_TIM12LPEN_Pos) /*!< 0x00000040 */
#define RCC_APB1LPENR_TIM12LPEN            RCC_APB1LPENR_TIM12LPEN_Msk
#define RCC_APB1LPENR_TIM13LPEN_Pos        (7U)
#define RCC_APB1LPENR_TIM13LPEN_Msk        (0x1UL << RCC_APB1LPENR_TIM13LPEN_Pos) /*!< 0x00000080 */
#define RCC_APB1LPENR_TIM13LPEN            RCC_APB1LPENR_TIM13LPEN_Msk
#define RCC_APB1LPENR_TIM14LPEN_Pos        (8U)
#define RCC_APB1LPENR_TIM14LPEN_Msk        (0x1UL << RCC_APB1LPENR_TIM14LPEN_Pos) /*!< 0x00000100 */
#define RCC_APB1LPENR_TIM14LPEN            RCC_APB1LPENR_TIM14LPEN_Msk
#define RCC_APB1LPENR_WWDGLPEN_Pos         (11U)
#define RCC_APB1LPENR_WWDGLPEN_Msk         (0x1UL << RCC_APB1LPENR_WWDGLPEN_Pos) /*!< 0x00000800 */
#define RCC_APB1LPENR_WWDGLPEN             RCC_APB1LPENR_WWDGLPEN_Msk
#define RCC_APB1LPENR_SPI2LPEN_Pos         (14U)
#define RCC_APB1LPENR_SPI2LPEN_Msk         (0x1UL << RCC_APB1LPENR_SPI2LPEN_Pos) /*!< 0x00004000 */
#define RCC_APB1LPENR_SPI2LPEN             RCC_APB1LPENR_SPI2LPEN_Msk
#define RCC_APB1LPENR_SPI3LPEN_Pos         (15U)
#define RCC_APB1LPENR_SPI3LPEN_Msk         (0x1UL << RCC_APB1LPENR_SPI3LPEN_Pos) /*!< 0x00008000 */
#define RCC_APB1LPENR_SPI3LPEN             RCC_APB1LPENR_SPI3LPEN_Msk
#define RCC_APB1LPENR_USART2LPEN_Pos       (17U)
#define RCC_APB1LPENR_USART2LPEN_Msk       (0x1UL << RCC_APB1LPENR_USART2LPEN_Pos) /*!< 0x00020000 */
#define RCC_APB1LPENR_USART2LPEN           RCC_APB1LPENR_USART2LPEN_Msk
#define RCC_APB1LPENR_USART3LPEN_Pos       (18U)
#define RCC_APB1LPENR_USART3LPEN_Msk       (0x1UL << RCC_APB1LPENR_USART3LPEN_Pos) /*!< 0x00040000 */
#define RCC_APB1LPENR_USART3LPEN           RCC_APB1LPENR_USART3LPEN_Msk
#define RCC_APB1LPENR_UART4LPEN_Pos        (19U)
#define RCC_APB1LPENR_UART4LPEN_Msk        (0x1UL << RCC_APB1LPENR_UART4LPEN_Pos) /*!< 0x00080000 */
#define RCC_APB1LPENR_UART4LPEN            RCC_APB1LPENR_UART4LPEN_Msk
#define RCC_APB1LPENR_UART5LPEN_Pos        (20U)
#define RCC_APB1LPENR_UART5LPEN_Msk        (0x1UL << RCC_APB1LPENR_UART5LPEN_Pos) /*!< 0x00100000 */
#define RCC_APB1LPENR_UART5LPEN            RCC_APB1LPENR_UART5LPEN_Msk
#define RCC_APB1LPENR_I2C1LPEN_Pos         (21U)
#define RCC_APB1LPENR_I2C1LPEN_Msk         (0x1UL << RCC_APB1LPENR_I2C1LPEN_Pos) /*!< 0x00200000 */
#define RCC_APB1LPENR_I2C1LPEN             RCC_APB1LPENR_I2C1LPEN_Msk
#define RCC_APB1LPENR_I2C2LPEN_Pos         (22U)
#define RCC_APB1LPENR_I2C2LPEN_Msk         (0x1UL << RCC_APB1LPENR_I2C2LPEN_Pos) /*!< 0x00400000 */
#define RCC_APB1LPENR_I2C2LPEN             RCC_APB1LPENR_I2C2LPEN_Msk
#define RCC_APB1LPENR_I2C3LPEN_Pos         (23U)
#define RCC_APB1LPENR_I2C3LPEN_Msk         (0x1UL << RCC_APB1LPENR_I2C3LPEN_Pos) /*!< 0x00800000 */
#define RCC_APB1LPENR_I2C3LPEN             RCC_APB1LPENR_I2C3LPEN_Msk
#define RCC_APB1LPENR_CAN1LPEN_Pos         (25U)
#define RCC_APB1LPENR_CAN1LPEN_Msk         (0x1UL << RCC_APB1LPENR_CAN1LPEN_Pos) /*!< 0x02000000 */
#define RCC_APB1LPENR_CAN1LPEN             RCC_APB1LPENR_CAN1LPEN_Msk
#define RCC_APB1LPENR_CAN2LPEN_Pos         (26U)
#define RCC_APB1LPENR_CAN2LPEN_Msk         (0x1UL << RCC_APB1LPENR_CAN2LPEN_Pos) /*!< 0x04000000 */
#define RCC_APB1LPENR_CAN2LPEN             RCC_APB1LPENR_CAN2LPEN_Msk
#define RCC_APB1LPENR_PWRLPEN_Pos          (28U)
#define RCC_APB1LPENR_PWRLPEN_Msk          (0x1UL << RCC_APB1LPENR_PWRLPEN_Pos) /*!< 0x10000000 */
#define RCC_APB1LPENR_PWRLPEN              RCC_APB1LPENR_PWRLPEN_Msk
#define RCC_APB1LPENR_DACLPEN_Pos          (29U)
#define RCC_APB1LPENR_DACLPEN_Msk          (0x1UL << RCC_APB1LPENR_DACLPEN_Pos) /*!< 0x20000000 */
#define RCC_APB1LPENR_DACLPEN              RCC_APB1LPENR_DACLPEN_Msk

/********************  Bit definition for RCC_APB2LPENR register  *************/
#define RCC_APB2LPENR_TIM1LPEN_Pos         (0U)
#define RCC_APB2LPENR_TIM1LPEN_Msk         (0x1UL << RCC_APB2LPENR_TIM1LPEN_Pos) /*!< 0x00000001 */
#define RCC_APB2LPENR_TIM1LPEN             RCC_APB2LPENR_TIM1LPEN_Msk
#define RCC_APB2LPENR_TIM8LPEN_Pos         (1U)
#define RCC_APB2LPENR_TIM8LPEN_Msk         (0x1UL << RCC_APB2LPENR_TIM8LPEN_Pos) /*!< 0x00000002 */
#define RCC_APB2LPENR_TIM8LPEN             RCC_APB2LPENR_TIM8LPEN_Msk
#define RCC_APB2LPENR_USART1LPEN_Pos       (4U)
#define RCC_APB2LPENR_USART1LPEN_Msk       (0x1UL << RCC_APB2LPENR_USART1LPEN_Pos) /*!< 0x00000010 */
#define RCC_APB2LPENR_USART1LPEN           RCC_APB2LPENR_USART1LPEN_Msk
#define RCC_APB2LPENR_USART6LPEN_Pos       (5U)
#define RCC_APB2LPENR_USART6LPEN_Msk       (0x1UL << RCC_APB2LPENR_USART6LPEN_Pos) /*!< 0x00000020 */
#define RCC_APB2LPENR_USART6LPEN           RCC_APB2LPENR_USART6LPEN_Msk
#define RCC_APB2LPENR_ADC1LPEN_Pos         (8U)
#define RCC_APB2LPENR_ADC1LPEN_Msk         (0x1UL << RCC_APB2LPENR_ADC1LPEN_Pos) /*!< 0x00000100 */
#define RCC_APB2LPENR_ADC1LPEN             RCC_APB2LPENR_ADC1LPEN_Msk
#define RCC_APB2LPENR_ADC2LPEN_Pos         (9U)
#define RCC_APB2LPENR_ADC2LPEN_Msk         (0x1UL << RCC_APB2LPENR_ADC2LPEN_Pos) /*!< 0x00000200 */
#define RCC_APB2LPENR_ADC2LPEN             RCC_APB2LPENR_ADC2LPEN_Msk
#define RCC_APB2LPENR_ADC3LPEN_Pos         (10U)
#define RCC_APB2LPENR_ADC3LPEN_Msk         (0x1UL << RCC_APB2LPENR_ADC3LPEN_Pos) /*!< 0x00000400 */
#define RCC_APB2LPENR_ADC3LPEN             RCC_APB2LPENR_ADC3LPEN_Msk
#define RCC_APB2LPENR_SDIOLPEN_Pos         (11U)
#define RCC_APB2LPENR_SDIOLPEN_Msk         (0x1UL << RCC_APB2LPENR_SDIOLPEN_Pos) /*!< 0x00000800 */
#define RCC_APB2LPENR_SDIOLPEN             RCC_APB2LPENR_SDIOLPEN_Msk
#define RCC_APB2LPENR_SPI1LPEN_Pos         (12U)
#define RCC_APB2LPENR_SPI1LPEN_Msk         (0x1UL << RCC_APB2LPENR_SPI1LPEN_Pos) /*!< 0x00001000 */
#define RCC_APB2LPENR_SPI1LPEN             RCC_APB2LPENR_SPI1LPEN_Msk
#define RCC_APB2LPENR_SYSCFGLPEN_Pos       (14U)
#define RCC_APB2LPENR_SYSCFGLPEN_Msk       (0x1UL << RCC_APB2LPENR_SYSCFGLPEN_Pos) /*!< 0x00004000 */
#define RCC_APB2LPENR_SYSCFGLPEN           RCC_APB2LPENR_SYSCFGLPEN_Msk
#define RCC_APB2LPENR_TIM9LPEN_Pos         (16U)
#define RCC_APB2LPENR_TIM9LPEN_Msk         (0x1UL << RCC_APB2LPENR_TIM9LPEN_Pos) /*!< 0x00010000 */
#define RCC_APB2LPENR_TIM9LPEN             RCC_APB2LPENR_TIM9LPEN_Msk
#define RCC_APB2LPENR_TIM10LPEN_Pos        (17U)
#define RCC_APB2LPENR_TIM10LPEN_Msk        (0x1UL << RCC_APB2LPENR_TIM10LPEN_Pos) /*!< 0x00020000 */
#define RCC_APB2LPENR_TIM10LPEN            RCC_APB2LPENR_TIM10LPEN_Msk
#define RCC_APB2LPENR_TIM11LPEN_Pos        (18U)
#define RCC_APB2LPENR_TIM11LPEN_Msk        (0x1UL << RCC_APB2LPENR_TIM11LPEN_Pos) /*!< 0x00040000 */
#define RCC_APB2LPENR_TIM11LPEN            RCC_APB2LPENR_TIM11LPEN_Msk

/********************  Bit definition for RCC_BDCR register  ******************/
#define RCC_BDCR_LSEON_Pos                 (0U)
#define RCC_BDCR_LSEON_Msk                 (0x1UL << RCC_BDCR_LSEON_Pos)        /*!< 0x00000001 */
#define RCC_BDCR_LSEON                     RCC_BDCR_LSEON_Msk
#define RCC_BDCR_LSERDY_Pos                (1U)
#define RCC_BDCR_LSERDY_Msk                (0x1UL << RCC_BDCR_LSERDY_Pos)       /*!< 0x00000002 */
#define RCC_BDCR_LSERDY                    RCC_BDCR_LSERDY_Msk
#define RCC_BDCR_LSEBYP_Pos                (2U)
#define RCC_BDCR_LSEBYP_Msk                (0x1UL << RCC_BDCR_LSEBYP_Pos)       /*!< 0x00000004 */
#define RCC_BDCR_LSEBYP                    RCC_BDCR_LSEBYP_Msk

#define RCC_BDCR_RTCSEL_Pos                (8U)
#define RCC_BDCR_RTCSEL_Msk                (0x3UL << RCC_BDCR_RTCSEL_Pos)       /*!< 0x00000300 */
#define RCC_BDCR_RTCSEL                    RCC_BDCR_RTCSEL_Msk
#define RCC_BDCR_RTCSEL_0                  (0x1UL << RCC_BDCR_RTCSEL_Pos)       /*!< 0x00000100 */
#define RCC_BDCR_RTCSEL_1                  (0x2UL << RCC_BDCR_RTCSEL_Pos)       /*!< 0x00000200 */

#define RCC_BDCR_RTCEN_Pos                 (15U)
#define RCC_BDCR_RTCEN_Msk                 (0x1UL << RCC_BDCR_RTCEN_Pos)        /*!< 0x00008000 */
#define RCC_BDCR_RTCEN                     RCC_BDCR_RTCEN_Msk
#define RCC_BDCR_BDRST_Pos                 (16U)
#define RCC_BDCR_BDRST_Msk                 (0x1UL << RCC_BDCR_BDRST_Pos)        /*!< 0x00010000 */
#define RCC_BDCR_BDRST                     RCC_BDCR_BDRST_Msk

/********************  Bit definition for RCC_CSR register  *******************/
#define RCC_CSR_LSION_Pos                  (0U)
#define RCC_CSR_LSION_Msk                  (0x1UL << RCC_CSR_LSION_Pos)         /*!< 0x00000001 */
#define RCC_CSR_LSION                      RCC_CSR_LSION_Msk
#define RCC_CSR_LSIRDY_Pos                 (1U)
#define RCC_CSR_LSIRDY_Msk                 (0x1UL << RCC_CSR_LSIRDY_Pos)        /*!< 0x00000002 */
#define RCC_CSR_LSIRDY                     RCC_CSR_LSIRDY_Msk
#define RCC_CSR_RMVF_Pos                   (24U)
#define RCC_CSR_RMVF_Msk                   (0x1UL << RCC_CSR_RMVF_Pos)          /*!< 0x01000000 */
#define RCC_CSR_RMVF                       RCC_CSR_RMVF_Msk
#define RCC_CSR_BORRSTF_Pos                (25U)
#define RCC_CSR_BORRSTF_Msk                (0x1UL << RCC_CSR_BORRSTF_Pos)       /*!< 0x02000000 */
#define RCC_CSR_BORRSTF                    RCC_CSR_BORRSTF_Msk
#define RCC_CSR_PINRSTF_Pos                (26U)
#define RCC_CSR_PINRSTF_Msk                (0x1UL << RCC_CSR_PINRSTF_Pos)       /*!< 0x04000000 */
#define RCC_CSR_PINRSTF                    RCC_CSR_PINRSTF_Msk
#define RCC_CSR_PORRSTF_Pos                (27U)
#define RCC_CSR_PORRSTF_Msk                (0x1UL << RCC_CSR_PORRSTF_Pos)       /*!< 0x08000000 */
#define RCC_CSR_PORRSTF                    RCC_CSR_PORRSTF_Msk
#define RCC_CSR_SFTRSTF_Pos                (28U)
#define RCC_CSR_SFTRSTF_Msk                (0x1UL << RCC_CSR_SFTRSTF_Pos)       /*!< 0x10000000 */
#define RCC_CSR_SFTRSTF                    RCC_CSR_SFTRSTF_Msk
#define RCC_CSR_IWDGRSTF_Pos               (29U)
#define RCC_CSR_IWDGRSTF_Msk               (0x1UL << RCC_CSR_IWDGRSTF_Pos)      /*!< 0x20000000 */
#define RCC_CSR_IWDGRSTF                   RCC_CSR_IWDGRSTF_Msk
#define RCC_CSR_WWDGRSTF_Pos               (30U)
#define RCC_CSR_WWDGRSTF_Msk               (0x1UL << RCC_CSR_WWDGRSTF_Pos)      /*!< 0x40000000 */
#define RCC_CSR_WWDGRSTF                   RCC_CSR_WWDGRSTF_Msk
#define RCC_CSR_LPWRRSTF_Pos               (31U)
#define RCC_CSR_LPWRRSTF_Msk               (0x1UL << RCC_CSR_LPWRRSTF_Pos)      /*!< 0x80000000 */
#define RCC_CSR_LPWRRSTF                   RCC_CSR_LPWRRSTF_Msk
/* Legacy defines */
#define RCC_CSR_PADRSTF                    RCC_CSR_PINRSTF
#define RCC_CSR_WDGRSTF                    RCC_CSR_IWDGRSTF

/********************  Bit definition for RCC_SSCGR register  *****************/
#define RCC_SSCGR_MODPER_Pos               (0U)
#define RCC_SSCGR_MODPER_Msk               (0x1FFFUL << RCC_SSCGR_MODPER_Pos)   /*!< 0x00001FFF */
#define RCC_SSCGR_MODPER                   RCC_SSCGR_MODPER_Msk
#define RCC_SSCGR_INCSTEP_Pos              (13U)
#define RCC_SSCGR_INCSTEP_Msk              (0x7FFFUL << RCC_SSCGR_INCSTEP_Pos)  /*!< 0x0FFFE000 */
#define RCC_SSCGR_INCSTEP                  RCC_SSCGR_INCSTEP_Msk
#define RCC_SSCGR_SPREADSEL_Pos            (30U)
#define RCC_SSCGR_SPREADSEL_Msk            (0x1UL << RCC_SSCGR_SPREADSEL_Pos)   /*!< 0x40000000 */
#define RCC_SSCGR_SPREADSEL                RCC_SSCGR_SPREADSEL_Msk
#define RCC_SSCGR_SSCGEN_Pos               (31U)
#define RCC_SSCGR_SSCGEN_Msk               (0x1UL << RCC_SSCGR_SSCGEN_Pos)      /*!< 0x80000000 */
#define RCC_SSCGR_SSCGEN                   RCC_SSCGR_SSCGEN_Msk

/********************  Bit definition for RCC_PLLI2SCFGR register  ************/
#define RCC_PLLI2SCFGR_PLLI2SN_Pos         (6U)
#define RCC_PLLI2SCFGR_PLLI2SN_Msk         (0x1FFUL << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00007FC0 */
#define RCC_PLLI2SCFGR_PLLI2SN             RCC_PLLI2SCFGR_PLLI2SN_Msk
#define RCC_PLLI2SCFGR_PLLI2SN_0           (0x001UL << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00000040 */
#define RCC_PLLI2SCFGR_PLLI2SN_1           (0x002UL << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00000080 */
#define RCC_PLLI2SCFGR_PLLI2SN_2           (0x004UL << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00000100 */
#define RCC_PLLI2SCFGR_PLLI2SN_3           (0x008UL << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00000200 */
#define RCC_PLLI2SCFGR_PLLI2SN_4           (0x010UL << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00000400 */
#define RCC_PLLI2SCFGR_PLLI2SN_5           (0x020UL << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00000800 */
#define RCC_PLLI2SCFGR_PLLI2SN_6           (0x040UL << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00001000 */
#define RCC_PLLI2SCFGR_PLLI2SN_7           (0x080UL << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00002000 */
#define RCC_PLLI2SCFGR_PLLI2SN_8           (0x100UL << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00004000 */

#define RCC_PLLI2SCFGR_PLLI2SR_Pos         (28U)
#define RCC_PLLI2SCFGR_PLLI2SR_Msk         (0x7UL << RCC_PLLI2SCFGR_PLLI2SR_Pos) /*!< 0x70000000 */
#define RCC_PLLI2SCFGR_PLLI2SR             RCC_PLLI2SCFGR_PLLI2SR_Msk
#define RCC_PLLI2SCFGR_PLLI2SR_0           (0x1UL << RCC_PLLI2SCFGR_PLLI2SR_Pos) /*!< 0x10000000 */
#define RCC_PLLI2SCFGR_PLLI2SR_1           (0x2UL << RCC_PLLI2SCFGR_PLLI2SR_Pos) /*!< 0x20000000 */
#define RCC_PLLI2SCFGR_PLLI2SR_2           (0x4UL << RCC_PLLI2SCFGR_PLLI2SR_Pos) /*!< 0x40000000 */



/******************************************************************************/
/*                                                                            */
/*                        Serial Peripheral Interface                         */
/*                                                                            */
/******************************************************************************/
#define SPI_I2S_FULLDUPLEX_SUPPORT                                             /*!< I2S Full-Duplex support */

/*******************  Bit definition for SPI_CR1 register  ********************/
#define SPI_CR1_CPHA_Pos            (0U)
#define SPI_CR1_CPHA_Msk            (0x1UL << SPI_CR1_CPHA_Pos)                 /*!< 0x00000001 */
#define SPI_CR1_CPHA                SPI_CR1_CPHA_Msk                           /*!<Clock Phase      */
#define SPI_CR1_CPOL_Pos            (1U)
#define SPI_CR1_CPOL_Msk            (0x1UL << SPI_CR1_CPOL_Pos)                 /*!< 0x00000002 */
#define SPI_CR1_CPOL                SPI_CR1_CPOL_Msk                           /*!<Clock Polarity   */
#define SPI_CR1_MSTR_Pos            (2U)
#define SPI_CR1_MSTR_Msk            (0x1UL << SPI_CR1_MSTR_Pos)                 /*!< 0x00000004 */
#define SPI_CR1_MSTR                SPI_CR1_MSTR_Msk                           /*!<Master Selection */

#define SPI_CR1_BR_Pos              (3U)
#define SPI_CR1_BR_Msk              (0x7UL << SPI_CR1_BR_Pos)                   /*!< 0x00000038 */
#define SPI_CR1_BR                  SPI_CR1_BR_Msk                             /*!<BR[2:0] bits (Baud Rate Control) */
#define SPI_CR1_BR_0                (0x1UL << SPI_CR1_BR_Pos)                   /*!< 0x00000008 */
#define SPI_CR1_BR_1                (0x2UL << SPI_CR1_BR_Pos)                   /*!< 0x00000010 */
#define SPI_CR1_BR_2                (0x4UL << SPI_CR1_BR_Pos)                   /*!< 0x00000020 */

#define SPI_CR1_SPE_Pos             (6U)
#define SPI_CR1_SPE_Msk             (0x1UL << SPI_CR1_SPE_Pos)                  /*!< 0x00000040 */
#define SPI_CR1_SPE                 SPI_CR1_SPE_Msk                            /*!<SPI Enable                          */
#define SPI_CR1_LSBFIRST_Pos        (7U)
#define SPI_CR1_LSBFIRST_Msk        (0x1UL << SPI_CR1_LSBFIRST_Pos)             /*!< 0x00000080 */
#define SPI_CR1_LSBFIRST            SPI_CR1_LSBFIRST_Msk                       /*!<Frame Format                        */
#define SPI_CR1_SSI_Pos             (8U)
#define SPI_CR1_SSI_Msk             (0x1UL << SPI_CR1_SSI_Pos)                  /*!< 0x00000100 */
#define SPI_CR1_SSI                 SPI_CR1_SSI_Msk                            /*!<Internal slave select               */
#define SPI_CR1_SSM_Pos             (9U)
#define SPI_CR1_SSM_Msk             (0x1UL << SPI_CR1_SSM_Pos)                  /*!< 0x00000200 */
#define SPI_CR1_SSM                 SPI_CR1_SSM_Msk                            /*!<Software slave management           */
#define SPI_CR1_RXONLY_Pos          (10U)
#define SPI_CR1_RXONLY_Msk          (0x1UL << SPI_CR1_RXONLY_Pos)               /*!< 0x00000400 */
#define SPI_CR1_RXONLY              SPI_CR1_RXONLY_Msk                         /*!<Receive only                        */
#define SPI_CR1_DFF_Pos             (11U)
#define SPI_CR1_DFF_Msk             (0x1UL << SPI_CR1_DFF_Pos)                  /*!< 0x00000800 */
#define SPI_CR1_DFF                 SPI_CR1_DFF_Msk                            /*!<Data Frame Format                   */
#define SPI_CR1_CRCNEXT_Pos         (12U)
#define SPI_CR1_CRCNEXT_Msk         (0x1UL << SPI_CR1_CRCNEXT_Pos)              /*!< 0x00001000 */
#define SPI_CR1_CRCNEXT             SPI_CR1_CRCNEXT_Msk                        /*!<Transmit CRC next                   */
#define SPI_CR1_CRCEN_Pos           (13U)
#define SPI_CR1_CRCEN_Msk           (0x1UL << SPI_CR1_CRCEN_Pos)                /*!< 0x00002000 */
#define SPI_CR1_CRCEN               SPI_CR1_CRCEN_Msk                          /*!<Hardware CRC calculation enable     */
#define SPI_CR1_BIDIOE_Pos          (14U)
#define SPI_CR1_BIDIOE_Msk          (0x1UL << SPI_CR1_BIDIOE_Pos)               /*!< 0x00004000 */
#define SPI_CR1_BIDIOE              SPI_CR1_BIDIOE_Msk                         /*!<Output enable in bidirectional mode */
#define SPI_CR1_BIDIMODE_Pos        (15U)
#define SPI_CR1_BIDIMODE_Msk        (0x1UL << SPI_CR1_BIDIMODE_Pos)             /*!< 0x00008000 */
#define SPI_CR1_BIDIMODE            SPI_CR1_BIDIMODE_Msk                       /*!<Bidirectional data mode enable      */

/*******************  Bit definition for SPI_CR2 register  ********************/
#define SPI_CR2_RXDMAEN_Pos         (0U)
#define SPI_CR2_RXDMAEN_Msk         (0x1UL << SPI_CR2_RXDMAEN_Pos)              /*!< 0x00000001 */
#define SPI_CR2_RXDMAEN             SPI_CR2_RXDMAEN_Msk                        /*!<Rx Buffer DMA Enable                 */
#define SPI_CR2_TXDMAEN_Pos         (1U)
#define SPI_CR2_TXDMAEN_Msk         (0x1UL << SPI_CR2_TXDMAEN_Pos)              /*!< 0x00000002 */
#define SPI_CR2_TXDMAEN             SPI_CR2_TXDMAEN_Msk                        /*!<Tx Buffer DMA Enable                 */
#define SPI_CR2_SSOE_Pos            (2U)
#define SPI_CR2_SSOE_Msk            (0x1UL << SPI_CR2_SSOE_Pos)                 /*!< 0x00000004 */
#define SPI_CR2_SSOE                SPI_CR2_SSOE_Msk                           /*!<SS Output Enable                     */
#define SPI_CR2_FRF_Pos             (4U)
#define SPI_CR2_FRF_Msk             (0x1UL << SPI_CR2_FRF_Pos)                  /*!< 0x00000010 */
#define SPI_CR2_FRF                 SPI_CR2_FRF_Msk                            /*!<Frame Format                         */
#define SPI_CR2_ERRIE_Pos           (5U)
#define SPI_CR2_ERRIE_Msk           (0x1UL << SPI_CR2_ERRIE_Pos)                /*!< 0x00000020 */
#define SPI_CR2_ERRIE               SPI_CR2_ERRIE_Msk                          /*!<Error Interrupt Enable               */
#define SPI_CR2_RXNEIE_Pos          (6U)
#define SPI_CR2_RXNEIE_Msk          (0x1UL << SPI_CR2_RXNEIE_Pos)               /*!< 0x00000040 */
#define SPI_CR2_RXNEIE              SPI_CR2_RXNEIE_Msk                         /*!<RX buffer Not Empty Interrupt Enable */
#define SPI_CR2_TXEIE_Pos           (7U)
#define SPI_CR2_TXEIE_Msk           (0x1UL << SPI_CR2_TXEIE_Pos)                /*!< 0x00000080 */
#define SPI_CR2_TXEIE               SPI_CR2_TXEIE_Msk                          /*!<Tx buffer Empty Interrupt Enable     */

/********************  Bit definition for SPI_SR register  ********************/
#define SPI_SR_RXNE_Pos             (0U)
#define SPI_SR_RXNE_Msk             (0x1UL << SPI_SR_RXNE_Pos)                  /*!< 0x00000001 */
#define SPI_SR_RXNE                 SPI_SR_RXNE_Msk                            /*!<Receive buffer Not Empty */
#define SPI_SR_TXE_Pos              (1U)
#define SPI_SR_TXE_Msk              (0x1UL << SPI_SR_TXE_Pos)                   /*!< 0x00000002 */
#define SPI_SR_TXE                  SPI_SR_TXE_Msk                             /*!<Transmit buffer Empty    */
#define SPI_SR_CHSIDE_Pos           (2U)
#define SPI_SR_CHSIDE_Msk           (0x1UL << SPI_SR_CHSIDE_Pos)                /*!< 0x00000004 */
#define SPI_SR_CHSIDE               SPI_SR_CHSIDE_Msk                          /*!<Channel side             */
#define SPI_SR_UDR_Pos              (3U)
#define SPI_SR_UDR_Msk              (0x1UL << SPI_SR_UDR_Pos)                   /*!< 0x00000008 */
#define SPI_SR_UDR                  SPI_SR_UDR_Msk                             /*!<Underrun flag            */
#define SPI_SR_CRCERR_Pos           (4U)
#define SPI_SR_CRCERR_Msk           (0x1UL << SPI_SR_CRCERR_Pos)                /*!< 0x00000010 */
#define SPI_SR_CRCERR               SPI_SR_CRCERR_Msk                          /*!<CRC Error flag           */
#define SPI_SR_MODF_Pos             (5U)
#define SPI_SR_MODF_Msk             (0x1UL << SPI_SR_MODF_Pos)                  /*!< 0x00000020 */
#define SPI_SR_MODF                 SPI_SR_MODF_Msk                            /*!<Mode fault               */
#define SPI_SR_OVR_Pos              (6U)
#define SPI_SR_OVR_Msk              (0x1UL << SPI_SR_OVR_Pos)                   /*!< 0x00000040 */
#define SPI_SR_OVR                  SPI_SR_OVR_Msk                             /*!<Overrun flag             */
#define SPI_SR_BSY_Pos              (7U)
#define SPI_SR_BSY_Msk              (0x1UL << SPI_SR_BSY_Pos)                   /*!< 0x00000080 */
#define SPI_SR_BSY                  SPI_SR_BSY_Msk                             /*!<Busy flag                */
#define SPI_SR_FRE_Pos              (8U)
#define SPI_SR_FRE_Msk              (0x1UL << SPI_SR_FRE_Pos)                   /*!< 0x00000100 */
#define SPI_SR_FRE                  SPI_SR_FRE_Msk                             /*!<Frame format error flag  */

/********************  Bit definition for SPI_DR register  ********************/
#define SPI_DR_DR_Pos               (0U)
#define SPI_DR_DR_Msk               (0xFFFFUL << SPI_DR_DR_Pos)                 /*!< 0x0000FFFF */
#define SPI_DR_DR                   SPI_DR_DR_Msk                              /*!<Data Register           */

/*******************  Bit definition for SPI_CRCPR register  ******************/
#define SPI_CRCPR_CRCPOLY_Pos       (0U)
#define SPI_CRCPR_CRCPOLY_Msk       (0xFFFFUL << SPI_CRCPR_CRCPOLY_Pos)         /*!< 0x0000FFFF */
#define SPI_CRCPR_CRCPOLY           SPI_CRCPR_CRCPOLY_Msk                      /*!<CRC polynomial register */

/******************  Bit definition for SPI_RXCRCR register  ******************/
#define SPI_RXCRCR_RXCRC_Pos        (0U)
#define SPI_RXCRCR_RXCRC_Msk        (0xFFFFUL << SPI_RXCRCR_RXCRC_Pos)          /*!< 0x0000FFFF */
#define SPI_RXCRCR_RXCRC            SPI_RXCRCR_RXCRC_Msk                       /*!<Rx CRC Register         */

/******************  Bit definition for SPI_TXCRCR register  ******************/
#define SPI_TXCRCR_TXCRC_Pos        (0U)
#define SPI_TXCRCR_TXCRC_Msk        (0xFFFFUL << SPI_TXCRCR_TXCRC_Pos)          /*!< 0x0000FFFF */
#define SPI_TXCRCR_TXCRC            SPI_TXCRCR_TXCRC_Msk                       /*!<Tx CRC Register         */

/******************  Bit definition for SPI_I2SCFGR register  *****************/
#define SPI_I2SCFGR_CHLEN_Pos       (0U)
#define SPI_I2SCFGR_CHLEN_Msk       (0x1UL << SPI_I2SCFGR_CHLEN_Pos)            /*!< 0x00000001 */
#define SPI_I2SCFGR_CHLEN           SPI_I2SCFGR_CHLEN_Msk                      /*!<Channel length (number of bits per audio channel) */

#define SPI_I2SCFGR_DATLEN_Pos      (1U)
#define SPI_I2SCFGR_DATLEN_Msk      (0x3UL << SPI_I2SCFGR_DATLEN_Pos)           /*!< 0x00000006 */
#define SPI_I2SCFGR_DATLEN          SPI_I2SCFGR_DATLEN_Msk                     /*!<DATLEN[1:0] bits (Data length to be transferred)  */
#define SPI_I2SCFGR_DATLEN_0        (0x1UL << SPI_I2SCFGR_DATLEN_Pos)           /*!< 0x00000002 */
#define SPI_I2SCFGR_DATLEN_1        (0x2UL << SPI_I2SCFGR_DATLEN_Pos)           /*!< 0x00000004 */

#define SPI_I2SCFGR_CKPOL_Pos       (3U)
#define SPI_I2SCFGR_CKPOL_Msk       (0x1UL << SPI_I2SCFGR_CKPOL_Pos)            /*!< 0x00000008 */
#define SPI_I2SCFGR_CKPOL           SPI_I2SCFGR_CKPOL_Msk                      /*!<steady state clock polarity               */

#define SPI_I2SCFGR_I2SSTD_Pos      (4U)
#define SPI_I2SCFGR_I2SSTD_Msk      (0x3UL << SPI_I2SCFGR_I2SSTD_Pos)           /*!< 0x00000030 */
#define SPI_I2SCFGR_I2SSTD          SPI_I2SCFGR_I2SSTD_Msk                     /*!<I2SSTD[1:0] bits (I2S standard selection) */
#define SPI_I2SCFGR_I2SSTD_0        (0x1UL << SPI_I2SCFGR_I2SSTD_Pos)           /*!< 0x00000010 */
#define SPI_I2SCFGR_I2SSTD_1        (0x2UL << SPI_I2SCFGR_I2SSTD_Pos)           /*!< 0x00000020 */

#define SPI_I2SCFGR_PCMSYNC_Pos     (7U)
#define SPI_I2SCFGR_PCMSYNC_Msk     (0x1UL << SPI_I2SCFGR_PCMSYNC_Pos)          /*!< 0x00000080 */
#define SPI_I2SCFGR_PCMSYNC         SPI_I2SCFGR_PCMSYNC_Msk                    /*!<PCM frame synchronization                 */

#define SPI_I2SCFGR_I2SCFG_Pos      (8U)
#define SPI_I2SCFGR_I2SCFG_Msk      (0x3UL << SPI_I2SCFGR_I2SCFG_Pos)           /*!< 0x00000300 */
#define SPI_I2SCFGR_I2SCFG          SPI_I2SCFGR_I2SCFG_Msk                     /*!<I2SCFG[1:0] bits (I2S configuration mode) */
#define SPI_I2SCFGR_I2SCFG_0        (0x1UL << SPI_I2SCFGR_I2SCFG_Pos)           /*!< 0x00000100 */
#define SPI_I2SCFGR_I2SCFG_1        (0x2UL << SPI_I2SCFGR_I2SCFG_Pos)           /*!< 0x00000200 */

#define SPI_I2SCFGR_I2SE_Pos        (10U)
#define SPI_I2SCFGR_I2SE_Msk        (0x1UL << SPI_I2SCFGR_I2SE_Pos)             /*!< 0x00000400 */
#define SPI_I2SCFGR_I2SE            SPI_I2SCFGR_I2SE_Msk                       /*!<I2S Enable         */
#define SPI_I2SCFGR_I2SMOD_Pos      (11U)
#define SPI_I2SCFGR_I2SMOD_Msk      (0x1UL << SPI_I2SCFGR_I2SMOD_Pos)           /*!< 0x00000800 */
#define SPI_I2SCFGR_I2SMOD          SPI_I2SCFGR_I2SMOD_Msk                     /*!<I2S mode selection */

/******************  Bit definition for SPI_I2SPR register  *******************/
#define SPI_I2SPR_I2SDIV_Pos        (0U)
#define SPI_I2SPR_I2SDIV_Msk        (0xFFUL << SPI_I2SPR_I2SDIV_Pos)            /*!< 0x000000FF */
#define SPI_I2SPR_I2SDIV            SPI_I2SPR_I2SDIV_Msk                       /*!<I2S Linear prescaler         */
#define SPI_I2SPR_ODD_Pos           (8U)
#define SPI_I2SPR_ODD_Msk           (0x1UL << SPI_I2SPR_ODD_Pos)                /*!< 0x00000100 */
#define SPI_I2SPR_ODD               SPI_I2SPR_ODD_Msk                          /*!<Odd factor for the prescaler */
#define SPI_I2SPR_MCKOE_Pos         (9U)
#define SPI_I2SPR_MCKOE_Msk         (0x1UL << SPI_I2SPR_MCKOE_Pos)              /*!< 0x00000200 */
#define SPI_I2SPR_MCKOE             SPI_I2SPR_MCKOE_Msk                        /*!<Master Clock Output Enable   */

/******************************************************************************/
/*                                                                            */
/*                                 SYSCFG                                     */
/*                                                                            */
/******************************************************************************/
/******************  Bit definition for SYSCFG_MEMRMP register  ***************/
#define SYSCFG_MEMRMP_MEM_MODE_Pos           (0U)
#define SYSCFG_MEMRMP_MEM_MODE_Msk           (0x3UL << SYSCFG_MEMRMP_MEM_MODE_Pos) /*!< 0x00000003 */
#define SYSCFG_MEMRMP_MEM_MODE               SYSCFG_MEMRMP_MEM_MODE_Msk        /*!< SYSCFG_Memory Remap Config */
#define SYSCFG_MEMRMP_MEM_MODE_0             (0x1UL << SYSCFG_MEMRMP_MEM_MODE_Pos) /*!< 0x00000001 */
#define SYSCFG_MEMRMP_MEM_MODE_1             (0x2UL << SYSCFG_MEMRMP_MEM_MODE_Pos) /*!< 0x00000002 */
/******************  Bit definition for SYSCFG_PMC register  ******************/
#define SYSCFG_PMC_MII_RMII_SEL_Pos          (23U)
#define SYSCFG_PMC_MII_RMII_SEL_Msk          (0x1UL << SYSCFG_PMC_MII_RMII_SEL_Pos) /*!< 0x00800000 */
#define SYSCFG_PMC_MII_RMII_SEL              SYSCFG_PMC_MII_RMII_SEL_Msk       /*!<Ethernet PHY interface selection */
/* Old MII_RMII_SEL bit definition, maintained for legacy purpose */
#define SYSCFG_PMC_MII_RMII             SYSCFG_PMC_MII_RMII_SEL

/*****************  Bit definition for SYSCFG_EXTICR1 register  ***************/
#define SYSCFG_EXTICR1_EXTI0_Pos             (0U)
#define SYSCFG_EXTICR1_EXTI0_Msk             (0xFUL << SYSCFG_EXTICR1_EXTI0_Pos) /*!< 0x0000000F */
#define SYSCFG_EXTICR1_EXTI0                 SYSCFG_EXTICR1_EXTI0_Msk          /*!<EXTI 0 configuration */
#define SYSCFG_EXTICR1_EXTI1_Pos             (4U)
#define SYSCFG_EXTICR1_EXTI1_Msk             (0xFUL << SYSCFG_EXTICR1_EXTI1_Pos) /*!< 0x000000F0 */
#define SYSCFG_EXTICR1_EXTI1                 SYSCFG_EXTICR1_EXTI1_Msk          /*!<EXTI 1 configuration */
#define SYSCFG_EXTICR1_EXTI2_Pos             (8U)
#define SYSCFG_EXTICR1_EXTI2_Msk             (0xFUL << SYSCFG_EXTICR1_EXTI2_Pos) /*!< 0x00000F00 */
#define SYSCFG_EXTICR1_EXTI2                 SYSCFG_EXTICR1_EXTI2_Msk          /*!<EXTI 2 configuration */
#define SYSCFG_EXTICR1_EXTI3_Pos             (12U)
#define SYSCFG_EXTICR1_EXTI3_Msk             (0xFUL << SYSCFG_EXTICR1_EXTI3_Pos) /*!< 0x0000F000 */
#define SYSCFG_EXTICR1_EXTI3                 SYSCFG_EXTICR1_EXTI3_Msk          /*!<EXTI 3 configuration */
/**
  * @brief   EXTI0 configuration
  */
#define SYSCFG_EXTICR1_EXTI0_PA              0x0000U                           /*!<PA[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PB              0x0001U                           /*!<PB[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PC              0x0002U                           /*!<PC[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PD              0x0003U                           /*!<PD[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PE              0x0004U                           /*!<PE[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PF              0x0005U                           /*!<PF[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PG              0x0006U                           /*!<PG[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PH              0x0007U                           /*!<PH[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PI              0x0008U                           /*!<PI[0] pin */

/**
  * @brief   EXTI1 configuration
  */
#define SYSCFG_EXTICR1_EXTI1_PA              0x0000U                           /*!<PA[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PB              0x0010U                           /*!<PB[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PC              0x0020U                           /*!<PC[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PD              0x0030U                           /*!<PD[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PE              0x0040U                           /*!<PE[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PF              0x0050U                           /*!<PF[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PG              0x0060U                           /*!<PG[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PH              0x0070U                           /*!<PH[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PI              0x0080U                           /*!<PI[1] pin */

/**
  * @brief   EXTI2 configuration
  */
#define SYSCFG_EXTICR1_EXTI2_PA              0x0000U                           /*!<PA[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PB              0x0100U                           /*!<PB[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PC              0x0200U                           /*!<PC[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PD              0x0300U                           /*!<PD[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PE              0x0400U                           /*!<PE[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PF              0x0500U                           /*!<PF[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PG              0x0600U                           /*!<PG[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PH              0x0700U                           /*!<PH[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PI              0x0800U                           /*!<PI[2] pin */

/**
  * @brief   EXTI3 configuration
  */
#define SYSCFG_EXTICR1_EXTI3_PA              0x0000U                           /*!<PA[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PB              0x1000U                           /*!<PB[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PC              0x2000U                           /*!<PC[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PD              0x3000U                           /*!<PD[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PE              0x4000U                           /*!<PE[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PF              0x5000U                           /*!<PF[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PG              0x6000U                           /*!<PG[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PH              0x7000U                           /*!<PH[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PI              0x8000U                           /*!<PI[3] pin */

/*****************  Bit definition for SYSCFG_EXTICR2 register  ***************/
#define SYSCFG_EXTICR2_EXTI4_Pos             (0U)
#define SYSCFG_EXTICR2_EXTI4_Msk             (0xFUL << SYSCFG_EXTICR2_EXTI4_Pos) /*!< 0x0000000F */
#define SYSCFG_EXTICR2_EXTI4                 SYSCFG_EXTICR2_EXTI4_Msk          /*!<EXTI 4 configuration */
#define SYSCFG_EXTICR2_EXTI5_Pos             (4U)
#define SYSCFG_EXTICR2_EXTI5_Msk             (0xFUL << SYSCFG_EXTICR2_EXTI5_Pos) /*!< 0x000000F0 */
#define SYSCFG_EXTICR2_EXTI5                 SYSCFG_EXTICR2_EXTI5_Msk          /*!<EXTI 5 configuration */
#define SYSCFG_EXTICR2_EXTI6_Pos             (8U)
#define SYSCFG_EXTICR2_EXTI6_Msk             (0xFUL << SYSCFG_EXTICR2_EXTI6_Pos) /*!< 0x00000F00 */
#define SYSCFG_EXTICR2_EXTI6                 SYSCFG_EXTICR2_EXTI6_Msk          /*!<EXTI 6 configuration */
#define SYSCFG_EXTICR2_EXTI7_Pos             (12U)
#define SYSCFG_EXTICR2_EXTI7_Msk             (0xFUL << SYSCFG_EXTICR2_EXTI7_Pos) /*!< 0x0000F000 */
#define SYSCFG_EXTICR2_EXTI7                 SYSCFG_EXTICR2_EXTI7_Msk          /*!<EXTI 7 configuration */

/**
  * @brief   EXTI4 configuration
  */
#define SYSCFG_EXTICR2_EXTI4_PA              0x0000U                           /*!<PA[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PB              0x0001U                           /*!<PB[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PC              0x0002U                           /*!<PC[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PD              0x0003U                           /*!<PD[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PE              0x0004U                           /*!<PE[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PF              0x0005U                           /*!<PF[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PG              0x0006U                           /*!<PG[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PH              0x0007U                           /*!<PH[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PI              0x0008U                           /*!<PI[4] pin */

/**
  * @brief   EXTI5 configuration
  */
#define SYSCFG_EXTICR2_EXTI5_PA              0x0000U                           /*!<PA[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PB              0x0010U                           /*!<PB[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PC              0x0020U                           /*!<PC[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PD              0x0030U                           /*!<PD[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PE              0x0040U                           /*!<PE[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PF              0x0050U                           /*!<PF[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PG              0x0060U                           /*!<PG[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PH              0x0070U                           /*!<PH[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PI              0x0080U                           /*!<PI[5] pin */

/**
  * @brief   EXTI6 configuration
  */
#define SYSCFG_EXTICR2_EXTI6_PA              0x0000U                           /*!<PA[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PB              0x0100U                           /*!<PB[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PC              0x0200U                           /*!<PC[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PD              0x0300U                           /*!<PD[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PE              0x0400U                           /*!<PE[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PF              0x0500U                           /*!<PF[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PG              0x0600U                           /*!<PG[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PH              0x0700U                           /*!<PH[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PI              0x0800U                           /*!<PI[6] pin */

/**
  * @brief   EXTI7 configuration
  */
#define SYSCFG_EXTICR2_EXTI7_PA              0x0000U                           /*!<PA[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PB              0x1000U                           /*!<PB[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PC              0x2000U                           /*!<PC[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PD              0x3000U                           /*!<PD[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PE              0x4000U                           /*!<PE[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PF              0x5000U                           /*!<PF[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PG              0x6000U                           /*!<PG[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PH              0x7000U                           /*!<PH[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PI              0x8000U                           /*!<PI[7] pin */

/*****************  Bit definition for SYSCFG_EXTICR3 register  ***************/
#define SYSCFG_EXTICR3_EXTI8_Pos             (0U)
#define SYSCFG_EXTICR3_EXTI8_Msk             (0xFUL << SYSCFG_EXTICR3_EXTI8_Pos) /*!< 0x0000000F */
#define SYSCFG_EXTICR3_EXTI8                 SYSCFG_EXTICR3_EXTI8_Msk          /*!<EXTI 8 configuration */
#define SYSCFG_EXTICR3_EXTI9_Pos             (4U)
#define SYSCFG_EXTICR3_EXTI9_Msk             (0xFUL << SYSCFG_EXTICR3_EXTI9_Pos) /*!< 0x000000F0 */
#define SYSCFG_EXTICR3_EXTI9                 SYSCFG_EXTICR3_EXTI9_Msk          /*!<EXTI 9 configuration */
#define SYSCFG_EXTICR3_EXTI10_Pos            (8U)
#define SYSCFG_EXTICR3_EXTI10_Msk            (0xFUL << SYSCFG_EXTICR3_EXTI10_Pos) /*!< 0x00000F00 */
#define SYSCFG_EXTICR3_EXTI10                SYSCFG_EXTICR3_EXTI10_Msk         /*!<EXTI 10 configuration */
#define SYSCFG_EXTICR3_EXTI11_Pos            (12U)
#define SYSCFG_EXTICR3_EXTI11_Msk            (0xFUL << SYSCFG_EXTICR3_EXTI11_Pos) /*!< 0x0000F000 */
#define SYSCFG_EXTICR3_EXTI11                SYSCFG_EXTICR3_EXTI11_Msk         /*!<EXTI 11 configuration */

/**
  * @brief   EXTI8 configuration
  */
#define SYSCFG_EXTICR3_EXTI8_PA              0x0000U                           /*!<PA[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PB              0x0001U                           /*!<PB[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PC              0x0002U                           /*!<PC[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PD              0x0003U                           /*!<PD[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PE              0x0004U                           /*!<PE[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PF              0x0005U                           /*!<PF[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PG              0x0006U                           /*!<PG[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PH              0x0007U                           /*!<PH[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PI              0x0008U                           /*!<PI[8] pin */

/**
  * @brief   EXTI9 configuration
  */
#define SYSCFG_EXTICR3_EXTI9_PA              0x0000U                           /*!<PA[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PB              0x0010U                           /*!<PB[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PC              0x0020U                           /*!<PC[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PD              0x0030U                           /*!<PD[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PE              0x0040U                           /*!<PE[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PF              0x0050U                           /*!<PF[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PG              0x0060U                           /*!<PG[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PH              0x0070U                           /*!<PH[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PI              0x0080U                           /*!<PI[9] pin */

/**
  * @brief   EXTI10 configuration
  */
#define SYSCFG_EXTICR3_EXTI10_PA             0x0000U                           /*!<PA[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PB             0x0100U                           /*!<PB[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PC             0x0200U                           /*!<PC[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PD             0x0300U                           /*!<PD[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PE             0x0400U                           /*!<PE[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PF             0x0500U                           /*!<PF[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PG             0x0600U                           /*!<PG[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PH             0x0700U                           /*!<PH[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PI             0x0800U                           /*!<PI[10] pin */

/**
  * @brief   EXTI11 configuration
  */
#define SYSCFG_EXTICR3_EXTI11_PA             0x0000U                           /*!<PA[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PB             0x1000U                           /*!<PB[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PC             0x2000U                           /*!<PC[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PD             0x3000U                           /*!<PD[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PE             0x4000U                           /*!<PE[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PF             0x5000U                           /*!<PF[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PG             0x6000U                           /*!<PG[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PH             0x7000U                           /*!<PH[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PI             0x8000U                           /*!<PI[11] pin */

/*****************  Bit definition for SYSCFG_EXTICR4 register  ***************/
#define SYSCFG_EXTICR4_EXTI12_Pos            (0U)
#define SYSCFG_EXTICR4_EXTI12_Msk            (0xFUL << SYSCFG_EXTICR4_EXTI12_Pos) /*!< 0x0000000F */
#define SYSCFG_EXTICR4_EXTI12                SYSCFG_EXTICR4_EXTI12_Msk         /*!<EXTI 12 configuration */
#define SYSCFG_EXTICR4_EXTI13_Pos            (4U)
#define SYSCFG_EXTICR4_EXTI13_Msk            (0xFUL << SYSCFG_EXTICR4_EXTI13_Pos) /*!< 0x000000F0 */
#define SYSCFG_EXTICR4_EXTI13                SYSCFG_EXTICR4_EXTI13_Msk         /*!<EXTI 13 configuration */
#define SYSCFG_EXTICR4_EXTI14_Pos            (8U)
#define SYSCFG_EXTICR4_EXTI14_Msk            (0xFUL << SYSCFG_EXTICR4_EXTI14_Pos) /*!< 0x00000F00 */
#define SYSCFG_EXTICR4_EXTI14                SYSCFG_EXTICR4_EXTI14_Msk         /*!<EXTI 14 configuration */
#define SYSCFG_EXTICR4_EXTI15_Pos            (12U)
#define SYSCFG_EXTICR4_EXTI15_Msk            (0xFUL << SYSCFG_EXTICR4_EXTI15_Pos) /*!< 0x0000F000 */
#define SYSCFG_EXTICR4_EXTI15                SYSCFG_EXTICR4_EXTI15_Msk         /*!<EXTI 15 configuration */

/**
  * @brief   EXTI12 configuration
  */
#define SYSCFG_EXTICR4_EXTI12_PA             0x0000U                           /*!<PA[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PB             0x0001U                           /*!<PB[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PC             0x0002U                           /*!<PC[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PD             0x0003U                           /*!<PD[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PE             0x0004U                           /*!<PE[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PF             0x0005U                           /*!<PF[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PG             0x0006U                           /*!<PG[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PH             0x0007U                           /*!<PH[12] pin */

/**
  * @brief   EXTI13 configuration
  */
#define SYSCFG_EXTICR4_EXTI13_PA             0x0000U                           /*!<PA[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PB             0x0010U                           /*!<PB[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PC             0x0020U                           /*!<PC[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PD             0x0030U                           /*!<PD[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PE             0x0040U                           /*!<PE[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PF             0x0050U                           /*!<PF[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PG             0x0060U                           /*!<PG[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PH             0x0070U                           /*!<PH[13] pin */

/**
  * @brief   EXTI14 configuration
  */
#define SYSCFG_EXTICR4_EXTI14_PA             0x0000U                           /*!<PA[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PB             0x0100U                           /*!<PB[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PC             0x0200U                           /*!<PC[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PD             0x0300U                           /*!<PD[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PE             0x0400U                           /*!<PE[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PF             0x0500U                           /*!<PF[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PG             0x0600U                           /*!<PG[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PH             0x0700U                           /*!<PH[14] pin */

/**
  * @brief   EXTI15 configuration
  */
#define SYSCFG_EXTICR4_EXTI15_PA             0x0000U                           /*!<PA[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PB             0x1000U                           /*!<PB[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PC             0x2000U                           /*!<PC[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PD             0x3000U                           /*!<PD[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PE             0x4000U                           /*!<PE[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PF             0x5000U                           /*!<PF[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PG             0x6000U                           /*!<PG[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PH             0x7000U                           /*!<PH[15] pin */

/******************  Bit definition for SYSCFG_CMPCR register  ****************/
#define SYSCFG_CMPCR_CMP_PD_Pos              (0U)
#define SYSCFG_CMPCR_CMP_PD_Msk              (0x1UL << SYSCFG_CMPCR_CMP_PD_Pos) /*!< 0x00000001 */
#define SYSCFG_CMPCR_CMP_PD                  SYSCFG_CMPCR_CMP_PD_Msk           /*!<Compensation cell ready flag */
#define SYSCFG_CMPCR_READY_Pos               (8U)
#define SYSCFG_CMPCR_READY_Msk               (0x1UL << SYSCFG_CMPCR_READY_Pos)  /*!< 0x00000100 */
#define SYSCFG_CMPCR_READY                   SYSCFG_CMPCR_READY_Msk            /*!<Compensation cell power-down */

/******************************************************************************/
/*                                                                            */
/*                                    TIM                                     */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for TIM_CR1 register  ********************/
#define TIM_CR1_CEN_Pos           (0U)
#define TIM_CR1_CEN_Msk           (0x1UL << TIM_CR1_CEN_Pos)                    /*!< 0x00000001 */
#define TIM_CR1_CEN               TIM_CR1_CEN_Msk                              /*!<Counter enable        */
#define TIM_CR1_UDIS_Pos          (1U)
#define TIM_CR1_UDIS_Msk          (0x1UL << TIM_CR1_UDIS_Pos)                   /*!< 0x00000002 */
#define TIM_CR1_UDIS              TIM_CR1_UDIS_Msk                             /*!<Update disable        */
#define TIM_CR1_URS_Pos           (2U)
#define TIM_CR1_URS_Msk           (0x1UL << TIM_CR1_URS_Pos)                    /*!< 0x00000004 */
#define TIM_CR1_URS               TIM_CR1_URS_Msk                              /*!<Update request source */
#define TIM_CR1_OPM_Pos           (3U)
#define TIM_CR1_OPM_Msk           (0x1UL << TIM_CR1_OPM_Pos)                    /*!< 0x00000008 */
#define TIM_CR1_OPM               TIM_CR1_OPM_Msk                              /*!<One pulse mode        */
#define TIM_CR1_DIR_Pos           (4U)
#define TIM_CR1_DIR_Msk           (0x1UL << TIM_CR1_DIR_Pos)                    /*!< 0x00000010 */
#define TIM_CR1_DIR               TIM_CR1_DIR_Msk                              /*!<Direction             */

#define TIM_CR1_CMS_Pos           (5U)
#define TIM_CR1_CMS_Msk           (0x3UL << TIM_CR1_CMS_Pos)                    /*!< 0x00000060 */
#define TIM_CR1_CMS               TIM_CR1_CMS_Msk                              /*!<CMS[1:0] bits (Center-aligned mode selection) */
#define TIM_CR1_CMS_0             (0x1UL << TIM_CR1_CMS_Pos)                    /*!< 0x0020 */
#define TIM_CR1_CMS_1             (0x2UL << TIM_CR1_CMS_Pos)                    /*!< 0x0040 */

#define TIM_CR1_ARPE_Pos          (7U)
#define TIM_CR1_ARPE_Msk          (0x1UL << TIM_CR1_ARPE_Pos)                   /*!< 0x00000080 */
#define TIM_CR1_ARPE              TIM_CR1_ARPE_Msk                             /*!<Auto-reload preload enable     */

#define TIM_CR1_CKD_Pos           (8U)
#define TIM_CR1_CKD_Msk           (0x3UL << TIM_CR1_CKD_Pos)                    /*!< 0x00000300 */
#define TIM_CR1_CKD               TIM_CR1_CKD_Msk                              /*!<CKD[1:0] bits (clock division) */
#define TIM_CR1_CKD_0             (0x1UL << TIM_CR1_CKD_Pos)                    /*!< 0x0100 */
#define TIM_CR1_CKD_1             (0x2UL << TIM_CR1_CKD_Pos)                    /*!< 0x0200 */

/*******************  Bit definition for TIM_CR2 register  ********************/
#define TIM_CR2_CCPC_Pos          (0U)
#define TIM_CR2_CCPC_Msk          (0x1UL << TIM_CR2_CCPC_Pos)                   /*!< 0x00000001 */
#define TIM_CR2_CCPC              TIM_CR2_CCPC_Msk                             /*!<Capture/Compare Preloaded Control        */
#define TIM_CR2_CCUS_Pos          (2U)
#define TIM_CR2_CCUS_Msk          (0x1UL << TIM_CR2_CCUS_Pos)                   /*!< 0x00000004 */
#define TIM_CR2_CCUS              TIM_CR2_CCUS_Msk                             /*!<Capture/Compare Control Update Selection */
#define TIM_CR2_CCDS_Pos          (3U)
#define TIM_CR2_CCDS_Msk          (0x1UL << TIM_CR2_CCDS_Pos)                   /*!< 0x00000008 */
#define TIM_CR2_CCDS              TIM_CR2_CCDS_Msk                             /*!<Capture/Compare DMA Selection            */

#define TIM_CR2_MMS_Pos           (4U)
#define TIM_CR2_MMS_Msk           (0x7UL << TIM_CR2_MMS_Pos)                    /*!< 0x00000070 */
#define TIM_CR2_MMS               TIM_CR2_MMS_Msk                              /*!<MMS[2:0] bits (Master Mode Selection) */
#define TIM_CR2_MMS_0             (0x1UL << TIM_CR2_MMS_Pos)                    /*!< 0x0010 */
#define TIM_CR2_MMS_1             (0x2UL << TIM_CR2_MMS_Pos)                    /*!< 0x0020 */
#define TIM_CR2_MMS_2             (0x4UL << TIM_CR2_MMS_Pos)                    /*!< 0x0040 */

#define TIM_CR2_TI1S_Pos          (7U)
#define TIM_CR2_TI1S_Msk          (0x1UL << TIM_CR2_TI1S_Pos)                   /*!< 0x00000080 */
#define TIM_CR2_TI1S              TIM_CR2_TI1S_Msk                             /*!<TI1 Selection */
#define TIM_CR2_OIS1_Pos          (8U)
#define TIM_CR2_OIS1_Msk          (0x1UL << TIM_CR2_OIS1_Pos)                   /*!< 0x00000100 */
#define TIM_CR2_OIS1              TIM_CR2_OIS1_Msk                             /*!<Output Idle state 1 (OC1 output)  */
#define TIM_CR2_OIS1N_Pos         (9U)
#define TIM_CR2_OIS1N_Msk         (0x1UL << TIM_CR2_OIS1N_Pos)                  /*!< 0x00000200 */
#define TIM_CR2_OIS1N             TIM_CR2_OIS1N_Msk                            /*!<Output Idle state 1 (OC1N output) */
#define TIM_CR2_OIS2_Pos          (10U)
#define TIM_CR2_OIS2_Msk          (0x1UL << TIM_CR2_OIS2_Pos)                   /*!< 0x00000400 */
#define TIM_CR2_OIS2              TIM_CR2_OIS2_Msk                             /*!<Output Idle state 2 (OC2 output)  */
#define TIM_CR2_OIS2N_Pos         (11U)
#define TIM_CR2_OIS2N_Msk         (0x1UL << TIM_CR2_OIS2N_Pos)                  /*!< 0x00000800 */
#define TIM_CR2_OIS2N             TIM_CR2_OIS2N_Msk                            /*!<Output Idle state 2 (OC2N output) */
#define TIM_CR2_OIS3_Pos          (12U)
#define TIM_CR2_OIS3_Msk          (0x1UL << TIM_CR2_OIS3_Pos)                   /*!< 0x00001000 */
#define TIM_CR2_OIS3              TIM_CR2_OIS3_Msk                             /*!<Output Idle state 3 (OC3 output)  */
#define TIM_CR2_OIS3N_Pos         (13U)
#define TIM_CR2_OIS3N_Msk         (0x1UL << TIM_CR2_OIS3N_Pos)                  /*!< 0x00002000 */
#define TIM_CR2_OIS3N             TIM_CR2_OIS3N_Msk                            /*!<Output Idle state 3 (OC3N output) */
#define TIM_CR2_OIS4_Pos          (14U)
#define TIM_CR2_OIS4_Msk          (0x1UL << TIM_CR2_OIS4_Pos)                   /*!< 0x00004000 */
#define TIM_CR2_OIS4              TIM_CR2_OIS4_Msk                             /*!<Output Idle state 4 (OC4 output)  */

/*******************  Bit definition for TIM_SMCR register  *******************/
#define TIM_SMCR_SMS_Pos          (0U)
#define TIM_SMCR_SMS_Msk          (0x7UL << TIM_SMCR_SMS_Pos)                   /*!< 0x00000007 */
#define TIM_SMCR_SMS              TIM_SMCR_SMS_Msk                             /*!<SMS[2:0] bits (Slave mode selection)    */
#define TIM_SMCR_SMS_0            (0x1UL << TIM_SMCR_SMS_Pos)                   /*!< 0x0001 */
#define TIM_SMCR_SMS_1            (0x2UL << TIM_SMCR_SMS_Pos)                   /*!< 0x0002 */
#define TIM_SMCR_SMS_2            (0x4UL << TIM_SMCR_SMS_Pos)                   /*!< 0x0004 */

#define TIM_SMCR_TS_Pos           (4U)
#define TIM_SMCR_TS_Msk           (0x7UL << TIM_SMCR_TS_Pos)                    /*!< 0x00000070 */
#define TIM_SMCR_TS               TIM_SMCR_TS_Msk                              /*!<TS[2:0] bits (Trigger selection)        */
#define TIM_SMCR_TS_0             (0x1UL << TIM_SMCR_TS_Pos)                    /*!< 0x0010 */
#define TIM_SMCR_TS_1             (0x2UL << TIM_SMCR_TS_Pos)                    /*!< 0x0020 */
#define TIM_SMCR_TS_2             (0x4UL << TIM_SMCR_TS_Pos)                    /*!< 0x0040 */

#define TIM_SMCR_MSM_Pos          (7U)
#define TIM_SMCR_MSM_Msk          (0x1UL << TIM_SMCR_MSM_Pos)                   /*!< 0x00000080 */
#define TIM_SMCR_MSM              TIM_SMCR_MSM_Msk                             /*!<Master/slave mode                       */

#define TIM_SMCR_ETF_Pos          (8U)
#define TIM_SMCR_ETF_Msk          (0xFUL << TIM_SMCR_ETF_Pos)                   /*!< 0x00000F00 */
#define TIM_SMCR_ETF              TIM_SMCR_ETF_Msk                             /*!<ETF[3:0] bits (External trigger filter) */
#define TIM_SMCR_ETF_0            (0x1UL << TIM_SMCR_ETF_Pos)                   /*!< 0x0100 */
#define TIM_SMCR_ETF_1            (0x2UL << TIM_SMCR_ETF_Pos)                   /*!< 0x0200 */
#define TIM_SMCR_ETF_2            (0x4UL << TIM_SMCR_ETF_Pos)                   /*!< 0x0400 */
#define TIM_SMCR_ETF_3            (0x8UL << TIM_SMCR_ETF_Pos)                   /*!< 0x0800 */

#define TIM_SMCR_ETPS_Pos         (12U)
#define TIM_SMCR_ETPS_Msk         (0x3UL << TIM_SMCR_ETPS_Pos)                  /*!< 0x00003000 */
#define TIM_SMCR_ETPS             TIM_SMCR_ETPS_Msk                            /*!<ETPS[1:0] bits (External trigger prescaler) */
#define TIM_SMCR_ETPS_0           (0x1UL << TIM_SMCR_ETPS_Pos)                  /*!< 0x1000 */
#define TIM_SMCR_ETPS_1           (0x2UL << TIM_SMCR_ETPS_Pos)                  /*!< 0x2000 */

#define TIM_SMCR_ECE_Pos          (14U)
#define TIM_SMCR_ECE_Msk          (0x1UL << TIM_SMCR_ECE_Pos)                   /*!< 0x00004000 */
#define TIM_SMCR_ECE              TIM_SMCR_ECE_Msk                             /*!<External clock enable     */
#define TIM_SMCR_ETP_Pos          (15U)
#define TIM_SMCR_ETP_Msk          (0x1UL << TIM_SMCR_ETP_Pos)                   /*!< 0x00008000 */
#define TIM_SMCR_ETP              TIM_SMCR_ETP_Msk                             /*!<External trigger polarity */

/*******************  Bit definition for TIM_DIER register  *******************/
#define TIM_DIER_UIE_Pos          (0U)
#define TIM_DIER_UIE_Msk          (0x1UL << TIM_DIER_UIE_Pos)                   /*!< 0x00000001 */
#define TIM_DIER_UIE              TIM_DIER_UIE_Msk                             /*!<Update interrupt enable */
#define TIM_DIER_CC1IE_Pos        (1U)
#define TIM_DIER_CC1IE_Msk        (0x1UL << TIM_DIER_CC1IE_Pos)                 /*!< 0x00000002 */
#define TIM_DIER_CC1IE            TIM_DIER_CC1IE_Msk                           /*!<Capture/Compare 1 interrupt enable   */
#define TIM_DIER_CC2IE_Pos        (2U)
#define TIM_DIER_CC2IE_Msk        (0x1UL << TIM_DIER_CC2IE_Pos)                 /*!< 0x00000004 */
#define TIM_DIER_CC2IE            TIM_DIER_CC2IE_Msk                           /*!<Capture/Compare 2 interrupt enable   */
#define TIM_DIER_CC3IE_Pos        (3U)
#define TIM_DIER_CC3IE_Msk        (0x1UL << TIM_DIER_CC3IE_Pos)                 /*!< 0x00000008 */
#define TIM_DIER_CC3IE            TIM_DIER_CC3IE_Msk                           /*!<Capture/Compare 3 interrupt enable   */
#define TIM_DIER_CC4IE_Pos        (4U)
#define TIM_DIER_CC4IE_Msk        (0x1UL << TIM_DIER_CC4IE_Pos)                 /*!< 0x00000010 */
#define TIM_DIER_CC4IE            TIM_DIER_CC4IE_Msk                           /*!<Capture/Compare 4 interrupt enable   */
#define TIM_DIER_COMIE_Pos        (5U)
#define TIM_DIER_COMIE_Msk        (0x1UL << TIM_DIER_COMIE_Pos)                 /*!< 0x00000020 */
#define TIM_DIER_COMIE            TIM_DIER_COMIE_Msk                           /*!<COM interrupt enable                 */
#define TIM_DIER_TIE_Pos          (6U)
#define TIM_DIER_TIE_Msk          (0x1UL << TIM_DIER_TIE_Pos)                   /*!< 0x00000040 */
#define TIM_DIER_TIE              TIM_DIER_TIE_Msk                             /*!<Trigger interrupt enable             */
#define TIM_DIER_BIE_Pos          (7U)
#define TIM_DIER_BIE_Msk          (0x1UL << TIM_DIER_BIE_Pos)                   /*!< 0x00000080 */
#define TIM_DIER_BIE              TIM_DIER_BIE_Msk                             /*!<Break interrupt enable               */
#define TIM_DIER_UDE_Pos          (8U)
#define TIM_DIER_UDE_Msk          (0x1UL << TIM_DIER_UDE_Pos)                   /*!< 0x00000100 */
#define TIM_DIER_UDE              TIM_DIER_UDE_Msk                             /*!<Update DMA request enable            */
#define TIM_DIER_CC1DE_Pos        (9U)
#define TIM_DIER_CC1DE_Msk        (0x1UL << TIM_DIER_CC1DE_Pos)                 /*!< 0x00000200 */
#define TIM_DIER_CC1DE            TIM_DIER_CC1DE_Msk                           /*!<Capture/Compare 1 DMA request enable */
#define TIM_DIER_CC2DE_Pos        (10U)
#define TIM_DIER_CC2DE_Msk        (0x1UL << TIM_DIER_CC2DE_Pos)                 /*!< 0x00000400 */
#define TIM_DIER_CC2DE            TIM_DIER_CC2DE_Msk                           /*!<Capture/Compare 2 DMA request enable */
#define TIM_DIER_CC3DE_Pos        (11U)
#define TIM_DIER_CC3DE_Msk        (0x1UL << TIM_DIER_CC3DE_Pos)                 /*!< 0x00000800 */
#define TIM_DIER_CC3DE            TIM_DIER_CC3DE_Msk                           /*!<Capture/Compare 3 DMA request enable */
#define TIM_DIER_CC4DE_Pos        (12U)
#define TIM_DIER_CC4DE_Msk        (0x1UL << TIM_DIER_CC4DE_Pos)                 /*!< 0x00001000 */
#define TIM_DIER_CC4DE            TIM_DIER_CC4DE_Msk                           /*!<Capture/Compare 4 DMA request enable */
#define TIM_DIER_COMDE_Pos        (13U)
#define TIM_DIER_COMDE_Msk        (0x1UL << TIM_DIER_COMDE_Pos)                 /*!< 0x00002000 */
#define TIM_DIER_COMDE            TIM_DIER_COMDE_Msk                           /*!<COM DMA request enable               */
#define TIM_DIER_TDE_Pos          (14U)
#define TIM_DIER_TDE_Msk          (0x1UL << TIM_DIER_TDE_Pos)                   /*!< 0x00004000 */
#define TIM_DIER_TDE              TIM_DIER_TDE_Msk                             /*!<Trigger DMA request enable           */

/********************  Bit definition for TIM_SR register  ********************/
#define TIM_SR_UIF_Pos            (0U)
#define TIM_SR_UIF_Msk            (0x1UL << TIM_SR_UIF_Pos)                     /*!< 0x00000001 */
#define TIM_SR_UIF                TIM_SR_UIF_Msk                               /*!<Update interrupt Flag              */
#define TIM_SR_CC1IF_Pos          (1U)
#define TIM_SR_CC1IF_Msk          (0x1UL << TIM_SR_CC1IF_Pos)                   /*!< 0x00000002 */
#define TIM_SR_CC1IF              TIM_SR_CC1IF_Msk                             /*!<Capture/Compare 1 interrupt Flag   */
#define TIM_SR_CC2IF_Pos          (2U)
#define TIM_SR_CC2IF_Msk          (0x1UL << TIM_SR_CC2IF_Pos)                   /*!< 0x00000004 */
#define TIM_SR_CC2IF              TIM_SR_CC2IF_Msk                             /*!<Capture/Compare 2 interrupt Flag   */
#define TIM_SR_CC3IF_Pos          (3U)
#define TIM_SR_CC3IF_Msk          (0x1UL << TIM_SR_CC3IF_Pos)                   /*!< 0x00000008 */
#define TIM_SR_CC3IF              TIM_SR_CC3IF_Msk                             /*!<Capture/Compare 3 interrupt Flag   */
#define TIM_SR_CC4IF_Pos          (4U)
#define TIM_SR_CC4IF_Msk          (0x1UL << TIM_SR_CC4IF_Pos)                   /*!< 0x00000010 */
#define TIM_SR_CC4IF              TIM_SR_CC4IF_Msk                             /*!<Capture/Compare 4 interrupt Flag   */
#define TIM_SR_COMIF_Pos          (5U)
#define TIM_SR_COMIF_Msk          (0x1UL << TIM_SR_COMIF_Pos)                   /*!< 0x00000020 */
#define TIM_SR_COMIF              TIM_SR_COMIF_Msk                             /*!<COM interrupt Flag                 */
#define TIM_SR_TIF_Pos            (6U)
#define TIM_SR_TIF_Msk            (0x1UL << TIM_SR_TIF_Pos)                     /*!< 0x00000040 */
#define TIM_SR_TIF                TIM_SR_TIF_Msk                               /*!<Trigger interrupt Flag             */
#define TIM_SR_BIF_Pos            (7U)
#define TIM_SR_BIF_Msk            (0x1UL << TIM_SR_BIF_Pos)                     /*!< 0x00000080 */
#define TIM_SR_BIF                TIM_SR_BIF_Msk                               /*!<Break interrupt Flag               */
#define TIM_SR_CC1OF_Pos          (9U)
#define TIM_SR_CC1OF_Msk          (0x1UL << TIM_SR_CC1OF_Pos)                   /*!< 0x00000200 */
#define TIM_SR_CC1OF              TIM_SR_CC1OF_Msk                             /*!<Capture/Compare 1 Overcapture Flag */
#define TIM_SR_CC2OF_Pos          (10U)
#define TIM_SR_CC2OF_Msk          (0x1UL << TIM_SR_CC2OF_Pos)                   /*!< 0x00000400 */
#define TIM_SR_CC2OF              TIM_SR_CC2OF_Msk                             /*!<Capture/Compare 2 Overcapture Flag */
#define TIM_SR_CC3OF_Pos          (11U)
#define TIM_SR_CC3OF_Msk          (0x1UL << TIM_SR_CC3OF_Pos)                   /*!< 0x00000800 */
#define TIM_SR_CC3OF              TIM_SR_CC3OF_Msk                             /*!<Capture/Compare 3 Overcapture Flag */
#define TIM_SR_CC4OF_Pos          (12U)
#define TIM_SR_CC4OF_Msk          (0x1UL << TIM_SR_CC4OF_Pos)                   /*!< 0x00001000 */
#define TIM_SR_CC4OF              TIM_SR_CC4OF_Msk                             /*!<Capture/Compare 4 Overcapture Flag */

/*******************  Bit definition for TIM_EGR register  ********************/
#define TIM_EGR_UG_Pos            (0U)
#define TIM_EGR_UG_Msk            (0x1UL << TIM_EGR_UG_Pos)                     /*!< 0x00000001 */
#define TIM_EGR_UG                TIM_EGR_UG_Msk                               /*!<Update Generation                         */
#define TIM_EGR_CC1G_Pos          (1U)
#define TIM_EGR_CC1G_Msk          (0x1UL << TIM_EGR_CC1G_Pos)                   /*!< 0x00000002 */
#define TIM_EGR_CC1G              TIM_EGR_CC1G_Msk                             /*!<Capture/Compare 1 Generation              */
#define TIM_EGR_CC2G_Pos          (2U)
#define TIM_EGR_CC2G_Msk          (0x1UL << TIM_EGR_CC2G_Pos)                   /*!< 0x00000004 */
#define TIM_EGR_CC2G              TIM_EGR_CC2G_Msk                             /*!<Capture/Compare 2 Generation              */
#define TIM_EGR_CC3G_Pos          (3U)
#define TIM_EGR_CC3G_Msk          (0x1UL << TIM_EGR_CC3G_Pos)                   /*!< 0x00000008 */
#define TIM_EGR_CC3G              TIM_EGR_CC3G_Msk                             /*!<Capture/Compare 3 Generation              */
#define TIM_EGR_CC4G_Pos          (4U)
#define TIM_EGR_CC4G_Msk          (0x1UL << TIM_EGR_CC4G_Pos)                   /*!< 0x00000010 */
#define TIM_EGR_CC4G              TIM_EGR_CC4G_Msk                             /*!<Capture/Compare 4 Generation              */
#define TIM_EGR_COMG_Pos          (5U)
#define TIM_EGR_COMG_Msk          (0x1UL << TIM_EGR_COMG_Pos)                   /*!< 0x00000020 */
#define TIM_EGR_COMG              TIM_EGR_COMG_Msk                             /*!<Capture/Compare Control Update Generation */
#define TIM_EGR_TG_Pos            (6U)
#define TIM_EGR_TG_Msk            (0x1UL << TIM_EGR_TG_Pos)                     /*!< 0x00000040 */
#define TIM_EGR_TG                TIM_EGR_TG_Msk                               /*!<Trigger Generation                        */
#define TIM_EGR_BG_Pos            (7U)
#define TIM_EGR_BG_Msk            (0x1UL << TIM_EGR_BG_Pos)                     /*!< 0x00000080 */
#define TIM_EGR_BG                TIM_EGR_BG_Msk                               /*!<Break Generation                          */

/******************  Bit definition for TIM_CCMR1 register  *******************/
#define TIM_CCMR1_CC1S_Pos        (0U)
#define TIM_CCMR1_CC1S_Msk        (0x3UL << TIM_CCMR1_CC1S_Pos)                 /*!< 0x00000003 */
#define TIM_CCMR1_CC1S            TIM_CCMR1_CC1S_Msk                           /*!<CC1S[1:0] bits (Capture/Compare 1 Selection) */
#define TIM_CCMR1_CC1S_0          (0x1UL << TIM_CCMR1_CC1S_Pos)                 /*!< 0x0001 */
#define TIM_CCMR1_CC1S_1          (0x2UL << TIM_CCMR1_CC1S_Pos)                 /*!< 0x0002 */

#define TIM_CCMR1_OC1FE_Pos       (2U)
#define TIM_CCMR1_OC1FE_Msk       (0x1UL << TIM_CCMR1_OC1FE_Pos)                /*!< 0x00000004 */
#define TIM_CCMR1_OC1FE           TIM_CCMR1_OC1FE_Msk                          /*!<Output Compare 1 Fast enable                 */
#define TIM_CCMR1_OC1PE_Pos       (3U)
#define TIM_CCMR1_OC1PE_Msk       (0x1UL << TIM_CCMR1_OC1PE_Pos)                /*!< 0x00000008 */
#define TIM_CCMR1_OC1PE           TIM_CCMR1_OC1PE_Msk                          /*!<Output Compare 1 Preload enable              */

#define TIM_CCMR1_OC1M_Pos        (4U)
#define TIM_CCMR1_OC1M_Msk        (0x7UL << TIM_CCMR1_OC1M_Pos)                 /*!< 0x00000070 */
#define TIM_CCMR1_OC1M            TIM_CCMR1_OC1M_Msk                           /*!<OC1M[2:0] bits (Output Compare 1 Mode)       */
#define TIM_CCMR1_OC1M_0          (0x1UL << TIM_CCMR1_OC1M_Pos)                 /*!< 0x0010 */
#define TIM_CCMR1_OC1M_1          (0x2UL << TIM_CCMR1_OC1M_Pos)                 /*!< 0x0020 */
#define TIM_CCMR1_OC1M_2          (0x4UL << TIM_CCMR1_OC1M_Pos)                 /*!< 0x0040 */

#define TIM_CCMR1_OC1CE_Pos       (7U)
#define TIM_CCMR1_OC1CE_Msk       (0x1UL << TIM_CCMR1_OC1CE_Pos)                /*!< 0x00000080 */
#define TIM_CCMR1_OC1CE           TIM_CCMR1_OC1CE_Msk                          /*!<Output Compare 1Clear Enable                 */

#define TIM_CCMR1_CC2S_Pos        (8U)
#define TIM_CCMR1_CC2S_Msk        (0x3UL << TIM_CCMR1_CC2S_Pos)                 /*!< 0x00000300 */
#define TIM_CCMR1_CC2S            TIM_CCMR1_CC2S_Msk                           /*!<CC2S[1:0] bits (Capture/Compare 2 Selection) */
#define TIM_CCMR1_CC2S_0          (0x1UL << TIM_CCMR1_CC2S_Pos)                 /*!< 0x0100 */
#define TIM_CCMR1_CC2S_1          (0x2UL << TIM_CCMR1_CC2S_Pos)                 /*!< 0x0200 */

#define TIM_CCMR1_OC2FE_Pos       (10U)
#define TIM_CCMR1_OC2FE_Msk       (0x1UL << TIM_CCMR1_OC2FE_Pos)                /*!< 0x00000400 */
#define TIM_CCMR1_OC2FE           TIM_CCMR1_OC2FE_Msk                          /*!<Output Compare 2 Fast enable                 */
#define TIM_CCMR1_OC2PE_Pos       (11U)
#define TIM_CCMR1_OC2PE_Msk       (0x1UL << TIM_CCMR1_OC2PE_Pos)                /*!< 0x00000800 */
#define TIM_CCMR1_OC2PE           TIM_CCMR1_OC2PE_Msk                          /*!<Output Compare 2 Preload enable              */

#define TIM_CCMR1_OC2M_Pos        (12U)
#define TIM_CCMR1_OC2M_Msk        (0x7UL << TIM_CCMR1_OC2M_Pos)                 /*!< 0x00007000 */
#define TIM_CCMR1_OC2M            TIM_CCMR1_OC2M_Msk                           /*!<OC2M[2:0] bits (Output Compare 2 Mode)       */
#define TIM_CCMR1_OC2M_0          (0x1UL << TIM_CCMR1_OC2M_Pos)                 /*!< 0x1000 */
#define TIM_CCMR1_OC2M_1          (0x2UL << TIM_CCMR1_OC2M_Pos)                 /*!< 0x2000 */
#define TIM_CCMR1_OC2M_2          (0x4UL << TIM_CCMR1_OC2M_Pos)                 /*!< 0x4000 */

#define TIM_CCMR1_OC2CE_Pos       (15U)
#define TIM_CCMR1_OC2CE_Msk       (0x1UL << TIM_CCMR1_OC2CE_Pos)                /*!< 0x00008000 */
#define TIM_CCMR1_OC2CE           TIM_CCMR1_OC2CE_Msk                          /*!<Output Compare 2 Clear Enable */

/*----------------------------------------------------------------------------*/

#define TIM_CCMR1_IC1PSC_Pos      (2U)
#define TIM_CCMR1_IC1PSC_Msk      (0x3UL << TIM_CCMR1_IC1PSC_Pos)               /*!< 0x0000000C */
#define TIM_CCMR1_IC1PSC          TIM_CCMR1_IC1PSC_Msk                         /*!<IC1PSC[1:0] bits (Input Capture 1 Prescaler) */
#define TIM_CCMR1_IC1PSC_0        (0x1UL << TIM_CCMR1_IC1PSC_Pos)               /*!< 0x0004 */
#define TIM_CCMR1_IC1PSC_1        (0x2UL << TIM_CCMR1_IC1PSC_Pos)               /*!< 0x0008 */

#define TIM_CCMR1_IC1F_Pos        (4U)
#define TIM_CCMR1_IC1F_Msk        (0xFUL << TIM_CCMR1_IC1F_Pos)                 /*!< 0x000000F0 */
#define TIM_CCMR1_IC1F            TIM_CCMR1_IC1F_Msk                           /*!<IC1F[3:0] bits (Input Capture 1 Filter)      */
#define TIM_CCMR1_IC1F_0          (0x1UL << TIM_CCMR1_IC1F_Pos)                 /*!< 0x0010 */
#define TIM_CCMR1_IC1F_1          (0x2UL << TIM_CCMR1_IC1F_Pos)                 /*!< 0x0020 */
#define TIM_CCMR1_IC1F_2          (0x4UL << TIM_CCMR1_IC1F_Pos)                 /*!< 0x0040 */
#define TIM_CCMR1_IC1F_3          (0x8UL << TIM_CCMR1_IC1F_Pos)                 /*!< 0x0080 */

#define TIM_CCMR1_IC2PSC_Pos      (10U)
#define TIM_CCMR1_IC2PSC_Msk      (0x3UL << TIM_CCMR1_IC2PSC_Pos)               /*!< 0x00000C00 */
#define TIM_CCMR1_IC2PSC          TIM_CCMR1_IC2PSC_Msk                         /*!<IC2PSC[1:0] bits (Input Capture 2 Prescaler)  */
#define TIM_CCMR1_IC2PSC_0        (0x1UL << TIM_CCMR1_IC2PSC_Pos)               /*!< 0x0400 */
#define TIM_CCMR1_IC2PSC_1        (0x2UL << TIM_CCMR1_IC2PSC_Pos)               /*!< 0x0800 */

#define TIM_CCMR1_IC2F_Pos        (12U)
#define TIM_CCMR1_IC2F_Msk        (0xFUL << TIM_CCMR1_IC2F_Pos)                 /*!< 0x0000F000 */
#define TIM_CCMR1_IC2F            TIM_CCMR1_IC2F_Msk                           /*!<IC2F[3:0] bits (Input Capture 2 Filter)       */
#define TIM_CCMR1_IC2F_0          (0x1UL << TIM_CCMR1_IC2F_Pos)                 /*!< 0x1000 */
#define TIM_CCMR1_IC2F_1          (0x2UL << TIM_CCMR1_IC2F_Pos)                 /*!< 0x2000 */
#define TIM_CCMR1_IC2F_2          (0x4UL << TIM_CCMR1_IC2F_Pos)                 /*!< 0x4000 */
#define TIM_CCMR1_IC2F_3          (0x8UL << TIM_CCMR1_IC2F_Pos)                 /*!< 0x8000 */

/******************  Bit definition for TIM_CCMR2 register  *******************/
#define TIM_CCMR2_CC3S_Pos        (0U)
#define TIM_CCMR2_CC3S_Msk        (0x3UL << TIM_CCMR2_CC3S_Pos)                 /*!< 0x00000003 */
#define TIM_CCMR2_CC3S            TIM_CCMR2_CC3S_Msk                           /*!<CC3S[1:0] bits (Capture/Compare 3 Selection)  */
#define TIM_CCMR2_CC3S_0          (0x1UL << TIM_CCMR2_CC3S_Pos)                 /*!< 0x0001 */
#define TIM_CCMR2_CC3S_1          (0x2UL << TIM_CCMR2_CC3S_Pos)                 /*!< 0x0002 */

#define TIM_CCMR2_OC3FE_Pos       (2U)
#define TIM_CCMR2_OC3FE_Msk       (0x1UL << TIM_CCMR2_OC3FE_Pos)                /*!< 0x00000004 */
#define TIM_CCMR2_OC3FE           TIM_CCMR2_OC3FE_Msk                          /*!<Output Compare 3 Fast enable           */
#define TIM_CCMR2_OC3PE_Pos       (3U)
#define TIM_CCMR2_OC3PE_Msk       (0x1UL << TIM_CCMR2_OC3PE_Pos)                /*!< 0x00000008 */
#define TIM_CCMR2_OC3PE           TIM_CCMR2_OC3PE_Msk                          /*!<Output Compare 3 Preload enable        */

#define TIM_CCMR2_OC3M_Pos        (4U)
#define TIM_CCMR2_OC3M_Msk        (0x7UL << TIM_CCMR2_OC3M_Pos)                 /*!< 0x00000070 */
#define TIM_CCMR2_OC3M            TIM_CCMR2_OC3M_Msk                           /*!<OC3M[2:0] bits (Output Compare 3 Mode) */
#define TIM_CCMR2_OC3M_0          (0x1UL << TIM_CCMR2_OC3M_Pos)                 /*!< 0x0010 */
#define TIM_CCMR2_OC3M_1          (0x2UL << TIM_CCMR2_OC3M_Pos)                 /*!< 0x0020 */
#define TIM_CCMR2_OC3M_2          (0x4UL << TIM_CCMR2_OC3M_Pos)                 /*!< 0x0040 */

#define TIM_CCMR2_OC3CE_Pos       (7U)
#define TIM_CCMR2_OC3CE_Msk       (0x1UL << TIM_CCMR2_OC3CE_Pos)                /*!< 0x00000080 */
#define TIM_CCMR2_OC3CE           TIM_CCMR2_OC3CE_Msk                          /*!<Output Compare 3 Clear Enable */

#define TIM_CCMR2_CC4S_Pos        (8U)
#define TIM_CCMR2_CC4S_Msk        (0x3UL << TIM_CCMR2_CC4S_Pos)                 /*!< 0x00000300 */
#define TIM_CCMR2_CC4S            TIM_CCMR2_CC4S_Msk                           /*!<CC4S[1:0] bits (Capture/Compare 4 Selection) */
#define TIM_CCMR2_CC4S_0          (0x1UL << TIM_CCMR2_CC4S_Pos)                 /*!< 0x0100 */
#define TIM_CCMR2_CC4S_1          (0x2UL << TIM_CCMR2_CC4S_Pos)                 /*!< 0x0200 */

#define TIM_CCMR2_OC4FE_Pos       (10U)
#define TIM_CCMR2_OC4FE_Msk       (0x1UL << TIM_CCMR2_OC4FE_Pos)                /*!< 0x00000400 */
#define TIM_CCMR2_OC4FE           TIM_CCMR2_OC4FE_Msk                          /*!<Output Compare 4 Fast enable    */
#define TIM_CCMR2_OC4PE_Pos       (11U)
#define TIM_CCMR2_OC4PE_Msk       (0x1UL << TIM_CCMR2_OC4PE_Pos)                /*!< 0x00000800 */
#define TIM_CCMR2_OC4PE           TIM_CCMR2_OC4PE_Msk                          /*!<Output Compare 4 Preload enable */

#define TIM_CCMR2_OC4M_Pos        (12U)
#define TIM_CCMR2_OC4M_Msk        (0x7UL << TIM_CCMR2_OC4M_Pos)                 /*!< 0x00007000 */
#define TIM_CCMR2_OC4M            TIM_CCMR2_OC4M_Msk                           /*!<OC4M[2:0] bits (Output Compare 4 Mode) */
#define TIM_CCMR2_OC4M_0          (0x1UL << TIM_CCMR2_OC4M_Pos)                 /*!< 0x1000 */
#define TIM_CCMR2_OC4M_1          (0x2UL << TIM_CCMR2_OC4M_Pos)                 /*!< 0x2000 */
#define TIM_CCMR2_OC4M_2          (0x4UL << TIM_CCMR2_OC4M_Pos)                 /*!< 0x4000 */

#define TIM_CCMR2_OC4CE_Pos       (15U)
#define TIM_CCMR2_OC4CE_Msk       (0x1UL << TIM_CCMR2_OC4CE_Pos)                /*!< 0x00008000 */
#define TIM_CCMR2_OC4CE           TIM_CCMR2_OC4CE_Msk                          /*!<Output Compare 4 Clear Enable */

/*----------------------------------------------------------------------------*/

#define TIM_CCMR2_IC3PSC_Pos      (2U)
#define TIM_CCMR2_IC3PSC_Msk      (0x3UL << TIM_CCMR2_IC3PSC_Pos)               /*!< 0x0000000C */
#define TIM_CCMR2_IC3PSC          TIM_CCMR2_IC3PSC_Msk                         /*!<IC3PSC[1:0] bits (Input Capture 3 Prescaler) */
#define TIM_CCMR2_IC3PSC_0        (0x1UL << TIM_CCMR2_IC3PSC_Pos)               /*!< 0x0004 */
#define TIM_CCMR2_IC3PSC_1        (0x2UL << TIM_CCMR2_IC3PSC_Pos)               /*!< 0x0008 */

#define TIM_CCMR2_IC3F_Pos        (4U)
#define TIM_CCMR2_IC3F_Msk        (0xFUL << TIM_CCMR2_IC3F_Pos)                 /*!< 0x000000F0 */
#define TIM_CCMR2_IC3F            TIM_CCMR2_IC3F_Msk                           /*!<IC3F[3:0] bits (Input Capture 3 Filter) */
#define TIM_CCMR2_IC3F_0          (0x1UL << TIM_CCMR2_IC3F_Pos)                 /*!< 0x0010 */
#define TIM_CCMR2_IC3F_1          (0x2UL << TIM_CCMR2_IC3F_Pos)                 /*!< 0x0020 */
#define TIM_CCMR2_IC3F_2          (0x4UL << TIM_CCMR2_IC3F_Pos)                 /*!< 0x0040 */
#define TIM_CCMR2_IC3F_3          (0x8UL << TIM_CCMR2_IC3F_Pos)                 /*!< 0x0080 */

#define TIM_CCMR2_IC4PSC_Pos      (10U)
#define TIM_CCMR2_IC4PSC_Msk      (0x3UL << TIM_CCMR2_IC4PSC_Pos)               /*!< 0x00000C00 */
#define TIM_CCMR2_IC4PSC          TIM_CCMR2_IC4PSC_Msk                         /*!<IC4PSC[1:0] bits (Input Capture 4 Prescaler) */
#define TIM_CCMR2_IC4PSC_0        (0x1UL << TIM_CCMR2_IC4PSC_Pos)               /*!< 0x0400 */
#define TIM_CCMR2_IC4PSC_1        (0x2UL << TIM_CCMR2_IC4PSC_Pos)               /*!< 0x0800 */

#define TIM_CCMR2_IC4F_Pos        (12U)
#define TIM_CCMR2_IC4F_Msk        (0xFUL << TIM_CCMR2_IC4F_Pos)                 /*!< 0x0000F000 */
#define TIM_CCMR2_IC4F            TIM_CCMR2_IC4F_Msk                           /*!<IC4F[3:0] bits (Input Capture 4 Filter) */
#define TIM_CCMR2_IC4F_0          (0x1UL << TIM_CCMR2_IC4F_Pos)                 /*!< 0x1000 */
#define TIM_CCMR2_IC4F_1          (0x2UL << TIM_CCMR2_IC4F_Pos)                 /*!< 0x2000 */
#define TIM_CCMR2_IC4F_2          (0x4UL << TIM_CCMR2_IC4F_Pos)                 /*!< 0x4000 */
#define TIM_CCMR2_IC4F_3          (0x8UL << TIM_CCMR2_IC4F_Pos)                 /*!< 0x8000 */

/*******************  Bit definition for TIM_CCER register  *******************/
#define TIM_CCER_CC1E_Pos         (0U)
#define TIM_CCER_CC1E_Msk         (0x1UL << TIM_CCER_CC1E_Pos)                  /*!< 0x00000001 */
#define TIM_CCER_CC1E             TIM_CCER_CC1E_Msk                            /*!<Capture/Compare 1 output enable                 */
#define TIM_CCER_CC1P_Pos         (1U)
#define TIM_CCER_CC1P_Msk         (0x1UL << TIM_CCER_CC1P_Pos)                  /*!< 0x00000002 */
#define TIM_CCER_CC1P             TIM_CCER_CC1P_Msk                            /*!<Capture/Compare 1 output Polarity               */
#define TIM_CCER_CC1NE_Pos        (2U)
#define TIM_CCER_CC1NE_Msk        (0x1UL << TIM_CCER_CC1NE_Pos)                 /*!< 0x00000004 */
#define TIM_CCER_CC1NE            TIM_CCER_CC1NE_Msk                           /*!<Capture/Compare 1 Complementary output enable   */
#define TIM_CCER_CC1NP_Pos        (3U)
#define TIM_CCER_CC1NP_Msk        (0x1UL << TIM_CCER_CC1NP_Pos)                 /*!< 0x00000008 */
#define TIM_CCER_CC1NP            TIM_CCER_CC1NP_Msk                           /*!<Capture/Compare 1 Complementary output Polarity */
#define TIM_CCER_CC2E_Pos         (4U)
#define TIM_CCER_CC2E_Msk         (0x1UL << TIM_CCER_CC2E_Pos)                  /*!< 0x00000010 */
#define TIM_CCER_CC2E             TIM_CCER_CC2E_Msk                            /*!<Capture/Compare 2 output enable                 */
#define TIM_CCER_CC2P_Pos         (5U)
#define TIM_CCER_CC2P_Msk         (0x1UL << TIM_CCER_CC2P_Pos)                  /*!< 0x00000020 */
#define TIM_CCER_CC2P             TIM_CCER_CC2P_Msk                            /*!<Capture/Compare 2 output Polarity               */
#define TIM_CCER_CC2NE_Pos        (6U)
#define TIM_CCER_CC2NE_Msk        (0x1UL << TIM_CCER_CC2NE_Pos)                 /*!< 0x00000040 */
#define TIM_CCER_CC2NE            TIM_CCER_CC2NE_Msk                           /*!<Capture/Compare 2 Complementary output enable   */
#define TIM_CCER_CC2NP_Pos        (7U)
#define TIM_CCER_CC2NP_Msk        (0x1UL << TIM_CCER_CC2NP_Pos)                 /*!< 0x00000080 */
#define TIM_CCER_CC2NP            TIM_CCER_CC2NP_Msk                           /*!<Capture/Compare 2 Complementary output Polarity */
#define TIM_CCER_CC3E_Pos         (8U)
#define TIM_CCER_CC3E_Msk         (0x1UL << TIM_CCER_CC3E_Pos)                  /*!< 0x00000100 */
#define TIM_CCER_CC3E             TIM_CCER_CC3E_Msk                            /*!<Capture/Compare 3 output enable                 */
#define TIM_CCER_CC3P_Pos         (9U)
#define TIM_CCER_CC3P_Msk         (0x1UL << TIM_CCER_CC3P_Pos)                  /*!< 0x00000200 */
#define TIM_CCER_CC3P             TIM_CCER_CC3P_Msk                            /*!<Capture/Compare 3 output Polarity               */
#define TIM_CCER_CC3NE_Pos        (10U)
#define TIM_CCER_CC3NE_Msk        (0x1UL << TIM_CCER_CC3NE_Pos)                 /*!< 0x00000400 */
#define TIM_CCER_CC3NE            TIM_CCER_CC3NE_Msk                           /*!<Capture/Compare 3 Complementary output enable   */
#define TIM_CCER_CC3NP_Pos        (11U)
#define TIM_CCER_CC3NP_Msk        (0x1UL << TIM_CCER_CC3NP_Pos)                 /*!< 0x00000800 */
#define TIM_CCER_CC3NP            TIM_CCER_CC3NP_Msk                           /*!<Capture/Compare 3 Complementary output Polarity */
#define TIM_CCER_CC4E_Pos         (12U)
#define TIM_CCER_CC4E_Msk         (0x1UL << TIM_CCER_CC4E_Pos)                  /*!< 0x00001000 */
#define TIM_CCER_CC4E             TIM_CCER_CC4E_Msk                            /*!<Capture/Compare 4 output enable                 */
#define TIM_CCER_CC4P_Pos         (13U)
#define TIM_CCER_CC4P_Msk         (0x1UL << TIM_CCER_CC4P_Pos)                  /*!< 0x00002000 */
#define TIM_CCER_CC4P             TIM_CCER_CC4P_Msk                            /*!<Capture/Compare 4 output Polarity               */
#define TIM_CCER_CC4NP_Pos        (15U)
#define TIM_CCER_CC4NP_Msk        (0x1UL << TIM_CCER_CC4NP_Pos)                 /*!< 0x00008000 */
#define TIM_CCER_CC4NP            TIM_CCER_CC4NP_Msk                           /*!<Capture/Compare 4 Complementary output Polarity */

/*******************  Bit definition for TIM_CNT register  ********************/
#define TIM_CNT_CNT_Pos           (0U)
#define TIM_CNT_CNT_Msk           (0xFFFFFFFFUL << TIM_CNT_CNT_Pos)                 /*!< 0xFFFFFFFF */
#define TIM_CNT_CNT               TIM_CNT_CNT_Msk                                  /*!<Counter Value            */

/*******************  Bit definition for TIM_PSC register  ********************/
#define TIM_PSC_PSC_Pos           (0U)
#define TIM_PSC_PSC_Msk           (0xFFFFUL << TIM_PSC_PSC_Pos)                 /*!< 0x0000FFFF */
#define TIM_PSC_PSC               TIM_PSC_PSC_Msk                              /*!<Prescaler Value          */

/*******************  Bit definition for TIM_ARR register  ********************/
#define TIM_ARR_ARR_Pos           (0U)
#define TIM_ARR_ARR_Msk           (0xFFFFFFFFUL << TIM_ARR_ARR_Pos)             /*!< 0xFFFFFFFF */
#define TIM_ARR_ARR               TIM_ARR_ARR_Msk                              /*!<actual auto-reload Value */

/*******************  Bit definition for TIM_RCR register  ********************/
#define TIM_RCR_REP_Pos           (0U)
#define TIM_RCR_REP_Msk           (0xFFUL << TIM_RCR_REP_Pos)                   /*!< 0x000000FF */
#define TIM_RCR_REP               TIM_RCR_REP_Msk                              /*!<Repetition Counter Value */

/*******************  Bit definition for TIM_CCR1 register  *******************/
#define TIM_CCR1_CCR1_Pos         (0U)
#define TIM_CCR1_CCR1_Msk         (0xFFFFUL << TIM_CCR1_CCR1_Pos)               /*!< 0x0000FFFF */
#define TIM_CCR1_CCR1             TIM_CCR1_CCR1_Msk                            /*!<Capture/Compare 1 Value  */

/*******************  Bit definition for TIM_CCR2 register  *******************/
#define TIM_CCR2_CCR2_Pos         (0U)
#define TIM_CCR2_CCR2_Msk         (0xFFFFUL << TIM_CCR2_CCR2_Pos)               /*!< 0x0000FFFF */
#define TIM_CCR2_CCR2             TIM_CCR2_CCR2_Msk                            /*!<Capture/Compare 2 Value  */

/*******************  Bit definition for TIM_CCR3 register  *******************/
#define TIM_CCR3_CCR3_Pos         (0U)
#define TIM_CCR3_CCR3_Msk         (0xFFFFUL << TIM_CCR3_CCR3_Pos)               /*!< 0x0000FFFF */
#define TIM_CCR3_CCR3             TIM_CCR3_CCR3_Msk                            /*!<Capture/Compare 3 Value  */

/*******************  Bit definition for TIM_CCR4 register  *******************/
#define TIM_CCR4_CCR4_Pos         (0U)
#define TIM_CCR4_CCR4_Msk         (0xFFFFUL << TIM_CCR4_CCR4_Pos)               /*!< 0x0000FFFF */
#define TIM_CCR4_CCR4             TIM_CCR4_CCR4_Msk                            /*!<Capture/Compare 4 Value  */

/*******************  Bit definition for TIM_BDTR register  *******************/
#define TIM_BDTR_DTG_Pos          (0U)
#define TIM_BDTR_DTG_Msk          (0xFFUL << TIM_BDTR_DTG_Pos)                  /*!< 0x000000FF */
#define TIM_BDTR_DTG              TIM_BDTR_DTG_Msk                             /*!<DTG[0:7] bits (Dead-Time Generator set-up) */
#define TIM_BDTR_DTG_0            (0x01UL << TIM_BDTR_DTG_Pos)                  /*!< 0x0001 */
#define TIM_BDTR_DTG_1            (0x02UL << TIM_BDTR_DTG_Pos)                  /*!< 0x0002 */
#define TIM_BDTR_DTG_2            (0x04UL << TIM_BDTR_DTG_Pos)                  /*!< 0x0004 */
#define TIM_BDTR_DTG_3            (0x08UL << TIM_BDTR_DTG_Pos)                  /*!< 0x0008 */
#define TIM_BDTR_DTG_4            (0x10UL << TIM_BDTR_DTG_Pos)                  /*!< 0x0010 */
#define TIM_BDTR_DTG_5            (0x20UL << TIM_BDTR_DTG_Pos)                  /*!< 0x0020 */
#define TIM_BDTR_DTG_6            (0x40UL << TIM_BDTR_DTG_Pos)                  /*!< 0x0040 */
#define TIM_BDTR_DTG_7            (0x80UL << TIM_BDTR_DTG_Pos)                  /*!< 0x0080 */

#define TIM_BDTR_LOCK_Pos         (8U)
#define TIM_BDTR_LOCK_Msk         (0x3UL << TIM_BDTR_LOCK_Pos)                  /*!< 0x00000300 */
#define TIM_BDTR_LOCK             TIM_BDTR_LOCK_Msk                            /*!<LOCK[1:0] bits (Lock Configuration) */
#define TIM_BDTR_LOCK_0           (0x1UL << TIM_BDTR_LOCK_Pos)                  /*!< 0x0100 */
#define TIM_BDTR_LOCK_1           (0x2UL << TIM_BDTR_LOCK_Pos)                  /*!< 0x0200 */

#define TIM_BDTR_OSSI_Pos         (10U)
#define TIM_BDTR_OSSI_Msk         (0x1UL << TIM_BDTR_OSSI_Pos)                  /*!< 0x00000400 */
#define TIM_BDTR_OSSI             TIM_BDTR_OSSI_Msk                            /*!<Off-State Selection for Idle mode */
#define TIM_BDTR_OSSR_Pos         (11U)
#define TIM_BDTR_OSSR_Msk         (0x1UL << TIM_BDTR_OSSR_Pos)                  /*!< 0x00000800 */
#define TIM_BDTR_OSSR             TIM_BDTR_OSSR_Msk                            /*!<Off-State Selection for Run mode  */
#define TIM_BDTR_BKE_Pos          (12U)
#define TIM_BDTR_BKE_Msk          (0x1UL << TIM_BDTR_BKE_Pos)                   /*!< 0x00001000 */
#define TIM_BDTR_BKE              TIM_BDTR_BKE_Msk                             /*!<Break enable                      */
#define TIM_BDTR_BKP_Pos          (13U)
#define TIM_BDTR_BKP_Msk          (0x1UL << TIM_BDTR_BKP_Pos)                   /*!< 0x00002000 */
#define TIM_BDTR_BKP              TIM_BDTR_BKP_Msk                             /*!<Break Polarity                    */
#define TIM_BDTR_AOE_Pos          (14U)
#define TIM_BDTR_AOE_Msk          (0x1UL << TIM_BDTR_AOE_Pos)                   /*!< 0x00004000 */
#define TIM_BDTR_AOE              TIM_BDTR_AOE_Msk                             /*!<Automatic Output enable           */
#define TIM_BDTR_MOE_Pos          (15U)
#define TIM_BDTR_MOE_Msk          (0x1UL << TIM_BDTR_MOE_Pos)                   /*!< 0x00008000 */
#define TIM_BDTR_MOE              TIM_BDTR_MOE_Msk                             /*!<Main Output enable                */

/*******************  Bit definition for TIM_DCR register  ********************/
#define TIM_DCR_DBA_Pos           (0U)
#define TIM_DCR_DBA_Msk           (0x1FUL << TIM_DCR_DBA_Pos)                   /*!< 0x0000001F */
#define TIM_DCR_DBA               TIM_DCR_DBA_Msk                              /*!<DBA[4:0] bits (DMA Base Address) */
#define TIM_DCR_DBA_0             (0x01UL << TIM_DCR_DBA_Pos)                   /*!< 0x0001 */
#define TIM_DCR_DBA_1             (0x02UL << TIM_DCR_DBA_Pos)                   /*!< 0x0002 */
#define TIM_DCR_DBA_2             (0x04UL << TIM_DCR_DBA_Pos)                   /*!< 0x0004 */
#define TIM_DCR_DBA_3             (0x08UL << TIM_DCR_DBA_Pos)                   /*!< 0x0008 */
#define TIM_DCR_DBA_4             (0x10UL << TIM_DCR_DBA_Pos)                   /*!< 0x0010 */

#define TIM_DCR_DBL_Pos           (8U)
#define TIM_DCR_DBL_Msk           (0x1FUL << TIM_DCR_DBL_Pos)                   /*!< 0x00001F00 */
#define TIM_DCR_DBL               TIM_DCR_DBL_Msk                              /*!<DBL[4:0] bits (DMA Burst Length) */
#define TIM_DCR_DBL_0             (0x01UL << TIM_DCR_DBL_Pos)                   /*!< 0x0100 */
#define TIM_DCR_DBL_1             (0x02UL << TIM_DCR_DBL_Pos)                   /*!< 0x0200 */
#define TIM_DCR_DBL_2             (0x04UL << TIM_DCR_DBL_Pos)                   /*!< 0x0400 */
#define TIM_DCR_DBL_3             (0x08UL << TIM_DCR_DBL_Pos)                   /*!< 0x0800 */
#define TIM_DCR_DBL_4             (0x10UL << TIM_DCR_DBL_Pos)                   /*!< 0x1000 */

/*******************  Bit definition for TIM_DMAR register  *******************/
#define TIM_DMAR_DMAB_Pos         (0U)
#define TIM_DMAR_DMAB_Msk         (0xFFFFUL << TIM_DMAR_DMAB_Pos)               /*!< 0x0000FFFF */
#define TIM_DMAR_DMAB             TIM_DMAR_DMAB_Msk                            /*!<DMA register for burst accesses                    */

/*******************  Bit definition for TIM_OR register  *********************/
#define TIM_OR_TI1_RMP_Pos        (0U)
#define TIM_OR_TI1_RMP_Msk        (0x3UL << TIM_OR_TI1_RMP_Pos)                 /*!< 0x00000003 */
#define TIM_OR_TI1_RMP            TIM_OR_TI1_RMP_Msk                           /*!< TI1_RMP[1:0] bits (TIM11 Input Capture 1 remap) */
#define TIM_OR_TI1_RMP_0          (0x1UL << TIM_OR_TI1_RMP_Pos)                 /*!< 0x00000001 */
#define TIM_OR_TI1_RMP_1          (0x2UL << TIM_OR_TI1_RMP_Pos)                 /*!< 0x00000002 */

#define TIM_OR_TI4_RMP_Pos        (6U)
#define TIM_OR_TI4_RMP_Msk        (0x3UL << TIM_OR_TI4_RMP_Pos)                 /*!< 0x000000C0 */
#define TIM_OR_TI4_RMP            TIM_OR_TI4_RMP_Msk                           /*!<TI4_RMP[1:0] bits (TIM5 Input 4 remap)             */
#define TIM_OR_TI4_RMP_0          (0x1UL << TIM_OR_TI4_RMP_Pos)                 /*!< 0x0040 */
#define TIM_OR_TI4_RMP_1          (0x2UL << TIM_OR_TI4_RMP_Pos)                 /*!< 0x0080 */
#define TIM_OR_ITR1_RMP_Pos       (10U)
#define TIM_OR_ITR1_RMP_Msk       (0x3UL << TIM_OR_ITR1_RMP_Pos)                /*!< 0x00000C00 */
#define TIM_OR_ITR1_RMP           TIM_OR_ITR1_RMP_Msk                          /*!<ITR1_RMP[1:0] bits (TIM2 Internal trigger 1 remap) */
#define TIM_OR_ITR1_RMP_0         (0x1UL << TIM_OR_ITR1_RMP_Pos)                /*!< 0x0400 */
#define TIM_OR_ITR1_RMP_1         (0x2UL << TIM_OR_ITR1_RMP_Pos)                /*!< 0x0800 */


/******************************************************************************/
/*                                                                            */
/*         Universal Synchronous Asynchronous Receiver Transmitter            */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for USART_SR register  *******************/
#define USART_SR_PE_Pos               (0U)
#define USART_SR_PE_Msk               (0x1UL << USART_SR_PE_Pos)                /*!< 0x00000001 */
#define USART_SR_PE                   USART_SR_PE_Msk                          /*!<Parity Error                 */
#define USART_SR_FE_Pos               (1U)
#define USART_SR_FE_Msk               (0x1UL << USART_SR_FE_Pos)                /*!< 0x00000002 */
#define USART_SR_FE                   USART_SR_FE_Msk                          /*!<Framing Error                */
#define USART_SR_NE_Pos               (2U)
#define USART_SR_NE_Msk               (0x1UL << USART_SR_NE_Pos)                /*!< 0x00000004 */
#define USART_SR_NE                   USART_SR_NE_Msk                          /*!<Noise Error Flag             */
#define USART_SR_ORE_Pos              (3U)
#define USART_SR_ORE_Msk              (0x1UL << USART_SR_ORE_Pos)               /*!< 0x00000008 */
#define USART_SR_ORE                  USART_SR_ORE_Msk                         /*!<OverRun Error                */
#define USART_SR_IDLE_Pos             (4U)
#define USART_SR_IDLE_Msk             (0x1UL << USART_SR_IDLE_Pos)              /*!< 0x00000010 */
#define USART_SR_IDLE                 USART_SR_IDLE_Msk                        /*!<IDLE line detected           */
#define USART_SR_RXNE_Pos             (5U)
#define USART_SR_RXNE_Msk             (0x1UL << USART_SR_RXNE_Pos)              /*!< 0x00000020 */
#define USART_SR_RXNE                 USART_SR_RXNE_Msk                        /*!<Read Data Register Not Empty */
#define USART_SR_TC_Pos               (6U)
#define USART_SR_TC_Msk               (0x1UL << USART_SR_TC_Pos)                /*!< 0x00000040 */
#define USART_SR_TC                   USART_SR_TC_Msk                          /*!<Transmission Complete        */
#define USART_SR_TXE_Pos              (7U)
#define USART_SR_TXE_Msk              (0x1UL << USART_SR_TXE_Pos)               /*!< 0x00000080 */
#define USART_SR_TXE                  USART_SR_TXE_Msk                         /*!<Transmit Data Register Empty */
#define USART_SR_LBD_Pos              (8U)
#define USART_SR_LBD_Msk              (0x1UL << USART_SR_LBD_Pos)               /*!< 0x00000100 */
#define USART_SR_LBD                  USART_SR_LBD_Msk                         /*!<LIN Break Detection Flag     */
#define USART_SR_CTS_Pos              (9U)
#define USART_SR_CTS_Msk              (0x1UL << USART_SR_CTS_Pos)               /*!< 0x00000200 */
#define USART_SR_CTS                  USART_SR_CTS_Msk                         /*!<CTS Flag                     */

/*******************  Bit definition for USART_DR register  *******************/
#define USART_DR_DR_Pos               (0U)
#define USART_DR_DR_Msk               (0x1FFUL << USART_DR_DR_Pos)              /*!< 0x000001FF */
#define USART_DR_DR                   USART_DR_DR_Msk                          /*!<Data value */

/******************  Bit definition for USART_BRR register  *******************/
#define USART_BRR_DIV_Fraction_Pos    (0U)
#define USART_BRR_DIV_Fraction_Msk    (0xFUL << USART_BRR_DIV_Fraction_Pos)     /*!< 0x0000000F */
#define USART_BRR_DIV_Fraction        USART_BRR_DIV_Fraction_Msk               /*!<Fraction of USARTDIV */
#define USART_BRR_DIV_Mantissa_Pos    (4U)
#define USART_BRR_DIV_Mantissa_Msk    (0xFFFUL << USART_BRR_DIV_Mantissa_Pos)   /*!< 0x0000FFF0 */
#define USART_BRR_DIV_Mantissa        USART_BRR_DIV_Mantissa_Msk               /*!<Mantissa of USARTDIV */

/******************  Bit definition for USART_CR1 register  *******************/
#define USART_CR1_SBK_Pos             (0U)
#define USART_CR1_SBK_Msk             (0x1UL << USART_CR1_SBK_Pos)              /*!< 0x00000001 */
#define USART_CR1_SBK                 USART_CR1_SBK_Msk                        /*!<Send Break                             */
#define USART_CR1_RWU_Pos             (1U)
#define USART_CR1_RWU_Msk             (0x1UL << USART_CR1_RWU_Pos)              /*!< 0x00000002 */
#define USART_CR1_RWU                 USART_CR1_RWU_Msk                        /*!<Receiver wakeup                        */
#define USART_CR1_RE_Pos              (2U)
#define USART_CR1_RE_Msk              (0x1UL << USART_CR1_RE_Pos)               /*!< 0x00000004 */
#define USART_CR1_RE                  USART_CR1_RE_Msk                         /*!<Receiver Enable                        */
#define USART_CR1_TE_Pos              (3U)
#define USART_CR1_TE_Msk              (0x1UL << USART_CR1_TE_Pos)               /*!< 0x00000008 */
#define USART_CR1_TE                  USART_CR1_TE_Msk                         /*!<Transmitter Enable                     */
#define USART_CR1_IDLEIE_Pos          (4U)
#define USART_CR1_IDLEIE_Msk          (0x1UL << USART_CR1_IDLEIE_Pos)           /*!< 0x00000010 */
#define USART_CR1_IDLEIE              USART_CR1_IDLEIE_Msk                     /*!<IDLE Interrupt Enable                  */
#define USART_CR1_RXNEIE_Pos          (5U)
#define USART_CR1_RXNEIE_Msk          (0x1UL << USART_CR1_RXNEIE_Pos)           /*!< 0x00000020 */
#define USART_CR1_RXNEIE              USART_CR1_RXNEIE_Msk                     /*!<RXNE Interrupt Enable                  */
#define USART_CR1_TCIE_Pos            (6U)
#define USART_CR1_TCIE_Msk            (0x1UL << USART_CR1_TCIE_Pos)             /*!< 0x00000040 */
#define USART_CR1_TCIE                USART_CR1_TCIE_Msk                       /*!<Transmission Complete Interrupt Enable */
#define USART_CR1_TXEIE_Pos           (7U)
#define USART_CR1_TXEIE_Msk           (0x1UL << USART_CR1_TXEIE_Pos)            /*!< 0x00000080 */
#define USART_CR1_TXEIE               USART_CR1_TXEIE_Msk                      /*!<TXE Interrupt Enable                   */
#define USART_CR1_PEIE_Pos            (8U)
#define USART_CR1_PEIE_Msk            (0x1UL << USART_CR1_PEIE_Pos)             /*!< 0x00000100 */
#define USART_CR1_PEIE                USART_CR1_PEIE_Msk                       /*!<PE Interrupt Enable                    */
#define USART_CR1_PS_Pos              (9U)
#define USART_CR1_PS_Msk              (0x1UL << USART_CR1_PS_Pos)               /*!< 0x00000200 */
#define USART_CR1_PS                  USART_CR1_PS_Msk                         /*!<Parity Selection                       */
#define USART_CR1_PCE_Pos             (10U)
#define USART_CR1_PCE_Msk             (0x1UL << USART_CR1_PCE_Pos)              /*!< 0x00000400 */
#define USART_CR1_PCE                 USART_CR1_PCE_Msk                        /*!<Parity Control Enable                  */
#define USART_CR1_WAKE_Pos            (11U)
#define USART_CR1_WAKE_Msk            (0x1UL << USART_CR1_WAKE_Pos)             /*!< 0x00000800 */
#define USART_CR1_WAKE                USART_CR1_WAKE_Msk                       /*!<Wakeup method                          */
#define USART_CR1_M_Pos               (12U)
#define USART_CR1_M_Msk               (0x1UL << USART_CR1_M_Pos)                /*!< 0x00001000 */
#define USART_CR1_M                   USART_CR1_M_Msk                          /*!<Word length                            */
#define USART_CR1_UE_Pos              (13U)
#define USART_CR1_UE_Msk              (0x1UL << USART_CR1_UE_Pos)               /*!< 0x00002000 */
#define USART_CR1_UE                  USART_CR1_UE_Msk                         /*!<USART Enable                           */
#define USART_CR1_OVER8_Pos           (15U)
#define USART_CR1_OVER8_Msk           (0x1UL << USART_CR1_OVER8_Pos)            /*!< 0x00008000 */
#define USART_CR1_OVER8               USART_CR1_OVER8_Msk                      /*!<USART Oversampling by 8 enable         */

/******************  Bit definition for USART_CR2 register  *******************/
#define USART_CR2_ADD_Pos             (0U)
#define USART_CR2_ADD_Msk             (0xFUL << USART_CR2_ADD_Pos)              /*!< 0x0000000F */
#define USART_CR2_ADD                 USART_CR2_ADD_Msk                        /*!<Address of the USART node            */
#define USART_CR2_LBDL_Pos            (5U)
#define USART_CR2_LBDL_Msk            (0x1UL << USART_CR2_LBDL_Pos)             /*!< 0x00000020 */
#define USART_CR2_LBDL                USART_CR2_LBDL_Msk                       /*!<LIN Break Detection Length           */
#define USART_CR2_LBDIE_Pos           (6U)
#define USART_CR2_LBDIE_Msk           (0x1UL << USART_CR2_LBDIE_Pos)            /*!< 0x00000040 */
#define USART_CR2_LBDIE               USART_CR2_LBDIE_Msk                      /*!<LIN Break Detection Interrupt Enable */
#define USART_CR2_LBCL_Pos            (8U)
#define USART_CR2_LBCL_Msk            (0x1UL << USART_CR2_LBCL_Pos)             /*!< 0x00000100 */
#define USART_CR2_LBCL                USART_CR2_LBCL_Msk                       /*!<Last Bit Clock pulse                 */
#define USART_CR2_CPHA_Pos            (9U)
#define USART_CR2_CPHA_Msk            (0x1UL << USART_CR2_CPHA_Pos)             /*!< 0x00000200 */
#define USART_CR2_CPHA                USART_CR2_CPHA_Msk                       /*!<Clock Phase                          */
#define USART_CR2_CPOL_Pos            (10U)
#define USART_CR2_CPOL_Msk            (0x1UL << USART_CR2_CPOL_Pos)             /*!< 0x00000400 */
#define USART_CR2_CPOL                USART_CR2_CPOL_Msk                       /*!<Clock Polarity                       */
#define USART_CR2_CLKEN_Pos           (11U)
#define USART_CR2_CLKEN_Msk           (0x1UL << USART_CR2_CLKEN_Pos)            /*!< 0x00000800 */
#define USART_CR2_CLKEN               USART_CR2_CLKEN_Msk                      /*!<Clock Enable                         */

#define USART_CR2_STOP_Pos            (12U)
#define USART_CR2_STOP_Msk            (0x3UL << USART_CR2_STOP_Pos)             /*!< 0x00003000 */
#define USART_CR2_STOP                USART_CR2_STOP_Msk                       /*!<STOP[1:0] bits (STOP bits) */
#define USART_CR2_STOP_0              (0x1UL << USART_CR2_STOP_Pos)             /*!< 0x1000 */
#define USART_CR2_STOP_1              (0x2UL << USART_CR2_STOP_Pos)             /*!< 0x2000 */

#define USART_CR2_LINEN_Pos           (14U)
#define USART_CR2_LINEN_Msk           (0x1UL << USART_CR2_LINEN_Pos)            /*!< 0x00004000 */
#define USART_CR2_LINEN               USART_CR2_LINEN_Msk                      /*!<LIN mode enable */

/******************  Bit definition for USART_CR3 register  *******************/
#define USART_CR3_EIE_Pos             (0U)
#define USART_CR3_EIE_Msk             (0x1UL << USART_CR3_EIE_Pos)              /*!< 0x00000001 */
#define USART_CR3_EIE                 USART_CR3_EIE_Msk                        /*!<Error Interrupt Enable      */
#define USART_CR3_IREN_Pos            (1U)
#define USART_CR3_IREN_Msk            (0x1UL << USART_CR3_IREN_Pos)             /*!< 0x00000002 */
#define USART_CR3_IREN                USART_CR3_IREN_Msk                       /*!<IrDA mode Enable            */
#define USART_CR3_IRLP_Pos            (2U)
#define USART_CR3_IRLP_Msk            (0x1UL << USART_CR3_IRLP_Pos)             /*!< 0x00000004 */
#define USART_CR3_IRLP                USART_CR3_IRLP_Msk                       /*!<IrDA Low-Power              */
#define USART_CR3_HDSEL_Pos           (3U)
#define USART_CR3_HDSEL_Msk           (0x1UL << USART_CR3_HDSEL_Pos)            /*!< 0x00000008 */
#define USART_CR3_HDSEL               USART_CR3_HDSEL_Msk                      /*!<Half-Duplex Selection       */
#define USART_CR3_NACK_Pos            (4U)
#define USART_CR3_NACK_Msk            (0x1UL << USART_CR3_NACK_Pos)             /*!< 0x00000010 */
#define USART_CR3_NACK                USART_CR3_NACK_Msk                       /*!<Smartcard NACK enable       */
#define USART_CR3_SCEN_Pos            (5U)
#define USART_CR3_SCEN_Msk            (0x1UL << USART_CR3_SCEN_Pos)             /*!< 0x00000020 */
#define USART_CR3_SCEN                USART_CR3_SCEN_Msk                       /*!<Smartcard mode enable       */
#define USART_CR3_DMAR_Pos            (6U)
#define USART_CR3_DMAR_Msk            (0x1UL << USART_CR3_DMAR_Pos)             /*!< 0x00000040 */
#define USART_CR3_DMAR                USART_CR3_DMAR_Msk                       /*!<DMA Enable Receiver         */
#define USART_CR3_DMAT_Pos            (7U)
#define USART_CR3_DMAT_Msk            (0x1UL << USART_CR3_DMAT_Pos)             /*!< 0x00000080 */
#define USART_CR3_DMAT                USART_CR3_DMAT_Msk                       /*!<DMA Enable Transmitter      */
#define USART_CR3_RTSE_Pos            (8U)
#define USART_CR3_RTSE_Msk            (0x1UL << USART_CR3_RTSE_Pos)             /*!< 0x00000100 */
#define USART_CR3_RTSE                USART_CR3_RTSE_Msk                       /*!<RTS Enable                  */
#define USART_CR3_CTSE_Pos            (9U)
#define USART_CR3_CTSE_Msk            (0x1UL << USART_CR3_CTSE_Pos)             /*!< 0x00000200 */
#define USART_CR3_CTSE                USART_CR3_CTSE_Msk                       /*!<CTS Enable                  */
#define USART_CR3_CTSIE_Pos           (10U)
#define USART_CR3_CTSIE_Msk           (0x1UL << USART_CR3_CTSIE_Pos)            /*!< 0x00000400 */
#define USART_CR3_CTSIE               USART_CR3_CTSIE_Msk                      /*!<CTS Interrupt Enable        */
#define USART_CR3_ONEBIT_Pos          (11U)
#define USART_CR3_ONEBIT_Msk          (0x1UL << USART_CR3_ONEBIT_Pos)           /*!< 0x00000800 */
#define USART_CR3_ONEBIT              USART_CR3_ONEBIT_Msk                     /*!<USART One bit method enable */

/******************  Bit definition for USART_GTPR register  ******************/
#define USART_GTPR_PSC_Pos            (0U)
#define USART_GTPR_PSC_Msk            (0xFFUL << USART_GTPR_PSC_Pos)            /*!< 0x000000FF */
#define USART_GTPR_PSC                USART_GTPR_PSC_Msk                       /*!<PSC[7:0] bits (Prescaler value) */
#define USART_GTPR_PSC_0              (0x01UL << USART_GTPR_PSC_Pos)            /*!< 0x0001 */
#define USART_GTPR_PSC_1              (0x02UL << USART_GTPR_PSC_Pos)            /*!< 0x0002 */
#define USART_GTPR_PSC_2              (0x04UL << USART_GTPR_PSC_Pos)            /*!< 0x0004 */
#define USART_GTPR_PSC_3              (0x08UL << USART_GTPR_PSC_Pos)            /*!< 0x0008 */
#define USART_GTPR_PSC_4              (0x10UL << USART_GTPR_PSC_Pos)            /*!< 0x0010 */
#define USART_GTPR_PSC_5              (0x20UL << USART_GTPR_PSC_Pos)            /*!< 0x0020 */
#define USART_GTPR_PSC_6              (0x40UL << USART_GTPR_PSC_Pos)            /*!< 0x0040 */
#define USART_GTPR_PSC_7              (0x80UL << USART_GTPR_PSC_Pos)            /*!< 0x0080 */

#define USART_GTPR_GT_Pos             (8U)
#define USART_GTPR_GT_Msk             (0xFFUL << USART_GTPR_GT_Pos)             /*!< 0x0000FF00 */
#define USART_GTPR_GT                 USART_GTPR_GT_Msk                        /*!<Guard time value */


/*
 * @brief Specific devices reset values definitions
 */
#define RCC_PLLCFGR_RST_VALUE              0x24003010U
#define RCC_PLLI2SCFGR_RST_VALUE           0x20003000U

#define RCC_MAX_FREQUENCY           168000000U         /*!< Max frequency of family in Hz*/
#define RCC_MAX_FREQUENCY_SCALE1    RCC_MAX_FREQUENCY  /*!< Maximum frequency for system clock at power scale1, in Hz */
#define RCC_MAX_FREQUENCY_SCALE2    144000000U         /*!< Maximum frequency for system clock at power scale2, in Hz */
#define RCC_PLLVCO_OUTPUT_MIN       100000000U       /*!< Frequency min for PLLVCO output, in Hz */
#define RCC_PLLVCO_INPUT_MIN           950000U       /*!< Frequency min for PLLVCO input, in Hz  */
#define RCC_PLLVCO_INPUT_MAX          2100000U       /*!< Frequency max for PLLVCO input, in Hz  */
#define RCC_PLLVCO_OUTPUT_MAX       432000000U       /*!< Frequency max for PLLVCO output, in Hz */

#define RCC_PLLN_MIN_VALUE                 50U
#define RCC_PLLN_MAX_VALUE                432U

#define FLASH_SCALE1_LATENCY1_FREQ   30000000U      /*!< HCLK frequency to set FLASH latency 1 in power scale 1  */
#define FLASH_SCALE1_LATENCY2_FREQ   60000000U      /*!< HCLK frequency to set FLASH latency 2 in power scale 1  */
#define FLASH_SCALE1_LATENCY3_FREQ   90000000U      /*!< HCLK frequency to set FLASH latency 3 in power scale 1  */
#define FLASH_SCALE1_LATENCY4_FREQ   120000000U     /*!< HCLK frequency to set FLASH latency 4 in power scale 1  */
#define FLASH_SCALE1_LATENCY5_FREQ   150000000U     /*!< HCLK frequency to set FLASH latency 5 in power scale 1  */

#define FLASH_SCALE2_LATENCY1_FREQ   30000000U      /*!< HCLK frequency to set FLASH latency 1 in power scale 2  */
#define FLASH_SCALE2_LATENCY2_FREQ   60000000U      /*!< HCLK frequency to set FLASH latency 2 in power scale 2  */
#define FLASH_SCALE2_LATENCY3_FREQ   90000000U      /*!< HCLK frequency to set FLASH latency 3 in power scale 2  */
#define FLASH_SCALE2_LATENCY4_FREQ   12000000U      /*!< HCLK frequency to set FLASH latency 4 in power scale 2  */


/**************************************************************************************************************
 * 																											  *
 * 												 Macro for IRQs					 							  *
 * 																											  *
 **************************************************************************************************************/

/**
 * @brief STM32F4XX Interrupt Number Definition, according to the selected device
 *        in @ref Library_configuration_section
 */
typedef enum
{
/******  Cortex-M4 Processor Exceptions Numbers ****************************************************************/
  NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                                          */
  MemoryManagement_IRQn       = -12,    /*!< 4 Cortex-M4 Memory Management Interrupt                           */
  BusFault_IRQn               = -11,    /*!< 5 Cortex-M4 Bus Fault Interrupt                                   */
  UsageFault_IRQn             = -10,    /*!< 6 Cortex-M4 Usage Fault Interrupt                                 */
  SVCall_IRQn                 = -5,     /*!< 11 Cortex-M4 SV Call Interrupt                                    */
  DebugMonitor_IRQn           = -4,     /*!< 12 Cortex-M4 Debug Monitor Interrupt                              */
  PendSV_IRQn                 = -2,     /*!< 14 Cortex-M4 Pend SV Interrupt                                    */
  SysTick_IRQn                = -1,     /*!< 15 Cortex-M4 System Tick Interrupt                                */
/******  STM32 specific Interrupt Numbers **********************************************************************/
  WWDG_IRQn                   = 0,      /*!< Window WatchDog Interrupt                                         */
  PVD_IRQn                    = 1,      /*!< PVD through EXTI Line detection Interrupt                         */
  TAMP_STAMP_IRQn             = 2,      /*!< Tamper and TimeStamp interrupts through the EXTI line             */
  RTC_WKUP_IRQn               = 3,      /*!< RTC Wakeup interrupt through the EXTI line                        */
  FLASH_IRQn                  = 4,      /*!< FLASH global Interrupt                                            */
  RCC_IRQn                    = 5,      /*!< RCC global Interrupt                                              */
  EXTI0_IRQn                  = 6,      /*!< EXTI Line0 Interrupt                                              */
  EXTI1_IRQn                  = 7,      /*!< EXTI Line1 Interrupt                                              */
  EXTI2_IRQn                  = 8,      /*!< EXTI Line2 Interrupt                                              */
  EXTI3_IRQn                  = 9,      /*!< EXTI Line3 Interrupt                                              */
  EXTI4_IRQn                  = 10,     /*!< EXTI Line4 Interrupt                                              */
  DMA1_Stream0_IRQn           = 11,     /*!< DMA1 Stream 0 global Interrupt                                    */
  DMA1_Stream1_IRQn           = 12,     /*!< DMA1 Stream 1 global Interrupt                                    */
  DMA1_Stream2_IRQn           = 13,     /*!< DMA1 Stream 2 global Interrupt                                    */
  DMA1_Stream3_IRQn           = 14,     /*!< DMA1 Stream 3 global Interrupt                                    */
  DMA1_Stream4_IRQn           = 15,     /*!< DMA1 Stream 4 global Interrupt                                    */
  DMA1_Stream5_IRQn           = 16,     /*!< DMA1 Stream 5 global Interrupt                                    */
  DMA1_Stream6_IRQn           = 17,     /*!< DMA1 Stream 6 global Interrupt                                    */
  ADC_IRQn                    = 18,     /*!< ADC1, ADC2 and ADC3 global Interrupts                             */
  CAN1_TX_IRQn                = 19,     /*!< CAN1 TX Interrupt                                                 */
  CAN1_RX0_IRQn               = 20,     /*!< CAN1 RX0 Interrupt                                                */
  CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                                                */
  CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                                                */
  EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                                     */
  TIM1_BRK_TIM9_IRQn          = 24,     /*!< TIM1 Break interrupt and TIM9 global interrupt                    */
  TIM1_UP_TIM10_IRQn          = 25,     /*!< TIM1 Update Interrupt and TIM10 global interrupt                  */
  TIM1_TRG_COM_TIM11_IRQn     = 26,     /*!< TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt */
  TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                                    */
  TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                             */
  TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                                             */
  TIM4_IRQn                   = 30,     /*!< TIM4 global Interrupt                                             */
  I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                              */
  I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                              */
  I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                              */
  I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                              */
  SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                             */
  SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                             */
  USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                                           */
  USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                                           */
  USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                                           */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                                   */
  RTC_Alarm_IRQn              = 41,     /*!< RTC Alarm (A and B) through EXTI Line Interrupt                   */
  OTG_FS_WKUP_IRQn            = 42,     /*!< USB OTG FS Wakeup through EXTI line interrupt                     */
  TIM8_BRK_TIM12_IRQn         = 43,     /*!< TIM8 Break Interrupt and TIM12 global interrupt                   */
  TIM8_UP_TIM13_IRQn          = 44,     /*!< TIM8 Update Interrupt and TIM13 global interrupt                  */
  TIM8_TRG_COM_TIM14_IRQn     = 45,     /*!< TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt */
  TIM8_CC_IRQn                = 46,     /*!< TIM8 Capture Compare global interrupt                             */
  DMA1_Stream7_IRQn           = 47,     /*!< DMA1 Stream7 Interrupt                                            */
  FSMC_IRQn                   = 48,     /*!< FSMC global Interrupt                                             */
  SDIO_IRQn                   = 49,     /*!< SDIO global Interrupt                                             */
  TIM5_IRQn                   = 50,     /*!< TIM5 global Interrupt                                             */
  SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                             */
  UART4_IRQn                  = 52,     /*!< UART4 global Interrupt                                            */
  UART5_IRQn                  = 53,     /*!< UART5 global Interrupt                                            */
  TIM6_DAC_IRQn               = 54,     /*!< TIM6 global and DAC1&2 underrun error  interrupts                 */
  TIM7_IRQn                   = 55,     /*!< TIM7 global interrupt                                             */
  DMA2_Stream0_IRQn           = 56,     /*!< DMA2 Stream 0 global Interrupt                                    */
  DMA2_Stream1_IRQn           = 57,     /*!< DMA2 Stream 1 global Interrupt                                    */
  DMA2_Stream2_IRQn           = 58,     /*!< DMA2 Stream 2 global Interrupt                                    */
  DMA2_Stream3_IRQn           = 59,     /*!< DMA2 Stream 3 global Interrupt                                    */
  DMA2_Stream4_IRQn           = 60,     /*!< DMA2 Stream 4 global Interrupt                                    */
  ETH_IRQn                    = 61,     /*!< Ethernet global Interrupt                                         */
  ETH_WKUP_IRQn               = 62,     /*!< Ethernet Wakeup through EXTI line Interrupt                       */
  CAN2_TX_IRQn                = 63,     /*!< CAN2 TX Interrupt                                                 */
  CAN2_RX0_IRQn               = 64,     /*!< CAN2 RX0 Interrupt                                                */
  CAN2_RX1_IRQn               = 65,     /*!< CAN2 RX1 Interrupt                                                */
  CAN2_SCE_IRQn               = 66,     /*!< CAN2 SCE Interrupt                                                */
  OTG_FS_IRQn                 = 67,     /*!< USB OTG FS global Interrupt                                       */
  DMA2_Stream5_IRQn           = 68,     /*!< DMA2 Stream 5 global interrupt                                    */
  DMA2_Stream6_IRQn           = 69,     /*!< DMA2 Stream 6 global interrupt                                    */
  DMA2_Stream7_IRQn           = 70,     /*!< DMA2 Stream 7 global interrupt                                    */
  USART6_IRQn                 = 71,     /*!< USART6 global interrupt                                           */
  I2C3_EV_IRQn                = 72,     /*!< I2C3 event interrupt                                              */
  I2C3_ER_IRQn                = 73,     /*!< I2C3 error interrupt                                              */
  OTG_HS_EP1_OUT_IRQn         = 74,     /*!< USB OTG HS End Point 1 Out global interrupt                       */
  OTG_HS_EP1_IN_IRQn          = 75,     /*!< USB OTG HS End Point 1 In global interrupt                        */
  OTG_HS_WKUP_IRQn            = 76,     /*!< USB OTG HS Wakeup through EXTI interrupt                          */
  OTG_HS_IRQn                 = 77,     /*!< USB OTG HS global interrupt                                       */
  DCMI_IRQn                   = 78,     /*!< DCMI global interrupt                                             */
  RNG_IRQn                    = 80,     /*!< RNG global Interrupt                                              */
  FPU_IRQn                    = 81      /*!< FPU global interrupt                                               */
} IRQn_Type;
/* Legacy define */
#define  HASH_RNG_IRQn      RNG_IRQn


/*
 *  Priority of IRQ
 *  (The lower the value, the greater the priority of corresponding interrupt)
 */
#define NVIC_PRIOR_0							0
#define NVIC_PRIOR_1							1
#define NVIC_PRIOR_2							2
#define NVIC_PRIOR_3							3
#define NVIC_PRIOR_4							4
#define NVIC_PRIOR_5							5
#define NVIC_PRIOR_6							6
#define NVIC_PRIOR_7							7
#define NVIC_PRIOR_8							8
#define NVIC_PRIOR_9							9
#define NVIC_PRIOR_10							10
#define NVIC_PRIOR_11							11
#define NVIC_PRIOR_12							12
#define NVIC_PRIOR_13							13
#define NVIC_PRIOR_14							14
#define NVIC_PRIOR_15							15


/**************************************************************************************************************
 * 																											  *
 * 											 User Macro Function				 							  *
 * 																											  *
 **************************************************************************************************************/

#define SET_BIT(REG, BIT)     ((REG) |= (BIT))

#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))

#define READ_BIT(REG, BIT)    ((REG) & (BIT))

#define CLEAR_REG(REG)        ((REG) = (0x0))

#define WRITE_REG(REG, VAL)   ((REG) = (VAL))

#define READ_REG(REG)         ((REG))

#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

#define POSITION_VAL(VAL)     (__CLZ(__RBIT(VAL)))

#define SET_FLAG(REG, FLAG)				SET_BIT(REG, FLAG)

#define CLEAR_FLAG(REG, FLAG)			CLEAR_BIT(REG, FLAG)

#define WAIT_FLAG_SET(REG, FLAG)		do{  while( !((REG) & (FLAG)) ); }while(0)

#define WAIT_FLAG_CLEAR(REG, FLAG)		do{  while( ((REG) & (FLAG)) ); }while(0)

#define GET_FLAG(REG, FLAG)				(((REG) & (FLAG)) == (FLAG))


/**************************************************************************************************************
 * 																											  *
 * 										Peripheral Macro defifnitions				 						  *
 * 						(which are too few to include entire Peripheral Header file) 						  *
 * 																											  *
 **************************************************************************************************************/

/**
  * @brief   ACR register byte 0 (Bits[7:0]) base address
  */
#define ACR_BYTE0_ADDRESS           0x40023C00U

/**
  * @brief  Enable the FLASH instruction cache.
  * @retval none
  */
#define FLASH_INSTRUCTION_CACHE_ENABLE()  (FLASH->ACR |= FLASH_ACR_ICEN)

/**
  * @brief  Enable the FLASH data cache.
  * @retval none
  */
#define FLASH_DATA_CACHE_ENABLE()  (FLASH->ACR |= FLASH_ACR_DCEN)

/**
  * @brief  Enable the FLASH prefetch buffer.
  * @retval none
  */
#define FLASH_PREFETCH_BUFFER_ENABLE()  (FLASH->ACR |= FLASH_ACR_PRFTEN)


/** @defgroup FLASH_Latency FLASH Latency
  * @{
  */
#define FLASH_LATENCY_0                FLASH_ACR_LATENCY_0WS   /*!< FLASH Zero Latency cycle      */
#define FLASH_LATENCY_1                FLASH_ACR_LATENCY_1WS   /*!< FLASH One Latency cycle       */
#define FLASH_LATENCY_2                FLASH_ACR_LATENCY_2WS   /*!< FLASH Two Latency cycles      */
#define FLASH_LATENCY_3                FLASH_ACR_LATENCY_3WS   /*!< FLASH Three Latency cycles    */
#define FLASH_LATENCY_4                FLASH_ACR_LATENCY_4WS   /*!< FLASH Four Latency cycles     */
#define FLASH_LATENCY_5                FLASH_ACR_LATENCY_5WS   /*!< FLASH Five Latency cycles     */
#define FLASH_LATENCY_6                FLASH_ACR_LATENCY_6WS   /*!< FLASH Six Latency cycles      */
#define FLASH_LATENCY_7                FLASH_ACR_LATENCY_7WS   /*!< FLASH Seven Latency cycles    */

/**
  * @brief  Set the FLASH Latency.
  * @param  __LATENCY__ FLASH Latency
  *         The value of this parameter depend on device used within the same series
  * @retval none
  */
#define __HAL_FLASH_SET_LATENCY(__LATENCY__) (*(__IO uint8_t *)ACR_BYTE0_ADDRESS = (uint8_t)(__LATENCY__))


/** @defgroup PWREx_Regulator_Voltage_Scale PWREx Regulator Voltage Scale
  * @{
  */
#define PWR_REGULATOR_VOLTAGE_SCALE1         PWR_CR_VOS             /* Scale 1 mode(default value at reset): the maximum value of fHCLK = 168 MHz. */
#define PWR_REGULATOR_VOLTAGE_SCALE2         0x00000000U            /* Scale 2 mode: the maximum value of fHCLK = 144 MHz. */


#define PWR_VOLTAGESCALING_CONFIG(__REGULATOR__) 		do {                                                     \
                                                            __IO uint32_t tmpreg = 0x00U;                        \
                                                            MODIFY_REG(PWR->CR, PWR_CR_VOS, (__REGULATOR__));   \
                                                            /* Delay after an RCC peripheral clock enabling */  \
                                                            tmpreg = READ_BIT(PWR->CR, PWR_CR_VOS);             \
                                                            UNUSED(tmpreg);                                     \
                                                          } while(0U)

/**************************************************************************************************************
 * 																											  *
 * 											Peripheral Header file					 						  *
 * 																											  *
 **************************************************************************************************************/

#include "stm32f407xx_rcc_driver.h"
#include "stm32f407xx_dma_driver.h"
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_tim_driver.h"
#include "stm32f407xx_usart_driver.h"
#include "stm32f407xx_adc_driver.h"
#include "stm32f407xx_dac_driver.h"


#endif /* __STM32F407xx_H */
