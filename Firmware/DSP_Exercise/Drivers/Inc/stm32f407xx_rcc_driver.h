/*
 * stm32f407xx_rcc_driver.h
 *
 *  Created on: 2021. 3. 7.
 *      Author: Ganghyeok Lim
 */

#ifndef INC_STM32F407XX_RCC_DRIVER_H_
#define INC_STM32F407XX_RCC_DRIVER_H_


#include "stm32f407xx.h"


/**
  * @brief  RCC PLL configuration structure definition
  */
typedef struct
{
  uint32_t PLLState;   /*!< The new state of the PLL.
                            This parameter can be a value of @ref RCC_PLL_Config                      */

  uint32_t PLLSource;  /*!< RCC_PLLSource: PLL entry clock source.
                            This parameter must be a value of @ref RCC_PLL_Clock_Source               */

  uint32_t PLLM;       /*!< PLLM: Division factor for PLL VCO input clock.
                            This parameter must be a number between Min_Data = 0 and Max_Data = 63    */

  uint32_t PLLN;       /*!< PLLN: Multiplication factor for PLL VCO output clock.
                            This parameter must be a number between Min_Data = 50 and Max_Data = 432
                            except for STM32F411xE devices where the Min_Data = 192 */

  uint32_t PLLP;       /*!< PLLP: Division factor for main system clock (SYSCLK).
                            This parameter must be a value of @ref RCC_PLLP_Clock_Divider             */

  uint32_t PLLQ;       /*!< PLLQ: Division factor for OTG FS, SDIO and RNG clocks.
                            This parameter must be a number between Min_Data = 2 and Max_Data = 15    */
}RCC_PLLInitTypeDef;


/**
  * @brief  RCC Internal/External Oscillator (HSE, HSI, LSE and LSI) configuration structure definition
  */
typedef struct
{
  uint32_t OscillatorType;       /*!< The oscillators to be configured.
                                      This parameter can be a value of @ref RCC_Oscillator_Type                   */

  uint32_t HSEState;             /*!< The new state of the HSE.
                                      This parameter can be a value of @ref RCC_HSE_Config                        */

  uint32_t LSEState;             /*!< The new state of the LSE.
                                      This parameter can be a value of @ref RCC_LSE_Config                        */

  uint32_t HSIState;             /*!< The new state of the HSI.
                                      This parameter can be a value of @ref RCC_HSI_Config                        */

  uint32_t HSICalibrationValue;  /*!< The HSI calibration trimming value (default is RCC_HSICALIBRATION_DEFAULT).
                                       This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x1F */

  uint32_t LSIState;             /*!< The new state of the LSI.
                                      This parameter can be a value of @ref RCC_LSI_Config                        */

  RCC_PLLInitTypeDef PLL;        /*!< PLL structure parameters                                                    */
}RCC_OscInitTypeDef;


/**
  * @brief  RCC System, AHB and APB busses clock configuration structure definition
  */
typedef struct
{
  uint32_t ClockType;             /*!< The clock to be configured.
                                       This parameter can be a value of @ref RCC_System_Clock_Type      */

  uint32_t SYSCLKSource;          /*!< The clock source (SYSCLKS) used as system clock.
                                       This parameter can be a value of @ref RCC_System_Clock_Source    */

  uint32_t AHBCLKDivider;         /*!< The AHB clock (HCLK) divider. This clock is derived from the system clock (SYSCLK).
                                       This parameter can be a value of @ref RCC_AHB_Clock_Source       */

  uint32_t APB1CLKDivider;        /*!< The APB1 clock (PCLK1) divider. This clock is derived from the AHB clock (HCLK).
                                       This parameter can be a value of @ref RCC_APB1_APB2_Clock_Source */

  uint32_t APB2CLKDivider;        /*!< The APB2 clock (PCLK2) divider. This clock is derived from the AHB clock (HCLK).
                                       This parameter can be a value of @ref RCC_APB1_APB2_Clock_Source */

}RCC_ClkInitTypeDef;


/** @defgroup RCC_Oscillator_Type Oscillator Type
  * @{
  */
#define RCC_OSCILLATORTYPE_NONE            0x00000000U
#define RCC_OSCILLATORTYPE_HSE             0x00000001U
#define RCC_OSCILLATORTYPE_HSI             0x00000002U
#define RCC_OSCILLATORTYPE_LSE             0x00000004U
#define RCC_OSCILLATORTYPE_LSI             0x00000008U

/** @defgroup RCC_HSE_Config HSE Config
  * @{
  */
#define RCC_HSE_OFF                      0x00000000U
#define RCC_HSE_ON                       RCC_CR_HSEON
#define RCC_HSE_BYPASS                   ((uint32_t)(RCC_CR_HSEBYP | RCC_CR_HSEON))

/** @defgroup RCC_LSE_Config LSE Config
  * @{
  */
#define RCC_LSE_OFF                    0x00000000U
#define RCC_LSE_ON                     RCC_BDCR_LSEON
#define RCC_LSE_BYPASS                 ((uint32_t)(RCC_BDCR_LSEBYP | RCC_BDCR_LSEON))

/** @defgroup RCC_HSI_Config HSI Config
  * @{
  */
#define RCC_HSI_OFF                      ((uint8_t)0x00)
#define RCC_HSI_ON                       ((uint8_t)0x01)

#define RCC_HSICALIBRATION_DEFAULT       0x10U         /* Default HSI calibration trimming value */

/** @defgroup RCC_LSI_Config LSI Config
  * @{
  */
#define RCC_LSI_OFF                      ((uint8_t)0x00)
#define RCC_LSI_ON                       ((uint8_t)0x01)

/** @defgroup RCC_PLL_Config PLL Config
  * @{
  */
#define RCC_PLL_NONE                      ((uint8_t)0x00)
#define RCC_PLL_OFF                       ((uint8_t)0x01)
#define RCC_PLL_ON                        ((uint8_t)0x02)

/** @defgroup RCC_PLLP_Clock_Divider PLLP Clock Divider
  * @{
  */
#define RCC_PLLP_DIV2                  0x00000002U
#define RCC_PLLP_DIV4                  0x00000004U
#define RCC_PLLP_DIV6                  0x00000006U
#define RCC_PLLP_DIV8                  0x00000008U

/** @defgroup RCC_PLL_Clock_Source PLL Clock Source
  * @{
  */
#define RCC_PLLSOURCE_HSI                RCC_PLLCFGR_PLLSRC_HSI
#define RCC_PLLSOURCE_HSE                RCC_PLLCFGR_PLLSRC_HSE

/** @defgroup RCC_System_Clock_Type System Clock Type
  * @{
  */
#define RCC_CLOCKTYPE_SYSCLK             0x00000001U
#define RCC_CLOCKTYPE_HCLK               0x00000002U
#define RCC_CLOCKTYPE_PCLK1              0x00000004U
#define RCC_CLOCKTYPE_PCLK2              0x00000008U

/** @defgroup RCC_System_Clock_Source System Clock Source
  * @{
  */
#define RCC_SYSCLKSOURCE_HSI             RCC_CFGR_SW_HSI
#define RCC_SYSCLKSOURCE_HSE             RCC_CFGR_SW_HSE
#define RCC_SYSCLKSOURCE_PLLCLK          RCC_CFGR_SW_PLL

/** @defgroup RCC_System_Clock_Source_Status System Clock Source Status
  * @{
  */
#define RCC_SYSCLKSOURCE_STATUS_HSI     RCC_CFGR_SWS_HSI   /*!< HSI used as system clock */
#define RCC_SYSCLKSOURCE_STATUS_HSE     RCC_CFGR_SWS_HSE   /*!< HSE used as system clock */
#define RCC_SYSCLKSOURCE_STATUS_PLLCLK  RCC_CFGR_SWS_PLL   /*!< PLL used as system clock */

/** @defgroup RCC_AHB_Clock_Source AHB Clock Source
  * @{
  */
#define RCC_SYSCLK_DIV1                  RCC_CFGR_HPRE_DIV1
#define RCC_SYSCLK_DIV2                  RCC_CFGR_HPRE_DIV2
#define RCC_SYSCLK_DIV4                  RCC_CFGR_HPRE_DIV4
#define RCC_SYSCLK_DIV8                  RCC_CFGR_HPRE_DIV8
#define RCC_SYSCLK_DIV16                 RCC_CFGR_HPRE_DIV16
#define RCC_SYSCLK_DIV64                 RCC_CFGR_HPRE_DIV64
#define RCC_SYSCLK_DIV128                RCC_CFGR_HPRE_DIV128
#define RCC_SYSCLK_DIV256                RCC_CFGR_HPRE_DIV256
#define RCC_SYSCLK_DIV512                RCC_CFGR_HPRE_DIV512

/** @defgroup RCC_APB1_APB2_Clock_Source APB1/APB2 Clock Source
  * @{
  */
#define RCC_HCLK_DIV1                    RCC_CFGR_PPRE1_DIV1
#define RCC_HCLK_DIV2                    RCC_CFGR_PPRE1_DIV2
#define RCC_HCLK_DIV4                    RCC_CFGR_PPRE1_DIV4
#define RCC_HCLK_DIV8                    RCC_CFGR_PPRE1_DIV8
#define RCC_HCLK_DIV16                   RCC_CFGR_PPRE1_DIV16

/** @defgroup RCC_MCO_Index MCO Index
  * @{
  */
#define RCC_MCO1                         0x00000000U
#define RCC_MCO2                         0x00000001U

/** @defgroup RCC_MCO1_Clock_Source MCO1 Clock Source
  * @{
  */
#define RCC_MCO1SOURCE_HSI               0x00000000U
#define RCC_MCO1SOURCE_LSE               RCC_CFGR_MCO1_0
#define RCC_MCO1SOURCE_HSE               RCC_CFGR_MCO1_1
#define RCC_MCO1SOURCE_PLLCLK            RCC_CFGR_MCO1

/** @defgroup RCC_MCOx_Clock_Prescaler MCOx Clock Prescaler
  * @{
  */
#define RCC_MCODIV_1                    0x00000000U
#define RCC_MCODIV_2                    RCC_CFGR_MCO1PRE_2
#define RCC_MCODIV_3                    ((uint32_t)RCC_CFGR_MCO1PRE_0 | RCC_CFGR_MCO1PRE_2)
#define RCC_MCODIV_4                    ((uint32_t)RCC_CFGR_MCO1PRE_1 | RCC_CFGR_MCO1PRE_2)
#define RCC_MCODIV_5                    RCC_CFGR_MCO1PRE

/** @defgroup RCC_Interrupt Interrupts
  * @{
  */
#define RCC_IT_LSIRDY                    ((uint8_t)0x01)
#define RCC_IT_LSERDY                    ((uint8_t)0x02)
#define RCC_IT_HSIRDY                    ((uint8_t)0x04)
#define RCC_IT_HSERDY                    ((uint8_t)0x08)
#define RCC_IT_PLLRDY                    ((uint8_t)0x10)
#define RCC_IT_PLLI2SRDY                 ((uint8_t)0x20)
#define RCC_IT_CSS                       ((uint8_t)0x80)


/** @defgroup RCC_Flag Flags
  *        Elements values convention: 0XXYYYYYb
  *           - YYYYY  : Flag position in the register
  *           - 0XX  : Register index
  *                 - 01: CR register
  *                 - 10: BDCR register
  *                 - 11: CSR register
  * @{
  */
/* Flags in the CR register */
#define RCC_FLAG_HSIRDY                  ((uint8_t)0x21)
#define RCC_FLAG_HSERDY                  ((uint8_t)0x31)
#define RCC_FLAG_PLLRDY                  ((uint8_t)0x39)
#define RCC_FLAG_PLLI2SRDY               ((uint8_t)0x3B)

/* Flags in the BDCR register */
#define RCC_FLAG_LSERDY                  ((uint8_t)0x41)

/* Flags in the CSR register */
#define RCC_FLAG_LSIRDY                  ((uint8_t)0x61)
#define RCC_FLAG_BORRST                  ((uint8_t)0x79)
#define RCC_FLAG_PINRST                  ((uint8_t)0x7A)
#define RCC_FLAG_PORRST                  ((uint8_t)0x7B)
#define RCC_FLAG_SFTRST                  ((uint8_t)0x7C)
#define RCC_FLAG_IWDGRST                 ((uint8_t)0x7D)
#define RCC_FLAG_WWDGRST                 ((uint8_t)0x7E)
#define RCC_FLAG_LPWRRST                 ((uint8_t)0x7F)


/** @defgroup RCC_MCO2_Clock_Source MCO2 Clock Source
  * @{
  */
#define RCC_MCO2SOURCE_SYSCLK            0x00000000U
#define RCC_MCO2SOURCE_PLLI2SCLK         RCC_CFGR_MCO2_0
#define RCC_MCO2SOURCE_HSE               RCC_CFGR_MCO2_1
#define RCC_MCO2SOURCE_PLLCLK            RCC_CFGR_MCO2


/**************************************************************************************************************
 * 																											  *
 * 												User Macro Definition										  *
 * 									  																		  *
 **************************************************************************************************************/

/*
 *	System Clock options
 */
#define SYSCLK_FREQ_168MHZ				168

#define MCO1_GPIO_PORT        			GPIOA
#define MCO1_PIN              			GPIO_PIN_8
#define MCO2_GPIO_PORT         			GPIOC
#define MCO2_PIN               			GPIO_PIN_9

/**************************************************************************************************************
 * 																											  *
 * 												User Macro Function											  *
 * 									  																		  *
 **************************************************************************************************************/

/* AHB1 Peripheral Clock Enable function */
#define RCC_GPIOA_CLK_ENABLE()				( SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN) )
#define RCC_GPIOB_CLK_ENABLE()				( SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN) )
#define RCC_GPIOC_CLK_ENABLE()				( SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN) )
#define RCC_GPIOD_CLK_ENABLE()				( SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIODEN) )
#define RCC_GPIOE_CLK_ENABLE()				( SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOEEN) )
#define RCC_BKPSRAMEN_CLK_ENABLE()			( SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_BKPSRAMEN) )
#define RCC_DMA1_CLK_ENABLE()				( SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA1EN) )
#define RCC_DMA2_CLK_ENABLE()				( SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA2EN) )

/* AHB1 Peripheral Clock Disable function */
#define RCC_GPIOA_CLK_DISABLE()				( CLEAR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN) )
#define RCC_GPIOB_CLK_DISABLE()				( CLEAR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN) )
#define RCC_GPIOC_CLK_DISABLE()				( CLEAR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN) )
#define RCC_GPIOD_CLK_DISABLE()				( CLEAR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIODEN) )
#define RCC_GPIOE_CLK_DISABLE()				( CLEAR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOEEN) )
#define RCC_BKPSRAMEN_CLK_DISABLE()			( CLEAR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_BKPSRAMEN) )
#define RCC_DMA1_CLK_DISABLE()				( CLEAR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA1EN) )
#define RCC_DMA2_CLK_DISABLE()				( CLEAR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA2EN) )


/* APB1 Peripheral Clock Enable function */
#define RCC_TIM2_CLK_ENABLE()				( SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM2EN) )
#define RCC_TIM3_CLK_ENABLE()				( SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM3EN) )
#define RCC_TIM4_CLK_ENABLE()				( SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM4EN) )
#define RCC_TIM5_CLK_ENABLE()				( SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM5EN) )
#define RCC_TIM6_CLK_ENABLE()				( SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM6EN) )
#define RCC_TIM7_CLK_ENABLE()				( SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM7EN) )
#define RCC_TIM12_CLK_ENABLE()				( SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM12EN) )
#define RCC_TIM13_CLK_ENABLE()				( SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM13EN) )
#define RCC_TIM14_CLK_ENABLE()				( SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM14EN) )
#define RCC_SPI2_CLK_ENABLE()				( SET_BIT(RCC->APB1ENR, RCC_APB1ENR_SPI2EN) )
#define RCC_SPI3_CLK_ENABLE()				( SET_BIT(RCC->APB1ENR, RCC_APB1ENR_SPI3EN) )
#define RCC_USART2_CLK_ENABLE()				( SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USART2EN) )
#define RCC_USART3_CLK_ENABLE()				( SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USART3EN) )
#define RCC_UART4_CLK_ENABLE()				( SET_BIT(RCC->APB1ENR, RCC_APB1ENR_UART4EN) )
#define RCC_UART5_CLK_ENABLE()				( SET_BIT(RCC->APB1ENR, RCC_APB1ENR_UART5EN) )
#define RCC_PWR_CLK_ENABLE()				( SET_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN) )
#define RCC_DAC_CLK_ENABLE()				( SET_BIT(RCC->APB1ENR, RCC_APB1ENR_DACEN) )

/* APB1 Peripheral Clock Disable function */
#define RCC_TIM2_CLK_DISABLE()				( CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM2EN) )
#define RCC_TIM3_CLK_DISABLE()				( CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM3EN) )
#define RCC_TIM4_CLK_DISABLE()				( CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM4EN) )
#define RCC_TIM5_CLK_DISABLE()				( CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM5EN) )
#define RCC_TIM6_CLK_DISABLE()				( CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM6EN) )
#define RCC_TIM7_CLK_DISABLE()				( CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM7EN) )
#define RCC_TIM12_CLK_DISABLE()				( CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM12EN) )
#define RCC_TIM13_CLK_DISABLE()				( CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM13EN) )
#define RCC_TIM14_CLK_DISABLE()				( CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM14EN) )
#define RCC_SPI2_CLK_DISABLE()				( CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_SPI2EN) )
#define RCC_SPI3_CLK_DISABLE()				( CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_SPI3EN) )
#define RCC_USART2_CLK_DISABLE()			( CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_USART2EN) )
#define RCC_USART3_CLK_DISABLE()			( CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_USART3EN) )
#define RCC_UART4_CLK_DISABLE()				( CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_UART4EN) )
#define RCC_UART5_CLK_DISABLE()				( CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_UART5EN) )
#define RCC_PWR_CLK_DISABLE()				( CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN) )
#define RCC_DAC_CLK_DISABLE()				( CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_DACEN) )

/* APB2 Peripheral Clock Enable function */
#define RCC_TIM1_CLK_ENABLE()				( SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM1EN) )
#define RCC_TIM8_CLK_ENABLE()				( SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM8EN) )
#define RCC_USART1_CLK_ENABLE()				( SET_BIT(RCC->APB2ENR, RCC_APB2ENR_USART1EN) )
#define RCC_USART6_CLK_ENABLE()				( SET_BIT(RCC->APB2ENR, RCC_APB2ENR_USART6EN) )
#define RCC_ADC1_CLK_ENABLE()				( SET_BIT(RCC->APB2ENR, RCC_APB2ENR_ADC1EN) )
#define RCC_ADC2_CLK_ENABLE()				( SET_BIT(RCC->APB2ENR, RCC_APB2ENR_ADC2EN) )
#define RCC_ADC3_CLK_ENABLE()				( SET_BIT(RCC->APB2ENR, RCC_APB2ENR_ADC3EN) )
#define RCC_SYSCFG_CLK_ENABLE()				( SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN) )
#define RCC_TIM9_CLK_ENABLE()				( SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM9EN) )
#define RCC_TIM10_CLK_ENABLE()				( SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM10EN) )
#define RCC_TIM11_CLK_ENABLE()				( SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM11EN) )

/* APB2 Peripheral Clock Disable function */
#define RCC_TIM1_CLK_DISABLE()				( CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM1EN) )
#define RCC_TIM8_CLK_DISABLE()				( CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM8EN) )
#define RCC_USART1_CLK_DISABLE()			( CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_USART1EN) )
#define RCC_USART6_CLK_DISABLE()			( CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_USART6EN) )
#define RCC_ADC1_CLK_DISABLE()				( CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_ADC1EN) )
#define RCC_ADC2_CLK_DISABLE()				( CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_ADC2EN) )
#define RCC_ADC3_CLK_DISABLE()				( CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_ADC3EN) )
#define RCC_SYSCFG_CLK_DISABLE()			( CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN) )
#define RCC_TIM9_CLK_DISABLE()				( CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM9EN) )
#define RCC_TIM10_CLK_DISABLE()				( CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM10EN) )
#define RCC_TIM11_CLK_DISABLE()				( CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM11EN) )


/* MCOx Clock Enable function */
#define MCO1_CLK_ENABLE()					( RCC_GPIOA_CLK_ENABLE() )
#define MCO2_CLK_ENABLE()   				( RCC_GPIOC_CLK_ENABLE() )

/**************************************************************************************************************
 * 																											  *
 * 											APIs supported by this driver									  *
 * 						For more information about the APIs, Check the function definitions					  *
 * 									  																		  *
 **************************************************************************************************************/

void RCC_OscConfig(RCC_OscInitTypeDef *RCC_OscInitStruct);
void RCC_ClockConfig(RCC_ClkInitTypeDef *RCC_ClkInitStruct, uint32_t FLatency);
void MCOConfig(uint32_t RCC_MCOx, uint32_t RCC_MCOSource, uint32_t RCC_MCODiv);
uint32_t RCC_GetHCLKFreq(void);
uint32_t RCC_GetPCLK1Freq(void);
uint32_t RCC_GetPCLK2Freq(void);



#endif /* INC_STM32F407XX_RCC_DRIVER_H_ */
