/*
 * stm32f407xx_tim_driver.h
 *
 *  Created on: 2021. 3. 8.
 *      Author: Ganghyeok Lim
 */

#ifndef INC_STM32F407XX_TIM_DRIVER_H_
#define INC_STM32F407XX_TIM_DRIVER_H_

#include "stm32f407xx.h"


/**
  * @brief  TIM Time base Configuration Structure definition
  */
typedef struct
{
  uint32_t Prescaler;         /*!< Specifies the prescaler value used to divide the TIM clock.
                                   This parameter can be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF */

  uint32_t CounterMode;       /*!< Specifies the counter mode.
                                   This parameter can be a value of @ref TIM_Counter_Mode */

  uint32_t Period;            /*!< Specifies the period value to be loaded into the active
                                   Auto-Reload Register at the next update event.
                                   This parameter can be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF.  */

  uint32_t ClockDivision;     /*!< Specifies the clock division.
                                   This parameter can be a value of @ref TIM_ClockDivision */

  uint32_t RepetitionCounter;  /*!< Specifies the repetition counter value. Each time the RCR downcounter
                                    reaches zero, an update event is generated and counting restarts
                                    from the RCR value (N).
                                    This means in PWM mode that (N+1) corresponds to:
                                        - the number of PWM periods in edge-aligned mode
                                        - the number of half PWM period in center-aligned mode
                                     GP timers: this parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF.
                                     Advanced timers: this parameter must be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF. */

  uint32_t AutoReloadPreload;  /*!< Specifies the auto-reload preload.
                                   This parameter can be a value of @ref TIM_AutoReloadPreload */
} TIM_Base_InitTypeDef;


/**
  * @brief  Clock Configuration Handle Structure definition
  */
typedef struct
{
  uint32_t ClockSource;     /*!< TIM clock sources
                                 This parameter can be a value of @ref TIM_Clock_Source */
  uint32_t ClockPolarity;   /*!< TIM clock polarity
                                 This parameter can be a value of @ref TIM_Clock_Polarity */
  uint32_t ClockPrescaler;  /*!< TIM clock prescaler
                                 This parameter can be a value of @ref TIM_Clock_Prescaler */
  uint32_t ClockFilter;     /*!< TIM clock filter
                                 This parameter can be a number between Min_Data = 0x0 and Max_Data = 0xF */
} TIM_ClockConfigTypeDef;


/**
  * @brief  TIM Master configuration Structure definition
  */
typedef struct
{
  uint32_t  MasterOutputTrigger;   /*!< Trigger output (TRGO) selection
                                        This parameter can be a value of @ref TIM_Master_Mode_Selection */
  uint32_t  MasterSlaveMode;       /*!< Master/slave mode selection
                                        This parameter can be a value of @ref TIM_Master_Slave_Mode
                                        @note When the Master/slave mode is enabled, the effect of
                                        an event on the trigger input (TRGI) is delayed to allow a
                                        perfect synchronization between the current timer and its
                                        slaves (through TRGO). It is not mandatory in case of timer
                                        synchronization mode. */
} TIM_MasterConfigTypeDef;


/**
  * @brief  TIM Time Base Handle Structure definition
  */
typedef struct
{
  TIM_TypeDef             	*Instance;     /*!< Register base address             */

  TIM_Base_InitTypeDef      Init;          /*!< TIM Time Base required parameters */

  __IO uint8_t   			State;         /*!< TIM operation state               */

} TIM_HandleTypeDef;



/** @defgroup TIM_ClearInput_Source TIM Clear Input Source
  * @{
  */
#define TIM_CLEARINPUTSOURCE_NONE           0x00000000U   /*!< OCREF_CLR is disabled */
#define TIM_CLEARINPUTSOURCE_ETR            0x00000001U   /*!< OCREF_CLR is connected to ETRF input */

/** @defgroup TIM_DMA_Base_address TIM DMA Base Address
  * @{
  */
#define TIM_DMABASE_CR1                    0x00000000U
#define TIM_DMABASE_CR2                    0x00000001U
#define TIM_DMABASE_SMCR                   0x00000002U
#define TIM_DMABASE_DIER                   0x00000003U
#define TIM_DMABASE_SR                     0x00000004U
#define TIM_DMABASE_EGR                    0x00000005U
#define TIM_DMABASE_CCMR1                  0x00000006U
#define TIM_DMABASE_CCMR2                  0x00000007U
#define TIM_DMABASE_CCER                   0x00000008U
#define TIM_DMABASE_CNT                    0x00000009U
#define TIM_DMABASE_PSC                    0x0000000AU
#define TIM_DMABASE_ARR                    0x0000000BU
#define TIM_DMABASE_RCR                    0x0000000CU
#define TIM_DMABASE_CCR1                   0x0000000DU
#define TIM_DMABASE_CCR2                   0x0000000EU
#define TIM_DMABASE_CCR3                   0x0000000FU
#define TIM_DMABASE_CCR4                   0x00000010U
#define TIM_DMABASE_BDTR                   0x00000011U
#define TIM_DMABASE_DCR                    0x00000012U
#define TIM_DMABASE_DMAR                   0x00000013U

/** @defgroup TIM_Event_Source TIM Event Source
  * @{
  */
#define TIM_EVENTSOURCE_UPDATE              TIM_EGR_UG     /*!< Reinitialize the counter and generates an update of the registers */
#define TIM_EVENTSOURCE_CC1                 TIM_EGR_CC1G   /*!< A capture/compare event is generated on channel 1 */
#define TIM_EVENTSOURCE_CC2                 TIM_EGR_CC2G   /*!< A capture/compare event is generated on channel 2 */
#define TIM_EVENTSOURCE_CC3                 TIM_EGR_CC3G   /*!< A capture/compare event is generated on channel 3 */
#define TIM_EVENTSOURCE_CC4                 TIM_EGR_CC4G   /*!< A capture/compare event is generated on channel 4 */
#define TIM_EVENTSOURCE_COM                 TIM_EGR_COMG   /*!< A commutation event is generated */
#define TIM_EVENTSOURCE_TRIGGER             TIM_EGR_TG     /*!< A trigger event is generated */
#define TIM_EVENTSOURCE_BREAK               TIM_EGR_BG     /*!< A break event is generated */

/** @defgroup TIM_Counter_Mode TIM Counter Mode
  * @{
  */
#define TIM_COUNTERMODE_UP                 0x00000000U                          /*!< Counter used as up-counter   */
#define TIM_COUNTERMODE_DOWN               TIM_CR1_DIR                          /*!< Counter used as down-counter */
#define TIM_COUNTERMODE_CENTERALIGNED1     TIM_CR1_CMS_0                        /*!< Center-aligned mode 1        */
#define TIM_COUNTERMODE_CENTERALIGNED2     TIM_CR1_CMS_1                        /*!< Center-aligned mode 2        */
#define TIM_COUNTERMODE_CENTERALIGNED3     TIM_CR1_CMS                          /*!< Center-aligned mode 3        */

/** @defgroup TIM_ClockDivision TIM Clock Division
  * @{
  */
#define TIM_CLOCKDIVISION_DIV1             0x00000000U                          /*!< Clock division: tDTS=tCK_INT   */
#define TIM_CLOCKDIVISION_DIV2             TIM_CR1_CKD_0                        /*!< Clock division: tDTS=2*tCK_INT */
#define TIM_CLOCKDIVISION_DIV4             TIM_CR1_CKD_1                        /*!< Clock division: tDTS=4*tCK_INT */

/** @defgroup TIM_AutoReloadPreload TIM Auto-Reload Preload
  * @{
  */
#define TIM_AUTORELOAD_PRELOAD_DISABLE                0x00000000U               /*!< TIMx_ARR register is not buffered */
#define TIM_AUTORELOAD_PRELOAD_ENABLE                 TIM_CR1_ARPE              /*!< TIMx_ARR register is buffered */

/** @defgroup TIM_Interrupt_definition TIM interrupt Definition
  * @{
  */
#define TIM_IT_UPDATE                      TIM_DIER_UIE                         /*!< Update interrupt            */
#define TIM_IT_CC1                         TIM_DIER_CC1IE                       /*!< Capture/Compare 1 interrupt */
#define TIM_IT_CC2                         TIM_DIER_CC2IE                       /*!< Capture/Compare 2 interrupt */
#define TIM_IT_CC3                         TIM_DIER_CC3IE                       /*!< Capture/Compare 3 interrupt */
#define TIM_IT_CC4                         TIM_DIER_CC4IE                       /*!< Capture/Compare 4 interrupt */
#define TIM_IT_COM                         TIM_DIER_COMIE                       /*!< Commutation interrupt       */
#define TIM_IT_TRIGGER                     TIM_DIER_TIE                         /*!< Trigger interrupt           */
#define TIM_IT_BREAK                       TIM_DIER_BIE                         /*!< Break interrupt             */

/** @defgroup TIM_Flag_definition TIM Flag Definition
  * @{
  */
#define TIM_FLAG_UPDATE                    TIM_SR_UIF                           /*!< Update interrupt flag         */
#define TIM_FLAG_CC1                       TIM_SR_CC1IF                         /*!< Capture/Compare 1 interrupt flag */
#define TIM_FLAG_CC2                       TIM_SR_CC2IF                         /*!< Capture/Compare 2 interrupt flag */
#define TIM_FLAG_CC3                       TIM_SR_CC3IF                         /*!< Capture/Compare 3 interrupt flag */
#define TIM_FLAG_CC4                       TIM_SR_CC4IF                         /*!< Capture/Compare 4 interrupt flag */
#define TIM_FLAG_COM                       TIM_SR_COMIF                         /*!< Commutation interrupt flag    */
#define TIM_FLAG_TRIGGER                   TIM_SR_TIF                           /*!< Trigger interrupt flag        */
#define TIM_FLAG_BREAK                     TIM_SR_BIF                           /*!< Break interrupt flag          */
#define TIM_FLAG_CC1OF                     TIM_SR_CC1OF                         /*!< Capture 1 overcapture flag    */
#define TIM_FLAG_CC2OF                     TIM_SR_CC2OF                         /*!< Capture 2 overcapture flag    */
#define TIM_FLAG_CC3OF                     TIM_SR_CC3OF                         /*!< Capture 3 overcapture flag    */
#define TIM_FLAG_CC4OF                     TIM_SR_CC4OF                         /*!< Capture 4 overcapture flag    */

/** @defgroup TIM_Channel TIM Channel
  * @{
  */
#define TIM_CHANNEL_1                      0x00000000U                          /*!< Capture/compare channel 1 identifier      */
#define TIM_CHANNEL_2                      0x00000004U                          /*!< Capture/compare channel 2 identifier      */
#define TIM_CHANNEL_3                      0x00000008U                          /*!< Capture/compare channel 3 identifier      */
#define TIM_CHANNEL_4                      0x0000000CU                          /*!< Capture/compare channel 4 identifier      */
#define TIM_CHANNEL_ALL                    0x0000003CU                          /*!< Global Capture/compare channel identifier  */

/** @defgroup TIM_Clock_Source TIM Clock Source
  * @{
  */
#define TIM_CLOCKSOURCE_ETRMODE2    TIM_SMCR_ETPS_1      /*!< External clock source mode 2                          */
#define TIM_CLOCKSOURCE_INTERNAL    TIM_SMCR_ETPS_0      /*!< Internal clock source                                 */
#define TIM_CLOCKSOURCE_ITR0        TIM_TS_ITR0          /*!< External clock source mode 1 (ITR0)                   */
#define TIM_CLOCKSOURCE_ITR1        TIM_TS_ITR1          /*!< External clock source mode 1 (ITR1)                   */
#define TIM_CLOCKSOURCE_ITR2        TIM_TS_ITR2          /*!< External clock source mode 1 (ITR2)                   */
#define TIM_CLOCKSOURCE_ITR3        TIM_TS_ITR3          /*!< External clock source mode 1 (ITR3)                   */
#define TIM_CLOCKSOURCE_TI1ED       TIM_TS_TI1F_ED       /*!< External clock source mode 1 (TTI1FP1 + edge detect.) */
#define TIM_CLOCKSOURCE_TI1         TIM_TS_TI1FP1        /*!< External clock source mode 1 (TTI1FP1)                */
#define TIM_CLOCKSOURCE_TI2         TIM_TS_TI2FP2        /*!< External clock source mode 1 (TTI2FP2)                */
#define TIM_CLOCKSOURCE_ETRMODE1    TIM_TS_ETRF          /*!< External clock source mode 1 (ETRF)                   */

/** @defgroup TIM_Clock_Polarity TIM Clock Polarity
  * @{
  */
#define TIM_CLOCKPOLARITY_INVERTED           TIM_ETRPOLARITY_INVERTED           /*!< Polarity for ETRx clock sources */
#define TIM_CLOCKPOLARITY_NONINVERTED        TIM_ETRPOLARITY_NONINVERTED        /*!< Polarity for ETRx clock sources */
#define TIM_CLOCKPOLARITY_RISING             TIM_INPUTCHANNELPOLARITY_RISING    /*!< Polarity for TIx clock sources */
#define TIM_CLOCKPOLARITY_FALLING            TIM_INPUTCHANNELPOLARITY_FALLING   /*!< Polarity for TIx clock sources */
#define TIM_CLOCKPOLARITY_BOTHEDGE           TIM_INPUTCHANNELPOLARITY_BOTHEDGE  /*!< Polarity for TIx clock sources */

/** @defgroup TIM_AOE_Bit_Set_Reset TIM Automatic Output Enable
  * @{
  */
#define TIM_AUTOMATICOUTPUT_DISABLE        0x00000000U                          /*!< MOE can be set only by software */
#define TIM_AUTOMATICOUTPUT_ENABLE         TIM_BDTR_AOE                         /*!< MOE can be set by software or automatically at the next update event
                                                                                    (if none of the break inputs BRK and BRK2 is active) */
/** @defgroup TIM_Master_Mode_Selection TIM Master Mode Selection
  * @{
  */
#define TIM_TRGO_RESET            0x00000000U                                      /*!< TIMx_EGR.UG bit is used as trigger output (TRGO)              */
#define TIM_TRGO_ENABLE           TIM_CR2_MMS_0                                    /*!< TIMx_CR1.CEN bit is used as trigger output (TRGO)             */
#define TIM_TRGO_UPDATE           TIM_CR2_MMS_1                                    /*!< Update event is used as trigger output (TRGO)                 */
#define TIM_TRGO_OC1              (TIM_CR2_MMS_1 | TIM_CR2_MMS_0)                  /*!< Capture or a compare match 1 is used as trigger output (TRGO) */
#define TIM_TRGO_OC1REF           TIM_CR2_MMS_2                                    /*!< OC1REF signal is used as trigger output (TRGO)                */
#define TIM_TRGO_OC2REF           (TIM_CR2_MMS_2 | TIM_CR2_MMS_0)                  /*!< OC2REF signal is used as trigger output(TRGO)                 */
#define TIM_TRGO_OC3REF           (TIM_CR2_MMS_2 | TIM_CR2_MMS_1)                  /*!< OC3REF signal is used as trigger output(TRGO)                 */
#define TIM_TRGO_OC4REF           (TIM_CR2_MMS_2 | TIM_CR2_MMS_1 | TIM_CR2_MMS_0)  /*!< OC4REF signal is used as trigger output(TRGO)                 */

/** @defgroup TIM_Master_Slave_Mode TIM Master/Slave Mode
  * @{
  */
#define TIM_MASTERSLAVEMODE_ENABLE         TIM_SMCR_MSM                         /*!< No action */
#define TIM_MASTERSLAVEMODE_DISABLE        0x00000000U                          /*!< Master/slave mode is selected */

/** @defgroup TIM_Slave_Mode TIM Slave mode
  * @{
  */
#define TIM_SLAVEMODE_DISABLE                0x00000000U                                        /*!< Slave mode disabled           */
#define TIM_SLAVEMODE_RESET                  TIM_SMCR_SMS_2                                     /*!< Reset Mode                    */
#define TIM_SLAVEMODE_GATED                  (TIM_SMCR_SMS_2 | TIM_SMCR_SMS_0)                  /*!< Gated Mode                    */
#define TIM_SLAVEMODE_TRIGGER                (TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1)                  /*!< Trigger Mode                  */
#define TIM_SLAVEMODE_EXTERNAL1              (TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0) /*!< External Clock Mode 1		   */


/**************************************************************************************************************
 * 																											  *
 * 												User Macro Definition										  *
 * 									  																		  *
 **************************************************************************************************************/

/*
 *  @TIM_State
 */
#define TIM_STATE_RESET								0
#define TIM_STATE_READY								1
#define TIM_STATE_BUSY								2
#define TIM_STATE_ERROR								3


/**************************************************************************************************************
 * 																											  *
 * 												User Macro Function											  *
 * 									  																		  *
 **************************************************************************************************************/

#define TIM_ENABLE(__HANDLE__)                 ((__HANDLE__)->Instance->CR1|=(TIM_CR1_CEN))

#define TIM_ENABLE_COUNTER(HANDLE)					((HANDLE)->Instance->CR1 |= (TIM_CR1_CEN))

#define TIM_DISABLE_COUNTER(HANDLE)					((HANDLE)->Instance->CR1 &= ~(TIM_CR1_CEN))

#define TIM_ENABLE_IT(HANDLE, INTERRUPT)    ((HANDLE)->Instance->DIER |= (INTERRUPT))

#define TIM_DISABLE_IT(HANDLE, INTERRUPT)   ((HANDLE)->Instance->DIER &= ~(INTERRUPT))

#define IS_TIM_SLAVEMODE_TRIGGER_ENABLED(__TRIGGER__) ((__TRIGGER__) == TIM_SLAVEMODE_TRIGGER)

#define IS_TIM_SLAVE_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || \
                                         ((INSTANCE) == TIM2) || \
                                         ((INSTANCE) == TIM3) || \
                                         ((INSTANCE) == TIM4) || \
                                         ((INSTANCE) == TIM5) || \
                                         ((INSTANCE) == TIM8) || \
                                         ((INSTANCE) == TIM9) || \
                                         ((INSTANCE) == TIM12))

#define IS_TIM_COUNTER_MODE_SELECT_INSTANCE(INSTANCE)  (((INSTANCE) == TIM1) || \
                                                        ((INSTANCE) == TIM2) || \
                                                        ((INSTANCE) == TIM3) || \
                                                        ((INSTANCE) == TIM4) || \
                                                        ((INSTANCE) == TIM5) || \
                                                        ((INSTANCE) == TIM8))

#define IS_TIM_CLOCK_DIVISION_INSTANCE(INSTANCE) (((INSTANCE) == TIM1)  || \
                                                  ((INSTANCE) == TIM2) || \
                                                  ((INSTANCE) == TIM3) || \
                                                  ((INSTANCE) == TIM4) || \
                                                  ((INSTANCE) == TIM5) || \
                                                  ((INSTANCE) == TIM8) || \
                                                  ((INSTANCE) == TIM9) || \
                                                  ((INSTANCE) == TIM10)|| \
                                                  ((INSTANCE) == TIM11)|| \
                                                  ((INSTANCE) == TIM12)|| \
                                                  ((INSTANCE) == TIM13)|| \
                                                  ((INSTANCE) == TIM14))

#define IS_TIM_REPETITION_COUNTER_INSTANCE(INSTANCE)  (((INSTANCE) == TIM1) || \
                                                       ((INSTANCE) == TIM8))




/**************************************************************************************************************
 * 																											  *
 * 											APIs supported by this driver									  *
 * 						For more information about the APIs, Check the function definitions					  *
 * 									  																		  *
 **************************************************************************************************************/

void TIM_Base_Init(TIM_HandleTypeDef *pTIMHandle);
void TIM_Base_MspInit(TIM_HandleTypeDef *pTIMHandle);
void TIM_Base_SetConfig(TIM_TypeDef *TIMx, TIM_Base_InitTypeDef *Structure);
void TIM_Base_Start(TIM_HandleTypeDef *pTIMHandle);
void TIM_Base_Start_IT(TIM_HandleTypeDef *pTIMHandle);
void TIM_PeripheralClockControl(TIM_TypeDef *TIMx, uint8_t En_or_Di);
void TIMEx_MasterConfigSynchronization(TIM_HandleTypeDef *pTIMHandle, TIM_MasterConfigTypeDef *sMasterConfig);
void TIM_IRQHandling(TIM_HandleTypeDef *pTIMHandle);
void TIM_PeriodElapsedCallback(TIM_HandleTypeDef *pTIMHandle);



#endif /* INC_STM32F407XX_TIM_DRIVER_H_ */
