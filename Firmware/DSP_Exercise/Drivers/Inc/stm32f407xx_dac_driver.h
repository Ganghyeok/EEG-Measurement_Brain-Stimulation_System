/*
 * stm32f407xx_dac_driver.h
 *
 *  Created on: 2021. 3. 11.
 *      Author: Ganghyeok Lim
 */

#ifndef INC_STM32F407XX_DAC_DRIVER_H_
#define INC_STM32F407XX_DAC_DRIVER_H_

#include "stm32f407xx.h"


/**
  * @brief State structures definition
  */
typedef enum
{
  DAC_STATE_RESET             = 0x00U,  /*!< DAC not yet initialized or disabled  */
  DAC_STATE_READY             = 0x01U,  /*!< DAC initialized and ready for use    */
  DAC_STATE_BUSY              = 0x02U,  /*!< DAC internal processing is ongoing   */
  DAC_STATE_TIMEOUT           = 0x03U,  /*!< DAC timeout state                    */
  DAC_STATE_ERROR             = 0x04U   /*!< DAC error state                      */
} DAC_StateTypeDef;


/**
  * @brief DAC handle Structure definition
  */
typedef struct
{
  DAC_TypeDef                 *Instance;     /*!< Register base address             */

  __IO DAC_StateTypeDef   State;         /*!< DAC communication state           */

  DMA_HandleTypeDef           *DMA_Handle1;  /*!< Pointer DMA handler for channel 1 */

  DMA_HandleTypeDef           *DMA_Handle2;  /*!< Pointer DMA handler for channel 2 */

  __IO uint32_t               ErrorCode;     /*!< DAC Error code                    */

} DAC_HandleTypeDef;



/**
  * @brief DAC Configuration regular Channel structure definition
  */
typedef struct
{
  uint32_t DAC_Trigger;       /*!< Specifies the external trigger for the selected DAC channel.
                                   This parameter can be a value of @ref DAC_trigger_selection */

  uint32_t DAC_OutputBuffer;  /*!< Specifies whether the DAC channel output buffer is enabled or disabled.
                                   This parameter can be a value of @ref DAC_output_buffer */
} DAC_ChannelConfTypeDef;



/** @defgroup DAC_Error_Code DAC Error Code
  * @{
  */
#define  DAC_ERROR_NONE              0x00U    /*!< No error                          */
#define  DAC_ERROR_DMAUNDERRUNCH1    0x01U    /*!< DAC channel1 DAM underrun error   */
#define  DAC_ERROR_DMAUNDERRUNCH2    0x02U    /*!< DAC channel2 DAM underrun error   */
#define  DAC_ERROR_DMA               0x04U    /*!< DMA error                         */

/** @defgroup DAC_trigger_selection DAC Trigger Selection
  * @{
  */
#define DAC_TRIGGER_NONE                   0x00000000U /*!< Conversion is automatic once the DAC1_DHRxxxx register                                    has been loaded, and not by external trigger */
#define DAC_TRIGGER_T2_TRGO                ((uint32_t)(DAC_CR_TSEL1_2 | DAC_CR_TEN1)) /*!< TIM2 TRGO selected as external conversion trigger for DAC channel */
#define DAC_TRIGGER_T4_TRGO                ((uint32_t)(DAC_CR_TSEL1_2 | DAC_CR_TSEL1_0 | DAC_CR_TEN1)) /*!< TIM4 TRGO selected as external conversion trigger for DAC channel */
#define DAC_TRIGGER_T5_TRGO                ((uint32_t)(DAC_CR_TSEL1_1 | DAC_CR_TSEL1_0 | DAC_CR_TEN1)) /*!< TIM5 TRGO selected as external conversion trigger for DAC channel */
#define DAC_TRIGGER_T6_TRGO                ((uint32_t)DAC_CR_TEN1) /*!< TIM6 TRGO selected as external conversion trigger for DAC channel */
#define DAC_TRIGGER_T7_TRGO                ((uint32_t)(DAC_CR_TSEL1_1 | DAC_CR_TEN1)) /*!< TIM7 TRGO selected as external conversion trigger for DAC channel */
#define DAC_TRIGGER_T8_TRGO                ((uint32_t)(DAC_CR_TSEL1_0 | DAC_CR_TEN1)) /*!< TIM8 TRGO selected as external conversion trigger for DAC channel */
#define DAC_TRIGGER_EXT_IT9                ((uint32_t)(DAC_CR_TSEL1_2 | DAC_CR_TSEL1_1 | DAC_CR_TEN1)) /*!< EXTI Line9 event selected as external conversion trigger for DAC channel */
#define DAC_TRIGGER_SOFTWARE               ((uint32_t)(DAC_CR_TSEL1 | DAC_CR_TEN1)) /*!< Conversion started by software trigger for DAC channel */

/** @defgroup DAC_output_buffer  DAC Output Buffer
  * @{
  */
#define DAC_OUTPUTBUFFER_ENABLE            0x00000000U
#define DAC_OUTPUTBUFFER_DISABLE           ((uint32_t)DAC_CR_BOFF1)

/** @defgroup DAC_Channel_selection DAC Channel Selection
  * @{
  */
#define DAC_CHANNEL_1                      0x00000000U
#define DAC_CHANNEL_2                      0x00000010U
#define DAC1_CHANNEL_1                     DAC_CHANNEL_1
#define DAC1_CHANNEL_2                     DAC_CHANNEL_2
#define DAC2_CHANNEL_1                     DAC_CHANNEL_1
#define DAC_WAVE_NONE                      0x00000000U
#define DAC_WAVE_NOISE                     DAC_CR_WAVE1_0
#define DAC_WAVE_TRIANGLE                  DAC_CR_WAVE1_1
#define DAC_WAVEGENERATION_NONE            DAC_WAVE_NONE
#define DAC_WAVEGENERATION_NOISE           DAC_WAVE_NOISE
#define DAC_WAVEGENERATION_TRIANGLE        DAC_WAVE_TRIANGLE

/** @defgroup DAC_data_alignment DAC Data Alignment
  * @{
  */
#define DAC_ALIGN_12B_R                    0x00000000U
#define DAC_ALIGN_12B_L                    0x00000004U
#define DAC_ALIGN_8B_R                     0x00000008U

/** @defgroup DAC_flags_definition DAC Flags Definition
  * @{
  */
#define DAC_FLAG_DMAUDR1                   ((uint32_t)DAC_SR_DMAUDR1)
#define DAC_FLAG_DMAUDR2                   ((uint32_t)DAC_SR_DMAUDR2)

/** @defgroup DAC_IT_definition DAC IT Definition
  * @{
  */
#define DAC_IT_DMAUDR1                   ((uint32_t)DAC_SR_DMAUDR1)
#define DAC_IT_DMAUDR2                   ((uint32_t)DAC_SR_DMAUDR2)

/**************************************************************************************************************
 * 																											  *
 * 												User Macro Definition										  *
 * 									  																		  *
 **************************************************************************************************************/




/**************************************************************************************************************
 * 																											  *
 * 												User Macro Function											  *
 * 									  																		  *
 **************************************************************************************************************/

/** @brief Enable the DAC interrupt
  * @param  __HANDLE__ specifies the DAC handle
  * @param  __INTERRUPT__ specifies the DAC interrupt.
  * @retval None
  */
#define DAC_ENABLE_IT(__HANDLE__, __INTERRUPT__) (((__HANDLE__)->Instance->CR) |= (__INTERRUPT__))

/** @brief Enable the DAC channel
  * @param  __HANDLE__ specifies the DAC handle.
  * @param  __DAC_Channel__ specifies the DAC channel
  * @retval None
  */
#define DAC_ENABLE(__HANDLE__, __DAC_Channel__) ((__HANDLE__)->Instance->CR |=  (DAC_CR_EN1 << (__DAC_Channel__)))


/**************************************************************************************************************
 * 																											  *
 * 											APIs supported by this driver									  *
 * 						For more information about the APIs, Check the function definitions					  *
 * 									  																		  *
 **************************************************************************************************************/
void DAC_Init(DAC_HandleTypeDef* pDACHandle);
void DAC_MspInit(DAC_HandleTypeDef* pDACHandle);
void DAC_Start_DMA(DAC_HandleTypeDef* pDACHandle, uint32_t Channel, uint32_t* pData, uint32_t Length, uint32_t Alignment);
void DAC_ConfigChannel(DAC_HandleTypeDef* pDACHandle, DAC_ChannelConfTypeDef* sConfig, uint32_t Channel);
void DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* pDACHandle);
void DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef* pDACHandle);
void DAC_ErrorCallbackCh1(DAC_HandleTypeDef *pDACHandle);
void DACEx_ConvCpltCallbackCh2(DAC_HandleTypeDef* pDACHandle);
void DACEx_ConvHalfCpltCallbackCh2(DAC_HandleTypeDef* pDACHandle);
void DACEx_ErrorCallbackCh2(DAC_HandleTypeDef *pDACHandle);
void DACEx_DMAUnderrunCallbackCh2(DAC_HandleTypeDef *pDACHandle);



#endif /* INC_STM32F407XX_DAC_DRIVER_H_ */
