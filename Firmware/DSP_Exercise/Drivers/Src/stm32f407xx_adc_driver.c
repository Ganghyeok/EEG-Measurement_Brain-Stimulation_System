/*
 * stm32f407xx_adc_driver.c
 *
 *  Created on: 2021. 3. 10.
 *      Author: Ganghyeok Lim
 */

#include "stm32f407xx.h"



extern uint32_t SystemCoreClock;


static void ADC_DMAConvCplt(DMA_HandleTypeDef *pDMAHandle);
static void ADC_DMAHalfConvCplt(DMA_HandleTypeDef *pDMAHandle);
static void ADC_DMAError(DMA_HandleTypeDef *pDMAHandle);

/**************************************************************************************************************
 * 																											  *
 * 											APIs supported by this driver									  *
 * 						For more information about the APIs, Check the function definitions					  *
 * 									  																		  *
 **************************************************************************************************************/

void ADC_Init(ADC_HandleTypeDef* pADCHandle)
{
	// 1. Init the Low level Hardware
	ADC_MspInit(pADCHandle);

	// 2. Configure the ADC when State of ADC is not ERROR_INTERNAL
	if(pADCHandle->State != ADC_STATE_ERROR_INTERNAL)
	{
		// 2-1. Set State of ADC
	    ADC_STATE_CLR_SET(pADCHandle->State,
	                      ADC_STATE_REG_BUSY | ADC_STATE_INJ_BUSY,
	                      ADC_STATE_BUSY_INTERNAL);

	    // 2-2. Set the ADCCLK Prescaler
	    ADC->CCR &= ~(ADC_CCR_ADCPRE);				// at here, ADC is Common type of ADC1/2/3
	    ADC->CCR |=  pADCHandle->Init.ClockPrescaler;

	    // 2-3. Set ADC Scan mode
	    pADCHandle->Instance->CR1 &= ~(ADC_CR1_SCAN);
	    pADCHandle->Instance->CR1 |=  ADC_CR1_SCANCONV(pADCHandle->Init.ScanConvMode);

	    // 2-4. Set ADC Resolution
	    pADCHandle->Instance->CR1 &= ~(ADC_CR1_RES);
	    pADCHandle->Instance->CR1 |=  pADCHandle->Init.Resolution;

	    // 2-5. Set ADC Data alignment
	    pADCHandle->Instance->CR2 &= ~(ADC_CR2_ALIGN);
	    pADCHandle->Instance->CR2 |= pADCHandle->Init.DataAlign;

	    // 2-6. If External Trigger is used
	    if(pADCHandle->Init.ExternalTrigConv != ADC_SOFTWARE_START)
	    {
	    	// Select External Trigger to Start Conversion
	    	pADCHandle->Instance->CR2 &= ~(ADC_CR2_EXTSEL);
	    	pADCHandle->Instance->CR2 |= pADCHandle->Init.ExternalTrigConv;

	    	// Select External Trigger Polarity
	    	pADCHandle->Instance->CR2 &= ~(ADC_CR2_EXTEN);
	    	pADCHandle->Instance->CR2 |= pADCHandle->Init.ExternalTrigConvEdge;
	    }
	    else
	    {
	    	// Reset External Trigger
	    	pADCHandle->Instance->CR2 &= ~(ADC_CR2_EXTSEL);
	    	pADCHandle->Instance->CR2 &= ~(ADC_CR2_EXTEN);
	    }

	    // 2-7. Enable or Disable ADC Continuous Conversion Mode
	    pADCHandle->Instance->CR2 &= ~(ADC_CR2_CONT);
	    pADCHandle->Instance->CR2 |= ADC_CR2_CONTINUOUS((uint32_t)pADCHandle->Init.ContinuousConvMode);

	    // 2-8. If ADC DisContinuous Mode is Enabled
	    if(pADCHandle->Init.DiscontinuousConvMode != DISABLE)
	    {
	    	// Enable the Selected ADC Regular Discontinuous Mode
	    	pADCHandle->Instance->CR1 |= (uint32_t)ADC_CR1_DISCEN;

	    	// Set the Number of channels to be converted in Discontinuous Mode
	    	pADCHandle->Instance->CR1 &= ~(ADC_CR1_DISCNUM);
	    	pADCHandle->Instance->CR1 |=  ADC_CR1_DISCONTINUOUS(pADCHandle->Init.NbrOfDiscConversion);
	    }
	    else
	    {
	    	// Disable the Selected ADC Regular Discontinuous Mode
	    	pADCHandle->Instance->CR1 &= ~(ADC_CR1_DISCEN);
	    }

	    // 2-9. Set ADC number of Conversion
	    pADCHandle->Instance->SQR1 &= ~(ADC_SQR1_L);
	    pADCHandle->Instance->SQR1 |=  ADC_SQR1(pADCHandle->Init.NbrOfConversion);

	    // 2-10. Enable or Disable ADC DMA Continuous Request
	    pADCHandle->Instance->CR2 &= ~(ADC_CR2_DDS);
	    pADCHandle->Instance->CR2 |= ADC_CR2_DMAContReq((uint32_t)pADCHandle->Init.DMAContinuousRequests);

	    // 2-11. Enable or Disable ADC End of Conversion Selection
	    pADCHandle->Instance->CR2 &= ~(ADC_CR2_EOCS);
	    pADCHandle->Instance->CR2 |= ADC_CR2_EOCSelection(pADCHandle->Init.EOCSelection);

	    // 2-12. Set the ADC State
	    ADC_STATE_CLR_SET(pADCHandle->State,
	                      ADC_STATE_BUSY_INTERNAL,
	                      ADC_STATE_READY);
	}
}


__weak void ADC_MspInit(ADC_HandleTypeDef* pADCHandle)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(pADCHandle);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the ADC_MspInit could be implemented in the user file
   */
}


void ADC_ConfigChannel(ADC_HandleTypeDef* pADCHandle, ADC_ChannelConfTypeDef* sConfig)
{
	__IO uint32_t counter = 0U;

	// 1-1. If Selected Channel is ADC_Channel_10 ... ADC_Channel_18
	if(sConfig->Channel > ADC_CHANNEL_9)
	{
		// Clear the old Sample time & Set the new Sample time
		pADCHandle->Instance->SMPR1 &= ~ADC_SMPR1(ADC_SMPR1_SMP10, sConfig->Channel);
		pADCHandle->Instance->SMPR1 |= ADC_SMPR1(sConfig->SamplingTime, sConfig->Channel);
	}
	// 1-2. If Selected Channel is ADC_Channel_[0..9]
	else
	{
		// Clear the old Sample time & Set the new Sample time
		pADCHandle->Instance->SMPR2 &= ~ADC_SMPR2(ADC_SMPR2_SMP0, sConfig->Channel);
		pADCHandle->Instance->SMPR2 |= ADC_SMPR2(sConfig->SamplingTime, sConfig->Channel);
	}


	// 2-1. If Rank is 1 to 6
	if (sConfig->Rank < 7U)
	{
		/* Clear the old SQx bits & Set the new SQx bits for the selected rank */
		pADCHandle->Instance->SQR3 &= ~ADC_SQR3_RK(ADC_SQR3_SQ1, sConfig->Rank);
		pADCHandle->Instance->SQR3 |= ADC_SQR3_RK(sConfig->Channel, sConfig->Rank);
	}
	// 2-2. If Rank is 7 to 12
	else if (sConfig->Rank < 13U)
	{
		/* Clear the old SQx bits & Set the new SQx bits for the selected rank */
		pADCHandle->Instance->SQR2 &= ~ADC_SQR2_RK(ADC_SQR2_SQ7, sConfig->Rank);
		pADCHandle->Instance->SQR2 |= ADC_SQR2_RK(sConfig->Channel, sConfig->Rank);
	}
	// 2-3 If Rank is 13 to 16
	else
	{
		/* Clear the old SQx bits & Set the SQx bits for the selected rank */
		pADCHandle->Instance->SQR1 &= ~ADC_SQR1_RK(ADC_SQR1_SQ13, sConfig->Rank);
		pADCHandle->Instance->SQR1 |= ADC_SQR1_RK(sConfig->Channel, sConfig->Rank);
	}


	// 3. If ADC1 Channel_18 is selected for VBAT Channel enable VBATE
	if ((pADCHandle->Instance == ADC1) && (sConfig->Channel == ADC_CHANNEL_VBAT))
	{
		/* Disable the TEMPSENSOR channel in case of using board with multiplixed ADC_CHANNEL_VBAT & ADC_CHANNEL_TEMPSENSOR*/
		if ((uint16_t)ADC_CHANNEL_TEMPSENSOR == (uint16_t)ADC_CHANNEL_VBAT)
		{
			ADC->CCR &= ~ADC_CCR_TSVREFE;
		}

		/* Enable the VBAT channel*/
		ADC->CCR |= ADC_CCR_VBATE;
	}

	// 4. If if ADC1 Channel_16 or Channel_18 is selected for Temperature sensor or Channel_17 is selected for VREFINT enable TSVREFE
	if ((pADCHandle->Instance == ADC1) && ((sConfig->Channel == ADC_CHANNEL_TEMPSENSOR) || (sConfig->Channel == ADC_CHANNEL_VREFINT)))
	{
		/* Disable the VBAT channel in case of using board with multiplixed ADC_CHANNEL_VBAT & ADC_CHANNEL_TEMPSENSOR*/
		if ((uint16_t)ADC_CHANNEL_TEMPSENSOR == (uint16_t)ADC_CHANNEL_VBAT)
		{
			ADC->CCR &= ~ADC_CCR_VBATE;
		}

		/* Enable the Temperature sensor and VREFINT channel*/
		ADC->CCR |= ADC_CCR_TSVREFE;

		if((sConfig->Channel == ADC_CHANNEL_TEMPSENSOR))
		{
			/* Delay for temperature sensor stabilization time */
			/* Compute number of CPU cycles to wait for */
			counter = (ADC_TEMPSENSOR_DELAY_US * (SystemCoreClock / 1000000U));

			while(counter != 0U)
			{
				counter--;
			}
		}
	}
}



void ADC_Start(ADC_HandleTypeDef* pADCHandle)
{
	__IO uint32_t counter = 0U;

	// 1. If ADC peripheral is Disable
	if((pADCHandle->Instance->CR2 & ADC_CR2_ADON) != ADC_CR2_ADON)
	{
		// 1. Enable the ADC peripheral
		ADC_ENABLE(pADCHandle);

		// 2. Give Delay for ADC stabilization time (Compute number of CPU cycles to wait for)
	    counter = (ADC_STAB_DELAY_US * (SystemCoreClock / 1000000U));

	    while(counter != 0U)
	    {
	    	counter--;
	    }
	}

	// 2. If ADC peripheral is enabled, Start conversion
	if(READ_BIT(pADCHandle->Instance->CR2, ADC_CR2_ADON) != RESET)
	{
		// 2-1. Set ADC state (Clear state bitfield related to regular group conversion results & Set state bitfield related to regular group operation)
	    ADC_STATE_CLR_SET(pADCHandle->State,
	                      ADC_STATE_READY | ADC_STATE_REG_EOC | ADC_STATE_REG_OVR,
	                      ADC_STATE_REG_BUSY);

	    // 2-2. Clear regular group conversion flag and overrun flag (To ensure of no unknown state from potential previous ADC operations)
	    ADC_CLEAR_FLAG(pADCHandle, ADC_FLAG_EOC | ADC_FLAG_OVR);

	    // 2-3. If instance of handle correspond to ADC1 and  no external trigger present enable software conversion of regular channels
		if((pADCHandle->Instance == ADC1) && ((pADCHandle->Instance->CR2 & ADC_CR2_EXTEN) == RESET))
		{
			/* Enable the selected ADC software conversion for regular group */
			pADCHandle->Instance->CR2 |= (uint32_t)ADC_CR2_SWSTART;
		}
	}
}



void ADC_Start_IT(ADC_HandleTypeDef* pADCHandle)
{
	__IO uint32_t counter = 0U;

	// 1. If ADC peripheral is Disable
	if((pADCHandle->Instance->CR2 & ADC_CR2_ADON) != ADC_CR2_ADON)
	{
		// 1-1. Enable the ADC peripheral
		ADC_ENABLE(pADCHandle);

		// 1-2. Give Delay for ADC stabilization time (Compute number of CPU cycles to wait for)
	    counter = (ADC_STAB_DELAY_US * (SystemCoreClock / 1000000U));

	    while(counter != 0U)
	    {
	    	counter--;
	    }
	}

	// 2. If ADC peripheral is enabled, Start conversion
	if(READ_BIT(pADCHandle->Instance->CR2, ADC_CR2_ADON) != RESET)
	{
		// 2-1. Set ADC state (Clear state bitfield related to regular group conversion results & Set state bitfield related to regular group operation)
	    ADC_STATE_CLR_SET(pADCHandle->State,
	                      ADC_STATE_READY | ADC_STATE_REG_EOC | ADC_STATE_REG_OVR,
	                      ADC_STATE_REG_BUSY);

	    // 2-2. Clear regular group conversion flag and overrun flag (To ensure of no unknown state from potential previous ADC operations)
	    ADC_CLEAR_FLAG(pADCHandle, ADC_FLAG_EOC | ADC_FLAG_OVR);

	    // 2-3. Enable end of conversion interrupt for regular group
	    ADC_ENABLE_IT(pADCHandle, (ADC_IT_EOC | ADC_IT_OVR));

	    // 2-4. If instance of handle correspond to ADC1 and no external trigger present enable software conversion of regular channels
		if((pADCHandle->Instance == ADC1) && ((pADCHandle->Instance->CR2 & ADC_CR2_EXTEN) == RESET))
		{
			/* Enable the selected ADC software conversion for regular group */
			pADCHandle->Instance->CR2 |= (uint32_t)ADC_CR2_SWSTART;
		}
	}
}



void ADC_Start_DMA(ADC_HandleTypeDef* pADCHandle, uint32_t* pData, uint32_t Length)
{
	__IO uint32_t counter = 0U;

	// 1. If ADC peripheral is Disable
	if((pADCHandle->Instance->CR2 & ADC_CR2_ADON) != ADC_CR2_ADON)
	{
		// 1-1. Enable the ADC peripheral
		ADC_ENABLE(pADCHandle);

		// 1-2. Give Delay for ADC stabilization time (Compute number of CPU cycles to wait for)
	    counter = (ADC_STAB_DELAY_US * (SystemCoreClock / 1000000U));

	    while(counter != 0U)
	    {
	    	counter--;
	    }
	}

	// 2. If ADC peripheral is enabled, Start conversion
	if(READ_BIT(pADCHandle->Instance->CR2, ADC_CR2_ADON) != RESET)
	{
		// 2-1. Set ADC state (Clear state bitfield related to regular group conversion results & Set state bitfield related to regular group operation)
	    ADC_STATE_CLR_SET(pADCHandle->State,
	                      ADC_STATE_READY | ADC_STATE_REG_EOC | ADC_STATE_REG_OVR,
	                      ADC_STATE_REG_BUSY);

	    // 2-2. Set the DMA transfer complete callback
	    pADCHandle->DMA_Handle->XferCpltCallback = ADC_DMAConvCplt;

	    // 2-3. Set the DMA half transfer complete callback
	    pADCHandle->DMA_Handle->XferHalfCpltCallback = ADC_DMAHalfConvCplt;

	    // 2-4. Set the DMA error callback
	    pADCHandle->DMA_Handle->XferErrorCallback = ADC_DMAError;

	    // 2-5. Clear regular group conversion flag and overrun flag (To ensure of no unknown state from potential previous ADC operations)
	    ADC_CLEAR_FLAG(pADCHandle, ADC_FLAG_EOC | ADC_FLAG_OVR);

	    // 2-6. Enable ADC overrun interrupt
	    ADC_ENABLE_IT(pADCHandle, ADC_IT_OVR);

	    // 2-7. Enable ADC DMA mode
	    pADCHandle->Instance->CR2 |= ADC_CR2_DMA;

	    // 2-8. Start the DMA channel
	    DMA_Start_IT(pADCHandle->DMA_Handle, (uint32_t)&pADCHandle->Instance->DR, (uint32_t)pData, Length);

	    // 2-9. Check if Multimode enabled
	    if(IS_BIT_CLR(ADC->CCR, ADC_CCR_MULTI))
	    {
	        if((pADCHandle->Instance == ADC1) || ((pADCHandle->Instance == ADC2) && ((ADC->CCR & ADC_CCR_MULTI_Msk) < ADC_CCR_MULTI_0)) \
	                                    || ((pADCHandle->Instance == ADC3) && ((ADC->CCR & ADC_CCR_MULTI_Msk) < ADC_CCR_MULTI_4)))
	        {
	        	/* if no external trigger present enable software conversion of regular channels */
	        	if((pADCHandle->Instance->CR2 & ADC_CR2_EXTEN) == RESET)
	        	{
	        		/* Enable the selected ADC software conversion for regular group */
	        		pADCHandle->Instance->CR2 |= (uint32_t)ADC_CR2_SWSTART;
	        	}
	        }
	    }
	    else
	    {
		    // If instance of handle correspond to ADC1 and no external trigger present enable software conversion of regular channels
			if((pADCHandle->Instance == ADC1) && ((pADCHandle->Instance->CR2 & ADC_CR2_EXTEN) == RESET))
			{
				/* Enable the selected ADC software conversion for regular group */
				pADCHandle->Instance->CR2 |= (uint32_t)ADC_CR2_SWSTART;
			}
	    }
	}
}



void ADC_PollForConversion(ADC_HandleTypeDef* pADCHandle)
{
	// 1. Wait until End of conversion flag Set
	while(!(ADC_GET_FLAG(pADCHandle, ADC_FLAG_EOC)));

	// 2. Clear regular group conversion flag
	ADC_CLEAR_FLAG(pADCHandle, ADC_FLAG_STRT | ADC_FLAG_EOC);

	// 3. Update ADC state machine
	SET_BIT(pADCHandle->State, ADC_STATE_REG_EOC);


	if(ADC_IS_SOFTWARE_START_REGULAR(pADCHandle)                   &&
	(pADCHandle->Init.ContinuousConvMode == DISABLE)            &&
	(IS_BIT_CLR(pADCHandle->Instance->SQR1, ADC_SQR1_L) ||
	IS_BIT_CLR(pADCHandle->Instance->CR2, ADC_CR2_EOCS)  )   )
	{
		// Set ADC state
		CLEAR_BIT(pADCHandle->State, ADC_STATE_REG_BUSY);

		if (IS_BIT_CLR(pADCHandle->State, ADC_STATE_INJ_BUSY))
		{
			SET_BIT(pADCHandle->State, ADC_STATE_READY);
		}
	}
}



uint32_t ADC_GetValue(ADC_HandleTypeDef* pADCHandle)
{
  /* Return the selected ADC converted value */
  return pADCHandle->Instance->DR;
}


void ADC_IRQHandling(ADC_HandleTypeDef* pADCHandle)
{
	uint32_t tmp1 = 0U, tmp2 = 0U;

	// 1. Check EOC flag of ADC & its interrupt source
	tmp1 = ADC_GET_FLAG(pADCHandle, ADC_FLAG_EOC);
	tmp2 = ADC_GET_IT_SOURCE(pADCHandle, ADC_IT_EOC);

	// 2. Check if Interrupt Handler is called by End of Conversion for Regular channels
	if(tmp1 && tmp2)
	{
		// 2-1. Update the State of ADC if it is not ERROR_INTERNAL
		if(pADCHandle->State != ADC_STATE_ERROR_INTERNAL)
		{
			// Set ADC state
			SET_BIT(pADCHandle->State, ADC_STATE_REG_EOC);
		}

		// 2-2. Determine whether any further conversion upcoming on group regular by external trigger, continuous mode or scan sequence on going.
		if(ADC_IS_SOFTWARE_START_REGULAR(pADCHandle)                   &&
			(pADCHandle->Init.ContinuousConvMode == DISABLE)            &&
			(IS_BIT_CLR(pADCHandle->Instance->SQR1, ADC_SQR1_L) ||
			IS_BIT_CLR(pADCHandle->Instance->CR2, ADC_CR2_EOCS)  )   )
		{
			if (IS_BIT_CLR(pADCHandle->State, ADC_STATE_REG_BUSY))
			{

				SET_BIT(pADCHandle->State, ADC_STATE_READY);
			}
		}

		// 2-3. Call Conversion Complete Callback
		ADC_ConvCpltCallback(pADCHandle);

		// 2-4. Clear regular group conversion flag
		ADC_CLEAR_FLAG(pADCHandle, ADC_FLAG_STRT | ADC_FLAG_EOC);
	}


	// 3. Check EOC flag of ADC & its interrupt source
	tmp1 = ADC_GET_FLAG(pADCHandle, ADC_FLAG_OVR);
	tmp2 = ADC_GET_IT_SOURCE(pADCHandle, ADC_IT_OVR);

	// 4. Check if Interrupt Handler is called by OVerrun Error for Regular channels
	if(tmp1 && tmp2)
	{
		// 4-1. Clear ADC Overrun error
		ADC_CLEAR_FLAG(pADCHandle, ADC_FLAG_OVR);

		// 4-2. Call Error Callback
		ADC_ErrorCallback(pADCHandle);

		// 4-3. Clear ADC Overrun error
		ADC_CLEAR_FLAG(pADCHandle, ADC_FLAG_OVR);
	}
}



__weak void ADC_ConvCpltCallback(ADC_HandleTypeDef* pADCHandle)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(pADCHandle);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the ADC_ConvCpltCallback could be implemented in the user file
   */
}


__weak void ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* pADCHandle)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(pADCHandle);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the ADC_ConvHalfCpltCallback could be implemented in the user file
   */
}


__weak void ADC_ErrorCallback(ADC_HandleTypeDef *pADCHandle)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(pADCHandle);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the ADC_ErrorCallback could be implemented in the user file
   */
}



static void ADC_DMAConvCplt(DMA_HandleTypeDef *pDMAHandle)
{
	// 1. Retrieve ADC handle corresponding to current DMA handle
	ADC_HandleTypeDef* pADCHandle = ( ADC_HandleTypeDef* )((DMA_HandleTypeDef* )pDMAHandle)->Parent;


	// 2. Update state machine on conversion status if not in error state
	if (IS_BIT_CLR(pADCHandle->State, ADC_STATE_ERROR_INTERNAL | ADC_STATE_ERROR_DMA))
	{
		// 2-1. Update ADC state machine
		SET_BIT(pADCHandle->State, ADC_STATE_REG_EOC);

		// 2-2. Determine whether any further conversion upcoming on group regular by external trigger, continuous mode or scan sequence on going.
		if(ADC_IS_SOFTWARE_START_REGULAR(pADCHandle)                   &&
		       (pADCHandle->Init.ContinuousConvMode == DISABLE)            &&
		       (IS_BIT_CLR(pADCHandle->Instance->SQR1, ADC_SQR1_L) ||
		        IS_BIT_CLR(pADCHandle->Instance->CR2, ADC_CR2_EOCS)  )   )
		{
			/* Disable ADC end of single conversion interrupt on group regular */
			ADC_DISABLE_IT(pADCHandle, ADC_IT_EOC);

			/* Set ADC state */
			CLEAR_BIT(pADCHandle->State, ADC_STATE_REG_BUSY);
		}

		ADC_ConvCpltCallback(pADCHandle);
	}

	// 3. If DMA and-or internal error occurred
	else
	{
		// 3-1. Call ADC Error Callback function
		if((pADCHandle->State & ADC_STATE_ERROR_INTERNAL) != 0UL)
		{
			ADC_ErrorCallback(pADCHandle);
		}
		// 3-2. Call DMA Error Callback function
		else
		{
			pADCHandle->DMA_Handle->XferErrorCallback(pDMAHandle);
		}
	}
}


static void ADC_DMAHalfConvCplt(DMA_HandleTypeDef *pDMAHandle)
{
	ADC_HandleTypeDef* pADCHandle = ( ADC_HandleTypeDef* )((DMA_HandleTypeDef* )pDMAHandle)->Parent;

	// Half conversion callback
	ADC_ConvHalfCpltCallback(pADCHandle);
}


static void ADC_DMAError(DMA_HandleTypeDef *pDMAHandle)
{
	ADC_HandleTypeDef* pADCHandle = ( ADC_HandleTypeDef* )((DMA_HandleTypeDef* )pDMAHandle)->Parent;

	// 1. Set the ADC state to DMA Error
	pADCHandle->State= ADC_STATE_ERROR_DMA;

	// 2. Call Error callback
	ADC_ErrorCallback(pADCHandle);
}









