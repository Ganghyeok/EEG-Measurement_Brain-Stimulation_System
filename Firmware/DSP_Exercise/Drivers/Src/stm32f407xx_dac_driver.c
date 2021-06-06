/*
 * stm32f407xx_dac_driver.c
 *
 *  Created on: 2021. 3. 11.
 *      Author: Ganghyeok Lim
 */

#include "stm32f407xx.h"



static void DAC_DMAConvCpltCh1(DMA_HandleTypeDef *pDMAHandle);
static void DAC_DMAHalfConvCpltCh1(DMA_HandleTypeDef *pDMAHandle);
static void DAC_DMAErrorCh1(DMA_HandleTypeDef *pDMAHandle);
static void DAC_DMAConvCpltCh2(DMA_HandleTypeDef *pDMAHandle);
static void DAC_DMAHalfConvCpltCh2(DMA_HandleTypeDef *pDMAHandle);
static void DAC_DMAErrorCh2(DMA_HandleTypeDef *pDMAHandle);

/**************************************************************************************************************
 * 																											  *
 * 											APIs supported by this driver									  *
 * 						For more information about the APIs, Check the function definitions					  *
 * 									  																		  *
 **************************************************************************************************************/

void DAC_Init(DAC_HandleTypeDef* pDACHandle)
{
	// 1. If DAC State is Reset
	if(pDACHandle->State == DAC_STATE_RESET)
	{
		// Init the Low level Hardware
		DAC_MspInit(pDACHandle);
	}

	// 2. Initialize the DAC State
	pDACHandle->State = DAC_STATE_BUSY;

	pDACHandle->State = DAC_STATE_READY;
}



__weak void DAC_MspInit(DAC_HandleTypeDef* pDACHandle)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(pDACHandle);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the DAC_MspInit could be implemented in the user file
   */
}



void DAC_Start_DMA(DAC_HandleTypeDef* pDACHandle, uint32_t Channel, uint32_t* pData, uint32_t Length, uint32_t Alignment)
{
	uint32_t tmpreg = 0U;

	// 1. Change DAC State
	pDACHandle->State = DAC_STATE_BUSY;

	// 2. If DAC Channel is Channel 1
	if(Channel == DAC_CHANNEL_1)
	{
		// 1-1. Set the DMA transfer complete callback for channel1
		pDACHandle->DMA_Handle1->XferCpltCallback = DAC_DMAConvCpltCh1;

		// 1-2. Set the DMA half transfer complete callback for channel1
		pDACHandle->DMA_Handle1->XferHalfCpltCallback = DAC_DMAHalfConvCpltCh1;

		// 1-3. Set the DMA error callback for channel1
		pDACHandle->DMA_Handle1->XferErrorCallback = DAC_DMAErrorCh1;

		// 1-4. Enable the selected DAC channel1 DMA request
		pDACHandle->Instance->CR |= DAC_CR_DMAEN1;

		// 1-5. Case of use of channel 1
		switch(Alignment)
		{
			case DAC_ALIGN_12B_R:
			{
		        /* Get DHR12R1 address */
		        tmpreg = (uint32_t)&pDACHandle->Instance->DHR12R1;
		        break;
			}

			case DAC_ALIGN_12B_L:
			{
		        /* Get DHR12L1 address */
		        tmpreg = (uint32_t)&pDACHandle->Instance->DHR12L1;
		        break;
			}

			case DAC_ALIGN_8B_R:
			{
		        /* Get DHR8R1 address */
		        tmpreg = (uint32_t)&pDACHandle->Instance->DHR8R1;
		        break;
			}

			default:
				break;
		}
	}

	// 3. If DAC Channel is Channel 2
	else
	{
		// 3-1. Set the DMA transfer complete callback for channel2
		pDACHandle->DMA_Handle2->XferCpltCallback = DAC_DMAConvCpltCh2;

		// 3-2. Set the DMA half transfer complete callback for channel2
		pDACHandle->DMA_Handle2->XferHalfCpltCallback = DAC_DMAHalfConvCpltCh2;

		// 3-3. Set the DMA error callback for channel2
		pDACHandle->DMA_Handle2->XferErrorCallback = DAC_DMAErrorCh2;

		// 3-4. Enable the selected DAC channel2 DMA request
		pDACHandle->Instance->CR |= DAC_CR_DMAEN2;

		// 3-5. Case of use of channel 2
		switch(Alignment)
		{
			case DAC_ALIGN_12B_R:
			{
		        /* Get DHR12R2 address */
		        tmpreg = (uint32_t)&pDACHandle->Instance->DHR12R2;
		        break;
			}

			case DAC_ALIGN_12B_L:
			{
		        /* Get DHR12L2 address */
		        tmpreg = (uint32_t)&pDACHandle->Instance->DHR12L2;
		        break;
			}

			case DAC_ALIGN_8B_R:
			{
		        /* Get DHR8R2 address */
		        tmpreg = (uint32_t)&pDACHandle->Instance->DHR8R2;
		        break;
			}

			default:
				break;
		}
	}

	// 4. Enable the DMA Stream
	if(Channel == DAC_CHANNEL_1)
	{
		/* Enable the DAC DMA underrun interrupt */
		DAC_ENABLE_IT(pDACHandle, DAC_IT_DMAUDR1);

		/* Enable the DMA Stream */
		DMA_Start_IT(pDACHandle->DMA_Handle1, (uint32_t)pData, tmpreg, Length);
	}
	else
	{
		/* Enable the DAC DMA underrun interrupt */
		DAC_ENABLE_IT(pDACHandle, DAC_IT_DMAUDR2);

		/* Enable the DMA Stream */
		DMA_Start_IT(pDACHandle->DMA_Handle2, (uint32_t)pData, tmpreg, Length);
	}

	// 5. Enable the Peripheral
	DAC_ENABLE(pDACHandle, Channel);
}



void DAC_ConfigChannel(DAC_HandleTypeDef* pDACHandle, DAC_ChannelConfTypeDef* sConfig, uint32_t Channel)
{
	uint32_t tmpreg1 = 0U, tmpreg2 = 0U;

	// 1. Change DAC state
	pDACHandle->State = DAC_STATE_BUSY;

	// 2. Get the DAC CR value
	tmpreg1 = pDACHandle->Instance->CR;

	// 3. Clear BOFFx, TENx, TSELx, WAVEx and MAMPx bits
	tmpreg1 &= ~(((uint32_t)(DAC_CR_MAMP1 | DAC_CR_WAVE1 | DAC_CR_TSEL1 | DAC_CR_TEN1 | DAC_CR_BOFF1)) << Channel);

	// 4. Configure for the selected DAC channel: buffer output, trigger
	// 	  Set TSELx and TENx bits according to DAC_Trigger value
	//    Set BOFFx bit according to DAC_OutputBuffer value
	tmpreg2 = (sConfig->DAC_Trigger | sConfig->DAC_OutputBuffer);

	// 5. Calculate CR register value depending on DAC_Channel
	tmpreg1 |= tmpreg2 << Channel;

	// 6. Write to DAC CR
	pDACHandle->Instance->CR = tmpreg1;

	// 7. Disable wave generation
	pDACHandle->Instance->CR &= ~(DAC_CR_WAVE1 << Channel);

	// 8. Change DAC state
	pDACHandle->State = DAC_STATE_READY;
}


__weak void DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* pDACHandle)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(pDACHandle);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the DAC_ConvCpltCallback could be implemented in the user file
   */
}


__weak void DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef* pDACHandle)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(pDACHandle);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the DAC_ConvHalfCpltCallbackCh1 could be implemented in the user file
   */
}


__weak void DAC_ErrorCallbackCh1(DAC_HandleTypeDef *pDACHandle)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(pDACHandle);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the DAC_ErrorCallbackCh1 could be implemented in the user file
   */
}


__weak void DACEx_ConvCpltCallbackCh2(DAC_HandleTypeDef* pDACHandle)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(pDACHandle);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the DAC_ConvCpltCallback could be implemented in the user file
   */
}


__weak void DACEx_ConvHalfCpltCallbackCh2(DAC_HandleTypeDef* pDACHandle)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(pDACHandle);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the DAC_ConvHalfCpltCallbackCh2 could be implemented in the user file
   */
}


__weak void DACEx_ErrorCallbackCh2(DAC_HandleTypeDef *pDACHandle)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(pDACHandle);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the DAC_ErrorCallback could be implemented in the user file
   */
}


__weak void DACEx_DMAUnderrunCallbackCh2(DAC_HandleTypeDef *pDACHandle)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(pDACHandle);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the DAC_DMAUnderrunCallbackCh2 could be implemented in the user file
   */
}


static void DAC_DMAConvCpltCh1(DMA_HandleTypeDef *pDMAHandle)
{
	DAC_HandleTypeDef* pDACHandle = ( DAC_HandleTypeDef* )((DMA_HandleTypeDef* )pDMAHandle)->Parent;

	DAC_ConvCpltCallbackCh1(pDACHandle);
	pDACHandle->State = DAC_STATE_READY;
}


static void DAC_DMAHalfConvCpltCh1(DMA_HandleTypeDef *pDMAHandle)
{
	DAC_HandleTypeDef* pDACHandle = ( DAC_HandleTypeDef* )((DMA_HandleTypeDef* )pDMAHandle)->Parent;

	/* Conversion complete callback */
	DAC_ConvHalfCpltCallbackCh1(pDACHandle);
}


static void DAC_DMAErrorCh1(DMA_HandleTypeDef *pDMAHandle)
{
	DAC_HandleTypeDef* pDACHandle = ( DAC_HandleTypeDef* )((DMA_HandleTypeDef* )pDMAHandle)->Parent;

	/* Set DAC error code to DMA error */
	pDACHandle->ErrorCode |= DAC_ERROR_DMA;

	DAC_ErrorCallbackCh1(pDACHandle);
	pDACHandle->State = DAC_STATE_READY;
}


static void DAC_DMAConvCpltCh2(DMA_HandleTypeDef *pDMAHandle)
{
	DAC_HandleTypeDef* pDACHandle = ( DAC_HandleTypeDef* )((DMA_HandleTypeDef* )pDMAHandle)->Parent;

	DACEx_ConvCpltCallbackCh2(pDACHandle);
	pDACHandle->State = DAC_STATE_READY;
}


static void DAC_DMAHalfConvCpltCh2(DMA_HandleTypeDef *pDMAHandle)
{
    DAC_HandleTypeDef* pDACHandle = ( DAC_HandleTypeDef* )((DMA_HandleTypeDef* )pDMAHandle)->Parent;

    /* Conversion complete callback */
    DACEx_ConvHalfCpltCallbackCh2(pDACHandle);
}


static void DAC_DMAErrorCh2(DMA_HandleTypeDef *pDMAHandle)
{
	DAC_HandleTypeDef* pDACHandle = ( DAC_HandleTypeDef* )((DMA_HandleTypeDef* )pDMAHandle)->Parent;

	/* Set DAC error code to DMA error */
	pDACHandle->ErrorCode |= DAC_ERROR_DMA;

	DACEx_ErrorCallbackCh2(pDACHandle);
	pDACHandle->State = DAC_STATE_READY;
}

