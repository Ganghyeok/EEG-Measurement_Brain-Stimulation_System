/*
 * stm32f407xx_dma_driver.c
 *
 *  Created on: 2021. 3. 9.
 *      Author: Ganghyeok Lim
 */

#include "stm32f407xx.h"


static uint32_t DMA_CalcBaseAndBitshift(DMA_HandleTypeDef *hdma);
static void DMA_SetConfig(DMA_HandleTypeDef *pDMAHandle, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);

/**************************************************************************************************************
 * 																											  *
 * 											APIs supported by this driver									  *
 * 						For more information about the APIs, Check the function definitions					  *
 * 									  																		  *
 **************************************************************************************************************/

void DMA_Init(DMA_HandleTypeDef *pDMAHandle)
{
	uint32_t config = 0x0U;
	DMA_Base_Registers *regs;

	// 1. Change DMA State to Busy
	pDMAHandle->State = DMA_STATE_BUSY;

	// 2. Disable DMA peripheral
	DMA_DISABLE(pDMAHandle);

	// 3. Check if the DMA Stream is disabled
	if((pDMAHandle->Instance->CR & DMA_SxCR_EN) != RESET)
	{
		pDMAHandle->State = DMA_STATE_ERROR;
		return;
	}

	// 4. Get the Current CR value
	config = pDMAHandle->Instance->CR;

	// 5. Clear CHSEL, MBURST, PBURST, PL, MSIZE, PSIZE, MINC, PINC, CIRC, DIR, CT and DBM bits
	config &= ((uint32_t)~(DMA_SxCR_CHSEL | DMA_SxCR_MBURST | DMA_SxCR_PBURST | \
					  DMA_SxCR_PL    | DMA_SxCR_MSIZE  | DMA_SxCR_PSIZE  | \
					  DMA_SxCR_MINC  | DMA_SxCR_PINC   | DMA_SxCR_CIRC   | \
					  DMA_SxCR_DIR   | DMA_SxCR_CT     | DMA_SxCR_DBM));

	// 6. Prepare the DMA Stream configuration
	config |=  pDMAHandle->Init.Channel             | pDMAHandle->Init.Direction        |
				pDMAHandle->Init.PeriphInc           | pDMAHandle->Init.MemInc           |
				pDMAHandle->Init.PeriphDataAlignment | pDMAHandle->Init.MemDataAlignment |
				pDMAHandle->Init.Mode                | pDMAHandle->Init.Priority;

	/* the Memory burst and peripheral burst are not used when the FIFO is disabled */
	if(pDMAHandle->Init.FIFOMode == DMA_FIFOMODE_ENABLE)
	{
	/* Get memory burst and peripheral burst */
		config |=  pDMAHandle->Init.MemBurst | pDMAHandle->Init.PeriphBurst;
	}

	/* Write to DMA Stream CR register */
	pDMAHandle->Instance->CR = config;

	/* Get the FCR register value */
	config = pDMAHandle->Instance->FCR;

	/* Clear Direct mode and FIFO threshold bits */
	config &= (uint32_t)~(DMA_SxFCR_DMDIS | DMA_SxFCR_FTH);

	/* Prepare the DMA Stream FIFO configuration */
	config |= pDMAHandle->Init.FIFOMode;

	/* The FIFO threshold is not used when the FIFO mode is disabled */
	if(pDMAHandle->Init.FIFOMode == DMA_FIFOMODE_ENABLE)
	{
		/* Get the FIFO threshold */
		config |= pDMAHandle->Init.FIFOThreshold;
	}

	/* Write to DMA Stream FCR */
	pDMAHandle->Instance->FCR = config;

	/* Initialize StreamBaseAddress and StreamIndex parameters to be used to calculate
	   DMA steam Base Address needed by HAL_DMA_IRQHandler() and HAL_DMA_PollForTransfer() */
	regs = (DMA_Base_Registers *)DMA_CalcBaseAndBitshift(pDMAHandle);

	/* Clear all interrupt flags */
	regs->IFCR = 0x3FU << pDMAHandle->StreamIndex;

	/* Initialize the error code */
	pDMAHandle->ErrorCode = DMA_ERROR_NONE;

	/* Initialize the DMA state */
	pDMAHandle->State = DMA_STATE_READY;
}


void DMA_Start_IT(DMA_HandleTypeDef *pDMAHandle, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength)
{
	/* calculate DMA base and stream number */
	DMA_Base_Registers *regs = (DMA_Base_Registers *)pDMAHandle->StreamBaseAddress;

	if(pDMAHandle->State == DMA_STATE_READY)
	{
	    /* Change DMA peripheral state */
		pDMAHandle->State = DMA_STATE_BUSY;

	    /* Initialize the error code */
		pDMAHandle->ErrorCode = DMA_ERROR_NONE;

	    /* Configure the source, destination address and the data length */
	    DMA_SetConfig(pDMAHandle, SrcAddress, DstAddress, DataLength);

	    /* Clear all interrupt flags at correct offset within the register */
	    regs->IFCR = 0x3FU << pDMAHandle->StreamIndex;

	    /* Enable Common interrupts*/
	    pDMAHandle->Instance->CR  |= DMA_IT_TC | DMA_IT_TE | DMA_IT_DME;

	    if(pDMAHandle->XferHalfCpltCallback != NULL)
		{
	    	pDMAHandle->Instance->CR  |= DMA_IT_HT;
		}

	    /* Enable the Peripheral */
	    DMA_ENABLE(pDMAHandle);
	}
	else
	{

	}
}


void DMA_RegisterCallback(DMA_HandleTypeDef *pDMAHandle, DMA_CallbackIDTypeDef CallbackID, void (* pCallback)(DMA_HandleTypeDef *_pDMAHandle))
{
	if(pDMAHandle->State == DMA_STATE_READY)
	{
		switch (CallbackID)
		{
			case  DMA_XFER_CPLT_CB_ID:
					pDMAHandle->XferCpltCallback = pCallback;
					break;

			case  DMA_XFER_HALFCPLT_CB_ID:
					pDMAHandle->XferHalfCpltCallback = pCallback;
					break;

			case  DMA_XFER_M1CPLT_CB_ID:
					pDMAHandle->XferM1CpltCallback = pCallback;
					break;

			case  DMA_XFER_M1HALFCPLT_CB_ID:
					pDMAHandle->XferM1HalfCpltCallback = pCallback;
					break;

			case  DMA_XFER_ERROR_CB_ID:
					pDMAHandle->XferErrorCallback = pCallback;
					break;

			case  DMA_XFER_ABORT_CB_ID:
					pDMAHandle->XferAbortCallback = pCallback;
					break;

			default:
				  break;
		}
	}

}


void DMA_IRQHandler(DMA_HandleTypeDef *pDMAHandle)
{
	uint32_t tmpisr;

	/* calculate DMA base and stream number */
	DMA_Base_Registers *regs = (DMA_Base_Registers *)pDMAHandle->StreamBaseAddress;

	tmpisr = regs->ISR;

	/* Transfer Complete Interrupt management ***********************************/
	if((tmpisr & (DMA_FLAG_TCIF0_4 << pDMAHandle->StreamIndex)) != RESET)
	{
		if(DMA_GET_IT_SOURCE(pDMAHandle, DMA_IT_TC) != RESET)
		{
			/* Clear the transfer complete flag */
			regs->IFCR = DMA_FLAG_TCIF0_4 << pDMAHandle->StreamIndex;

			/* Disable the transfer complete interrupt */
			pDMAHandle->Instance->CR  &= ~(DMA_IT_TC);

			/* Change the DMA state */
			pDMAHandle->State = DMA_STATE_READY;

	        if(pDMAHandle->XferCpltCallback != NULL)
	        {
				/* Transfer complete callback */
	        	pDMAHandle->XferCpltCallback(pDMAHandle);
	        }
		}
	}



	/* FIFO Error Interrupt management ******************************************/
	if ((tmpisr & (DMA_FLAG_FEIF0_4 << pDMAHandle->StreamIndex)) != RESET)
	{
		asm("NOP");
	}

	/* Direct Mode Error Interrupt management ***********************************/
	if ((tmpisr & (DMA_FLAG_DMEIF0_4 << pDMAHandle->StreamIndex)) != RESET)
	{
		asm("NOP");
	}

	/* Half Transfer Complete Interrupt management ******************************/
	if ((tmpisr & (DMA_FLAG_HTIF0_4 << pDMAHandle->StreamIndex)) != RESET)
	{
		if(DMA_GET_IT_SOURCE(pDMAHandle, DMA_IT_HT) != RESET)
		{
			/* Clear the half transfer complete flag */
			regs->IFCR = DMA_FLAG_HTIF0_4 << pDMAHandle->StreamIndex;

			/* Multi_Buffering mode enabled */
			if(((pDMAHandle->Instance->CR) & (uint32_t)(DMA_SxCR_DBM)) != RESET)
			{
				/* Current memory buffer used is Memory 0 */
				if((pDMAHandle->Instance->CR & DMA_SxCR_CT) == RESET)
				{
					if(pDMAHandle->XferHalfCpltCallback != NULL)
					{
						/* Half transfer callback */
						pDMAHandle->XferHalfCpltCallback(pDMAHandle);
					}
				}

				/* Current memory buffer used is Memory 1 */
		        else
		        {
					if(pDMAHandle->XferM1HalfCpltCallback != NULL)
					{
						/* Half transfer callback */
						pDMAHandle->XferM1HalfCpltCallback(pDMAHandle);
					}
		        }
			}
			/* Disable the half transfer interrupt if the DMA mode is not CIRCULAR */
			else
			{
				/* Disable the half transfer interrupt if the DMA mode is not CIRCULAR */
				if((pDMAHandle->Instance->CR & DMA_SxCR_CIRC) == RESET)
				{
					/* Disable the half transfer interrupt */
					pDMAHandle->Instance->CR  &= ~(DMA_IT_HT);
				}

				if(pDMAHandle->XferHalfCpltCallback != NULL)
				{
					/* Half transfer callback */
					pDMAHandle->XferHalfCpltCallback(pDMAHandle);
				}
			}
		}
	}
}



static uint32_t DMA_CalcBaseAndBitshift(DMA_HandleTypeDef *pDMAHandle)
{
	uint32_t stream_number = (((uint32_t)pDMAHandle->Instance & 0xFFU) - 16U) / 24U;

	/* lookup table for necessary bitshift of flags within status registers */
	static const uint8_t flagBitshiftOffset[8U] = {0U, 6U, 16U, 22U, 0U, 6U, 16U, 22U};
	pDMAHandle->StreamIndex = flagBitshiftOffset[stream_number];

	if (stream_number > 3U)
	{
		/* return pointer to HISR and HIFCR */
		pDMAHandle->StreamBaseAddress = (((uint32_t)pDMAHandle->Instance & (uint32_t)(~0x3FFU)) + 4U);
	}
	else
	{
		/* return pointer to LISR and LIFCR */
		pDMAHandle->StreamBaseAddress = ((uint32_t)pDMAHandle->Instance & (uint32_t)(~0x3FFU));
	}

	return pDMAHandle->StreamBaseAddress;
}


static void DMA_SetConfig(DMA_HandleTypeDef *pDMAHandle, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength)
{
	/* Clear DBM bit */
	pDMAHandle->Instance->CR &= (uint32_t)(~DMA_SxCR_DBM);

	/* Configure DMA Stream data length */
	pDMAHandle->Instance->NDTR = DataLength;

	/* Memory to Peripheral */
	if((pDMAHandle->Init.Direction) == DMA_MEMORY_TO_PERIPH)
	{
		/* Configure DMA Stream destination address */
		pDMAHandle->Instance->PAR = DstAddress;

		/* Configure DMA Stream source address */
		pDMAHandle->Instance->M0AR = SrcAddress;
	}

	/* Peripheral to Memory */
	else
	{
		/* Configure DMA Stream source address */
		pDMAHandle->Instance->PAR = SrcAddress;

		/* Configure DMA Stream destination address */
		pDMAHandle->Instance->M0AR = DstAddress;
	}
}

