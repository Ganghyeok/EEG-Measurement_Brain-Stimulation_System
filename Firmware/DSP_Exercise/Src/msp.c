/*
 * msp.c
 *
 *  Created on: Mar 7, 2021
 *      Author: Ganghyeok Lim
 */

#include "main.h"


void TIM_Base_MspInit(TIM_HandleTypeDef *pTIMHandle)
{
	if(pTIMHandle->Instance == TIM6)
	{
		// 1. Configure GPIO for TIM6
		// TIM6 is used for just time base generation so that GPIO config is not needed

		// 2. Configure CLOCK for TIM6
		RCC_TIM6_CLK_ENABLE();

		// 3. Configure NVIC for TIM6
		NVIC_IRQConfig(TIM6_DAC_IRQn, 0, ENABLE);
	}
	else if(pTIMHandle->Instance == TIM7)
	{
		// 1. Configure GPIO for TIM7
		// TIM7 is used for just time base generation so that GPIO config is not needed

		// 2. Configure CLOCK for TIM7
		RCC_TIM7_CLK_ENABLE();

		// 3. Configure NVIC for TIM7
		NVIC_IRQConfig(TIM7_IRQn, 0, ENABLE);
	}
}


void UART_MspInit(UART_HandleTypeDef *pUSARTHandle)
{
	if(pUSARTHandle->Instance == USART1)
	{
		GPIO_InitTypeDef GPIO_InitStruct = {0};

		// 1. USART1 Clock Enable
		RCC_USART1_CLK_ENABLE();

		// 2. GPIO Clock Enable
	   	RCC_GPIOB_CLK_ENABLE();

	   	// 3. USART1 GPIO Configuration (PB6 -> USART1_TX, PB7 -> USART1_RX)
	    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
	    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	    GPIO_Init(GPIOB, &GPIO_InitStruct);

	    // 4. USART1 DMA Init
	    DMA2Handle_UART.Instance = DMA2_Stream7;
	    DMA2Handle_UART.Init.Channel = DMA_CHANNEL_4;
	    DMA2Handle_UART.Init.Direction = DMA_MEMORY_TO_PERIPH;
	    DMA2Handle_UART.Init.PeriphInc = DMA_PINC_DISABLE;
	    DMA2Handle_UART.Init.MemInc = DMA_MINC_ENABLE;
	    DMA2Handle_UART.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	    DMA2Handle_UART.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	    DMA2Handle_UART.Init.Mode = DMA_NORMAL;
	    DMA2Handle_UART.Init.Priority = DMA_PRIORITY_LOW;
	    DMA2Handle_UART.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
	    DMA2Handle_UART.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
	    DMA2Handle_UART.Init.MemBurst = DMA_MBURST_SINGLE;
	    DMA2Handle_UART.Init.PeriphBurst = DMA_PBURST_SINGLE;
	    DMA_Init(&DMA2Handle_UART);

	    LINKDMA(pUSARTHandle,hdmatx,DMA2Handle_UART);

	    // 5. USART1 NVIC Configuration
	    NVIC_IRQConfig(USART1_IRQn, 2, ENABLE);
	}
}



void ADC_MspInit(ADC_HandleTypeDef* pADCHandle)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	if(pADCHandle->Instance == ADC1)
	{
		// 1. ADC1 Clock Enable
		RCC_ADC1_CLK_ENABLE();

		// 2. GPIO Clock Enable
		RCC_GPIOA_CLK_ENABLE();

		// 3. ADC1 GPIO Configuration (PA0-WKUP -> ADC1_IN0)
	    GPIO_InitStruct.Pin = GPIO_PIN_0;
	    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_Init(GPIOA, &GPIO_InitStruct);

	    // 4. ADC1 DMA Init
	    DMA2Handle_ADC.Instance = DMA2_Stream4;
	    DMA2Handle_ADC.Init.Channel = DMA_CHANNEL_0;
	    DMA2Handle_ADC.Init.Direction = DMA_PERIPH_TO_MEMORY;
	    DMA2Handle_ADC.Init.PeriphInc = DMA_PINC_DISABLE;
	    DMA2Handle_ADC.Init.MemInc = DMA_MINC_ENABLE;
	    DMA2Handle_ADC.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	    DMA2Handle_ADC.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
	    DMA2Handle_ADC.Init.Mode = DMA_NORMAL;
	    DMA2Handle_ADC.Init.Priority = DMA_PRIORITY_HIGH;
	    DMA2Handle_ADC.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	    DMA_Init(&DMA2Handle_ADC);

	    LINKDMA(pADCHandle,DMA_Handle,DMA2Handle_ADC);

	    // 5. ADC NVIC Configuration
	    NVIC_IRQConfig(ADC_IRQn, 0, ENABLE);
	}
}


void DAC_MspInit(DAC_HandleTypeDef* pDACHandle)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	if(pDACHandle->Instance == DAC)
	{
		// 1. Enable DAC Clock
		RCC_DAC_CLK_ENABLE();

		// 2. Enable GPIOA Clock
		RCC_GPIOA_CLK_ENABLE();

		// 3. DAC GPIO Configuration (PA4 -> DAC_OUT1)
	    GPIO_InitStruct.Pin = GPIO_PIN_4;
	    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_Init(GPIOA, &GPIO_InitStruct);

	    // 4. DAC DMA Init
	    DMA1Handle_DAC.Instance = DMA1_Stream5;
	    DMA1Handle_DAC.Init.Channel = DMA_CHANNEL_7;
	    DMA1Handle_DAC.Init.Direction = DMA_MEMORY_TO_PERIPH;
	    DMA1Handle_DAC.Init.PeriphInc = DMA_PINC_DISABLE;
	    DMA1Handle_DAC.Init.MemInc = DMA_MINC_ENABLE;
	    DMA1Handle_DAC.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
	    DMA1Handle_DAC.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
	    DMA1Handle_DAC.Init.Mode = DMA_CIRCULAR;						// DMA_CIRCULAR
	    DMA1Handle_DAC.Init.Priority = DMA_PRIORITY_MEDIUM;
	    DMA1Handle_DAC.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	    DMA_Init(&DMA1Handle_DAC);

	    LINKDMA(pDACHandle,DMA_Handle1,DMA1Handle_DAC);
	}
}


void TFT_MspInit(TFT_HandleTypeDef *pTFTHandle)
{
	GPIO_InitTypeDef GPIOInit;

	memset(&GPIOInit, 0, sizeof(GPIOInit));

	if(pTFTHandle->Instance == TFT1)
	{
		/************************************************************************
		 *		  Low level init GPIO of TFT Control pins and Data pins			*
		 ***********************************************************************/

		// 1. Initialize GPIO for Control Pins (nRST, nCS, RS, nWR) to GPIO Output Push-Pull mode
		pTFTHandle->Init.GPIOx_TFT_Control = GPIOA;
		pTFTHandle->Init.GPIO_Pin_TFT_nRST = GPIO_PIN_8;
		pTFTHandle->Init.GPIO_Pin_TFT_nCS = GPIO_PIN_9;
		pTFTHandle->Init.GPIO_Pin_TFT_RS = GPIO_PIN_10;
		pTFTHandle->Init.GPIO_Pin_TFT_nWR = GPIO_PIN_11;
		pTFTHandle->Init.GPIO_Pins_TFT_Control = pTFTHandle->Init.GPIO_Pin_TFT_nRST | pTFTHandle->Init.GPIO_Pin_TFT_nCS | pTFTHandle->Init.GPIO_Pin_TFT_RS | pTFTHandle->Init.GPIO_Pin_TFT_nWR;

		GPIOInit.Pin = pTFTHandle->Init.GPIO_Pins_TFT_Control;
		GPIOInit.Mode = GPIO_MODE_OUTPUT_PP;
		GPIOInit.Pull = GPIO_NOPULL;
		GPIOInit.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_Init(pTFTHandle->Init.GPIOx_TFT_Control, &GPIOInit);
		Delay_ms(10);

		GPIO_ModifyPin(pTFTHandle->Init.GPIOx_TFT_Control, pTFTHandle->Init.GPIO_Pin_TFT_nRST | pTFTHandle->Init.GPIO_Pin_TFT_nCS | pTFTHandle->Init.GPIO_Pin_TFT_nWR, pTFTHandle->Init.GPIO_Pin_TFT_RS);

		// 2. Initialize GPIO for Data Pins (D0 ~ D15) to GPIO Output mode
		pTFTHandle->Init.GPIOx_TFT_Data = GPIOD;
		pTFTHandle->Init.GPIO_Pins_TFT_Data = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;

		GPIOInit.Pin = pTFTHandle->Init.GPIO_Pins_TFT_Data;
		GPIOInit.Mode = GPIO_MODE_OUTPUT_PP;
		GPIOInit.Pull = GPIO_NOPULL;
		GPIOInit.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_Init(pTFTHandle->Init.GPIOx_TFT_Data, &GPIOInit);
		Delay_ms(10);

		GPIO_WritePort(pTFTHandle->Init.GPIOx_TFT_Data, pTFTHandle->Init.GPIO_Pins_TFT_Data, GPIO_PIN_RESET);
	}
}

