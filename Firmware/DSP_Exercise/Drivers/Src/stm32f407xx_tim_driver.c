/*
 * stm32f407xx_tim_driver.c
 *
 *  Created on: 2021. 3. 8.
 *      Author: Ganghyeok Lim
 */

#include "stm32f407xx.h"


/**************************************************************************************************************
 * 																											  *
 * 											APIs supported by this driver									  *
 * 						For more information about the APIs, Check the function definitions					  *
 * 									  																		  *
 **************************************************************************************************************/

void TIM_Base_Init(TIM_HandleTypeDef *pTIMHandle)
{
	// 1. Check state of TIMx is RESET
	if(pTIMHandle->State == TIM_STATE_RESET)
	{
		// Init Low level hardware of TIM : GPIO, CLOCK
		TIM_Base_MspInit(pTIMHandle);
	}

	// 2. Set the TIM State
	pTIMHandle->State = TIM_STATE_BUSY;

	// 3. Set the Time Base configuration
	TIM_Base_SetConfig(pTIMHandle->Instance, &pTIMHandle->Init);

	// 4. Init the TIM state
	pTIMHandle->State = TIM_STATE_READY;
}


__weak void TIM_Base_MspInit(TIM_HandleTypeDef *pTIMHandle)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(pTIMHandle);

	/* NOTE : This function should not be modified, when the callback is needed,
            	the TIM_Base_MspInit could be implemented in the user file
	 */
}


void TIM_Base_SetConfig(TIM_TypeDef *TIMx, TIM_Base_InitTypeDef *Structure)
{
	uint32_t tmpcr1;
	tmpcr1 = TIMx->CR1;

	if(IS_TIM_COUNTER_MODE_SELECT_INSTANCE(TIMx))
	{
	    /* Select the Counter Mode */
	    tmpcr1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS);
	    tmpcr1 |= Structure->CounterMode;
	}

	if(IS_TIM_CLOCK_DIVISION_INSTANCE(TIMx))
	{
		/* Set the clock division */
		tmpcr1 &= ~TIM_CR1_CKD;
		tmpcr1 |= (uint32_t)Structure->ClockDivision;
	}

    /* Set the auto-reload preload */
    MODIFY_REG(tmpcr1, TIM_CR1_ARPE, Structure->AutoReloadPreload);

    TIMx->CR1 = tmpcr1;

    /* Set the Autoreload value */
    TIMx->ARR = (uint32_t)Structure->Period;

    /* Set the Prescaler value */
    TIMx->PSC = Structure->Prescaler;

    if(IS_TIM_REPETITION_COUNTER_INSTANCE(TIMx))
    {
		/* Set the Repetition Counter value */
    	TIMx->RCR = Structure->RepetitionCounter;
    }

    // Generate an update event to reload the Prescaler and the repetition counter (only for advanced timer) value immediately
    TIMx->EGR = TIM_EGR_UG;
}



void TIM_Base_Start(TIM_HandleTypeDef *pTIMHandle)
{
	uint32_t tmpsmcr;

	// 1. Set the TIM State
	pTIMHandle->State = TIM_STATE_BUSY;

	// 2. Enable the Peripheral, except in trigger mode where enable is automatically done with trigger
	tmpsmcr = pTIMHandle->Instance->SMCR & TIM_SMCR_SMS;

	if (!IS_TIM_SLAVEMODE_TRIGGER_ENABLED(tmpsmcr))
	{
		TIM_ENABLE(pTIMHandle);
	}

	// 3. Change the TIM State
	pTIMHandle->State = TIM_STATE_READY;
}



void TIM_Base_Start_IT(TIM_HandleTypeDef *pTIMHandle)
{
	uint32_t tmpsmcr;

	// 1. Enable the TIM Update interrupt
	TIM_ENABLE_IT(pTIMHandle, TIM_IT_UPDATE);

	// 2. Enable the Peripheral, except in trigger mode where enable is automatically done with trigger
	tmpsmcr = pTIMHandle->Instance->SMCR & TIM_SMCR_SMS;

	if (!IS_TIM_SLAVEMODE_TRIGGER_ENABLED(tmpsmcr))
	{
		TIM_ENABLE(pTIMHandle);
	}
}



void TIM_PeripheralClockControl(TIM_TypeDef *TIMx, uint8_t En_or_Di)
{
	if(En_or_Di == ENABLE)
	{
		if(TIMx == TIM1)		RCC_TIM1_CLK_ENABLE();
		else if(TIMx == TIM2)	RCC_TIM2_CLK_ENABLE();
		else if(TIMx == TIM3)	RCC_TIM3_CLK_ENABLE();
		else if(TIMx == TIM4)	RCC_TIM4_CLK_ENABLE();
		else if(TIMx == TIM5)	RCC_TIM5_CLK_ENABLE();
		else if(TIMx == TIM6)	RCC_TIM6_CLK_ENABLE();
		else if(TIMx == TIM7)	RCC_TIM7_CLK_ENABLE();
		else if(TIMx == TIM8)	RCC_TIM8_CLK_ENABLE();
		else if(TIMx == TIM9)	RCC_TIM9_CLK_ENABLE();
		else if(TIMx == TIM10)	RCC_TIM10_CLK_ENABLE();
		else if(TIMx == TIM11)	RCC_TIM11_CLK_ENABLE();
		else if(TIMx == TIM12)	RCC_TIM12_CLK_ENABLE();
		else if(TIMx == TIM13)	RCC_TIM13_CLK_ENABLE();
		else if(TIMx == TIM14)	RCC_TIM14_CLK_ENABLE();
	}
	else if(En_or_Di == DISABLE)
	{
		if(TIMx == TIM1)		RCC_TIM1_CLK_DISABLE();
		else if(TIMx == TIM2)	RCC_TIM2_CLK_DISABLE();
		else if(TIMx == TIM3)	RCC_TIM3_CLK_DISABLE();
		else if(TIMx == TIM4)	RCC_TIM4_CLK_DISABLE();
		else if(TIMx == TIM5)	RCC_TIM5_CLK_DISABLE();
		else if(TIMx == TIM6)	RCC_TIM6_CLK_DISABLE();
		else if(TIMx == TIM7)	RCC_TIM7_CLK_DISABLE();
		else if(TIMx == TIM8)	RCC_TIM8_CLK_DISABLE();
		else if(TIMx == TIM9)	RCC_TIM9_CLK_DISABLE();
		else if(TIMx == TIM10)	RCC_TIM10_CLK_DISABLE();
		else if(TIMx == TIM11)	RCC_TIM11_CLK_DISABLE();
		else if(TIMx == TIM12)	RCC_TIM12_CLK_DISABLE();
		else if(TIMx == TIM13)	RCC_TIM13_CLK_DISABLE();
		else if(TIMx == TIM14)	RCC_TIM14_CLK_DISABLE();
	}
}


void TIMEx_MasterConfigSynchronization(TIM_HandleTypeDef *pTIMHandle, TIM_MasterConfigTypeDef *sMasterConfig)
{
	uint32_t tmpcr2;
	uint32_t tmpsmcr;

	// 1. Change the TIM State
	pTIMHandle->State = TIM_STATE_BUSY;

	// 2. Get the TIMx CR2 register value
	tmpcr2 = pTIMHandle->Instance->CR2;

	// 3. Get the TIMx SMCR register value
	tmpsmcr = pTIMHandle->Instance->SMCR;

	// 4. Reset the MMS Bits
	tmpcr2 &= ~TIM_CR2_MMS;

	// 5. Select the TRGO source
	tmpcr2 |=  sMasterConfig->MasterOutputTrigger;

	// 6. Update TIMx CR2
	pTIMHandle->Instance->CR2 = tmpcr2;

	if(IS_TIM_SLAVE_INSTANCE(pTIMHandle->Instance))
	{
		/* Reset the MSM Bit */
		tmpsmcr &= ~TIM_SMCR_MSM;

		/* Set master mode */
		tmpsmcr |= sMasterConfig->MasterSlaveMode;

		/* Update TIMx SMCR */
		pTIMHandle->Instance->SMCR = tmpsmcr;
	}

	// 7. Change the TIM State
	pTIMHandle->State = TIM_STATE_READY;
}


void TIM_IRQHandling(TIM_HandleTypeDef *pTIMHandle)
{
	/* Interrupt handling for TIM */

	uint32_t temp1, temp2;

	// 1. Handle for interrupt generated by Update Event
	temp1 = READ_BIT(pTIMHandle->Instance->SR, TIM_SR_UIF);
	temp2 = READ_BIT(pTIMHandle->Instance->DIER, TIM_DIER_UIE);

	if(temp1 && temp2)
	{
		// This interrupt is generated by Update Event
		CLEAR_FLAG(TIM6->SR, TIM_SR_UIF);

		TIM_PeriodElapsedCallback(pTIMHandle);
	}
}


__weak void TIM_PeriodElapsedCallback(TIM_HandleTypeDef *pTIMHandle)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(pTIMHandle);

	/* NOTE : This function should not be modified, when the callback is needed,
				the TIM_PeriodElapsedCallback could be implemented in the user file
	 */
}










