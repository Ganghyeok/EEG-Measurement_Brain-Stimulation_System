/*
 * stm32f407xx_gpio_driver.c
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

void GPIO_Init(GPIO_TypeDef *GPIOx, GPIO_InitTypeDef *GPIO_Init)
{

	uint32_t position;
	uint32_t ioposition = 0x00U;
	uint32_t iocurrent = 0x00U;
	uint32_t temp = 0x00U;

	/* GPIO Clock enable */
	GPIOx_ClockEnable(GPIOx);

	for(position = 0U; position < GPIO_NUMBER; position++)
	{
		/* Get the IO position */
		ioposition = 0x01U << position;

		/* Get the current IO position */
		iocurrent = (uint32_t)(GPIO_Init->Pin) & ioposition;

		if(iocurrent == ioposition)
		{
			/*--------------------- GPIO Mode Configuration ------------------------*/

			/* In case of Output or Alternate function mode selection */
			if((GPIO_Init->Mode == GPIO_MODE_OUTPUT_PP) || (GPIO_Init->Mode == GPIO_MODE_AF_PP) ||
				  (GPIO_Init->Mode == GPIO_MODE_OUTPUT_OD) || (GPIO_Init->Mode == GPIO_MODE_AF_OD))
			{
				temp = GPIOx->OSPEEDR;
				temp &= ~(GPIO_OSPEEDER_OSPEEDR0 << (position * 2U));
				temp |= (GPIO_Init->Speed << (position * 2U));
				GPIOx->OSPEEDR = temp;

				/* Configure the IO Output Type */
				temp = GPIOx->OTYPER;
				temp &= ~(GPIO_OTYPER_OT_0 << position) ;
				temp |= (((GPIO_Init->Mode & GPIO_OUTPUT_TYPE) >> 4U) << position);
				GPIOx->OTYPER = temp;
			}

			/* Activate the Pull-up or Pull down resistor for the current IO */
			temp = GPIOx->PUPDR;
			temp &= ~(GPIO_PUPDR_PUPDR0 << (position * 2U));
			temp |= ((GPIO_Init->Pull) << (position * 2U));
			GPIOx->PUPDR = temp;

			/* In case of Alternate function mode selection */
			if((GPIO_Init->Mode == GPIO_MODE_AF_PP) || (GPIO_Init->Mode == GPIO_MODE_AF_OD))
			{
				/* Configure Alternate function mapped with the current IO */
				temp = GPIOx->AFR[position >> 3U];
				temp &= ~(0xFU << ((uint32_t)(position & 0x07U) * 4U)) ;
				temp |= ((uint32_t)(GPIO_Init->Alternate) << (((uint32_t)position & 0x07U) * 4U));
				GPIOx->AFR[position >> 3U] = temp;
			}

			/* Configure IO Direction mode (Input, Output, Alternate or Analog) */
			temp = GPIOx->MODER;
			temp &= ~(GPIO_MODER_MODER0 << (position * 2U));
			temp |= ((GPIO_Init->Mode & GPIO_MODE) << (position * 2U));
			GPIOx->MODER = temp;


			/*--------------------- EXTI Mode Configuration ------------------------*/

			/* Configure the External Interrupt or event for the current IO */
			if((GPIO_Init->Mode & EXTI_MODE) == EXTI_MODE)
			{
				/* Enable SYSCFG Clock */
				RCC_SYSCFG_CLK_ENABLE();

				temp = SYSCFG->EXTICR[position >> 2U];
				temp &= ~(0x0FU << (4U * (position & 0x03U)));
				temp |= ((uint32_t)(GPIO_GET_INDEX(GPIOx)) << (4U * (position & 0x03U)));
				SYSCFG->EXTICR[position >> 2U] = temp;

				/* Clear EXTI line configuration */
				temp = EXTI->IMR;
				temp &= ~((uint32_t)iocurrent);
				if((GPIO_Init->Mode & GPIO_MODE_IT) == GPIO_MODE_IT)
				{
					temp |= iocurrent;
				}
				EXTI->IMR = temp;

				temp = EXTI->EMR;
				temp &= ~((uint32_t)iocurrent);
				if((GPIO_Init->Mode & GPIO_MODE_EVT) == GPIO_MODE_EVT)
				{
					temp |= iocurrent;
				}
				EXTI->EMR = temp;

				/* Clear Rising Falling edge configuration */
				temp = EXTI->RTSR;
				temp &= ~((uint32_t)iocurrent);
				if((GPIO_Init->Mode & RISING_EDGE) == RISING_EDGE)
				{
					temp |= iocurrent;
				}
				EXTI->RTSR = temp;

				temp = EXTI->FTSR;
				temp &= ~((uint32_t)iocurrent);
				if((GPIO_Init->Mode & FALLING_EDGE) == FALLING_EDGE)
				{
					temp |= iocurrent;
				}
				EXTI->FTSR = temp;
			}
		}
	}
}


void GPIO_DeInit(GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin)
{
	// to be implemented
}



/********************************** IO operation functions **********************************/


uint8_t GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	if((uint16_t)GPIOx->IDR & GPIO_Pin)		return GPIO_PIN_SET;
	else									return GPIO_PIN_RESET;
}


void GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint8_t PinState)
{
	if(PinState == GPIO_PIN_SET)
	{
		GPIOx->BSRR |= GPIO_Pin;
	}
	else
	{
		GPIOx->BSRR |= ((uint32_t)GPIO_Pin << 16);
	}
}


void GPIO_ModifyPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin_To_Set, uint16_t GPIO_Pin_To_Reset)
{
	uint32_t config = 0;

	config = ((uint32_t)GPIO_Pin_To_Reset << 16U) | (GPIO_Pin_To_Set);

	GPIOx->BSRR |= config;
}


void GPIO_WritePort(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint8_t PinState)
{
	GPIOx->BSRR = (((0xFFFFU & ~(GPIO_Pin)) << 16U) | (GPIO_Pin));
}


void GPIO_WriteData(GPIO_TypeDef *GPIOx, uint16_t Data)
{
	GPIOx->ODR = (uint16_t)Data;
}


void GPIO_TogglePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	if( (GPIOx->ODR & GPIO_Pin) )
	{
		// State of pin was LOW
		GPIOx->BSRR |= ((uint32_t)GPIO_Pin << 16);
	}
	else
	{
		// State of pin was HIGH
		GPIOx->BSRR |= GPIO_Pin;
	}
}


void EXTI_IRQHandling(uint32_t GPIO_Pin)
{
	/* Interrupt handling for EXTI */
	uint32_t extiSrcPin;

	extiSrcPin = EXTI->PR & GPIO_Pin;

	if(extiSrcPin != 0)
	{
		EXTI->PR |= extiSrcPin;		// Clear the pending event from EXTI line
		EXTI_Callback(extiSrcPin);		// Call EXTI_Callback function
	}
}


__weak void EXTI_Callback(uint32_t ExtiSrcPin)
{
	/* Prevent unused argument(s) compilation warning */
		UNUSED(ExtiSrcPin);

	/* NOTE : This function should not be modified, when the callback is needed,
	 * 		  the EXTI_Callback could be implemented in the user file
	 * 		  (This is a weak implementation. The user application may override this function)
	 */
}
