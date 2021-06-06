/*
 * it.c
 *
 *  Created on: Mar 7, 2021
 *      Author: Ganghyeok Lim
 */

#include "main.h"


void TIM6_DAC_IRQHandler(void)
{
	TIM_IRQHandling(&TIM6Handle);
}


void USART1_IRQHandler(void)
{
	UART_IRQHandler(&USART1Handle);
}


void ADC_IRQHandler(void)
{
	ADC_IRQHandling(&ADC1Handle);
}


void DMA1_Stream5_IRQHandler(void)
{
	DMA_IRQHandler(&DMA1Handle_DAC);
}


void DMA2_Stream4_IRQHandler(void)
{
	DMA_IRQHandler(&DMA2Handle_ADC);
}


void DMA2_Stream7_IRQHandler(void)
{
	DMA_IRQHandler(&DMA2Handle_UART);
}


void EXTI0_IRQHandler(void)
{
	EXTI_IRQHandling(GPIO_PIN_0);
//	EXTI->PR |= GPIO_PIN_0;
//
//	KEY1_count++;
//	GPIO_TogglePin(GPIOB, GPIO_PIN_12);
}


void EXTI1_IRQHandler(void)
{
	EXTI_IRQHandling(GPIO_PIN_1);
//	EXTI->PR |= GPIO_PIN_1;
//
//	KEY2_count++;
//	GPIO_TogglePin(GPIOB, GPIO_PIN_12);
}


void EXTI2_IRQHandler(void)
{
	EXTI_IRQHandling(GPIO_PIN_2);
//	EXTI->PR |= GPIO_PIN_2;
//
//	KEY3_count++;
//	GPIO_TogglePin(GPIOB, GPIO_PIN_12);
}


void EXTI3_IRQHandler(void)
{
	EXTI_IRQHandling(GPIO_PIN_3);
//	EXTI->PR |= GPIO_PIN_3;
//
//	KEY4_count++;
//	GPIO_TogglePin(GPIOB, GPIO_PIN_12);
}


void NMI_Handler(void)
{
	while(1);
}


void HardFault_Handler(void)
{
	while(1);
}


void MemManage_Handler(void)
{
	while(1);
}


void BusFault_Handler(void)
{
	while(1);
}


void UsageFault_Handler(void)
{
	while(1);
}
