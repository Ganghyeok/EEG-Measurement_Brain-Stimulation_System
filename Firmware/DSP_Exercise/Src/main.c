/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

#include "main.h"


int main(void)
{
	// 1. Initialize MCU
	System_Init();

	// 2. Memset Handle structure to 0
	MemsetHandleStructure();

	// 3. Initialize On Chip Peripherals
	DMA1_Init();
	DMA2_Init();
	TIM6_Init();
	UART1_Init();
	ADC1_Init();
	DAC1_Init();
	TIM7_Init();

	// 4. Start Timers
	TIM_Base_Start_IT(&TIM6Handle);
	TIM_Base_Start(&TIM7Handle);
	Delay_ms(1000);	// if you do not add this delay, then Filtered output does not show at DAC output pin

	// 5. Initialize Off Chip Peripherals
	TFT1_Init();
	LED1_Init();
	KEY_Init();
	//BUZZER_Init();
	Stimulator_Init();
	Delay_ms(100);


	State = STATE_TIME_DOMAIN;
	FilterState = ENABLE;

	DAC_Start_DMA(&DAC1Handle, DAC1_CHANNEL_1, &DacOut, 1, DAC_ALIGN_12B_R);


	while(1)
	{
		switch(State)
		{
			case STATE_INTRO :
			{
				State_Intro();
				break;
			}

			case STATE_TIME_DOMAIN :
			{
				State_Time_Domain();
				break;
			}

			case STATE_TIME_DOMAIN_SPLIT :
			{
				State_Time_Domain_Split();
				break;
			}

			case STATE_FREQ_DOMAIN :
			{
				State_Freq_Domain();
				break;
			}

			case STATE_FILTER_TEST :
			{
				State_Filter_Test();
				break;
			}

			case STATE_STIMULATOR_TEST :
			{
				State_Stimulator_Test();
				break;
			}

			case STATE_ETC_TEST :
			{
				State_Etc_Test();
				break;
			}

			default :
				break;
		}
	}
}


