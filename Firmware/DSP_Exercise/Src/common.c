/*
 * common.c
 *
 *  Created on: Mar 7, 2021
 *      Author: Ganghyeok Lim
 */

#include "common.h"


uint8_t TFlag = 0;

/********************************************************************************************************************
 * 																											  		*
 *												User Common Function												*
 * 																											  		*
 ********************************************************************************************************************/

void System_Init(void)
{
	// 1. Enable Instruction Cache
	FLASH_INSTRUCTION_CACHE_ENABLE();

	// 2. Enable Data Cache
	FLASH_DATA_CACHE_ENABLE();

	// 3. Enable Prefetch Buffer
	FLASH_PREFETCH_BUFFER_ENABLE();

	// 4. Set Access privileges for coprocessor(FPU_CPACR) to 'Full Access(0b11)'
	SCB->CPACR |= (0xF << 20U);		// If you did not set 'CPn', you cannot access to FPU
									// and Arithmetic operation of Float value will cause NOCP fault.
									// FPU_CPACR can be accessed through SCB (I don't know why)
	RCC_SYSCFG_CLK_ENABLE();
	RCC_PWR_CLK_ENABLE();

	SystemClock_Config(SYSCLK_FREQ_168MHZ);
}


void SystemClock_Config(uint8_t clockFreq)
{
	RCC_OscInitTypeDef oscInit;
	RCC_ClkInitTypeDef clkInit;
	uint8_t FLatency;

	memset(&oscInit, 0, sizeof(oscInit));
	memset(&clkInit, 0, sizeof(clkInit));

	RCC_PWR_CLK_ENABLE();
	PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	oscInit.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	oscInit.HSEState = RCC_HSE_ON;
	oscInit.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	oscInit.PLL.PLLState = RCC_PLL_ON;

	switch(clockFreq)
	{
		case SYSCLK_FREQ_168MHZ :
		{
			oscInit.PLL.PLLM = 4;		// DIV4
			oscInit.PLL.PLLN = 168;		// MUL168
			oscInit.PLL.PLLP = 0;		// DIV2
			clkInit.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
			clkInit.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
			clkInit.AHBCLKDivider = RCC_SYSCLK_DIV1;	// HCLK  : 168MHz
			clkInit.APB1CLKDivider = RCC_HCLK_DIV4;		// PCLK1 : 42MHz
			clkInit.APB2CLKDivider = RCC_HCLK_DIV2;		// PCLK2 : 84MHz

			FLatency = FLASH_LATENCY_5;

			break;
		}

		default :
		{
			break;
		}
	}

	RCC_OscConfig(&oscInit);
	RCC_ClockConfig(&clkInit, FLatency);
}


/* Wait until timer interrupt */
static void WaitTFlag(void)
{
	while (!TFlag) ;
	TFlag = FLAG_RESET;
}


/* Waiting time = (Timer Period)*cnt */
static void WaitTFlagCnt(uint64_t cnt)
{
	uint64_t i;

	TFlag = FLAG_RESET;

	for (i=0; i<cnt; i++)
	{
		WaitTFlag();
	}
}


void Delay_us(uint64_t time_us)
{
	// minimum time delay & time delay unit is 10[us]
	if(time_us >= 10)
	{
		WaitTFlagCnt(time_us/10);
	}
	else
	{
		WaitTFlagCnt(1);	// 10[us]
	}
}


void Delay_ms(uint64_t time_ms)
{
	// minimum time delay is 1[ms]
	if(time_ms >= 1)
	{
		WaitTFlagCnt(time_ms*100);
	}
	else
	{
		WaitTFlagCnt(100);	// 1[ms]
	}
}


void NVIC_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t En_or_Di)
{
	if(En_or_Di == ENABLE)
	{
		if(IRQNumber < 32)
		{
			// IRQ0 ~ IRQ31
			NVIC->ISER[0] = (uint32_t)(1UL << (uint32_t)IRQNumber);
		}
		else if(IRQNumber < 64)
		{
			// IRQ32 ~ IRQ63
			NVIC->ISER[1] = (uint32_t)(1UL << (uint32_t)(IRQNumber % 32));
		}
		else if(IRQNumber < 96)
		{
			// IRQ64 ~ IRQ95
			NVIC->ISER[2] = (uint32_t)(1UL << (uint32_t)(IRQNumber % 64));
		}

	}
	else if(En_or_Di == DISABLE)
	{
		if(IRQNumber < 32)
		{
			// IRQ0 ~ IRQ31
			NVIC->ICER[0] = (uint32_t)(1UL << (uint32_t)IRQNumber);
		}
		else if(IRQNumber < 64)
		{
			// IRQ32 ~ IRQ63
			NVIC->ICER[1] = (uint32_t)(1UL << (uint32_t)(IRQNumber % 32));
		}
		else if(IRQNumber < 96)
		{
			// IRQ64 ~ IRQ95
			NVIC->ICER[2] = (uint32_t)(1UL << (uint32_t)(IRQNumber % 64));
		}
	}

	// IRQ Priority configuration
	NVIC->IPR[IRQNumber] = (IRQPriority << 4UL);
}


void Delay_us_polling(uint32_t time_us)
{
	register uint32_t i, j;

	for(i = 0; i < (time_us / 10); i++)
	{
		for(j = 0; j < 0x4D; j++)
		{
			asm volatile ("NOP");
		}
	}
}


void Delay_ms_polling(uint32_t time_ms)
{
	Delay_us(time_ms * 1000);
}
