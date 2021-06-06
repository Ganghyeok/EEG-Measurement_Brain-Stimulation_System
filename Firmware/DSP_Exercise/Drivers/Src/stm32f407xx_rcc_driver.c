/*
 * stm32f407xx_rcc_driver.c
 *
 *  Created on: 2021. 3. 7.
 *      Author: Ganghyeok Lim
 */

#include "stm32f407xx.h"
#include "main.h"


/**************************************************************************************************************
 * 																											  *
 * 											APIs supported by this driver									  *
 * 						For more information about the APIs, Check the function definitions					  *
 * 									  																		  *
 **************************************************************************************************************/

void RCC_OscConfig(RCC_OscInitTypeDef *RCC_OscInitStruct)
{
	/*------------------------------- HSE Configuration ------------------------*/
	if(((RCC_OscInitStruct->OscillatorType) & RCC_OSCILLATORTYPE_HSE) == RCC_OSCILLATORTYPE_HSE)
	{
		if(RCC_OscInitStruct->HSEState == RCC_CR_HSEON)
		{
			SET_BIT(RCC->CR, RCC_CR_HSEON);				// Enable HSE
			WAIT_FLAG_SET(RCC->CR, RCC_CR_HSERDY);		// Wait until HSERDY flag is set
		}
	}

	/*-------------------------------- PLL Configuration -----------------------*/
	if((RCC_OscInitStruct->PLL.PLLState) == RCC_PLL_ON)
	{
		// 1. Disable PLL
		CLEAR_BIT(RCC->CR, RCC_CR_PLLON);

		// 2. Configure PLL source and PLL multiplication factor
        MODIFY_REG(RCC->PLLCFGR, ( (RCC_PLLCFGR_PLLSRC) | (RCC_PLLCFGR_PLLM) | (RCC_PLLCFGR_PLLN) | (RCC_PLLCFGR_PLLP) ), ( (RCC_OscInitStruct->PLL.PLLSource) | (RCC_OscInitStruct->PLL.PLLM) | (RCC_OscInitStruct->PLL.PLLN << RCC_PLLCFGR_PLLN_Pos) | (RCC_OscInitStruct->PLL.PLLP << RCC_PLLCFGR_PLLP_Pos) ));

        // 3. Enable PLL
        SET_BIT(RCC->CR, RCC_CR_PLLON);

        // 4. Wait until PLL is ready
        WAIT_FLAG_SET(RCC->CR, RCC_CR_PLLRDY);
	}
}


void RCC_ClockConfig(RCC_ClkInitTypeDef *RCC_ClkInitStruct, uint32_t FLatency)
{
	// 1. Configure FLASH Latency
	MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, FLatency);

	// 2. Configure APB prescaler
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_HCLK_DIV16);					// need to be changed
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, (RCC_HCLK_DIV16 << 3));			// need to be changed

	// 3. Configure AHB prescaler
    MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_ClkInitStruct->AHBCLKDivider);

	// 4. Configure SYSCLK
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_ClkInitStruct->SYSCLKSource);

    // 5. Wait until SYSCLK is PLLCLK
    WAIT_FLAG_SET(RCC->CFGR, RCC_CFGR_SWS_PLL);

    /*-------------------------- PCLK1 Configuration ---------------------------*/
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_ClkInitStruct->APB1CLKDivider);

    /*-------------------------- PCLK2 Configuration ---------------------------*/
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, ((RCC_ClkInitStruct->APB2CLKDivider) << 3U));
}

/**
  * @brief  Selects the clock source to output on MCO1 pin(PA8) or on MCO2 pin(PC9).
  * @note   PA8/PC9 should be configured in alternate function mode.
  * @param  RCC_MCOx specifies the output direction for the clock source.
  *          This parameter can be one of the following values:
  *            @arg RCC_MCO1: Clock source to output on MCO1 pin(PA8).
  *            @arg RCC_MCO2: Clock source to output on MCO2 pin(PC9).
  * @param  RCC_MCOSource specifies the clock source to output.
  *          This parameter can be one of the following values:
  *            @arg RCC_MCO1SOURCE_HSI: HSI clock selected as MCO1 source
  *            @arg RCC_MCO1SOURCE_LSE: LSE clock selected as MCO1 source
  *            @arg RCC_MCO1SOURCE_HSE: HSE clock selected as MCO1 source
  *            @arg RCC_MCO1SOURCE_PLLCLK: main PLL clock selected as MCO1 source
  *            @arg RCC_MCO2SOURCE_SYSCLK: System clock (SYSCLK) selected as MCO2 source
  *            @arg RCC_MCO2SOURCE_PLLI2SCLK: PLLI2S clock selected as MCO2 source, available for all STM32F4 devices except STM32F410xx
  *            @arg RCC_MCO2SOURCE_I2SCLK: I2SCLK clock selected as MCO2 source, available only for STM32F410Rx devices
  *            @arg RCC_MCO2SOURCE_HSE: HSE clock selected as MCO2 source
  *            @arg RCC_MCO2SOURCE_PLLCLK: main PLL clock selected as MCO2 source
  * @param  RCC_MCODiv specifies the MCOx prescaler.
  *          This parameter can be one of the following values:
  *            @arg RCC_MCODIV_1: no division applied to MCOx clock
  *            @arg RCC_MCODIV_2: division by 2 applied to MCOx clock
  *            @arg RCC_MCODIV_3: division by 3 applied to MCOx clock
  *            @arg RCC_MCODIV_4: division by 4 applied to MCOx clock
  *            @arg RCC_MCODIV_5: division by 5 applied to MCOx clock
  * @note  For STM32F410Rx devices to output I2SCLK clock on MCO2 you should have
  *        at last one of the SPI clocks enabled (SPI1, SPI2 or SPI5).
  * @retval None
  */
void MCOConfig(uint32_t RCC_MCOx, uint32_t RCC_MCOSource, uint32_t RCC_MCODiv)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	if(RCC_MCOx == RCC_MCO1)
	{
	    /* MCO1 Clock Enable */
	    MCO1_CLK_ENABLE();

	    /* Configure the MCO1 pin in alternate function mode */
	    GPIO_InitStruct.Pin = MCO1_PIN;
	    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
	    GPIO_Init(MCO1_GPIO_PORT, &GPIO_InitStruct);

	    /* Mask MCO1 and MCO1PRE[2:0] bits then Select MCO1 clock source and prescaler */
	    MODIFY_REG(RCC->CFGR, (RCC_CFGR_MCO1 | RCC_CFGR_MCO1PRE), (RCC_MCOSource | RCC_MCODiv));
	  }

	  else if(RCC_MCOx == RCC_MCO2)
	  {
	    /* MCO2 Clock Enable */
	    MCO2_CLK_ENABLE();

	    /* Configure the MCO2 pin in alternate function mode */
	    GPIO_InitStruct.Pin = MCO2_PIN;
	    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
	    GPIO_Init(MCO2_GPIO_PORT, &GPIO_InitStruct);

	    /* Mask MCO2 and MCO2PRE[2:0] bits then Select MCO2 clock source and prescaler */
	    MODIFY_REG(RCC->CFGR, (RCC_CFGR_MCO2 | RCC_CFGR_MCO2PRE), (RCC_MCOSource | (RCC_MCODiv << 3U)));
	}
}


uint32_t RCC_GetHCLKFreq(void)
{
  return SystemCoreClock;
}


uint32_t RCC_GetPCLK1Freq(void)
{
  /* Get HCLK source and Compute PCLK1 frequency ---------------------------*/
  return (RCC_GetHCLKFreq() >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1)>> RCC_CFGR_PPRE1_Pos]);
}


uint32_t RCC_GetPCLK2Freq(void)
{
  /* Get HCLK source and Compute PCLK2 frequency ---------------------------*/
  return (RCC_GetHCLKFreq() >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE2)>> RCC_CFGR_PPRE2_Pos]);
}
