/*
 * common.h
 *
 *  Created on: Mar 7, 2021
 *      Author: GangHyeok Lim
 */

#ifndef COMMON_H_
#define COMMON_H_


#include "stm32f407xx.h"

extern uint8_t TFlag;

/* User Common functions */
void System_Init(void);
void SystemClock_Config(uint8_t clockFreq);
void Delay_us(uint64_t time_us);
void Delay_ms(uint64_t time_ms);
void NVIC_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t En_or_Di);
void Delay_us_polling(uint32_t time_us);
void Delay_ms_polling(uint32_t time_ms);

#endif /* COMMON_H_ */
