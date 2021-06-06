/*
 * main.h
 *
 *  Created on: Mar 7, 2021
 *      Author: Ganghyeok Lim
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "stm32f407xx.h"
#include "common.h"
#include "tft.h"
#include "DSP.h"


/********************************************************************************************************************
 *																													*
 *									  				 Core Specific													*
 *																													*
 ********************************************************************************************************************/
extern uint32_t SystemCoreClock;
extern const uint8_t AHBPrescTable[16];
extern const uint8_t APBPrescTable[8];


/********************************************************************************************************************
 *																													*
 *							 				 Application Specific Variables											*
 *																													*
 ********************************************************************************************************************/

/************************************************************************
 * 						 Peripheral Handle Definitions					*
 ************************************************************************/
extern TIM_HandleTypeDef			TIM6Handle;
extern TIM_HandleTypeDef			TIM7Handle;
extern UART_HandleTypeDef			USART1Handle;
extern DMA_HandleTypeDef			DMA2Handle_UART;
extern ADC_HandleTypeDef			ADC1Handle;
extern DMA_HandleTypeDef			DMA2Handle_ADC;
extern DAC_HandleTypeDef			DAC1Handle;
extern DMA_HandleTypeDef			DMA1Handle_DAC;
extern TFT_HandleTypeDef			TFT1Handle;

/************************************************************************
 * 							 		State								*
 ************************************************************************/
/*
 *  State related Macros
 */
#define STATE_INTRO					0
#define STATE_TIME_DOMAIN			1
#define STATE_TIME_DOMAIN_SPLIT		2
#define STATE_FREQ_DOMAIN			3
#define STATE_FILTER_TEST			4
#define STATE_STIMULATOR_TEST		5
#define STATE_ETC_TEST				6

/*
 *  State related variables
 */
extern uint8_t State;


/************************************************************************
 * 				 Timer prescaling variable related Macros				*
 ************************************************************************/
// In TIM_PeriodElapsedCallback
#define TIMER_PERIOD_COUNT_SAMPLING_FREQ_5000		(20-1)

// In ADC_ConvCpltCallback for Time domain
#define DOWN_SAMPLING_COUNT_TIME_UNIT_10MS			(1-1)
#define DOWN_SAMPLING_COUNT_TIME_UNIT_20MS			(2-1)
#define DOWN_SAMPLING_COUNT_TIME_UNIT_50MS			(5-1)
#define DOWN_SAMPLING_COUNT_TIME_UNIT_100MS			(10-1)
#define DOWN_SAMPLING_COUNT_TIME_UNIT_200MS			(20-1)

// In ADC_ConvCpltCallback for Frequency domain
#define DOWN_SAMPLING_COUNT_FREQ_UNIT_5HZ			(100-1)			// FFT's Fs is 50Hz 	(Folding freq is 25Hz)
#define DOWN_SAMPLING_COUNT_FREQ_UNIT_10HZ			(50-1)			// FFT's Fs is 100Hz 	(Folding freq is 50Hz)
#define DOWN_SAMPLING_COUNT_FREQ_UNIT_20HZ			(25-1)			// FFT's Fs is 200Hz 	(Folding freq is 100Hz)
#define DOWN_SAMPLING_COUNT_FREQ_UNIT_50HZ			(10-1)			// FFT's Fs is 500Hz 	(Folding freq is 250Hz)
#define DOWN_SAMPLING_COUNT_FREQ_UNIT_100HZ			(5-1)			// FFT's Fs is 1000Hz 	(Folding freq is 500Hz)
#define DOWN_SAMPLING_COUNT_FREQ_UNIT_500HZ			(1-1)			// FFT's Fs is 5000Hz 	(Folding freq is 2500Hz)

/************************************************************************
 * 					 Figure Grid Unit for Each Domain					*
 ************************************************************************/
// For Time domain
#define TIME_UNIT_10MS				0
#define TIME_UNIT_20MS				1
#define TIME_UNIT_50MS				2
#define TIME_UNIT_100MS				3
#define TIME_UNIT_200MS				4

// For Frequency domain
#define FREQ_UNIT_5HZ				0								// FFT's Fs is 50Hz 	(Folding freq is 25Hz)
#define FREQ_UNIT_10HZ				1								// FFT's Fs is 100Hz 	(Folding freq is 50Hz)
#define FREQ_UNIT_20HZ				2								// FFT's Fs is 200Hz 	(Folding freq is 100Hz)
#define FREQ_UNIT_50HZ				3								// FFT's Fs is 500Hz 	(Folding freq is 250Hz)
#define FREQ_UNIT_100HZ				4								// FFT's Fs is 1000Hz 	(Folding freq is 500Hz)
#define FREQ_UNIT_500HZ				5								// FFT's Fs is 5000Hz 	(Folding freq is 2500Hz)


/************************************************************************
 * 							Filter related Variables					*
 ************************************************************************/
extern uint32_t DacOut;
extern uint8_t FilterState;


/********************************************************************************************************************
 *																													*
 *											 Application Specific C Functions										*
 *																													*
 ********************************************************************************************************************/

/************************************************************************
 * 						Prototype for static functions					*
 ************************************************************************/


/************************************************************************
 * 					User Peripheral Initialization functions			*
 ************************************************************************/
extern void LED1_Init(void);
extern void KEY_Init(void);
extern void BUZZER_Init(void);
extern void Stimulator_Init(void);


/************************************************************************
 * 							  State Functions							*
 ************************************************************************/
extern void State_Intro(void);
extern void State_Time_Domain(void);
extern void State_Time_Domain_Split(void);
extern void State_Freq_Domain(void);
extern void State_Filter_Test(void);
extern void State_Stimulator_Test(void);
extern void State_Etc_Test(void);

/************************************************************************
 * 						  Digital Filter functions						*
 ************************************************************************/
extern double NotchFilter(void);

/************************************************************************
 * 						  	Stimulator functions						*
 ************************************************************************/
extern void Stimulator_DetectCondition(void);
extern void Stimulator_Start(void);
extern void Stimulator_Stop(void);

/************************************************************************
 * 						  	  Graph funcitons							*
 ************************************************************************/
extern void Draw_Axis(TFT_HandleTypeDef *pTFTHandle);
extern void Change_TimeUnit(uint8_t direction);
extern void Change_FreqUnit(uint8_t direction);
extern void Clear_Graph(TFT_HandleTypeDef *pTFTHandle);

/************************************************************************
 * 							   Key functions							*
 ************************************************************************/
extern void Key_InputDetection(void);

/************************************************************************
 * 				  Vendor Peripheral Initialization functions			*
 ************************************************************************/
extern void MemsetHandleStructure(void);
extern void DMA1_Init(void);
extern void DMA2_Init(void);
extern void TIM6_Init(void);
extern void TIM7_Init(void);
extern void UART1_Init(void);
extern void ADC1_Init(void);
extern void DAC1_Init(void);
extern void TFT1_Init(void);

/************************************************************************
 * 				 	   Peripheral Callback functions					*
 ************************************************************************/



/********************************************************************************************************************
 *																													*
 *									   				  DSP Specific													*
 *																													*
 ********************************************************************************************************************/

/************************************************************************
 * 			    DSP User configurable parameters definitions			*
 ************************************************************************/
#define N							256		// Length of Block (must be a power of two)
#define MF							31		// Length of FIR filter
#define L							226		// Length of Input data to be processed at a time => L should be (N-MF+1)

/************************************************************************
 * 			    			   DSP variables							*
 ************************************************************************/
extern uint8_t						InFullFlag;
extern double						InputBuffer[L];
extern double						OutputBuffer[L];
extern uint8_t						OutFullFlag;



#endif /* MAIN_H_ */
