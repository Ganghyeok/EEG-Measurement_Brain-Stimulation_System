/*
 * user_func.c
 *
 *  Created on: Mar 7, 2021
 *      Author: Ganghyeok Lim
 */

#include "main.h"

/********************************************************************************************************************
 *																													*
 *									  				 Core Specific													*
 *																													*
 ********************************************************************************************************************/
uint32_t SystemCoreClock        = 168000000;
const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8]  = {0, 0, 0, 0, 1, 2, 3, 4};

/********************************************************************************************************************
 *																													*
 *							 				 Application Specific Variables											*
 *																													*
 ********************************************************************************************************************/

/************************************************************************
 * 						 Peripheral Handle Definitions					*
 ************************************************************************/
TIM_HandleTypeDef						TIM6Handle;
TIM_HandleTypeDef						TIM7Handle;
UART_HandleTypeDef						USART1Handle;
DMA_HandleTypeDef						DMA2Handle_UART;
ADC_HandleTypeDef						ADC1Handle;
DMA_HandleTypeDef						DMA2Handle_ADC;
DAC_HandleTypeDef						DAC1Handle;
DMA_HandleTypeDef						DMA1Handle_DAC;
TFT_HandleTypeDef						TFT1Handle;

/************************************************************************
 * 							 		State								*
 ************************************************************************/
/*
 *  State related variables
 */
uint8_t	 State = STATE_INTRO;

/************************************************************************
 * 							 		 ADC								*
 ************************************************************************/
uint32_t AdcValue              		= 0;		// Variable to hold the result of A/D Conversion
uint32_t TimeDomainSample[250]      = {0,};		// Array to hold the results of A/D Conversion
uint32_t TimeDomainSampleFltrd[250] = {0,};		// Array to hold the filtered results of A/D Converion by Notch filter
uint16_t SampleIdx                  = 0;		// Index variable for ADC samples
uint8_t  AdcCmpltFlag 				= 0;		// Flag variable to notify the completion of A/D Conversion
uint16_t DownSamplingCount			= 0;		// Count variable to prescale Timer period
Complex FFT_Buffer[N] 				= {0,};		// Array to hold the samples of input signal or FFT result of that
double FFT_Mag[N]                   = {0,};		// Magnitude array of FFT result

uint16_t TimerPeriodCountMax        = TIMER_PERIOD_COUNT_SAMPLING_FREQ_5000;	// Max value of Timer prescaling variable
uint16_t DownSamplingCountMax       = DOWN_SAMPLING_COUNT_TIME_UNIT_10MS;		// Max value of Timer prescaling variable

/************************************************************************
 * 							 		 DAC								*
 ************************************************************************/
uint32_t DacOut;		// DAC output buffer variable

/************************************************************************
 * 							 		Graph								*
 ************************************************************************/
uint16_t TimeUnit    = 					TIME_UNIT_10MS;
uint16_t PrvTimeUnit = 					TIME_UNIT_10MS;

uint16_t FreqUnit    = 					FREQ_UNIT_5HZ;
uint16_t PrvFreqUnit = 					FREQ_UNIT_5HZ;

/************************************************************************
 * 							 		 Key								*
 ************************************************************************/
uint8_t KeyFlag_Mode = FLAG_RESET;
uint8_t KeyFlag_Up   = FLAG_RESET;
uint8_t KeyFlag_Down = FLAG_RESET;
uint8_t KeyFlag_Set  = FLAG_RESET;

uint32_t KeyTime_Mode  = 0;
uint32_t KeyTime_Up    = 0;
uint32_t KeyTime_Down  = 0;
uint32_t KeyTime_Set = 0;

uint16_t KeyCount_Mode = 0;
uint16_t KeyCount_Up   = 0;
uint16_t KeyCount_Down = 0;
uint16_t KeyCount_Set  = 0;

/************************************************************************
 * 							 		Filter								*
 ************************************************************************/
double r, Fs, F0, f0, w0, b0;		// Notch filter parameters
uint8_t FilterState = DISABLE;		// Variable to hold the state of Filter (Enable or Disable)

/************************************************************************
 * 							 	  Stimualtor							*
 ************************************************************************/
uint8_t StimulatorState     = DISABLE;
uint32_t StimulatorCount    = 0;
uint32_t StimulatorCountMax = 500000;	// Duration of Stimulator Operation is 5[sec] ( 5s / 10us = 500000)


/********************************************************************************************************************
 *																													*
 *											 Application Specific C Functions										*
 *																													*
 ********************************************************************************************************************/

/************************************************************************
 * 						Prototype for static functions					*
 ************************************************************************/
static void Label_Axis(void);

/************************************************************************
 * 					User Peripheral Initialization functions			*
 ************************************************************************/
void LED1_Init(void)
{
	GPIO_InitTypeDef GPIO_LED;

	memset(&GPIO_LED, 0, sizeof(GPIO_LED));

	GPIO_LED.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_LED.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_LED.Pin = GPIO_PIN_12;

	GPIO_Init(GPIOA, &GPIO_LED);
}


void KEY_Init(void)
{
	GPIO_InitTypeDef GPIO_KEY;

	memset(&GPIO_KEY, 0, sizeof(GPIO_KEY));

	GPIO_KEY.Mode = GPIO_MODE_IT_FALLING;
	GPIO_KEY.Pull = GPIO_PULLUP;

	GPIO_KEY.Pin = GPIO_PIN_0;
	GPIO_Init(GPIOB, &GPIO_KEY);
	NVIC_IRQConfig(EXTI0_IRQn, 3, ENABLE);

	GPIO_KEY.Pin = GPIO_PIN_1;
	GPIO_Init(GPIOB, &GPIO_KEY);
	NVIC_IRQConfig(EXTI1_IRQn, 3, ENABLE);

	GPIO_KEY.Pin = GPIO_PIN_2;
	GPIO_Init(GPIOB, &GPIO_KEY);
	NVIC_IRQConfig(EXTI2_IRQn, 3, ENABLE);

	GPIO_KEY.Pin = GPIO_PIN_3;
	GPIO_Init(GPIOB, &GPIO_KEY);
	NVIC_IRQConfig(EXTI3_IRQn, 3, ENABLE);
}


void BUZZER_Init(void)
{
	GPIO_InitTypeDef GPIO_BUZZER;

	memset(&GPIO_BUZZER, 0, sizeof(GPIO_BUZZER));

	GPIO_BUZZER.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_BUZZER.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_BUZZER.Pin = GPIO_PIN_12;

	GPIO_Init(GPIOB, &GPIO_BUZZER);

	GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
}


void Stimulator_Init(void)
{
	// 1. GPIO Init for Opto-Coupler
	GPIO_InitTypeDef GPIO_OPTO;

	memset(&GPIO_OPTO, 0, sizeof(GPIO_OPTO));

	GPIO_OPTO.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_OPTO.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

	GPIO_OPTO.Pin = GPIO_PIN_0;
	GPIO_Init(GPIOC, &GPIO_OPTO);

	GPIO_OPTO.Pin = GPIO_PIN_1;
	GPIO_Init(GPIOC, &GPIO_OPTO);

	GPIO_OPTO.Pin = GPIO_PIN_2;
	GPIO_Init(GPIOC, &GPIO_OPTO);

	GPIO_OPTO.Pin = GPIO_PIN_3;
	GPIO_Init(GPIOC, &GPIO_OPTO);


	// 2. GPIO Init for Gate Driver(H_SIDEx, L_SIDEx)
	GPIO_InitTypeDef GPIO_GD;

	memset(&GPIO_GD, 0, sizeof(GPIO_GD));

	GPIO_GD.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_GD.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

	GPIO_GD.Pin = GPIO_PIN_4;
	GPIO_Init(GPIOC, &GPIO_GD);

	GPIO_GD.Pin = GPIO_PIN_5;
	GPIO_Init(GPIOC, &GPIO_GD);

	GPIO_GD.Pin = GPIO_PIN_6;
	GPIO_Init(GPIOC, &GPIO_GD);

	GPIO_GD.Pin = GPIO_PIN_7;
	GPIO_Init(GPIOC, &GPIO_GD);


	// 3. GPIO Init for Passive Discharge
	GPIO_InitTypeDef GPIO_PD;

	memset(&GPIO_PD, 0, sizeof(GPIO_PD));

	GPIO_PD.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_PD.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

	GPIO_PD.Pin = GPIO_PIN_8;
	GPIO_Init(GPIOC, &GPIO_PD);

	// 4. Disable Stimulator
	Stimulator_Stop();
}


/************************************************************************
 * 							  State Functions							*
 ************************************************************************/
void State_Intro(void)
{
	TFT_Rectangle(&TFT1Handle, 0, 0, 319, 239, Blue);
	TFT_Rectangle(&TFT1Handle, 1, 1, 318, 238, Blue);
	TFT_String_Large(&TFT1Handle, 3, 6, White, Magenta, (uint8_t*)" EEG Measurement &  ");
	TFT_String_Large(&TFT1Handle, 3, 10, White, Magenta, (uint8_t*)" Stimulation System ");
	TFT_String(&TFT1Handle, 6, 20, White, Magenta, (uint8_t*)" Designed by Ganghyeok Lim ");
	while(1);
}


void State_Time_Domain(void)
{
	uint16_t y_val;

	// 1. Re-initialize variables
	DownSamplingCountMax = DOWN_SAMPLING_COUNT_TIME_UNIT_10MS;
	TimeUnit = DOWN_SAMPLING_COUNT_TIME_UNIT_10MS;				// TimeUnit = PrvTimeUnit

	// 2. Draw Figure Axis of Time domain
	TFT_Clear_Screen(&TFT1Handle);
	TFT_String(&TFT1Handle, 14, 0, White, Blue, (uint8_t *)" Time-Domain ");
	Draw_Axis(&TFT1Handle);

	// 3. Re-initialize Key related variables
	KeyCount_Mode = 0;
	KeyCount_Up = 0;
	KeyCount_Down = 0;


	while(1)
	{
		if(AdcCmpltFlag == 1)
		{
			Clear_Graph(&TFT1Handle);

			for(int x = 33; x <= 282; x++)
			{
				y_val = (-1)*62./1365.*TimeDomainSample[x-33] + 215;
				TFT_Pixel(&TFT1Handle, x, y_val, Green);

				y_val = (-1)*62./1365.*TimeDomainSampleFltrd[x-33] + 215;
				TFT_Pixel(&TFT1Handle, x, y_val, Yellow);
			}

			AdcCmpltFlag = 0;
		}

		// If 'Mode' key is pressed
		if(KeyCount_Mode >= 1)
		{
			KeyCount_Mode = 0;
			KeyCount_Up = 0;
			KeyCount_Down = 0;
			DownSamplingCount = 0;
			PrvTimeUnit = TimeUnit;
			State = STATE_TIME_DOMAIN_SPLIT;
			break;
		}

		// If 'Up' key is pressed
		else if(KeyCount_Up >= 1)
		{
			KeyCount_Up = 0;
			Change_TimeUnit(UP);	// Increase Time unit
		}

		// If 'Down' key is pressed
		else if(KeyCount_Down >= 1)
		{
			KeyCount_Down = 0;
			Change_TimeUnit(DOWN);	// Decrease Time unit
		}

		Delay_ms(1000);
	}
}


void State_Time_Domain_Split(void)
{
	uint16_t y_val;

	// 1. Re-initialize variables
	DownSamplingCountMax = DOWN_SAMPLING_COUNT_TIME_UNIT_10MS;
	TimeUnit = DOWN_SAMPLING_COUNT_TIME_UNIT_10MS;				// TimeUnit = PrvTimeUnit

	// 2. Draw Figure Axis of Time domain
	TFT_Clear_Screen(&TFT1Handle);
	TFT_String(&TFT1Handle, 14, 0, White, Blue, (uint8_t *)" Time-Domain ");
	Draw_Axis(&TFT1Handle);

	// 3. Re-initialize Key related variables
	KeyCount_Mode = 0;
	KeyCount_Up = 0;
	KeyCount_Down = 0;


	while(1)
	{
		if(AdcCmpltFlag == 1)
		{
			Clear_Graph(&TFT1Handle);

			for(int x = 33; x <= 282; x++)	// 250 samples
			{
				y_val = (-1)*17./819.*TimeDomainSample[x-33] + 114;
				TFT_Pixel(&TFT1Handle, x, y_val, Green);

				y_val = (-1)*17./819.*TimeDomainSampleFltrd[x-33] + 215;
				TFT_Pixel(&TFT1Handle, x, y_val, Yellow);
			}

			AdcCmpltFlag = 0;
		}

		// If 'Mode' key is pressed
		if(KeyCount_Mode >= 1)
		{
			KeyCount_Mode = 0;
			KeyCount_Up = 0;
			KeyCount_Down = 0;
			DownSamplingCount = 0;
			PrvTimeUnit = TimeUnit;
			State = STATE_FREQ_DOMAIN;
			break;
		}

		// If 'Up' key is pressed
		else if(KeyCount_Up >= 1)
		{
			KeyCount_Up = 0;
			Change_TimeUnit(UP);
		}

		// If 'Down' key is pressed
		else if(KeyCount_Down >= 1)
		{
			KeyCount_Down = 0;
			Change_TimeUnit(DOWN);
		}

		Delay_ms(1000);
	}
}


void State_Freq_Domain(void)
{


	// 1. Re-initialize variables
	DownSamplingCountMax = DOWN_SAMPLING_COUNT_FREQ_UNIT_5HZ;
	FreqUnit = FREQ_UNIT_5HZ;

	// 2. Draw Figure Axis of Frequency domain
	TFT_Clear_Screen(&TFT1Handle);
	TFT_String(&TFT1Handle, 14, 0, White, Blue, (uint8_t *)" Freq-Domain ");
	Draw_Axis(&TFT1Handle);

	// 3. Re-initialize Key related variables
	KeyCount_Mode = 0;
	KeyCount_Up = 0;
	KeyCount_Down = 0;

	while(1)
	{
		// 1. If A/D Conversion is Completed
		if(AdcCmpltFlag == 1)
		{
			// Clear previous FFT figure
			Clear_Graph(&TFT1Handle);

			// Execute FFT operation
			FFT(FFT_Buffer);

			// Calculate Magnitude of FFT results
			for(int i=0; i<N; i++)
			{
				FFT_Mag[i] = sqrt(FFT_Buffer[i].real * FFT_Buffer[i].real + FFT_Buffer[i].imag * FFT_Buffer[i].imag) / N;
			}

			// Draw Vertical bars for Magnitude of FFT results
			for(int i=1; i<128; i++)
			{
				unsigned int height = 0;

				height = (unsigned int)(360.*FFT_Mag[i] + 0.5);
				if(height >= 180.) height = 180.;

				TFT_Line(&TFT1Handle, 30+2*i, 219, 30+2*i, 219 - height, Magenta); // draw new bar
			}

			AdcCmpltFlag = 0;
		}


		// 2. Check the State of Stimulator
		if(StimulatorState == ENABLE)
		{
			Stimulator_Start();
		}
		else if(StimulatorState == DISABLE)
		{
			Stimulator_Stop();
		}


		// 3. If 'Mode' key is pressed
		if(KeyCount_Mode >= 1)
		{
			KeyCount_Mode = 0;
			KeyCount_Up = 0;
			KeyCount_Down = 0;
			DownSamplingCount = 0;
			State = STATE_TIME_DOMAIN;
			break;
		}

		// 4. If 'Up' key is pressed
		else if(KeyCount_Up >= 1)
		{
			KeyCount_Up = 0;
			Change_FreqUnit(UP);
		}

		// 5. If 'Down' key is pressed
		else if(KeyCount_Down >= 1)
		{
			KeyCount_Down = 0;
			Change_FreqUnit(DOWN);
		}
	}
}


void State_Filter_Test(void)
{
	// Function generator AC sweep setting
	// Sweep type : Log sweep
	// Sweep time : 4.0[sec]
	// Start freq : 1.0[mHz]
	// Stop freq  : 100[Hz]

	while(1);
}


void State_Stimulator_Test(void)
{
	// 1. Passive Discharge OFF
	GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);

	// 2. Select Current value of DAC
	GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);


	while(1)
	{
		// 2. All OFF and Enable Passive Discharge
		GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);	// H_SIDE1 : LOW (-> H_MOS1 Turn OFF)
		GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);	// L_SIDE1 : LOW (-> L_MOS1 Turn OFF)
		GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);	// H_SIDE2 : LOW (-> H_MOS2 Turn OFF)
		GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);	// L_SIDE2 : LOW (-> L_MOS2 Turn OFF)
		Delay_us(10);
		GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);	// Passive Discharge ON
		Delay_ms(4);
		GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);		// Passive Discharge OFF
		Delay_us(10);

		// 3. M1, M4 ON
		GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);		// H_SIDE1 : HIGH (-> H_MOS1 Turn ON)
		GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);		// L_SIDE2 : HIGH (-> L_MOS2 Turn ON)
		Delay_ms(1);

		// 4. All OFF for very short time
		GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);	// H_SIDE1 : LOW (-> H_MOS1 Turn OFF)
		GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);	// L_SIDE1 : LOW (-> L_MOS1 Turn OFF)
		GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);	// H_SIDE2 : LOW (-> H_MOS2 Turn OFF)
		GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);	// L_SIDE2 : LOW (-> L_MOS2 Turn OFF)
		Delay_us(100);

		// 5. M2, M3 ON
		GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);		// H_SIDE2 : HIGH (-> H_MOS2 Turn On)
		GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);		// L_SIDE1 : HIGH (-> L_MOS1 Turn On)
		Delay_ms(1);
	}
}


void State_Etc_Test(void)
{
	while(1)
	{
		GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
		Delay_ms(500);
	}
}


/************************************************************************
 * 						  Digital Filter functions						*
 ************************************************************************/
double NotchFilter(void)
{
	double Input;
	double Output;
	static double InputPrv[2] = {0,};
	static double OutputPrv[2] = {0,};
	static uint8_t InitialCondCount = 0;

	// 1. Figure out the Sampling frequency
	Fs = 5000;		// Sampling frequency

	// 2. Set Filter parameters
	r = 0.99;		// Distance from origin to pole		// Original 'r' value = 0.85
	F0 = 60;		// Analog Center frequency of Notch Filter
	f0 = F0 / Fs;	// Normalized Center frequency of Notch Filter
	w0 = 2*pi*f0; 	// F0 = 60Hz (w0 = 2*pi*f0)
	b0 = 1;			// PassBand gain

	// 2. Scaliing ADC Sample value to the range of -1.65 ~ +1.65
	Input = (double)AdcValue * 3.3 / 4095. - 1.65;

	// 3. Calculate Filter output
	if(InitialCondCount == 0)
	{
		// 1st Filter output when all initial values are zero
		Output = b0*Input;

		InitialCondCount = 1;
	}
	else if(InitialCondCount == 1)
	{
		// 2nd Filter output when all initial values are zero
		Output = 2*r*cos(w0)*OutputPrv[0] + b0*Input - 2*b0*cos(w0)*InputPrv[0];

		InitialCondCount = 2;
	}
	else
	{
		// Filter output for n>=2
		Output = 2*r*cos(w0)*OutputPrv[0] - r*r*OutputPrv[1] + b0*Input - 2*b0*cos(w0)*InputPrv[0] + b0*InputPrv[1];
	}

	// 4. Store Past value for Next Filter calculation
	InputPrv[1] = InputPrv[0];
	InputPrv[0] = Input;
	OutputPrv[1] = OutputPrv[0];
	OutputPrv[0] = Output;

	// 5. Load Filter output value to DAC buffer
	DacOut = (uint32_t)( (Output + 1.65) * 4095. / 3.3 );


	return Output;
}


/************************************************************************
 * 						  	Stimulator functions						*
 ************************************************************************/
void Stimulator_DetectCondition(void)
{
	if(State == STATE_FREQ_DOMAIN)
	{
		// 1. Check if the Stimulator operation condition is satisfied
		if(FreqUnit == FREQ_UNIT_5HZ)
		{
			if(FFT_Mag[25] >= 0.12)			// FFT_Mag[20] is corresponding to 4Hz in FREQ_UNIT_5HZ
			{
				StimulatorState = ENABLE;
			}
			else if(FFT_Mag[26] >= 0.12)			// FFT_Mag[20] is corresponding to 4Hz in FREQ_UNIT_5HZ
			{
				StimulatorState = ENABLE;
			}
			else
			{
				StimulatorState = DISABLE;
			}
		}
		else if(FreqUnit == FREQ_UNIT_10HZ)		// FFT_Mag[10] is corresponding to 4Hz in FREQ_UNIT_10HZ
		{
			if(FFT_Mag[10] >= 0.358)
			{
				StimulatorState = ENABLE;
			}
			else
			{
				StimulatorState = DISABLE;
			}
		}
		else
		{
			StimulatorState = DISABLE;
		}


//		// 2. Check if StimulatorState is ENABLE then, count time until it reaches max value
//		if(StimulatorState == ENABLE)
//		{
//			if(StimulatorCount >= StimulatorCountMax)
//			{
//				StimulatorCount = 0;
//				StimulatorState = DISABLE;
//			}
//			else
//			{
//				StimulatorCount++;
//			}
//		}
	}
}


void Stimulator_Start(void)
{
	// 1. Passive Discharge OFF
	GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);

	// 2. Select Current value of DAC
	GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
	Delay_ms(1);

	// 3. M1, M4 ON
	GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);		// H_SIDE1 : HIGH (-> H_MOS1 Turn ON)
	GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);		// L_SIDE2 : HIGH (-> L_MOS2 Turn ON)
	Delay_ms(1);

	// 4. All OFF for very short time
	GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);	// H_SIDE1 : LOW (-> H_MOS1 Turn OFF)
	GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);	// L_SIDE1 : LOW (-> L_MOS1 Turn OFF)
	GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);	// H_SIDE2 : LOW (-> H_MOS2 Turn OFF)
	GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);	// L_SIDE2 : LOW (-> L_MOS2 Turn OFF)
	Delay_us(100);

	// 5. M2, M3 ON
	GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);		// H_SIDE2 : HIGH (-> H_MOS2 Turn On)
	GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);		// L_SIDE1 : HIGH (-> L_MOS1 Turn On)
	Delay_ms(1);

	// 6. All OFF and Enable Passive Discharge
	GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);	// H_SIDE1 : LOW (-> H_MOS1 Turn OFF)
	GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);	// L_SIDE1 : LOW (-> L_MOS1 Turn OFF)
	GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);	// H_SIDE2 : LOW (-> H_MOS2 Turn OFF)
	GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);	// L_SIDE2 : LOW (-> L_MOS2 Turn OFF)
	Delay_us(10);
	GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);	// Passive Discharge ON
	Delay_ms(4);
	GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);		// Passive Discharge OFF
	Delay_us(10);
}


void Stimulator_Stop(void)
{
	// 1. Disable All Opto-coupler that controls Current Steering DAC
	GPIO_WritePin(GPIOC, (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3), GPIO_PIN_RESET);

	// 2. Disable All MOSFETs of Bi-phasic Bipolar Current Driver
	GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);		// ~H_SIDE1 : HIGH (-> H_MOS1 Turn OFF)
	GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);		// ~L_SIDE1 : HIGH (-> L_MOS1 Turn OFF)
	GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);		// ~H_SIDE2 : HIGH (-> H_MOS2 Turn OFF)
	GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);		// ~L_SIDE2 : HIGH (-> L_MOS2 Turn OFF)

	// 3. Disable Passive Discharge
	GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);		// ~DISCHARGE : HIGH
}


/************************************************************************
 * 						  	  Graph funcitons							*
 ************************************************************************/
void Draw_Axis(TFT_HandleTypeDef *pTFTHandle)
{
	if(State == STATE_TIME_DOMAIN)
	{
		/* X-axis */

		// 1. Draw x-axis
		TFT_Line(&TFT1Handle, 32, 216, 310, 216, White);	// Horizontal line of x-axis
		TFT_Line(&TFT1Handle, 305, 211, 310, 216, White);	// Upper arrow head of x-axis
		TFT_Line(&TFT1Handle, 305, 221, 310, 216, White);	// Lower arrow head of x-axis

		// 2. Draw dotted lines for x-axis
		for(int i = 82; i <= 319; i = i + 50)
		{
			for(int j = 29; j <= 216; j = j + 5)
			{
				TFT_Pixel(&TFT1Handle, i, j, White);
			}
		}


		/* Y-axis */

		// 1. Draw y-axis
		TFT_Line(&TFT1Handle, 32, 216, 32, 23, White);		// Vertical line of y-axis
		TFT_Line(&TFT1Handle, 27, 28, 32, 23, White);		// Left arrow head of y-axis
		TFT_Line(&TFT1Handle, 37, 28, 32, 23, White);		// Right arrow head of y-axis

		// 2. Draw dotted lines for y-axis
		for(int j = 46; j <= 186; j = j + 28)
		{
			for(int i = 33; i <= 310; i = i + 5)
			{
				TFT_Pixel(&TFT1Handle, i, j, White);
			}
		}

		Label_Axis();
	}

	else if(State == STATE_TIME_DOMAIN_SPLIT)
	{
		/* X-axis of Figure1 */

		// 1. Draw x-axis
		TFT_Line(&TFT1Handle, 32, 115, 310, 115, White);	// Horizontal line of x-axis
		TFT_Line(&TFT1Handle, 305, 110, 310, 115, White);	// Upper arrow head of x-axis
		TFT_Line(&TFT1Handle, 305, 120, 310, 115, White);	// Lower arrow head of x-axis

		// 2. Draw dotted lines for x-axis
		for(int i = 82; i <= 319; i = i + 50)
		{
			for(int j = 29; j <= 115; j = j + 5)
			{
				TFT_Pixel(&TFT1Handle, i, j, White);
			}
		}


		/* Y-axis of Figure1 */

		// 1. Draw y-axis
		TFT_Line(&TFT1Handle, 32, 23, 32, 115, White);		// Vertical line of y-axis
		TFT_Line(&TFT1Handle, 27, 28, 32, 23, White);		// Left arrow head of y-axis
		TFT_Line(&TFT1Handle, 37, 28, 32, 23, White);		// Right arrow head of y-axis

		// 2. Draw dotted lines for y-axis
		for(int j = 37; j <= 89; j = j + 26)
		{
			for(int i = 33; i <= 310; i = i + 5)
			{
				TFT_Pixel(&TFT1Handle, i, j, White);
			}
		}


		/* X-axis of Figure2 */

		// 1. Draw x-axis
		TFT_Line(&TFT1Handle, 32, 216, 310, 216, White);	// Horizontal line of x-axis
		TFT_Line(&TFT1Handle, 305, 211, 310, 216, White);	// Upper arrow head of x-axis
		TFT_Line(&TFT1Handle, 305, 221, 310, 216, White);	// Lower arrow head of x-axis

		// 2. Draw dotted lines for x-axis
		for(int i = 82; i <= 319; i = i + 50)
		{
			for(int j = 130; j <= 216; j = j + 5)
			{
				TFT_Pixel(&TFT1Handle, i, j, White);
			}
		}


		/* Y-axis of Figure2 */

		// 1. Draw y-axis
		TFT_Line(&TFT1Handle, 32, 124, 32, 216, White);		// Vertical line of y-axis
		TFT_Line(&TFT1Handle, 27, 129, 32, 124, White);		// Left arrow head of y-axis
		TFT_Line(&TFT1Handle, 37, 129, 32, 124, White);		// Right arrow head of y-axis

		// 2. Draw dotted lines for y-axis
		for(int j = 138; j <= 190; j = j + 26)
		{
			for(int i = 33; i <= 310; i = i + 5)
			{
				TFT_Pixel(&TFT1Handle, i, j, White);
			}
		}

		Label_Axis();
	}

	else if(State == STATE_FREQ_DOMAIN)
	{
		/* X-axis of Frequency Spectrum */

		TFT_Line(&TFT1Handle, 30, 220, 310, 220, White);	// Horizontal line of x-axis
		TFT_Line(&TFT1Handle, 305,215, 310, 220, White);	// Upper arrow head of x-axis
		TFT_Line(&TFT1Handle, 305,225, 310, 220, White);	// Lower arrow head of x-axis

		TFT_Line(&TFT1Handle, 80, 220, 80, 222, White);		// x-axis scale
		TFT_Line(&TFT1Handle, 130, 220, 130, 222, White);
		TFT_Line(&TFT1Handle, 180, 220, 180, 222, White);
		TFT_Line(&TFT1Handle, 230, 220, 230, 222, White);
		TFT_Line(&TFT1Handle, 280, 220, 280, 222, White);

		/* Y-axis of Frequency Spectrum */
		TFT_Line(&TFT1Handle, 30,  28,  30, 220, White);	// Vertical line of y-axis
		TFT_Line(&TFT1Handle, 35,  33,  30,  28, White);	// Left arrow head of y-axis
		TFT_Line(&TFT1Handle, 25,  33,  30,  28, White);	// Right arrow head of y-axis

		for(int y = 40; y <= 202; y += 18)					// y-axis scale
		{
			TFT_Line(&TFT1Handle, 28,y, 30,y, White);
		}

		Label_Axis();
	}
}


static void Label_Axis(void)
{
	if( (State == STATE_TIME_DOMAIN) || (State == STATE_TIME_DOMAIN_SPLIT) )
	{
		/*****************************************************
		 * 				Erase Previous Time Unit	  		 *
		 *****************************************************/

		/* Window Re-Setting */

		// 1. x = 0 ~ 319
		TFT_Write(&TFT1Handle, 0x02, 0U >> 8);
		TFT_Write(&TFT1Handle, 0x03, 0U & 0x00FF);
		TFT_Write(&TFT1Handle, 0x04, 319U >> 8);
		TFT_Write(&TFT1Handle, 0x05, 319U & 0x00FF);

		// 2. y = 222 ~ 239
		TFT_Write(&TFT1Handle, 0x06, 0x0000);
		TFT_Write(&TFT1Handle, 0x07, 222U);
		TFT_Write(&TFT1Handle, 0x08, 0x0000);
		TFT_Write(&TFT1Handle, 0x09, 239U);

		TFT_Command(&TFT1Handle, 0x22);

		// 3. Erase Previous Time unit by Filling the Window with Black
		for(uint16_t i = 0; i < 320; i++)
		{
			for(uint16_t j = 0; j < 18; j++)
			{
				TFT_Data(&TFT1Handle, Black);
			}
		}

		/* Return the Window setting to its Original state */

		// 1. x = 0 ~ 319
		TFT_Write(&TFT1Handle, 0x02, 0x0000);
		TFT_Write(&TFT1Handle, 0x03, 0x0000);
		TFT_Write(&TFT1Handle, 0x04, 0x0001);
		TFT_Write(&TFT1Handle, 0x05, 0x003F);


		// 2. y = 0 ~ 239
		TFT_Write(&TFT1Handle, 0x06, 0x0000);
		TFT_Write(&TFT1Handle, 0x07, 0x0000);
		TFT_Write(&TFT1Handle, 0x08, 0x0000);
		TFT_Write(&TFT1Handle, 0x09, 0x00EF);

		TFT_Command(&TFT1Handle, 0x22);


		/*****************************************************
		 * 					 Print Time Unit	 	 		 *
		 *****************************************************/

		/* Labeling X-axis */

		if(TimeUnit == TIME_UNIT_10MS)
		{
			// 1. Print Time label[ms] corresponding to the x-axis scale
			TFT_Color(&TFT1Handle, Cyan, Black);
			TFT_English_pixel(&TFT1Handle, 28, 222, '0');		// time label of 0ms

			TFT_English_pixel(&TFT1Handle, 74, 222, '1');		// time label of 10ms
			TFT_English_pixel(&TFT1Handle, 82, 222, '0');

			TFT_English_pixel(&TFT1Handle, 124, 222, '2');		// time label of 20ms
			TFT_English_pixel(&TFT1Handle, 132, 222, '0');

			TFT_English_pixel(&TFT1Handle, 174, 222, '3');		// time label of 30ms
			TFT_English_pixel(&TFT1Handle, 182, 222, '0');

			TFT_English_pixel(&TFT1Handle, 224, 222, '4');		// time label of 40ms
			TFT_English_pixel(&TFT1Handle, 232, 222, '0');

			// 2. Print Time unit[ms] to x-axis
			TFT_Color(&TFT1Handle, Magenta, Black);
			TFT_English_pixel(&TFT1Handle, 288, 222, '[');		// time unit[ms]
			TFT_English_pixel(&TFT1Handle, 296, 222, 'm');
			TFT_English_pixel(&TFT1Handle, 304, 222, 's');
			TFT_English_pixel(&TFT1Handle, 312, 222, ']');
		}
		else if(TimeUnit == TIME_UNIT_20MS)
		{
			// 1. Print Time label[ms] corresponding to the x-axis scale
			TFT_Color(&TFT1Handle, Cyan, Black);
			TFT_English_pixel(&TFT1Handle, 28, 222, '0');		// time label of 0ms

			TFT_English_pixel(&TFT1Handle, 74, 222, '2');		// time label of 20ms
			TFT_English_pixel(&TFT1Handle, 82, 222, '0');

			TFT_English_pixel(&TFT1Handle, 124, 222, '4');		// time label of 40ms
			TFT_English_pixel(&TFT1Handle, 132, 222, '0');

			TFT_English_pixel(&TFT1Handle, 174, 222, '6');		// time label of 60ms
			TFT_English_pixel(&TFT1Handle, 182, 222, '0');

			TFT_English_pixel(&TFT1Handle, 224, 222, '8');		// time label of 80ms
			TFT_English_pixel(&TFT1Handle, 232, 222, '0');

			// 2. Print Time unit[ms] to x-axis
			TFT_Color(&TFT1Handle, Magenta, Black);
			TFT_English_pixel(&TFT1Handle, 288, 222, '[');		// time unit[ms]
			TFT_English_pixel(&TFT1Handle, 296, 222, 'm');
			TFT_English_pixel(&TFT1Handle, 304, 222, 's');
			TFT_English_pixel(&TFT1Handle, 312, 222, ']');
		}
		else if(TimeUnit == TIME_UNIT_50MS)
		{
			// 1. Print Time label[ms] corresponding to the x-axis scale
			TFT_Color(&TFT1Handle, Cyan, Black);
			TFT_English_pixel(&TFT1Handle, 28, 222, '0');		// time label of 0ms

			TFT_English_pixel(&TFT1Handle, 74, 222, '5');		// time label of 50ms
			TFT_English_pixel(&TFT1Handle, 82, 222, '0');

			TFT_English_pixel(&TFT1Handle, 120, 222, '1');		// time label of 100ms
			TFT_English_pixel(&TFT1Handle, 128, 222, '0');
			TFT_English_pixel(&TFT1Handle, 136, 222, '0');

			TFT_English_pixel(&TFT1Handle, 170, 222, '1');		// time label of 150ms
			TFT_English_pixel(&TFT1Handle, 178, 222, '5');
			TFT_English_pixel(&TFT1Handle, 186, 222, '0');

			TFT_English_pixel(&TFT1Handle, 220, 222, '2');		// time label of 200ms
			TFT_English_pixel(&TFT1Handle, 228, 222, '0');
			TFT_English_pixel(&TFT1Handle, 236, 222, '0');

			// 2. Print Time unit[ms] to x-axis
			TFT_Color(&TFT1Handle, Magenta, Black);
			TFT_English_pixel(&TFT1Handle, 288, 222, '[');		// time unit[ms]
			TFT_English_pixel(&TFT1Handle, 296, 222, 'm');
			TFT_English_pixel(&TFT1Handle, 304, 222, 's');
			TFT_English_pixel(&TFT1Handle, 312, 222, ']');
		}
		else if(TimeUnit == TIME_UNIT_100MS)
		{
			// 1. Print Time label[ms] corresponding to the x-axis scale
			TFT_Color(&TFT1Handle, Cyan, Black);
			TFT_English_pixel(&TFT1Handle, 28, 222, '0');		// time label of 0ms

			TFT_English_pixel(&TFT1Handle, 70, 222, '1');		// time label of 100ms
			TFT_English_pixel(&TFT1Handle, 78, 222, '0');
			TFT_English_pixel(&TFT1Handle, 86, 222, '0');

			TFT_English_pixel(&TFT1Handle, 120, 222, '2');		// time label of 200ms
			TFT_English_pixel(&TFT1Handle, 128, 222, '0');
			TFT_English_pixel(&TFT1Handle, 136, 222, '0');

			TFT_English_pixel(&TFT1Handle, 170, 222, '3');		// time label of 300ms
			TFT_English_pixel(&TFT1Handle, 178, 222, '0');
			TFT_English_pixel(&TFT1Handle, 186, 222, '0');

			TFT_English_pixel(&TFT1Handle, 220, 222, '4');		// time label of 400ms
			TFT_English_pixel(&TFT1Handle, 228, 222, '0');
			TFT_English_pixel(&TFT1Handle, 236, 222, '0');

			// 2. Print Time unit[ms] to x-axis
			TFT_Color(&TFT1Handle, Magenta, Black);
			TFT_English_pixel(&TFT1Handle, 288, 222, '[');		// time unit[ms]
			TFT_English_pixel(&TFT1Handle, 296, 222, 'm');
			TFT_English_pixel(&TFT1Handle, 304, 222, 's');
			TFT_English_pixel(&TFT1Handle, 312, 222, ']');
		}
		else if(TimeUnit == TIME_UNIT_200MS)
		{
			// 1. Print Time label[ms] corresponding to the x-axis scale
			TFT_Color(&TFT1Handle, Cyan, Black);
			TFT_English_pixel(&TFT1Handle, 28, 222, '0');		// time label of 0ms

			TFT_English_pixel(&TFT1Handle, 70, 222, '2');		// time label of 200ms
			TFT_English_pixel(&TFT1Handle, 78, 222, '0');
			TFT_English_pixel(&TFT1Handle, 86, 222, '0');

			TFT_English_pixel(&TFT1Handle, 120, 222, '4');		// time label of 400ms
			TFT_English_pixel(&TFT1Handle, 128, 222, '0');
			TFT_English_pixel(&TFT1Handle, 136, 222, '0');

			TFT_English_pixel(&TFT1Handle, 170, 222, '6');		// time label of 600ms
			TFT_English_pixel(&TFT1Handle, 178, 222, '0');
			TFT_English_pixel(&TFT1Handle, 186, 222, '0');

			TFT_English_pixel(&TFT1Handle, 220, 222, '8');		// time label of 800ms
			TFT_English_pixel(&TFT1Handle, 228, 222, '0');
			TFT_English_pixel(&TFT1Handle, 236, 222, '0');

			// 2. Print Time unit[ms] to x-axis
			TFT_Color(&TFT1Handle, Magenta, Black);
			TFT_English_pixel(&TFT1Handle, 288, 222, '[');		// time unit[ms]
			TFT_English_pixel(&TFT1Handle, 296, 222, 'm');
			TFT_English_pixel(&TFT1Handle, 304, 222, 's');
			TFT_English_pixel(&TFT1Handle, 312, 222, ']');
		}


		/* Labeling Y-axis */

		if(State == STATE_TIME_DOMAIN)
		{
			// 1. Print Voltage label[V] corresponding to the y-axis scale
			TFT_Color(&TFT1Handle, Cyan, Black);
			TFT_English_pixel(&TFT1Handle, 4, 39, '3');			// Voltage label of +3.0[V]
			TFT_English_pixel(&TFT1Handle, 12, 39, '.');
			TFT_English_pixel(&TFT1Handle, 20, 39, '0');

			TFT_English_pixel(&TFT1Handle, 4, 67, '2');			// Voltage label of +2.5[V]
			TFT_English_pixel(&TFT1Handle, 12, 67, '.');
			TFT_English_pixel(&TFT1Handle, 20, 67, '5');

			TFT_English_pixel(&TFT1Handle, 4, 95, '2');			// Voltage label of +2.0[V]
			TFT_English_pixel(&TFT1Handle, 12, 95, '.');
			TFT_English_pixel(&TFT1Handle, 20, 95, '0');

			TFT_English_pixel(&TFT1Handle, 4, 123, '1');		// Voltage label of +1.5[V]
			TFT_English_pixel(&TFT1Handle, 12, 123, '.');
			TFT_English_pixel(&TFT1Handle, 20, 123, '5');

			TFT_English_pixel(&TFT1Handle, 4, 151, '1');		// Voltage label of +1.0[V]
			TFT_English_pixel(&TFT1Handle, 12, 151, '.');
			TFT_English_pixel(&TFT1Handle, 20, 151, '0');

			TFT_English_pixel(&TFT1Handle, 4, 179, '0');		// Voltage label of +0.5[V]
			TFT_English_pixel(&TFT1Handle, 12, 179, '.');
			TFT_English_pixel(&TFT1Handle, 20, 179, '5');

			// 3. Print Voltage unit[V] to y-axis
			TFT_Color(&TFT1Handle, Magenta, Black);
			TFT_English_pixel(&TFT1Handle, 2, 12, '[');
			TFT_English_pixel(&TFT1Handle, 10, 12, 'V');
			TFT_English_pixel(&TFT1Handle, 18, 12, ']');
		}
		else if(State == STATE_TIME_DOMAIN_SPLIT)
		{
			/* Labeling Y-axis for Figure1 & 2 */

			// 1. Print Voltage label[V] corresponding to the y-axis scale for Figure1
			TFT_Color(&TFT1Handle, Cyan, Black);
			TFT_English_pixel(&TFT1Handle, 4, 30, '3');			// Voltage label of +3.0[V]
			TFT_English_pixel(&TFT1Handle, 12, 30, '.');
			TFT_English_pixel(&TFT1Handle, 20, 30, '0');

			TFT_English_pixel(&TFT1Handle, 4, 56, '2');			// Voltage label of +2.0[V]
			TFT_English_pixel(&TFT1Handle, 12, 56, '.');
			TFT_English_pixel(&TFT1Handle, 20, 56, '0');

			TFT_English_pixel(&TFT1Handle, 4, 82, '1');			// Voltage label of +1.0[V]
			TFT_English_pixel(&TFT1Handle, 12, 82, '.');
			TFT_English_pixel(&TFT1Handle, 20, 82, '0');

			// 2. Print Voltage label[V] corresponding to the y-axis scale for Figure2
			TFT_Color(&TFT1Handle, Cyan, Black);
			TFT_English_pixel(&TFT1Handle, 4, 131, '3');			// Voltage label of +3.0[V]
			TFT_English_pixel(&TFT1Handle, 12, 131, '.');
			TFT_English_pixel(&TFT1Handle, 20, 131, '0');

			TFT_English_pixel(&TFT1Handle, 4, 157, '2');			// Voltage label of +2.0[V]
			TFT_English_pixel(&TFT1Handle, 12, 157, '.');
			TFT_English_pixel(&TFT1Handle, 20, 157, '0');

			TFT_English_pixel(&TFT1Handle, 4, 183, '1');			// Voltage label of +1.0[V]
			TFT_English_pixel(&TFT1Handle, 12, 183, '.');
			TFT_English_pixel(&TFT1Handle, 20, 183, '0');

			// 3. Print Voltage unit[V] to y-axis
			TFT_Color(&TFT1Handle, Magenta, Black);
			TFT_English_pixel(&TFT1Handle, 2, 12, '[');
			TFT_English_pixel(&TFT1Handle, 10, 12, 'V');
			TFT_English_pixel(&TFT1Handle, 18, 12, ']');
		}
	}

	else if(State == STATE_FREQ_DOMAIN)
	{
		/*****************************************************
		 * 				Erase Previous Freq Unit	  		 *
		 *****************************************************/

		/* Window Re-Setting */

		// 1. x = 0 ~ 265
		TFT_Write(&TFT1Handle, 0x02, 0U >> 8);
		TFT_Write(&TFT1Handle, 0x03, 0U & 0x00FF);
		TFT_Write(&TFT1Handle, 0x04, 265U >> 8);
		TFT_Write(&TFT1Handle, 0x05, 265U & 0x00FF);

		// 2. y = 225 ~ 239
		TFT_Write(&TFT1Handle, 0x06, 0x0000);
		TFT_Write(&TFT1Handle, 0x07, 225U);
		TFT_Write(&TFT1Handle, 0x08, 0x0000);
		TFT_Write(&TFT1Handle, 0x09, 239U);

		TFT_Command(&TFT1Handle, 0x22);

		// 3. Erase Previous Freq unit by Filling the Window with Black
		for(uint16_t i = 0; i < 266; i++)
		{
			for(uint16_t j = 0; j < 15; j++)
			{
				TFT_Data(&TFT1Handle, Black);
			}
		}

		/* Return the Window setting to its Original state */

		// 1. x = 0 ~ 319
		TFT_Write(&TFT1Handle, 0x02, 0x0000);
		TFT_Write(&TFT1Handle, 0x03, 0x0000);
		TFT_Write(&TFT1Handle, 0x04, 0x0001);
		TFT_Write(&TFT1Handle, 0x05, 0x003F);


		// 2. y = 0 ~ 239
		TFT_Write(&TFT1Handle, 0x06, 0x0000);
		TFT_Write(&TFT1Handle, 0x07, 0x0000);
		TFT_Write(&TFT1Handle, 0x08, 0x0000);
		TFT_Write(&TFT1Handle, 0x09, 0x00EF);

		TFT_Command(&TFT1Handle, 0x22);


		/*****************************************************
		 * 					 Print Freq Unit	 	 		 *
		 *****************************************************/

		/* Labeling X-axis */

		if(FreqUnit == FREQ_UNIT_5HZ)
		{
			// 1. Print Freq label[Hz] corresponding to the x-axis scale
			TFT_Color(&TFT1Handle, White,Black);

			TFT_English_pixel(&TFT1Handle, 26, 222, '0');		// freq label of 0Hz

			TFT_English_pixel(&TFT1Handle, 76, 222, '5');		// freq label of 5Hz

			TFT_English_pixel(&TFT1Handle, 122, 222, '1');		// freq label of 10Hz
			TFT_English_pixel(&TFT1Handle, 130, 222, '0');

			TFT_English_pixel(&TFT1Handle, 172, 222, '1');		// freq label of 15Hz
			TFT_English_pixel(&TFT1Handle, 180, 222, '5');

			TFT_English_pixel(&TFT1Handle, 222, 222, '2');		// freq label of 20Hz
			TFT_English_pixel(&TFT1Handle, 230, 222, '0');

			// 2. Print Freq unit[Hz] to x-axis
			TFT_Color(&TFT1Handle, Magenta,Black);
			TFT_English_pixel(&TFT1Handle, 266,223, '[');		// [Hz]
			TFT_English_pixel(&TFT1Handle, 276,223, 'H');
			TFT_English_pixel(&TFT1Handle, 284,223, 'z');
			TFT_English_pixel(&TFT1Handle, 292,223, ']');
		}
		else if(FreqUnit == FREQ_UNIT_10HZ)
		{
			// 1. Print Freq label[Hz] corresponding to the x-axis scale
			TFT_Color(&TFT1Handle, White,Black);

			TFT_English_pixel(&TFT1Handle, 26, 222, '0');		// freq label of 0Hz

			TFT_English_pixel(&TFT1Handle, 72, 222, '1');		// freq label of 10Hz
			TFT_English_pixel(&TFT1Handle, 80, 222, '0');

			TFT_English_pixel(&TFT1Handle, 122, 222, '2');		// freq label of 20Hz
			TFT_English_pixel(&TFT1Handle, 130, 222, '0');

			TFT_English_pixel(&TFT1Handle, 172, 222, '3');		// freq label of 30Hz
			TFT_English_pixel(&TFT1Handle, 180, 222, '0');

			TFT_English_pixel(&TFT1Handle, 222, 222, '4');		// freq label of 40Hz
			TFT_English_pixel(&TFT1Handle, 230, 222, '0');

			// 2. Print Freq unit[Hz] to x-axis
			TFT_Color(&TFT1Handle, Magenta,Black);
			TFT_English_pixel(&TFT1Handle, 266,223, '[');		// [Hz]
			TFT_English_pixel(&TFT1Handle, 276,223, 'H');
			TFT_English_pixel(&TFT1Handle, 284,223, 'z');
			TFT_English_pixel(&TFT1Handle, 292,223, ']');
		}
		else if(FreqUnit == FREQ_UNIT_20HZ)
		{
			// 1. Print Freq label[Hz] corresponding to the x-axis scale
			TFT_Color(&TFT1Handle, White,Black);

			TFT_English_pixel(&TFT1Handle, 26, 222, '0');		// freq label of 0Hz

			TFT_English_pixel(&TFT1Handle, 72, 222, '2');		// freq label of 20Hz
			TFT_English_pixel(&TFT1Handle, 80, 222, '0');

			TFT_English_pixel(&TFT1Handle, 122, 222, '4');		// freq label of 40Hz
			TFT_English_pixel(&TFT1Handle, 130, 222, '0');

			TFT_English_pixel(&TFT1Handle, 172, 222, '6');		// freq label of 60Hz
			TFT_English_pixel(&TFT1Handle, 180, 222, '0');

			TFT_English_pixel(&TFT1Handle, 222, 222, '8');		// freq label of 80Hz
			TFT_English_pixel(&TFT1Handle, 230, 222, '0');

			// 2. Print Freq unit[Hz] to x-axis
			TFT_Color(&TFT1Handle, Magenta,Black);
			TFT_English_pixel(&TFT1Handle, 266,223, '[');		// [Hz]
			TFT_English_pixel(&TFT1Handle, 276,223, 'H');
			TFT_English_pixel(&TFT1Handle, 284,223, 'z');
			TFT_English_pixel(&TFT1Handle, 292,223, ']');
		}
		else if(FreqUnit == FREQ_UNIT_50HZ)
		{
			// 1. Print Freq label[Hz] corresponding to the x-axis scale
			TFT_Color(&TFT1Handle, White,Black);

			TFT_English_pixel(&TFT1Handle, 26, 222, '0');		// freq label of 0Hz

			TFT_English_pixel(&TFT1Handle, 72, 222, '5');		// freq label of 50Hz
			TFT_English_pixel(&TFT1Handle, 80, 222, '0');

			TFT_English_pixel(&TFT1Handle, 118, 222, '1');		// freq label of 100Hz
			TFT_English_pixel(&TFT1Handle, 126, 222, '0');
			TFT_English_pixel(&TFT1Handle, 134, 222, '0');

			TFT_English_pixel(&TFT1Handle, 168, 222, '1');		// freq label of 150Hz
			TFT_English_pixel(&TFT1Handle, 176, 222, '5');
			TFT_English_pixel(&TFT1Handle, 184, 222, '0');

			TFT_English_pixel(&TFT1Handle, 218, 222, '2');		// freq label of 200Hz
			TFT_English_pixel(&TFT1Handle, 226, 222, '0');
			TFT_English_pixel(&TFT1Handle, 234, 222, '0');

			// 2. Print Freq unit[Hz] to x-axis
			TFT_Color(&TFT1Handle, Magenta,Black);
			TFT_English_pixel(&TFT1Handle, 266,223, '[');		// [Hz]
			TFT_English_pixel(&TFT1Handle, 276,223, 'H');
			TFT_English_pixel(&TFT1Handle, 284,223, 'z');
			TFT_English_pixel(&TFT1Handle, 292,223, ']');
		}
		else if(FreqUnit == FREQ_UNIT_100HZ)
		{
			// 1. Print Freq label[Hz] corresponding to the x-axis scale
			TFT_Color(&TFT1Handle, White,Black);

			TFT_English_pixel(&TFT1Handle, 26, 222, '0');		// freq label of 0Hz

			TFT_English_pixel(&TFT1Handle, 68, 222, '1');		// freq label of 100Hz
			TFT_English_pixel(&TFT1Handle, 76, 222, '0');
			TFT_English_pixel(&TFT1Handle, 84, 222, '0');

			TFT_English_pixel(&TFT1Handle, 118, 222, '2');		// freq label of 200Hz
			TFT_English_pixel(&TFT1Handle, 126, 222, '0');
			TFT_English_pixel(&TFT1Handle, 134, 222, '0');

			TFT_English_pixel(&TFT1Handle, 168, 222, '3');		// freq label of 300Hz
			TFT_English_pixel(&TFT1Handle, 176, 222, '0');
			TFT_English_pixel(&TFT1Handle, 184, 222, '0');

			TFT_English_pixel(&TFT1Handle, 218, 222, '4');		// freq label of 400Hz
			TFT_English_pixel(&TFT1Handle, 226, 222, '0');
			TFT_English_pixel(&TFT1Handle, 234, 222, '0');

			// 2. Print Freq unit[Hz] to x-axis
			TFT_Color(&TFT1Handle, Magenta,Black);
			TFT_English_pixel(&TFT1Handle, 266,223, '[');		// [Hz]
			TFT_English_pixel(&TFT1Handle, 276,223, 'H');
			TFT_English_pixel(&TFT1Handle, 284,223, 'z');
			TFT_English_pixel(&TFT1Handle, 292,223, ']');
		}
		else if(FreqUnit == FREQ_UNIT_500HZ)
		{
			// 1. Print Freq label[Hz] corresponding to the x-axis scale
			TFT_Color(&TFT1Handle, White,Black);

			TFT_English_pixel(&TFT1Handle, 26, 222, '0');		// freq label of 0Hz

			TFT_English_pixel(&TFT1Handle, 68, 222, '0');		// freq label of 0.5kHz
			TFT_English_pixel(&TFT1Handle, 76, 222, '.');
			TFT_English_pixel(&TFT1Handle, 84, 222, '5');

			TFT_English_pixel(&TFT1Handle, 118, 222, '1');		// freq label of 1kHz
			TFT_English_pixel(&TFT1Handle, 126, 222, '.');
			TFT_English_pixel(&TFT1Handle, 134, 222, '0');

			TFT_English_pixel(&TFT1Handle, 168, 222, '1');		// freq label of 1.5kHz
			TFT_English_pixel(&TFT1Handle, 176, 222, '.');
			TFT_English_pixel(&TFT1Handle, 184, 222, '5');

			TFT_English_pixel(&TFT1Handle, 218, 222, '2');		// freq label of 2kHz
			TFT_English_pixel(&TFT1Handle, 226, 222, '.');
			TFT_English_pixel(&TFT1Handle, 234, 222, '0');

			// 2. Print Freq unit[Hz] to x-axis
			TFT_Color(&TFT1Handle, Magenta,Black);
			TFT_English_pixel(&TFT1Handle, 258,223, '[');
			TFT_English_pixel(&TFT1Handle, 266,223, 'k');		// [Hz]
			TFT_English_pixel(&TFT1Handle, 276,223, 'H');
			TFT_English_pixel(&TFT1Handle, 284,223, 'z');
			TFT_English_pixel(&TFT1Handle, 292,223, ']');
		}


		/* Labeling Y-axis */

		// 1. Print Magnitude label[%] corresponding to the y-axis scale for Frequency spectrum
		TFT_Color(&TFT1Handle, White,Black);
		TFT_English_pixel(&TFT1Handle, 6,195, '1');			// 10
		TFT_English_pixel(&TFT1Handle, 14,195, '0');
		TFT_English_pixel(&TFT1Handle, 6,177, '2');         // 20
		TFT_English_pixel(&TFT1Handle, 14,177, '0');
		TFT_English_pixel(&TFT1Handle, 6,159, '3');       	// 30
		TFT_English_pixel(&TFT1Handle, 14,159, '0');
		TFT_English_pixel(&TFT1Handle, 6,141, '4');         // 40
		TFT_English_pixel(&TFT1Handle, 14,141, '0');
		TFT_English_pixel(&TFT1Handle, 6,123, '5');         // 50
		TFT_English_pixel(&TFT1Handle, 14,123, '0');
		TFT_English_pixel(&TFT1Handle, 6,105, '6');         // 60
		TFT_English_pixel(&TFT1Handle, 14,105, '0');
		TFT_English_pixel(&TFT1Handle, 6, 87, '7');         // 70
		TFT_English_pixel(&TFT1Handle, 14, 87, '0');
		TFT_English_pixel(&TFT1Handle, 6, 69, '8');         // 80
		TFT_English_pixel(&TFT1Handle, 14, 69, '0');
		TFT_English_pixel(&TFT1Handle, 6, 51, '9');         // 90
		TFT_English_pixel(&TFT1Handle, 14, 51, '0');
		TFT_English_pixel(&TFT1Handle, 0,  33, '1');		// 100
		TFT_English_pixel(&TFT1Handle, 8, 33, '0');
		TFT_English_pixel(&TFT1Handle, 16, 33, '0');

		// 3. Print Magnitude unit[%] to y-axis
		TFT_Color(&TFT1Handle, Magenta,Black);
		TFT_English_pixel(&TFT1Handle, 2,  16, '[');        // [%]
		TFT_English_pixel(&TFT1Handle, 10, 16, '%');
		TFT_English_pixel(&TFT1Handle, 18, 16, ']');


	}

	else if(State == STATE_FILTER_TEST)
	{

	}


}


void Change_TimeUnit(uint8_t direction)
{
	if((State == STATE_TIME_DOMAIN) || (State == STATE_TIME_DOMAIN_SPLIT))
	{
		if(direction == UP)
		{
			if(TimeUnit >= TIME_UNIT_200MS)			TimeUnit = TIME_UNIT_200MS;
			else									TimeUnit++;
		}

		else if(direction == DOWN)
		{
			if(TimeUnit <= TIME_UNIT_10MS)			TimeUnit = TIME_UNIT_10MS;
			else									TimeUnit--;
		}


		Label_Axis();

		switch(TimeUnit)
		{
			case TIME_UNIT_10MS :
			{
				DownSamplingCountMax = DOWN_SAMPLING_COUNT_TIME_UNIT_10MS;
				break;
			}

			case TIME_UNIT_20MS :
			{
				DownSamplingCountMax = DOWN_SAMPLING_COUNT_TIME_UNIT_20MS;
				break;
			}

			case TIME_UNIT_50MS :
			{
				DownSamplingCountMax = DOWN_SAMPLING_COUNT_TIME_UNIT_50MS;
				break;
			}

			case TIME_UNIT_100MS :
			{
				DownSamplingCountMax = DOWN_SAMPLING_COUNT_TIME_UNIT_100MS;
				break;
			}

			case TIME_UNIT_200MS :
			{
				DownSamplingCountMax = DOWN_SAMPLING_COUNT_TIME_UNIT_200MS;
				break;
			}


			default :
				break;
		}
	}
}


void Change_FreqUnit(uint8_t direction)
{
	if(direction == UP)
	{
		if(FreqUnit >= FREQ_UNIT_500HZ)		FreqUnit = FREQ_UNIT_500HZ;
		else								FreqUnit++;
	}

	else if(direction == DOWN)
	{
		if(FreqUnit <= FREQ_UNIT_5HZ)		FreqUnit = FREQ_UNIT_5HZ;
		else								FreqUnit--;
	}

	Label_Axis();

	switch(FreqUnit)
	{
		case FREQ_UNIT_5HZ :
		{
			DownSamplingCountMax = DOWN_SAMPLING_COUNT_FREQ_UNIT_5HZ;
			break;
		}

		case FREQ_UNIT_10HZ :
		{
			DownSamplingCountMax = DOWN_SAMPLING_COUNT_FREQ_UNIT_10HZ;
			break;
		}

		case FREQ_UNIT_20HZ :
		{
			DownSamplingCountMax = DOWN_SAMPLING_COUNT_FREQ_UNIT_20HZ;
			break;
		}

		case FREQ_UNIT_50HZ :
		{
			DownSamplingCountMax = DOWN_SAMPLING_COUNT_FREQ_UNIT_50HZ;
			break;
		}

		case FREQ_UNIT_100HZ :
		{
			DownSamplingCountMax = DOWN_SAMPLING_COUNT_FREQ_UNIT_100HZ;
			break;
		}

		case FREQ_UNIT_500HZ :
		{
			DownSamplingCountMax = DOWN_SAMPLING_COUNT_FREQ_UNIT_500HZ;
			break;
		}

		default :
			break;
	}
}


void Clear_Graph(TFT_HandleTypeDef *pTFTHandle)
{
	if(State == STATE_TIME_DOMAIN)
	{
		/* Window Re-Setting */

		// 1. x = 33 ~ 304
		TFT_Write(pTFTHandle, 0x02, 33U >> 8);
		TFT_Write(pTFTHandle, 0x03, 33U & 0x00FF);
		TFT_Write(pTFTHandle, 0x04, 304U >> 8);
		TFT_Write(pTFTHandle, 0x05, 304U & 0x00FF);

		// 2. y = 29 ~ 215
		TFT_Write(pTFTHandle, 0x06, 0x0000);
		TFT_Write(pTFTHandle, 0x07, 29U);
		TFT_Write(pTFTHandle, 0x08, 0x0000);
		TFT_Write(pTFTHandle, 0x09, 215U);

		TFT_Command(pTFTHandle, 0x22);

		// 3. Clear graph of Figure1 by Filling the Window with Black
		for(uint16_t i = 0; i < 272; i++)
		{
			for(uint16_t j = 0; j < 187; j++)
			{
				TFT_Data(pTFTHandle, Black);
			}
		}

		// 4-1. Draw dotted grid for x-axis
		for(int i = 82; i <= 319; i = i + 50)
		{
			for(int j = 29; j <= 216; j = j + 5)
			{
				TFT_Pixel(&TFT1Handle, i, j, White);
			}
		}

		// 4-2. Draw dotted grid for y-axis
		for(int j = 46; j <= 186; j = j + 28)
		{
			for(int i = 33; i <= 310; i = i + 5)
			{
				TFT_Pixel(&TFT1Handle, i, j, White);
			}
		}


		/* Return the Window setting to its Original state */

		// 1. x = 0 ~ 319
		TFT_Write(pTFTHandle, 0x02, 0x0000);
		TFT_Write(pTFTHandle, 0x03, 0x0000);
		TFT_Write(pTFTHandle, 0x04, 0x0001);
		TFT_Write(pTFTHandle, 0x05, 0x003F);


		// 2. y = 0 ~ 239
		TFT_Write(pTFTHandle, 0x06, 0x0000);
		TFT_Write(pTFTHandle, 0x07, 0x0000);
		TFT_Write(pTFTHandle, 0x08, 0x0000);
		TFT_Write(pTFTHandle, 0x09, 0x00EF);

		TFT_Command(pTFTHandle, 0x22);
	}

	else if(State == STATE_TIME_DOMAIN_SPLIT)
	{
		/* Window Re-Setting for Figure1 */

		// 1. x = 33 ~ 304
		TFT_Write(pTFTHandle, 0x02, 33U >> 8);
		TFT_Write(pTFTHandle, 0x03, 33U & 0x00FF);
		TFT_Write(pTFTHandle, 0x04, 304U >> 8);
		TFT_Write(pTFTHandle, 0x05, 304U & 0x00FF);

		// 2. y = 29 ~ 114
		TFT_Write(pTFTHandle, 0x06, 0x0000);
		TFT_Write(pTFTHandle, 0x07, 29U);
		TFT_Write(pTFTHandle, 0x08, 0x0000);
		TFT_Write(pTFTHandle, 0x09, 114U);

		TFT_Command(pTFTHandle, 0x22);

		// 3. Clear graph of Figure1 by Filling the Window with Black
		for(uint16_t i = 0; i < 272; i++)
		{
			for(uint16_t j = 0; j < 86; j++)
			{
				TFT_Data(pTFTHandle, Black);
			}
		}

		// 4-1. Draw dotted grid for x-axis
		for(int i = 82; i <= 319; i = i + 50)
		{
			for(int j = 29; j <= 115; j = j + 5)
			{
				TFT_Pixel(&TFT1Handle, i, j, White);
			}
		}

		// 4-2. Draw dotted grid for y-axis
		for(int j = 37; j <= 89; j = j + 26)
		{
			for(int i = 33; i <= 310; i = i + 5)
			{
				TFT_Pixel(&TFT1Handle, i, j, White);
			}
		}


		/* Window Re-Setting for Figure2 */

		// 1. x = 33 ~ 304
		TFT_Write(pTFTHandle, 0x02, 33U >> 8);
		TFT_Write(pTFTHandle, 0x03, 33U & 0x00FF);
		TFT_Write(pTFTHandle, 0x04, 304U >> 8);
		TFT_Write(pTFTHandle, 0x05, 304U & 0x00FF);

		// 2. y = 130 ~ 215
		TFT_Write(pTFTHandle, 0x06, 0x0000);
		TFT_Write(pTFTHandle, 0x07, 130U);
		TFT_Write(pTFTHandle, 0x08, 0x0000);
		TFT_Write(pTFTHandle, 0x09, 215U);

		TFT_Command(pTFTHandle, 0x22);

		// 3. Clear graph of Figure1 by Filling the Window with Black
		for(uint16_t i = 0; i < 272; i++)
		{
			for(uint16_t j = 0; j < 86; j++)
			{
				TFT_Data(pTFTHandle, Black);
			}
		}

		// 4-1. Draw dotted grid for x-axis
		for(int i = 82; i <= 319; i = i + 50)
		{
			for(int j = 130; j <= 216; j = j + 5)
			{
				TFT_Pixel(&TFT1Handle, i, j, White);
			}
		}

		// 4-2. Draw dotted grid for y-axis
		for(int j = 138; j <= 190; j = j + 26)
		{
			for(int i = 33; i <= 310; i = i + 5)
			{
				TFT_Pixel(&TFT1Handle, i, j, White);
			}
		}


		/* Return the Window setting to its Original state */

		// 1. x = 0 ~ 319
		TFT_Write(pTFTHandle, 0x02, 0x0000);
		TFT_Write(pTFTHandle, 0x03, 0x0000);
		TFT_Write(pTFTHandle, 0x04, 0x0001);
		TFT_Write(pTFTHandle, 0x05, 0x003F);


		// 2. y = 0 ~ 239
		TFT_Write(pTFTHandle, 0x06, 0x0000);
		TFT_Write(pTFTHandle, 0x07, 0x0000);
		TFT_Write(pTFTHandle, 0x08, 0x0000);
		TFT_Write(pTFTHandle, 0x09, 0x00EF);

		TFT_Command(pTFTHandle, 0x22);
	}

	else if(State == STATE_FREQ_DOMAIN)
	{
		/* Window Re-Setting */

		// 1. x = 31 ~ 304
		TFT_Write(pTFTHandle, 0x02, 31U >> 8);
		TFT_Write(pTFTHandle, 0x03, 31U & 0x00FF);
		TFT_Write(pTFTHandle, 0x04, 304U >> 8);
		TFT_Write(pTFTHandle, 0x05, 304U & 0x00FF);

		// 2. y = 34 ~ 219
		TFT_Write(pTFTHandle, 0x06, 0x0000);
		TFT_Write(pTFTHandle, 0x07, 34U);
		TFT_Write(pTFTHandle, 0x08, 0x0000);
		TFT_Write(pTFTHandle, 0x09, 219U);

		TFT_Command(pTFTHandle, 0x22);

		// 3. Clear graph by Filling the Window with Black
		for(uint16_t i = 0; i < 274; i++)
		{
			for(uint16_t j = 0; j < 186; j++)
			{
				TFT_Data(pTFTHandle, Black);
			}
		}


		/* Return the Window setting to its Original state */

		// 1. x = 0 ~ 319
		TFT_Write(pTFTHandle, 0x02, 0x0000);
		TFT_Write(pTFTHandle, 0x03, 0x0000);
		TFT_Write(pTFTHandle, 0x04, 0x0001);
		TFT_Write(pTFTHandle, 0x05, 0x003F);


		// 2. y = 0 ~ 239
		TFT_Write(pTFTHandle, 0x06, 0x0000);
		TFT_Write(pTFTHandle, 0x07, 0x0000);
		TFT_Write(pTFTHandle, 0x08, 0x0000);
		TFT_Write(pTFTHandle, 0x09, 0x00EF);

		TFT_Command(pTFTHandle, 0x22);
	}
}


void Key_InputDetection(void)
{
	/* Mode Key Press Detection */
	if(KeyFlag_Mode == FLAG_SET)
	{
		KeyTime_Mode++;

		if(KeyTime_Mode >= 20000)
		{
			/* Key is still pressed */
			if(READ_BIT(GPIOB->IDR, GPIO_PIN_0) == 0)
			{
				// to be implemented
			}

			/* Key is not pressed anymore */
			else
			{
				KeyCount_Mode++;

				KeyFlag_Mode = FLAG_RESET;
				KeyTime_Mode = 0;
			}
		}
	}

	/* Up Key Press Detection */
	else if(KeyFlag_Up == FLAG_SET)
	{
		KeyTime_Up++;

		if(KeyTime_Up >= 20000)
		{
			/* Key is still pressed */
			if(READ_BIT(GPIOB->IDR, GPIO_PIN_1) == 0)
			{
				// to be implemented
			}

			/* Key is not pressed anymore */
			else
			{
				KeyCount_Up++;

				KeyFlag_Up = FLAG_RESET;
				KeyTime_Up = 0;
			}
		}
	}

	/* Down Key Press Detection */
	else if(KeyFlag_Down == FLAG_SET)
	{
		KeyTime_Down++;

		if(KeyTime_Down >= 20000)
		{
			/* Key is still pressed */
			if(READ_BIT(GPIOB->IDR, GPIO_PIN_2) == 0)
			{
				// to be implemented
			}

			/* Key is not pressed anymore */
			else
			{
				KeyCount_Down++;

				KeyFlag_Down = FLAG_RESET;
				KeyTime_Down = 0;
			}
		}
	}

	/* Start/Stop Key Press Detection */
	else if(KeyFlag_Set == FLAG_SET)
	{
		KeyTime_Set++;

		if(KeyTime_Set >= 20000)
		{
			/* Key is still pressed */
			if(READ_BIT(GPIOB->IDR, GPIO_PIN_3) == 0)
			{
				// to be implemented
			}

			/* Key is not pressed anymore */
			else
			{
				KeyCount_Set++;

				KeyFlag_Set = FLAG_RESET;
				KeyTime_Set = 0;
			}
		}
	}
}


/************************************************************************
 * 				  Vendor Peripheral Initialization functions			*
 ************************************************************************/
void MemsetHandleStructure(void)
{
	memset(&TIM6Handle, 0, sizeof(TIM6Handle));
	memset(&TIM7Handle, 0, sizeof(TIM7Handle));
	memset(&USART1Handle, 0, sizeof(USART1Handle));
	memset(&DMA2Handle_UART, 0, sizeof(DMA2Handle_UART));
	memset(&ADC1Handle, 0, sizeof(ADC1Handle));
	memset(&DMA2Handle_ADC, 0, sizeof(DMA2Handle_ADC));
	memset(&DAC1Handle, 0, sizeof(DAC1Handle));
	memset(&DMA1Handle_DAC, 0, sizeof(DMA1Handle_DAC));
	memset(&TFT1Handle, 0, sizeof(TFT1Handle));
}


void DMA1_Init(void)
{
	// 1. DMA1 Clock Enable
	RCC_DMA1_CLK_ENABLE();

	// 2. Configure NVIC Setting of DMA1_Stream5
	NVIC_IRQConfig(DMA1_Stream5_IRQn, 2, ENABLE);
}


void DMA2_Init(void)
{
	// 1. DMA2 Clock Enable
	RCC_DMA2_CLK_ENABLE();

	// 2. Configure NVIC Setting of DMA2_Stream7
	NVIC_IRQConfig(DMA2_Stream7_IRQn, 2, ENABLE);

	// 3. Configure NVIC Setting of DMA2_Stream4
	NVIC_IRQConfig(DMA2_Stream4_IRQn, 2, ENABLE);
}


void TIM6_Init(void)
{
	// Init TIM6 Base
	TIM6Handle.Instance = TIM6;
	TIM6Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	TIM6Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	TIM6Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	TIM6Handle.Init.Prescaler = (84-1);		// 84MHz / 84 = 1MHz
	TIM6Handle.Init.Period = (10-1);		// 1MHz / 10 = 100kHz
	TIM_Base_Init(&TIM6Handle);
}


void TIM7_Init(void)
{
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	// origin : Prescaler = 84-1, Period = 10-1

	TIM7Handle.Instance = TIM7;
	TIM7Handle.Init.Prescaler = (8400-1); 	// 84MHz / 8400 = 10kHz
	TIM7Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	TIM7Handle.Init.Period = (10-1);		// 10kHz / 10 = 1kHz(When Fs=1kHz)	(When Fs=100Hz : 10kHz / 100 = 100Hz)
	TIM7Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	TIM_Base_Init(&TIM7Handle);

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	TIMEx_MasterConfigSynchronization(&TIM7Handle, &sMasterConfig);
}


void UART1_Init(void)
{
	USART1Handle.Instance = USART1;
	USART1Handle.Init.Mode = UART_MODE_TX;
	USART1Handle.Init.OverSampling = UART_OVERSAMPLING_16;
	USART1Handle.Init.BaudRate = USART_STD_BAUD_115200;
	USART1Handle.Init.Parity = UART_PARITY_NONE;
	USART1Handle.Init.StopBits = UART_STOPBITS_1;
	USART1Handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	USART1Handle.Init.WordLength = UART_WORDLENGTH_8B;

	UART_Init(&USART1Handle);
}


void ADC1_Init(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};

	// 1. Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	ADC1Handle.Instance = ADC1;
	ADC1Handle.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	ADC1Handle.Init.Resolution = ADC_RESOLUTION_12B;
	ADC1Handle.Init.ScanConvMode = DISABLE;
	ADC1Handle.Init.ContinuousConvMode = DISABLE;
	ADC1Handle.Init.DiscontinuousConvMode = DISABLE;
	ADC1Handle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	ADC1Handle.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	ADC1Handle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	ADC1Handle.Init.NbrOfConversion = 1;
	ADC1Handle.Init.DMAContinuousRequests = ENABLE;
	ADC1Handle.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	ADC_Init(&ADC1Handle);

	// 2. Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	ADC_ConfigChannel(&ADC1Handle, &sConfig);
}


void DAC1_Init(void)
{
	DAC_ChannelConfTypeDef sConfig = {0};


	DAC1Handle.Instance = DAC;
	DAC_Init(&DAC1Handle);

	sConfig.DAC_Trigger = DAC_TRIGGER_T7_TRGO;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
	DAC_ConfigChannel(&DAC1Handle, &sConfig, DAC_CHANNEL_1);
}


void TFT1_Init(void)
{
	TFT1Handle.Instance = TFT1;
	TFT1Handle.ScreenMode = 'L';
	TFT1Handle.XcharacterLimit = 40;
	TFT1Handle.YcharacterLimit = 30;
	TFT1Handle.XcharacterLimit_Large = 26;
	TFT1Handle.YcharacterLimit_Large = 30;
	TFT1Handle.nextline_flag = 0;
	TFT1Handle.cursor_flag = 0;
	TFT1Handle.underscore_flag = 0;
	TFT1Handle.outline_flag = 0;
	TFT1Handle.Kfont_type = 'M';

	TFT_Init(&TFT1Handle);
}


/************************************************************************
 * 				 	   Peripheral Callback functions					*
 ************************************************************************/
void TIM_PeriodElapsedCallback(TIM_HandleTypeDef *pTIMHandle)
{
	static int TimerPeriodCount = 0;

	if(pTIMHandle->Instance == TIM6)
	{
		/* Timer Period of TIM6 is 10us */

		// TFlag is used for Delay functions
		TFlag = FLAG_SET;

		// Detect Key inputs
		Key_InputDetection();

		// Case of Time-domain State
		if((State == STATE_TIME_DOMAIN) || (State == STATE_TIME_DOMAIN_SPLIT))
		{
			// A/D Conversion is executed every 200us (Base TimeUnit : TIME_UNIT_10MS)
			if(TimerPeriodCount >= TimerPeriodCountMax)
			{
				TimerPeriodCount = 0;

				ADC_Start_DMA(&ADC1Handle, &AdcValue, 1);
			}
			else
			{
				TimerPeriodCount++;
			}
		}

		// Case of Freq-domain State
		else if(State == STATE_FREQ_DOMAIN)
		{
			// 1. A/D Conversion is executed every 200us (Base FreqUnit : FREQ_UNIT_5000HZ)
			if(TimerPeriodCount >= TimerPeriodCountMax)
			{
				TimerPeriodCount = 0;

				ADC_Start_DMA(&ADC1Handle, &AdcValue, 1);
			}
			else
			{
				TimerPeriodCount++;
			}

			// 2. Detect the Condition for Stimulator to operate
			Stimulator_DetectCondition();
		}
	}
}


void USART_ApplicationEventCallback(UART_HandleTypeDef *pUSARTHandle, uint8_t AppEV)
{
	if(pUSARTHandle->Instance == USART1)
	{
		if(AppEV == USART_EVENT_TX_CMPLT)
		{
			asm("NOP");
		}
	}
}


void ADC_ConvCpltCallback(ADC_HandleTypeDef* pADCHandle)
{
	/* A/D Conversion is completed every 200us */

	double FilterOutput;

	// 1. Filter Input data with Notch Filter
	FilterOutput = NotchFilter();

	// 2. Do tasks according to State
	switch(State)
	{
		/*************************************************************************/
		/* 								Time domain								 */
		/*************************************************************************/

		case STATE_TIME_DOMAIN :
		{
			if(DownSamplingCount >= DownSamplingCountMax)
			{
				if(AdcCmpltFlag == 0)
				{
					TimeDomainSample[SampleIdx] = AdcValue;
					TimeDomainSampleFltrd[SampleIdx] = (uint32_t)( (FilterOutput + 1.65) * 4095. / 3.3 );

					if(SampleIdx >= 249)
					{
						SampleIdx = 0;
						AdcCmpltFlag = 1;
					}
					else
					{
						SampleIdx++;
					}
				}

				DownSamplingCount = 0;
			}
			else
			{
				DownSamplingCount++;
			}

			break;
		}


		/*************************************************************************/
		/* 							 Time domain split							 */
		/*************************************************************************/

		case STATE_TIME_DOMAIN_SPLIT :
		{
			if(DownSamplingCount >= DownSamplingCountMax)
			{
				if(AdcCmpltFlag == 0)
				{
					TimeDomainSample[SampleIdx] = AdcValue;
					TimeDomainSampleFltrd[SampleIdx] = (uint32_t)( (FilterOutput + 1.65) * 4095. / 3.3 );

					if(SampleIdx >= 249)
					{
						SampleIdx = 0;
						AdcCmpltFlag = 1;
					}
					else
					{
						SampleIdx++;
					}
				}

				DownSamplingCount = 0;
			}
			else
			{
				DownSamplingCount++;
			}

			break;
		}

		/*************************************************************************/
		/* 							  Frequency domain							 */
		/*************************************************************************/

		case STATE_FREQ_DOMAIN :
		{
			if(DownSamplingCount >= DownSamplingCountMax)
			{
				if(AdcCmpltFlag == 0)
				{
					if(FilterState == DISABLE)
					{
						FFT_Buffer[SampleIdx].real = (double)AdcValue * 3.3 / 4095. - 1.65;
						FFT_Buffer[SampleIdx].imag = 0;
					}
					else if(FilterState == ENABLE)
					{
						FFT_Buffer[SampleIdx].real = FilterOutput;
						FFT_Buffer[SampleIdx].imag = 0;
					}

					if(SampleIdx >= N-1)
					{
						SampleIdx = 0;
						AdcCmpltFlag = 1;
					}
					else
					{
						SampleIdx++;
					}
				}

				DownSamplingCount = 0;
			}
			else
			{
				DownSamplingCount++;
			}

			break;
		}


		/*************************************************************************/
		/* 								ETC TEST								 */
		/*************************************************************************/

		case STATE_ETC_TEST :
		{

			break;
		}

		default :

			break;
	}

}


void DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* pDACHandle)
{
	if(pDACHandle->Instance == DAC1)
	{
		asm("NOP");
	}
}


void EXTI_Callback(uint32_t ExtiSrcPin)
{
	if(ExtiSrcPin == GPIO_PIN_0)
	{
		NVIC->ICPR[0] = (uint32_t)(1UL << 6);
		KeyFlag_Mode = FLAG_SET;
	}

	else if(ExtiSrcPin == GPIO_PIN_1)
	{
		NVIC->ICPR[0] = (uint32_t)(1UL << 7);
		KeyFlag_Up = FLAG_SET;
	}

	else if(ExtiSrcPin == GPIO_PIN_2)
	{
		NVIC->ICPR[0] = (uint32_t)(1UL << 8);
		KeyFlag_Down = FLAG_SET;
	}

	else if(ExtiSrcPin == GPIO_PIN_3)
	{
		NVIC->ICPR[0] = (uint32_t)(1UL << 9);
		KeyFlag_Set = FLAG_SET;
	}
}

