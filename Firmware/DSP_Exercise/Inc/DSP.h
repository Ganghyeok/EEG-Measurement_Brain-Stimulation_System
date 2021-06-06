/*
 * DSP.h
 *
 *  Created on: 2021. 3. 16.
 *      Author: Ganghyeok Lim
 */

#ifndef DSP_H_
#define DSP_H_

#include "main.h"



/* Constants */
#define pi		3.14159265358979f


typedef struct
{
	double real;
	double imag;
} Complex;


/* DSP APIs */
Complex CplxAdd(Complex a, Complex b);
Complex CplxSub(Complex a, Complex b);
Complex CplxMul(Complex a, Complex b);
void Permute(Complex* seq);
void GetTwiddle_FFT(int stage);
void GetTwiddle_IFFT(int stage);
void FFT(Complex* seq);
void IFFT(Complex* seq);
void Filter_Init(void);
void HSBOlpSav(void);




#endif /* DSP_H_ */
