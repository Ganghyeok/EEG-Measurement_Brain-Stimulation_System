/*
 * DSP.c
 *
 *  Created on: 2021. 3. 16.
 *      Author: Ganghyeok Lim
 */

#include "DSP.h"


/* About FFT, IFFT */
uint16_t		NumStage;		// Number of Stage in N-point FFT
Complex			w[N / 2];		// Twiddle Factor

/* About Filtering (High-Speed Block Overlap-Saving method) */
Complex			h[N] = { 0, };
double			InputBuffer[L];
double			OutputBuffer[L];
Complex			Block[N] = { 0, };
double			OldBlockEnd[MF - 1] = { 0, };
Complex			H[N];
Complex			Xk[N], Yk[N];
Complex			yk[N];
uint8_t			InFullFlag = 0;
uint8_t			OutFullFlag = 0;


/* ================================================================================================================================ */

Complex CplxAdd(Complex a, Complex b)
{
	Complex c;

	c.real = a.real + b.real;
	c.imag = a.imag + b.imag;

	return c;
}


Complex CplxSub(Complex a, Complex b)
{
	Complex c;

	c.real = a.real - b.real;
	c.imag = a.imag - b.imag;

	return c;
}


Complex CplxMul(Complex a, Complex b)
{
	Complex c;

	c.real = a.real * b.real - a.imag * b.imag;
	c.imag = a.real * b.imag + a.imag * b.real;

	return c;
}


void Permute(Complex *seq)
{
	Complex temp;
	int m = 0;
	int i = 1;

	for (int n = 1; n <= N; n++)
	{
		if (n > i)
		{
			temp = seq[n - 1];
			seq[n - 1] = seq[i - 1];
			seq[i - 1] = temp;
		}

		m = N / 2;

		while ((m >= 2) && (i > m))
		{
			i = i - m;
			m = m / 2;
		}

		i = i + m;
	}
}


void GetTwiddle_FFT(int stage)
{
	uint16_t n = pow(2, stage);

	// Caculate the Twiddle factor of each stage by using Euler's formula
	for (uint16_t k = 0; k < (n / 2); k++)
	{
		w[k].real = cos((-2) * pi * k / n);
		w[k].imag = sin((-2) * pi * k / n);
	}
}


void GetTwiddle_IFFT(int stage)
{
	uint16_t n = pow(2, stage);

	// Caculate the Twiddle factor of each stage by using Euler's formula
	for (uint16_t k = 0; k < (n / 2); k++)
	{
		w[k].real = cos((+2) * pi * k / n);
		w[k].imag = sin((+2) * pi * k / n);
	}
}


void FFT(Complex *seq)
{
	int NumGroup, i, j, M;
	Complex temp;

	// 1. Find the # of Stage in N-point FFT
	NumStage = (uint16_t)log2(N);

	// 2. Permute the input signal prior to calculation of FFT algorithm
	Permute(seq);

	// 3. Execute FFT algorithm
	for (int stage = 1; stage <= NumStage; stage++)
	{
		GetTwiddle_FFT(stage);
		NumGroup = N / (int)pow(2, stage);
		M = (int)pow(2, (stage - 1));

		for (int g = 0; g <= (NumGroup - 1); g++)
		{
			for (int k = 0; k < M; k++)
			{
				i = (int)pow(2, stage) * g + k;
				j = i + M;
				temp = CplxMul(w[k], seq[j]);
				seq[j] = CplxSub(seq[i], temp);
				seq[i] = CplxAdd(seq[i], temp);
			}
		}
	}
}


void IFFT(Complex *seq)
{
	int NumGroup, i, j, M;
	Complex temp;

	// 1. Permute the FFT result prior to calculation of IFFT algorithm
	Permute(seq);

	// 2. Divide N to each of FFT
	for (int i = 0; i < N; i++)
	{
		seq[i].real = seq[i].real / N;
		seq[i].imag = seq[i].imag / N;
	}

	// 3. Execute IFFT algorithm
	for (int stage = 1; stage <= NumStage; stage++)
	{
		GetTwiddle_IFFT(stage);
		NumGroup = N / (int)pow(2, stage);
		M = (int)pow(2, (stage - 1));

		for (int g = 0; g <= (NumGroup - 1); g++)
		{
			for (int k = 0; k < M; k++)
			{
				i = (int)pow(2, stage) * g + k;
				j = i + M;
				temp = CplxMul(w[k], seq[j]);
				seq[j] = CplxSub(seq[i], temp);
				seq[i] = CplxAdd(seq[i], temp);
			}
		}
	}
}


void Filter_Init(void)
{
	/* Filter 1 */
//	double K = 1.06;
//
//	h[0].real = K*0.5;
//	h[1].real = K*0.25;
//	h[2].real = K*0.125;
//	h[3].real = K*0.0625;

	/* Filter 2 */
	h[0].real = 0.000199512328641;
	h[1].real = -0.002708453461401;
	h[2].real = -0.002400461099957;
	h[3].real = 0.003546543555809;
	h[4].real = 0.008266607456720;
	h[5].real = 0.000012109690648;
	h[6].real = -0.015608300819736;
	h[7].real = -0.012905580320708;
	h[8].real = 0.017047710292001;
	h[9].real = 0.036435951059014;
	h[10].real = 0.000019292305776;
	h[11].real = -0.065652005307521;
	h[12].real = -0.057621325403582;
	h[13].real = 0.090301607282890;
	h[14].real = 0.300096964940136;
	h[15].real = 0.400022084144842;
	h[16].real = 0.300096964940136;
	h[17].real = 0.090301607282890;
	h[18].real = -0.057621325403582;
	h[19].real = -0.065652005307521;
	h[20].real = 0.000019292305776;
	h[21].real = 0.036435951059014;
	h[22].real = 0.017047710292001;
	h[23].real = -0.012905580320708;
	h[24].real = -0.015608300819736;
	h[25].real = 0.000012109690648;
	h[26].real = 0.008266607456720;
	h[27].real = 0.003546543555809;
	h[28].real = -0.002400461099957;
	h[29].real = -0.002708453461401;
	h[30].real = 0.000199512328641;



	// 2. Execute N-point FFT of h[N]
	FFT(h);		// In here, h[N] is equal to H[N], Becuase of In-place Execution
}


void HSBOlpSav(void)
{
	// 1. Fill Block[N] with OldBlockEnd[M-1] and InputBuffer[L]
	for (int i = 0; i < MF - 1; i++)
	{
		Block[i].real = OldBlockEnd[i];				// Samples of OldBlockEnd are all real value
	}

	for (int i = 0; i < L; i++)
	{
		Block[i + MF - 1].real = InputBuffer[i];	// Samples of InputBuffer are all real value
	}

	// 2. Transfer Last (MF-1) Samples of Block[N] to OldBlockEnd[M-1] for Next HSBOlpSav function call
	for (int i = 0; i < MF - 1; i++)
	{
		OldBlockEnd[i] = Block[N - MF + 1 + i].real;
	}

	// 3. Execute N-point FFT of Block[N]
	FFT(Block);		// In here, Block[N] is equal to Xk[N], Becuase of In-place Execution

	// 4. Do Term-wise Multiplication of Xk[N] and H[N]
	for (int i = 0; i < N; i++)
	{
		yk[i] = CplxMul(Block[i], h[i]);		// In here, "h[n] is equal to H[N]" and "yk[N] is equal to Yk[N] becuase of Next In-place Execution"
	}

	// 5. Execute N-point IFFT of Yk[N]
	IFFT(yk);

	// 6. Throw away First (M-1) Samples and Take Last L samples of yk[N], Then Transfer it to OutputBuffer[L]
	for (int i = 0; i < L; i++)
	{
		OutputBuffer[i] = yk[N - L + i].real;
	}
}




