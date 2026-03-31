// stm32f7_fft.c 

#include <math.h>
#include "stm32f7_display.h"

#define SOURCE_FILE_NAME "stm32f7_fft.c"

#define PI 3.14159265358979
// choose N from 32, 64, 128, 256
#define N 128
#define TESTFREQ 1800.0
#define SAMPLING_FREQ 8000.0

typedef struct
{
  float real;
  float imag;
} COMPLEX;

#include "fft.h"

COMPLEX samples[N];
COMPLEX twiddle[N];


int main()
{    
  int n;
	
  stm32f7_LCD_init(SAMPLING_FREQ, SOURCE_FILE_NAME, GRAPH);
	
  for (n=0 ; n< N ; n++)
  {
    twiddle[n].real = cos(PI*n/N);
    twiddle[n].imag = -sin(PI*n/N);
  }	

  for(n=0 ; n<N ; n++)
  {
  samples[n].real = cos(2*PI*TESTFREQ*n/SAMPLING_FREQ);
  samples[n].imag = 0.0;
  }
  BSP_LED_On(LED1);
  fft(samples,N,twiddle);
  BSP_LED_Off(LED1);
// if N is equal to 256, pass N/2 in place of N to function plotWave()
  plotWave(&samples->real, N, 0, 1); 
  while(1){}
}	


