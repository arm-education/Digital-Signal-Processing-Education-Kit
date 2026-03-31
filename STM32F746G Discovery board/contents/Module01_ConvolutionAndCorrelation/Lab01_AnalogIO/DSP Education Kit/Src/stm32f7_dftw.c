// stm32f7_dftw.c 

#include "stm32f7_display.h"

#define SOURCE_FILE_NAME "stm32f7_dftw.c "

#define N 128
#define TESTFREQ 1800.0
#define SAMPLING_FREQ 8000.0

typedef struct
{
  float real;
  float imag;
} COMPLEX;

COMPLEX samples[N];
COMPLEX twiddle[N];

void dftw(COMPLEX *x, COMPLEX *w)
{
}

int main()
{    
  int n;

  stm32f7_LCD_init(SAMPLING_FREQ, SOURCE_FILE_NAME, GRAPH);
	
  for(n=0 ; n<N ; n++)
  {
    samples[n].real = cos(2*PI*TESTFREQ*n/SAMPLING_FREQ);
    samples[n].imag = 0.0;
  }
	
  BSP_LED_On(LED1);
  dftw(samples,twiddle);
  BSP_LED_Off(LED1);
  plotWave(&samples->real, N/2, 0, 1);
  while(1){}
}	


