// stm32f7_dft.c 

#include "stm32f7_display.h"

#define SOURCE_FILE_NAME "stm32f7_dft.c"

#define N 128
#define TESTFREQ 1800.0f
#define SAMPLING_FREQ 8000.0f

typedef struct
{
  float real;
  float imag;
} COMPLEX;

COMPLEX samples[N];

void dft(COMPLEX *x)
{
}

int main()
{    
  int n;
	
  for(n=0 ; n<N ; n++)
  {
    samples[n].real = cos(2*PI*TESTFREQ*n/SAMPLING_FREQ);
    samples[n].imag = 0.0f;	
  }
	
  stm32f7_LCD_init(SAMPLING_FREQ, SOURCE_FILE_NAME, GRAPH);
  BSP_LED_On(LED1);
  dft(samples);          //call DFT function
  BSP_LED_Off(LED1);
  plotWave(&samples->real, N, 0, 1);
  while(1){}
}	
