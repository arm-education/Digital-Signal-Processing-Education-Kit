// stm32f7_dft.c 

#include "stm32f7_display.h"
#include <math.h> 
#include "audio.h"

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
	COMPLEX result[N];
    int k, n;
    
    for (k = 0; k < N; k++)
    {
        result[k].real = 0.0;
        result[k].imag = 0.0;
        
        for (n = 0; n < N; n++)
        {
            result[k].real += x[n].real * cos(2 * PI * k * n / N)
                            + x[n].imag * sin(2 * PI * k * n / N);
            result[k].imag += x[n].imag * cos(2 * PI * k * n / N)
                            - x[n].real * sin(2 * PI * k * n / N);
        }
    }
    
    for (k = 0; k < N; k++)
    {
        x[k] = result[k];
    }
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
