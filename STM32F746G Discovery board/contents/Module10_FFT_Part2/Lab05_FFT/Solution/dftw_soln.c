// stm32f7_dftw.c 

#include "stm32f7_display.h"
#include <math.h> 
#include "audio.h"

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
	    COMPLEX result[N];
    int k, n;
    
    for (k = 0; k < N; k++)
    {
        result[k].real = 0.0;
        result[k].imag = 0.0;
        
        for (n = 0; n < N; n++)
        {
            result[k].real += x[n].real * w[(n * k) % N].real
                            + x[n].imag * w[(n * k) % N].imag;
            result[k].imag += x[n].imag * w[(n * k) % N].real
                            - x[n].real * w[(n * k) % N].imag;
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

  
	
	    for(n = 0; n < N; n++)
    {
        twiddle[n].real =  cos(2 * PI * n / N);
        twiddle[n].imag = -sin(2 * PI * n / N);
    }
	
  for(n=0 ; n<N ; n++)
  {
    samples[n].real = cos(2*PI*TESTFREQ*n/SAMPLING_FREQ);
    samples[n].imag = 0.0;
  }
	
	stm32f7_LCD_init(SAMPLING_FREQ, SOURCE_FILE_NAME, GRAPH);
	
  BSP_LED_On(LED1);
  dftw(samples,twiddle);
  BSP_LED_Off(LED1);
  plotWave(&samples->real, N/2, 0, 1);
  while(1){}
}	


