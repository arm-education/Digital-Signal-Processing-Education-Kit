// stm32f7_fft_CMSIS.c 

#define ARM_MATH_CM4

#include "stm32f7_wm8994_init.h"
#include "stm32f7_display.h"

#define SOURCE_FILE_NAME "stm32f7_fft_CMSIS.c"

#define N 128
#define TESTFREQ 900.0
#define SAMPLING_FREQ 8000.0

float32_t samples[2*N];

int main()
{    
  int n;
	
  stm32f7_LCD_init(SAMPLING_FREQ, SOURCE_FILE_NAME, GRAPH);
	
  for(n=0 ; n<N ; n++)
  {
    samples[2*n] = arm_cos_f32(2*PI*TESTFREQ*n/SAMPLING_FREQ);
    samples[2*n+1] = 0.0;
  }

  plotWave(samples, N, 0, 1);
  BSP_LED_On(LED1);
  arm_cfft_f32(&arm_cfft_sR_f32_len128, samples, 0, 1);
  BSP_LED_Off(LED1);
  proceed_statement();
  while(1)
  {
    while(CheckForUserInput() != 1){}	
    plotWave(samples, N, 0, 1);
  }
}	
