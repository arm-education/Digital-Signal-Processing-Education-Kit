// stm32f7_average_intr.c

#include "stm32f7_wm8994_init.h"
#include "stm32f7_display.h"

#define SOURCE_FILE_NAME "stm32f7_average_intr.c"
#define N 5

extern int16_t rx_sample_L;
extern int16_t rx_sample_R;
extern int16_t tx_sample_L;
extern int16_t tx_sample_R;

float32_t h[N];
// float32_t h[N] = {0.08333, 0.25, 0.3333333, 0.25, 0.08333};
float32_t x[N];

void BSP_AUDIO_SAI_Interrupt_CallBack()
{
// when we arrive at this interrupt service routine (callback)
// the most recent input sample values are (already) in global variables
// rx_sample_L and rx_sample_R
// this routine should write new output sample values in
// global variables tx_sample_L and tx_sample_R
  int16_t i;
  float32_t yn = 0.0;

  x[0] = (float32_t)(rx_sample_L);
  for (i=0 ; i<N ; i++) yn += h[i]*x[i];
  for (i=(N-1) ; i>0 ; i--) x[i] = x[i-1];
  tx_sample_L = (int16_t)(yn);
  tx_sample_R = tx_sample_L;

  BSP_LED_Toggle(LED1);

  return;
}

int main(void)
{  
  int i;

  for (i=0 ; i<N ; i++) h[i] = 1.0/N;

  stm32f7_wm8994_init(AUDIO_FREQUENCY_8K,
                      IO_METHOD_INTR,
                      INPUT_DEVICE_INPUT_LINE_1,
                      OUTPUT_DEVICE_HEADPHONE,
                      WM8994_HP_OUT_ANALOG_GAIN_0DB,
                      WM8994_LINE_IN_GAIN_0DB,
                      WM8994_DMIC_GAIN_9DB,
                      SOURCE_FILE_NAME,
	              NOGRAPH);
  
  while(1){}
}
