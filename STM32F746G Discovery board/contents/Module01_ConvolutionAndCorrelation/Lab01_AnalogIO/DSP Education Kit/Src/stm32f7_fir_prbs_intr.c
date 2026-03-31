// stm32f7_fir_prbs_intr.c

#include "stm32f7_wm8994_init.h"
#include "bp1750.h"
#include "stm32f7_display.h"

#define SOURCE_FILE_NAME "stm32f7_fir_prbs_intr.c"

float32_t x[N];

extern int16_t rx_sample_L;
extern int16_t rx_sample_R;
extern int16_t tx_sample_L;
extern int16_t tx_sample_R;

void BSP_AUDIO_SAI_Interrupt_CallBack(void)
{
  int16_t i;
  float32_t yn = 0.0;

  BSP_LED_On(LED1);
  x[0] = (float32_t)prbs(8000);
  for (i=0 ; i<N ; i++) yn += h[i]*x[i];
  for (i=(N-1) ; i>0 ; i--) x[i] = x[i-1];
  tx_sample_R = tx_sample_L = (int16_t)(yn);
  BSP_LED_Off(LED1);

  return;
}


int main(void)
{  
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

