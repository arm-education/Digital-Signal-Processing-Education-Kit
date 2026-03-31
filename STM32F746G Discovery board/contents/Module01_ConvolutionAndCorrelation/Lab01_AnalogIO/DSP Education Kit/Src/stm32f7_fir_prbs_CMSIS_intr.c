// stm32f7_fir_prbs_CMSIS_intr.c

#include "stm32f7_wm8994_init.h"
#include "stm32f7_display.h"
#include "bp1750.h"

#define SOURCE_FILE_NAME "stm32f7_fir_prbs_CMSIS_intr.c"

extern int16_t rx_sample_L;
extern int16_t rx_sample_R;
extern int16_t tx_sample_L;
extern int16_t tx_sample_R;

float32_t x, y, state[N];
arm_fir_instance_f32 S;

void BSP_AUDIO_SAI_Interrupt_CallBack()
{
// when we arrive at this interrupt service routine (callback)
// the most recent input sample values are (already) in global variables
// rx_sample_L and rx_sample_R
// this routine should write new output sample values in
// global variables tx_sample_L and tx_sample_R

  x = (float32_t)(prbs(8000));
  BSP_LED_On(LED1);
  arm_fir_f32(&S,&x,&y,1);
  BSP_LED_Off(LED1);
  tx_sample_L = (int16_t)(y);
  tx_sample_R = tx_sample_L;

  return;
}

int main(void)
{  

  arm_fir_init_f32(&S, N, h, state, 1); 

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
