// stm32f7_sine_intr.c

#include "stm32f7_wm8994_init.h"
#include "stm32f7_display.h"

#define SOURCE_FILE_NAME "stm32f7_sine_intr.c"
#define SAMPLING_FREQ 8000

extern int16_t rx_sample_L;
extern int16_t rx_sample_R;
extern int16_t tx_sample_L;
extern int16_t tx_sample_R;

float32_t sine_frequency = 367.0;
float32_t amplitude = 10000.0;
float32_t theta_increment;
float32_t theta = 0.0;

void BSP_AUDIO_SAI_Interrupt_CallBack()
{
// when we arrive at this interrupt service routine (callback)
// the most recent input sample values are (already) in global variables
// rx_sample_L and rx_sample_R
// this routine should write new output sample values in
// global variables tx_sample_L and tx_sample_R

  theta_increment = 2*PI*sine_frequency/SAMPLING_FREQ;
  theta += theta_increment;
  if (theta > 2*PI) theta -= 2*PI;
  BSP_LED_On(LED1);
//  tx_sample_L = (int16_t)(amplitude*sin(theta));
//  tx_sample_L = (int16_t)(amplitude*sinf(theta));
  tx_sample_L = (int16_t)(amplitude*arm_sin_f32(theta));
  tx_sample_R = tx_sample_L;
  BSP_LED_Off(LED1);
  plotSamplesIntr(tx_sample_L, 32);
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
                      GRAPH);
  
  while(1){}
}
