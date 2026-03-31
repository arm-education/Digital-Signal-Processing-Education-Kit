// stm32f7_dimpulse_intr.c

#include "stm32f7_wm8994_init.h"
#include "stm32f7_display.h"

#define SOURCE_FILE_NAME "stm32f7_dimpulse_intr.c"
#define IMPULSE_PERIOD 256
#define IMPULSE_AMPLITUDE 20000

extern int16_t rx_sample_L;
extern int16_t rx_sample_R;
extern int16_t tx_sample_L;
extern int16_t tx_sample_R;

int impulse_count = 0;

void BSP_AUDIO_SAI_Interrupt_CallBack()
{
// when we arrive at this interrupt service routine (callback)
// the most recent input sample values are (already) in global variables
// rx_sample_L and rx_sample_R
// this routine should write new output sample values in
// global variables tx_sample_L and tx_sample_R

  if (impulse_count == 0)
    {
      tx_sample_L = IMPULSE_AMPLITUDE;
    }
    else
    {  
      tx_sample_L = 0;
    }
  impulse_count = (impulse_count + 1) % IMPULSE_PERIOD;
  tx_sample_R = tx_sample_L;
  plotSamplesIntr(tx_sample_L, 256);
  BSP_LED_Toggle(LED1);
  return;
}

int main(void)
{  
  stm32f7_wm8994_init(AUDIO_FREQUENCY_8K,
                      IO_METHOD_INTR,
                      INPUT_DEVICE_DIGITAL_MICROPHONE_2,
                      OUTPUT_DEVICE_HEADPHONE,
                      WM8994_HP_OUT_ANALOG_GAIN_0DB,
                      WM8994_LINE_IN_GAIN_0DB,
                      WM8994_DMIC_GAIN_9DB,
                      SOURCE_FILE_NAME,
                      GRAPH);
  
  while(1){}
}
