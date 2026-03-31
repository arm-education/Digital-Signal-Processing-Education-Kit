// stm32f7_delay_intr.c

#include "stm32f7_wm8994_init.h"
#include "stm32f7_display.h"

#define SOURCE_FILE_NAME "stm32f7_delay_intr.c"
#define DELAY_BUF_SIZE 24000

extern int16_t rx_sample_L;
extern int16_t rx_sample_R;
extern int16_t tx_sample_L;
extern int16_t tx_sample_R;

int16_t buffer[DELAY_BUF_SIZE];
int16_t bufptr = 0;

void BSP_AUDIO_SAI_Interrupt_CallBack()
{
// when we arrive at this interrupt service routine (callback)
// the most recent input sample values are (already) in global variables
// rx_sample_L and rx_sample_R
// this routine should write new output sample values in
// global variables tx_sample_L and tx_sample_R

  int16_t delayed_sample;

  delayed_sample = buffer[bufptr];
  tx_sample_L = delayed_sample + rx_sample_L;
  buffer[bufptr] = rx_sample_L;
  bufptr = (bufptr+1) % DELAY_BUF_SIZE;
  tx_sample_R = tx_sample_L;
  BSP_LED_Toggle(LED1);
  return;
}

int main(void)
{  
  stm32f7_wm8994_init(AUDIO_FREQUENCY_48K,
                      IO_METHOD_INTR,
                      INPUT_DEVICE_DIGITAL_MICROPHONE_2,
                      OUTPUT_DEVICE_HEADPHONE,
                      WM8994_HP_OUT_ANALOG_GAIN_6DB,
                      WM8994_LINE_IN_GAIN_0DB,
                      WM8994_DMIC_GAIN_17DB,
                      SOURCE_FILE_NAME,
	              NOGRAPH);
  while(1){}
}
