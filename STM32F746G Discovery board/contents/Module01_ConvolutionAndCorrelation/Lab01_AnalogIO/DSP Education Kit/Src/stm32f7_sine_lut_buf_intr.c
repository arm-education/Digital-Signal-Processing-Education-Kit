// stm32f7_sine_lut_buf_intr.c

#include "stm32f7_wm8994_init.h"
#include "stm32f7_display.h"

#define SOURCE_FILE_NAME "stm32f7_sine_lut_buf_intr.c"
#define LOOPLENGTH 8
#define BUFFER_LENGTH 100

extern int16_t rx_sample_L;
extern int16_t rx_sample_R;
extern int16_t tx_sample_L;
extern int16_t tx_sample_R;

int16_t sine_table[LOOPLENGTH] = {0, 7071, 10000, 7071, 0, -7071, -10000, -7071};
int16_t sine_ptr = 0;  // pointer into lookup table
float32_t buffer[BUFFER_LENGTH];
int16_t buf_ptr = 0;   // pointer into buffer

void BSP_AUDIO_SAI_Interrupt_CallBack()
{
// when we arrive at this interrupt service routine (callback)
// the most recent input sample values are (already) in global variables
// rx_sample_L and rx_sample_R
// this routine should write new output sample values in
// global variables tx_sample_L and tx_sample_R

  tx_sample_L = sine_table[sine_ptr];
  tx_sample_R = tx_sample_L;
  buffer[buf_ptr] = tx_sample_L;
	sine_ptr = (sine_ptr+1)%LOOPLENGTH;
	buf_ptr = (buf_ptr+1)%BUFFER_LENGTH;
	
  BSP_LED_Toggle(LED1);

  return;
}

int main(void)
{  
  stm32f7_wm8994_init(AUDIO_FREQUENCY_8K,
                      IO_METHOD_INTR,
                      INPUT_DEVICE_DIGITAL_MICROPHONE_2,
                      OUTPUT_DEVICE_HEADPHONE,
                      WM8994_HP_OUT_ANALOG_GAIN_6DB,
                      WM8994_LINE_IN_GAIN_0DB,
                      WM8994_DMIC_GAIN_9DB,
                      SOURCE_FILE_NAME,
                      GRAPH);
	plotSamples(sine_table, LOOPLENGTH, 32);
  while(1){}
}
