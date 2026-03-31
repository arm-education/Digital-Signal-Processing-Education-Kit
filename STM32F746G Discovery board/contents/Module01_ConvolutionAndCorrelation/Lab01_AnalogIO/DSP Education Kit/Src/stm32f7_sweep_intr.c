// stm32f7_sweep_intr.c

#include "stm32f7_wm8994_init.h"
#include "stm32f7_display.h"

#include "sine8000_table.h"  // one cycle with 8000 points

#define SOURCE_FILE_NAME "stm32f7_sweep_intr.c"

#define SAMPLING_FREQ 8000.0
#define N 8000
#define START_FREQ 500.0
#define STOP_FREQ 3800.0
#define START_INCR START_FREQ*N/SAMPLING_FREQ
#define STOP_INCR STOP_FREQ*N/SAMPLING_FREQ
#define SWEEPTIME 4
#define DELTA_INCR (STOP_INCR - START_INCR)/(N*SWEEPTIME)

int16_t amplitude = 10;
float32_t float_index = 0.0;
float32_t float_incr = START_INCR;
int16_t i;

extern int16_t rx_sample_L;
extern int16_t rx_sample_R;
extern int16_t tx_sample_L;
extern int16_t tx_sample_R;

void BSP_AUDIO_SAI_Interrupt_CallBack()
{
// when we arrive at this interrupt service routine (callback)
// the most recent input sample values are (already) in global variables
// rx_sample_L and rx_sample_R
// this routine should write new output sample values in
// global variables tx_sample_L and tx_sample_R

  float_incr += DELTA_INCR;
  if (float_incr > STOP_INCR) float_incr = START_INCR;
  float_index += float_incr;
  if (float_index > N) float_index -= N;
  i = (int16_t)(float_index);
  tx_sample_L = (amplitude*sine8000[i]);
  tx_sample_R = tx_sample_L;

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
                      NOGRAPH);
  
  while(1){}
}
