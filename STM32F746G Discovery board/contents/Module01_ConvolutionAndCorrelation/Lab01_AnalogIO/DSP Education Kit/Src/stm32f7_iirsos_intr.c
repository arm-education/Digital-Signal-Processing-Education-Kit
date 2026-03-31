// stm32f7_iirsos_intr.c

#include "stm32f7_wm8994_init.h"
#include "stm32f7_display.h"
#include "elliptic.h"

#define SOURCE_FILE_NAME "stm32f7_iirsos_intr.c"

extern int16_t rx_sample_L;
extern int16_t rx_sample_R;
extern int16_t tx_sample_L;
extern int16_t tx_sample_R;

float w[NUM_SECTIONS][2] = {0};

void BSP_AUDIO_SAI_Interrupt_CallBack()
{
  int16_t section;    // second order section number
  float32_t input;    // input to each section
  float32_t wn, yn;   // intermediate and output values

  input =(float32_t)(rx_sample_L);
    for (section=0 ; section< NUM_SECTIONS ; section++)
    {
      wn = input - a[section][1]*w[section][0]
           - a[section][2]*w[section][1];
      yn = b[section][0]*wn + b[section][1]*w[section][0]
           + b[section][2]*w[section][1];
      w[section][1] = w[section][0];
      w[section][0] = wn;
      input = yn; 
    }
    tx_sample_L  = (int16_t)(yn);
    tx_sample_R = tx_sample_L;

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
                      WM8994_DMIC_GAIN_0DB,
                      SOURCE_FILE_NAME,
                      NOGRAPH);
  while(1){}
}
