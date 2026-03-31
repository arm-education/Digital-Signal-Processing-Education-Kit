// stm32f7_impinviir_intr.c

#include "stm32f7_wm8994_init.h"
#include "stm32f7_display.h"

#define SOURCE_FILE_NAME "stm32f7_impinviir_intr.c"

extern int16_t rx_sample_L;
extern int16_t rx_sample_R;
extern int16_t tx_sample_L;
extern int16_t tx_sample_R;
float32_t xn1 = 0.0; // previous input and output sample values
float32_t yn1 = 0.0; // initialised to zero
float32_t yn2 = 0.0;

void BSP_AUDIO_SAI_Interrupt_CallBack()
{
  float32_t xn, yn;   // input and output values

  xn =(float32_t)(rx_sample_L);

  /***********************************************************************
   insert code to compute new output sample here, i.e.
   y(n) = 0.48255x(n-1) + 0.71624315y(n-1) - 0.38791310y(n-2)
   also update stored previous sample values, i.e.
   y(n-2), y(n-1), and x(n-1)
  ***********************************************************************/
      yn = 0.48255f * xn1 + 0.71624315f * yn1 - 0.3879131f * yn2;     
      yn2 = yn1;
      yn1 = yn;
      xn1 = xn;
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
