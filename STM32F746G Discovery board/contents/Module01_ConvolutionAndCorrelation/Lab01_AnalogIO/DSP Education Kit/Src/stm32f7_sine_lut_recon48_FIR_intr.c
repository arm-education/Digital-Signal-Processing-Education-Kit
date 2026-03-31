// stm32f7_sine_lut_recon48_FIR_intr.c

#include "stm32f7_wm8994_init.h"
#include "recon484fird.h"
#include "stm32f7_display.h"

#define SOURCE_FILE_NAME "stm32f7_sine_lut_recon48_FIR_intr.c"

#define LOOPLENGTH 8
#define OVERSAMPLING_RATIO 6

float32_t x, y, state[N];
arm_fir_instance_f32 S;

extern int16_t rx_sample_L;
extern int16_t rx_sample_R;
extern int16_t tx_sample_L;
extern int16_t tx_sample_R;

int16_t sine_table[LOOPLENGTH] = {0, 7071, 10000, 7071, 0, -7071, -10000, -7071};
int16_t sine_ptr = 0;  // pointer into lookup table
static int count = 0;

extern DAC_TypeDef *Instance;

void BSP_AUDIO_SAI_Interrupt_CallBack()
{
// when we arrive at this interrupt service routine (callback)
// the most recent input sample values are (already) in global variables
// rx_sample_L and rx_sample_R
// this routine should write new output sample values in
// global variables tx_sample_L and tx_sample_R
  int16_t y_bit16;
  __IO uint32_t tmp = 0;

  count++;
  if (count >= OVERSAMPLING_RATIO)
  {
    count = 0;
    x = (float32_t)(sine_table[sine_ptr]);
    sine_ptr = (sine_ptr+1) % LOOPLENGTH;
  }
  else
  {
    x = 0.0;
  }
  arm_fir_f32(&S,&x,&y,1);
  y_bit16 = (int16_t)(y);

  plotSamplesIntr(y_bit16, 128);

  tx_sample_L = 0;
  tx_sample_R = tx_sample_L;

  tmp = (uint32_t)Instance; 
  tmp += 8;
// scaling of 16-bit 2's complement value intended to be written to
// 16-bit WM8994 codec should perhaps instead be ((y_bit16/16) + 2048)
// but for the sample values used in this example, ((y_bit16/5) + 2000)
// works okay. 
  *(__IO uint32_t *) tmp = (uint32_t)((y_bit16+10000)/5);
  Instance->SWTRIGR |= (uint32_t)DAC_SWTRIGR_SWTRIG1;

  return;
}

int main(void)
{  
  arm_fir_init_f32(&S, N, h, state, 1); 
  stm32f7_wm8994_init(AUDIO_FREQUENCY_48K,
                      IO_METHOD_INTR,
                      INPUT_DEVICE_INPUT_LINE_1,
                      OUTPUT_DEVICE_HEADPHONE,
                      WM8994_HP_OUT_ANALOG_GAIN_0DB,
                      WM8994_LINE_IN_GAIN_0DB,
                      WM8994_DMIC_GAIN_0DB,
                      SOURCE_FILE_NAME,
                      GRAPH);

  while(1){}
}
