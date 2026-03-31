// stm32f7_iirsosadapt_intr.c
// uses normalised LMS

#include "stm32f7_wm8994_init.h"
#include "stm32f7_display.h"
#include "elliptic_bandpass.h"

#define BLOCK_SIZE 1
#define NUM_TAPS 256

#define SOURCE_FILE_NAME "stm32f7_iirsosadapt_intr.c"

float32_t beta = 1E-3; // using normalised LMS !
float w[NUM_SECTIONS][2] = {0.0f, 0.0f};
float32_t x[NUM_TAPS];
float32_t h[NUM_TAPS];

extern int16_t rx_sample_L;
extern int16_t rx_sample_R;
extern int16_t tx_sample_L;
extern int16_t tx_sample_R;

float32_t cmplx_buf[2*PING_PONG_BUFFER_SIZE];
float32_t *cmplx_buf_ptr;
float32_t outbuffer[PING_PONG_BUFFER_SIZE];
volatile int intr_flag = 0;

void BSP_AUDIO_SAI_Interrupt_CallBack()
{
  float32_t input;
  int16_t section, i;
  float32_t  wn, yn, adapt_out, error, adapt_in, dummy, energy;

  BSP_LED_On(LED1);
  input = (float32_t)(prbs(8000));
  adapt_in = input;
  for (section=0 ; section<NUM_SECTIONS ; section++)
  {
    wn = input - a[section][1]*w[section][0]
               - a[section][2]*w[section][1];
    yn = b[section][0]*wn + b[section][1]*w[section][0]
                          + b[section][2]*w[section][1];
    w[section][1] = w[section][0];
    w[section][0] = wn;
    input = yn;
  }
  x[0] = adapt_in;
  adapt_out = 0.0;
		
  energy = 0.0;
  for (i=0;i<NUM_TAPS; i++)
  {
    adapt_out += (h[i]*x[i]);
    energy+= x[i]*x[i];
  }
  error = yn - adapt_out;
  for (i = NUM_TAPS-1; i >= 0; i--)      // update weights
  {                               // and delay line
    dummy = beta*error;
    dummy = dummy*x[i];
    h[i] = h[i] + dummy/energy;
    x[i] = x[i-1];
  } 
  BSP_LED_Off(LED1);
  tx_sample_L = (int16_t)(error);
  tx_sample_R = (int16_t)(error);
  return;
}


int main(void)
{  
  int i = 0;
  int button = 0;
 
  stm32f7_wm8994_init(AUDIO_FREQUENCY_8K,
                      IO_METHOD_INTR,
                      INPUT_DEVICE_INPUT_LINE_1,
                      OUTPUT_DEVICE_HEADPHONE,
                      WM8994_HP_OUT_ANALOG_GAIN_6DB,
                      WM8994_LINE_IN_GAIN_0DB,
                      WM8994_DMIC_GAIN_0DB,
                      SOURCE_FILE_NAME,
	              GRAPH);
  while(1)
  {
    button = checkButtonFlag();
    if (button == 1) 
    {
      for(i=0; i<NUM_TAPS; i++)
      {
        cmplx_buf[2*i] = h[i];
        cmplx_buf[2*i + 1] = 0.0;
      }
      arm_cfft_f32(&arm_cfft_sR_f32_len256, (float32_t *)(cmplx_buf), 0, 1);
      arm_cmplx_mag_f32((float32_t *)(cmplx_buf),(float32_t *)(outbuffer), NUM_TAPS);
      plotLogFFT(outbuffer, NUM_TAPS, LIVE);
    }
    else if (button == 0)
    {
      plotLMS(h, NUM_TAPS, LIVE);
    }
  }
}


