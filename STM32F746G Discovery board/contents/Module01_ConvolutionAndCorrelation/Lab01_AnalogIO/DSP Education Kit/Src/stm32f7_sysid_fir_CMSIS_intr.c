// stm32f7_sysid_fir_CMSIS_intr.c
// uses normalised LMS

#include "stm32f7_wm8994_init.h"
#include "display.h"
#include "bp1750.h"

#define BLOCK_SIZE 1
#define NUM_TAPS 256

#define SOURCE_FILE_NAME "stm32f7_sysid_fir_CMSIS_intr.c"

float32_t beta = 1E-2; // using normalised LMS !
float32_t firStateF32[BLOCK_SIZE + NUM_TAPS - 1];
float32_t firCoeffs32[NUM_TAPS] = { 0.0f };
arm_lms_norm_instance_f32 S;
arm_fir_instance_f32 T;
float32_t state[N];
float32_t yn, adapt_in, adapt_out, adapt_err,input;
uint32_t status;

extern int16_t rx_sample_L;
extern int16_t rx_sample_R;
extern int16_t tx_sample_L;
extern int16_t tx_sample_R;

float32_t cmplx_buf[2*NUM_TAPS];
float32_t *cmplx_buf_ptr;
float32_t outbuffer[NUM_TAPS];
volatile int intr_flag = 0;

void BSP_AUDIO_SAI_Interrupt_CallBack()
{
// when we arrive at this interrupt service routine (callback)
// the most recent input sample values are (already) in global variables
// rx_sample_L and rx_sample_R
// this routine should write new output sample values in
// global variables tx_sample_L and tx_sample_R
	
  input = (float32_t)(rx_sample_L);	
  adapt_in = (float32_t)(prbs(8000));
  arm_fir_f32(&T,&adapt_in,&yn,1);
  tx_sample_L = (int16_t)(yn);
  tx_sample_R = (int16_t)(yn);
  arm_lms_norm_f32(&S, &adapt_in, &input, &adapt_out, &adapt_err, BLOCK_SIZE);	
  intr_flag = 1;
  return;
}


int main(void)
{  
	int i = 0;
	int button = 0;
  arm_lms_norm_init_f32(&S, NUM_TAPS, (float32_t *)&firCoeffs32[0], &firStateF32[0], beta, BLOCK_SIZE);
  arm_fir_init_f32(&T, N, h, state, 1); 
 
  stm32f7_wm8994_init(AUDIO_FREQUENCY_8K,
                      IO_METHOD_INTR,
                      INPUT_DEVICE_INPUT_LINE_1,
                      OUTPUT_DEVICE_HEADPHONE,
                      WM8994_HP_OUT_ANALOG_GAIN_0DB,
                      WM8994_LINE_IN_GAIN_0DB,
                      WM8994_DMIC_GAIN_0DB,
                      SOURCE_FILE_NAME,
                      GRAPH);
	
 // the following two registers control the equaliser function
 //     AUDIO_IO_Write(AUDIO_I2C_ADDRESS, 0x480, 0x6319);
 //     AUDIO_IO_Write(AUDIO_I2C_ADDRESS, 0x481, 0x0300);
 // the following register controls the HPF function
 //     AUDIO_IO_Write(AUDIO_I2C_ADDRESS, 0x410, 0x3800); 
 // 0x410 for LINE IN 0x411 for DMIC2 1800, 3800, 7800
 // the following register controls the de-emphasis function
 //     AUDIO_IO_Write(AUDIO_I2C_ADDRESS, 0x420, 0x0006);
 	
  while(1)
  {
    button = checkButtonFlag();
    if (button == 1)
    {
      for (i=0; i<NUM_TAPS; i++)
      {
        cmplx_buf[2*i] = firCoeffs32[i];
        cmplx_buf[2*i + 1] = 0.0;
      }
      arm_cfft_f32(&arm_cfft_sR_f32_len256, (float32_t *)(cmplx_buf), 0, 1);
      arm_cmplx_mag_f32((float32_t *)(cmplx_buf),(float32_t *)(outbuffer), NUM_TAPS);
      plotLogFFT(outbuffer, NUM_TAPS, LIVE);
    } 
    else if (button == 0)
    {
      plotLMS(firCoeffs32, NUM_TAPS, LIVE);
    }
  }
}


