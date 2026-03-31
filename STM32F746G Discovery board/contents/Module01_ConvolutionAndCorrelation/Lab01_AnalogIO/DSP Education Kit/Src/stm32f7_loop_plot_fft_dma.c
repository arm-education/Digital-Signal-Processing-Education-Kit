// stm32f7_loop_dma.c
// ensure that PING_PONG_BUFFER_SIZE is 256 for this program (if plotting)
// #defined in stm32f7_wm8994_init.h 

#include "stm32f7_wm8994_init.h"

#define MAGNITUDE_SCALING_FACTOR 512
#define TRIGGER 12000
#define N (PING_PONG_BUFFER_SIZE)

extern volatile int32_t TX_buffer_empty; // these may not need to be int32_t
extern volatile int32_t RX_buffer_full; // they were extern volatile int16_t in F4 version
extern int16_t rx_buffer_proc, tx_buffer_proc; // will be assigned token values PING or PONG


float32_t cmplx_buf[2*N];
float32_t *cmplx_buf_ptr;
float32_t outbuffer[N];
int16_t inbuffer[N];


void process_buffer(void) // this function processes one DMA transfer block worth of data
{
  int i;
  int16_t *rx_buf, *tx_buf;
	
  if (rx_buffer_proc == PING) {rx_buf = (int16_t *)PING_IN;}
  else {rx_buf = (int16_t *)PONG_IN;}
  if (tx_buffer_proc == PING) {tx_buf = (int16_t *)PING_OUT;}
  else {tx_buf = (int16_t *)PONG_OUT;}
	
  for (i=0 ; i<(PING_PONG_BUFFER_SIZE) ; i++)
  {
    cmplx_buf[i*2] = (float32_t)(*rx_buf++);
    cmplx_buf[(i*2)+1] = 0.0;
		*rx_buf++;
		inbuffer[i] = (int16_t)cmplx_buf[i*2]/4;
  }
  cmplx_buf_ptr = cmplx_buf;
	arm_cfft_f32(&arm_cfft_sR_f32_len256,(float32_t *)(cmplx_buf), 0, 1);
	arm_cmplx_mag_f32((float32_t *)(cmplx_buf), outbuffer,N);
//	outbuffer[0] = (float32_t)(TRIGGER*MAGNITUDE_SCALING_FACTOR);
	
	plotFFT(outbuffer, PING_PONG_BUFFER_SIZE, 1);
	for (i=0 ; i<(PING_PONG_BUFFER_SIZE) ; i++)
  {
//    *tx_buf++ = (short)(outbuffer[i]/MAGNITUDE_SCALING_FACTOR);
    *tx_buf++ = (short)(inbuffer[i]);
    *tx_buf++ = (short)(inbuffer[i]);
	} 

  RX_buffer_full = 0;
  TX_buffer_empty = 0;
}

int main(void)
{  
  
  stm32f7_wm8994_init(AUDIO_FREQUENCY_8K,
                      IO_METHOD_DMA,
                      INPUT_DEVICE_DIGITAL_MICROPHONE_2,
                      OUTPUT_DEVICE_HEADPHONE,
                      WM8994_HP_OUT_ANALOG_GAIN_0DB,
                      WM8994_LINE_IN_GAIN_0DB,
                      WM8994_DMIC_GAIN_17DB);
 
  while(1)
  {
    while(!(RX_buffer_full && TX_buffer_empty)){}
    BSP_LED_On(LED1);
    process_buffer();
    BSP_LED_Off(LED1);
  }
}

