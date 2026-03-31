// stm32f7_fft256_dma.c

#include "stm32f7_wm8994_init.h"
#include "stm32f7_display.h"
#include "hamming256.h"

#define SOURCE_FILE_NAME "stm32f7_fft256_dma.c"

extern volatile int32_t TX_buffer_empty; // these may not need to be int32_t
extern volatile int32_t RX_buffer_full; // they were extern volatile int16_t in F4 version
extern int16_t rx_buffer_proc, tx_buffer_proc; // will be assigned token values PING or PONG

#define N (PING_PONG_BUFFER_SIZE)
#define MAGNITUDE_SCALING_FACTOR 32
#define TRIGGER 32000

typedef struct
{
  float real;
  float imag;
} COMPLEX;

#include "fft.h"

COMPLEX twiddle[N];
COMPLEX cbuf[N];
int16_t sinebuf[N];
float32_t outbuffer[N];
float32_t inbuffer[N];

void process_buffer()
{
  int i;
  int16_t *rx_buf, *tx_buf;
  int16_t left_sample, right_sample;

  // determine which buffers to use
  if (rx_buffer_proc == PING) {rx_buf = (int16_t *)PING_IN;}
  else {rx_buf = (int16_t *)PONG_IN;}
  if (tx_buffer_proc == PING) {tx_buf = (int16_t *)PING_OUT;}
  else {tx_buf = (int16_t *)PONG_OUT;}
	
  for (i = 0; i < N ; i++) 
  {
    left_sample = *rx_buf++;
    right_sample = *rx_buf++;
    cbuf[i].real = ((float)left_sample);
//    cbuf[i].real = ((float)left_sample)*hamming[i];
    cbuf[i].imag = 0.0;
    inbuffer[i] = cbuf[i].real;
  } 

  fft(cbuf,N,twiddle);
  arm_cmplx_mag_f32((float32_t *)(cbuf), outbuffer,N);
  for (i = 0; i < N ; i++) 
  {
    outbuffer[i] = outbuffer[i]/MAGNITUDE_SCALING_FACTOR;
    if (i==0)
      *tx_buf++ = TRIGGER;
    else
      *tx_buf++ = (int16_t)(outbuffer[i]);
    *tx_buf++ = inbuffer[i];
  }

  TX_buffer_empty = 0;
  RX_buffer_full = 0;
}

int main(void)
{
  int n;
  for (n=0 ; n< N ; n++)
  {
    twiddle[n].real = cos(PI*n/N);
    twiddle[n].imag = -sin(PI*n/N);
  }	

  stm32f7_wm8994_init(AUDIO_FREQUENCY_8K,
                      IO_METHOD_DMA,
                      INPUT_DEVICE_INPUT_LINE_1,
                      OUTPUT_DEVICE_HEADPHONE,
                      WM8994_HP_OUT_ANALOG_GAIN_6DB,
                      WM8994_LINE_IN_GAIN_0DB,
                      WM8994_DMIC_GAIN_0DB,
                      SOURCE_FILE_NAME,
                      GRAPH);
  while(1)
  {
    while (!(RX_buffer_full && TX_buffer_empty)){}
    BSP_LED_On(LED1);
    process_buffer();
    plotFFT(outbuffer, PING_PONG_BUFFER_SIZE, NO_AUTO_SCALING);
    BSP_LED_Off(LED1);
  }
}
