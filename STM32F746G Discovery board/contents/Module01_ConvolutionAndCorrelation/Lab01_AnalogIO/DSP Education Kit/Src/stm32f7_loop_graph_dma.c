// stm32f7_loop_graph_dma.c

#include "stm32f7_wm8994_init.h"
#include "stm32f7_display.h"

#define PLOTBUFSIZE 128

#define BLOCK_SIZE 1

#define SOURCE_FILE_NAME "stm32f7_loop_graph_dma.c"

extern volatile int32_t TX_buffer_empty; // these may not need to be int32_t
extern volatile int32_t RX_buffer_full; // they were extern volatile int16_t in F4 version
extern int16_t rx_buffer_proc, tx_buffer_proc; // will be assigned token values PING or PONG

float32_t x[PING_PONG_BUFFER_SIZE];

float32_t cmplx_buf[2*PING_PONG_BUFFER_SIZE];
float32_t outbuffer[PING_PONG_BUFFER_SIZE] = { 0.0f };

void process_buffer(void) // this function processes one DMA transfer block of data
{
  int i;
  int16_t *rx_buf, *tx_buf;
	
  if (rx_buffer_proc == PING) {rx_buf = (int16_t *)PING_IN;}
  else {rx_buf = (int16_t *)PONG_IN;}
  if (tx_buffer_proc == PING) {tx_buf = (int16_t *)PING_OUT;}
  else {tx_buf = (int16_t *)PONG_OUT;}
	
  for (i=0 ; i<(PING_PONG_BUFFER_SIZE) ; i++)
  {
    x[i] = (float32_t)(*rx_buf);
    *tx_buf++ = *rx_buf++;
    *tx_buf++ = *rx_buf++;
    cmplx_buf[i*2] = x[i]; // real part
    cmplx_buf[(i*2)+1] = 0.0; // imaginary part
  }

  RX_buffer_full = 0;
  TX_buffer_empty = 0;
}

int main(void)
{ 
	int i;
	int button = 0;
	
  stm32f7_wm8994_init(AUDIO_FREQUENCY_8K,
                      IO_METHOD_DMA,
                      INPUT_DEVICE_DIGITAL_MICROPHONE_2,
                      OUTPUT_DEVICE_HEADPHONE,
                      WM8994_HP_OUT_ANALOG_GAIN_0DB,
                      WM8994_LINE_IN_GAIN_0DB,
                      WM8994_DMIC_GAIN_9DB,
                      SOURCE_FILE_NAME,
                      GRAPH);
  while(1)
  {
    while(!(RX_buffer_full && TX_buffer_empty)){}
    BSP_LED_On(LED1);
    process_buffer();
    button = checkButtonFlag();
    if(button == 1) 
    {
      for(i=0; i<PING_PONG_BUFFER_SIZE; i++)
      {
        cmplx_buf[2*i] = x[i];
        cmplx_buf[2*i + 1] = 0.0;
      }
      arm_cfft_f32(&arm_cfft_sR_f32_len256, (float32_t *)(cmplx_buf), 0, 1);
      arm_cmplx_mag_f32((float32_t *)(cmplx_buf),(float32_t *)(outbuffer), PING_PONG_BUFFER_SIZE);
      plotLogFFT(outbuffer, PING_PONG_BUFFER_SIZE, LIVE);
    }
    else
    {
      plotWave(x, PLOTBUFSIZE, LIVE, ARRAY);
    }
    BSP_LED_Off(LED1);
  }
}

