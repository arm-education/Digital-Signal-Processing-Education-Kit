// stm32f7_loop_buf_dma.c

#include "stm32f7_wm8994_init.h"
#include "stm32f7_display.h"

#define SOURCE_FILE_NAME "stm32f7_loop_buf_dma.c"

#define BUFFER_SIZE PING_PONG_BUFFER_SIZE

extern volatile int32_t TX_buffer_empty; // these may not need to be int32_t
extern volatile int32_t RX_buffer_full; // they were extern volatile int16_t in F4 version
extern int16_t rx_buffer_proc, tx_buffer_proc; // will be assigned token values PING or PONG

float32_t rbuffer[BUFFER_SIZE];
int16_t rbufptr = 0;
float32_t lbuffer[BUFFER_SIZE];
int16_t lbufptr = 0;

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
    *tx_buf++ = *rx_buf++;  
    lbuffer[lbufptr] = (float32_t)(*tx_buf);
    lbufptr = (lbufptr+1) % BUFFER_SIZE;
    *tx_buf++ = *rx_buf++;
    rbuffer[rbufptr] = (float32_t)(*tx_buf);
    rbufptr = (rbufptr+1) % BUFFER_SIZE;
  }
  plotWaveNoAutoScale(rbuffer, 128);
  RX_buffer_full = 0;
  TX_buffer_empty = 0;
}

int main(void)
{  
  stm32f7_wm8994_init(AUDIO_FREQUENCY_8K,
                      IO_METHOD_DMA,
                      INPUT_DEVICE_INPUT_LINE_1,
                      OUTPUT_DEVICE_HEADPHONE,
                      WM8994_HP_OUT_ANALOG_GAIN_0DB,
                      WM8994_LINE_IN_GAIN_0DB,
                      WM8994_DMIC_GAIN_0DB,
                      SOURCE_FILE_NAME,
                      GRAPH);
  
  while(1)
  {
    while(!(RX_buffer_full && TX_buffer_empty)){}
    process_buffer();
  }
}

