// stm32f7_loop_dma.c

#include "stm32f7_wm8994_init.h"
#include "stm32f7_display.h"

#define SOURCE_FILE_NAME "stm32f7_loop_dma.c"

extern volatile int32_t TX_buffer_empty; // these may not need to be int32_t
extern volatile int32_t RX_buffer_full; // they were extern volatile int16_t in F4 version
extern int16_t rx_buffer_proc, tx_buffer_proc; // will be assigned token values PING or PONG


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
    *tx_buf++ = *rx_buf++;
  }

  RX_buffer_full = 0;
  TX_buffer_empty = 0;
}

int main(void)
{ 	
  stm32f7_wm8994_init(AUDIO_FREQUENCY_48K,
                      IO_METHOD_DMA,
                      INPUT_DEVICE_INPUT_LINE_1,
                      OUTPUT_DEVICE_HEADPHONE,
                      WM8994_HP_OUT_ANALOG_GAIN_0DB,
                      WM8994_LINE_IN_GAIN_0DB,
                      WM8994_DMIC_GAIN_9DB,
                      SOURCE_FILE_NAME,
                      NOGRAPH);
  while(1)
  {
    while(!(RX_buffer_full && TX_buffer_empty)){}
    process_buffer();
  }
}

