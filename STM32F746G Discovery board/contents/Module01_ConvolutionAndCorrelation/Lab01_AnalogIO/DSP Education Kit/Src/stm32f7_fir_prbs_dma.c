// stm32f7_fir_prbs_dma.c

#include "stm32f7_wm8994_init.h"
#include "maf5.h"
#include "stm32f7_display.h"

#define SOURCE_FILE_NAME "stm32f7_fir_prbs_dma.c"

extern volatile int32_t TX_buffer_empty; // these may not need to be int32_t
extern volatile int32_t RX_buffer_full; // they were extern volatile int16_t in F4 version
extern int16_t rx_buffer_proc, tx_buffer_proc; // will be assigned token values PING or PONG

float32_t x[N], y[PING_PONG_BUFFER_SIZE];

void process_buffer(void) // this function processes one DMA transfer block worth of data
{
  int i,k;
  int16_t *rx_buf, *tx_buf;
	
  if (rx_buffer_proc == PING) {rx_buf = (int16_t *)PING_IN;}
  else {rx_buf = (int16_t *)PONG_IN;}
  if (tx_buffer_proc == PING) {tx_buf = (int16_t *)PING_OUT;}
  else {tx_buf = (int16_t *)PONG_OUT;}
  BSP_LED_On(LED1);
  for (i=0 ; i<(PING_PONG_BUFFER_SIZE) ; i++)
  {
    y[i] = 0.0;
    x[0] = (float32_t)(prbs(8000));
    for (k=0 ; k<N ; k++) y[i] += h[k]*x[k];
    for (k=(N-1) ; k>0 ; k--) x[k] = x[k-1];
  }
  BSP_LED_Off(LED1);
  for (i=0 ; i<(PING_PONG_BUFFER_SIZE) ; i++)
  {
    *tx_buf++ = (short)(y[i]);
    *tx_buf++ = (short)(y[i]);
  } 
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
                      NOGRAPH);
  
  while(1)
  {
    while(!(RX_buffer_full && TX_buffer_empty)){}
    process_buffer();
  }
}

