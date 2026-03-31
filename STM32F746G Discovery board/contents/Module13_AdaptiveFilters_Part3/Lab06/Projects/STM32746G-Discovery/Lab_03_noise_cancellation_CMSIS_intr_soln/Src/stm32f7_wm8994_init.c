// stm32f7_wm8994_init.c

// this file contains function definitions specific to DSP education kit implementation
// of audio in and out on F7 Discovery

#include "stm32f7_wm8994_init.h"

// structures and functions used to generate PRBS sequence
 typedef union 
{
 uint16_t value;
 struct 
 {
	 unsigned char bit0 : 1;
	 unsigned char bit1 : 1;
	 unsigned char bit2 : 1;
	 unsigned char bit3 : 1;
	 unsigned char bit4 : 1;
	 unsigned char bit5 : 1;
	 unsigned char bit6 : 1;
	 unsigned char bit7 : 1;
	 unsigned char bit8 : 1;
	 unsigned char bit9 : 1;
	 unsigned char bit10 : 1;
	 unsigned char bit11 : 1;
	 unsigned char bit12 : 1;
	 unsigned char bit13 : 1;
	 unsigned char bit14 : 1;
	 unsigned char bit15 : 1;
  } bits;
} shift_register;

shift_register sreg = {0x0001};

short prbs(int16_t noise_level)
{
  char fb;
  
	fb =((sreg.bits.bit15)+(sreg.bits.bit14)+(sreg.bits.bit3)+(sreg.bits.bit1))%2;
  sreg.value = sreg.value << 1;
  sreg.bits.bit0 = fb;			      
  if(fb == 0)	return(-noise_level); else return(noise_level);
}

uint32_t prand_seed = 1;       // used in function prand()

uint32_t rand31_next()
{
  uint32_t hi, lo;

  lo = 16807 * (prand_seed & 0xFFFF);
  hi = 16807 * (prand_seed >> 16);

  lo += (hi & 0x7FFF) << 16;
  lo += hi >> 15;

  if (lo > 0x7FFFFFFF) lo -= 0x7FFFFFFF;

  return(prand_seed = (uint32_t)lo);
}

// function returns random number in range +/- 8192
int16_t prand()
{
return ((int16_t)(rand31_next()>>18)-4096);
}

// flags similar to those used in STM32F4 program examples
volatile int32_t TX_buffer_empty = 0; // these may not need to be int32_t
volatile int32_t RX_buffer_full = 0; // they were extern volatile int16_t in F4 version
int16_t rx_buffer_proc, tx_buffer_proc; // will be assigned token values PING or PONG

int16_t rx_sample_L;
int16_t rx_sample_R;
int16_t tx_sample_L;
int16_t tx_sample_R;

// essentially this is the interrupt service routine called when input DMA transfer to
// buffer PING_IN has completed
void BSP_AUDIO_IN_TransferComplete_CallBack(void)
{
	rx_buffer_proc = PING;
  RX_buffer_full = 1;
  return;
}

// essentially this is the interrupt service routine called when input DMA transfer to
// buffer PONG_IN has completed
void BSP_AUDIO_IN_TransferCompleteM1_CallBack(void)
{
 rx_buffer_proc = PONG;
  RX_buffer_full = 1;
  return;
}

void BSP_AUDIO_IN_Error_CallBack(void)
{
  /* This function is called when an Interrupt due to transfer error on or peripheral
     error occurs. */
  while (BSP_PB_GetState(BUTTON_KEY) != RESET)
  {
    return;
  }
  /* could also generate a system reset to recover from the error */
  /* .... */
}

// essentially this is the interrupt service routine called when output DMA transfer from
// buffer PING_OUT has completed
void BSP_AUDIO_OUT_TransferComplete_CallBack(void)
{
  tx_buffer_proc = PING;
  TX_buffer_empty = 1;
  return;
}

// essentially this is the interrupt service routine called when output DMA transfer from
// buffer PONG_OUT has completed
void BSP_AUDIO_OUT_TransferCompleteM1_CallBack(void)
{
	tx_buffer_proc = PONG;
  TX_buffer_empty = 1;
  return;
}

void BSP_AUDIO_OUT_Error_CallBack(void)
{
  /* Stop the program with an infinite loop */
  while (BSP_PB_GetState(BUTTON_KEY) != RESET)
  {
    return;
  }
}

/**
  * @brief EXTI line detection callbacks.
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  static uint32_t debounce_time = 0;
	
//	BSP_TS_GetState(&Position);
	
  if (GPIO_Pin == KEY_BUTTON_PIN)
  {
     //Prevent debounce effect for user key 
    if ((HAL_GetTick() - debounce_time) > 50)
    {
      debounce_time = HAL_GetTick();
    }
  }
	else if (GPIO_Pin == AUDIO_IN_INT_GPIO_PIN)
  {
     //Audio IN interrupt 
  }
}

#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif /* USE_FULL_ASSERT */

DAC_TypeDef *Instance;
GPIO_InitTypeDef          GPIO_InitStruct;

void DAC12_Config(void){
	 Instance  = DAC;

//   Could we incorporate the following in init() fn?
/* Enable GPIO clock ****************************************/
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_DAC_CLK_ENABLE();

  /*##-2- Configure peripheral GPIO ##########################################*/
  /* DAC Channel1 GPIO pin configuration */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		
  Instance->CR = 0x3D;
}

// overall initialisation function called from main function
void stm32f7_wm8994_init(uint32_t fs, int16_t io_method, int16_t select_input, int16_t select_output, int16_t headphone_gain, int16_t line_in_gain, int16_t dmic_gain, char * name, int graph)
{
  BSP_LED_Init(LED1);   // initialise LED on GPIO pin P   (also accessible on arduino header)
  BSP_GPIO_Init();      // initialise diagnostic GPIO pin P   (accessible on arduino header)
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_GPIO); // configure the  blue user pushbutton in GPIO mode
  BSP_SDRAM_Init();
	DAC12_Config();
  init_LCD(fs, name, io_method, graph);
		
  switch(io_method)
  {
    case IO_METHOD_DMA:

      BSP_AUDIO_IN_OUT_Init(select_input, select_output, fs);
      wm8994_SetVolume(AUDIO_I2C_ADDRESS, headphone_gain, line_in_gain, dmic_gain);
		  memset((uint16_t*)PING_IN, 0, PING_PONG_BUFFER_SIZE*4);
      memset((uint16_t*)PONG_IN, 0, PING_PONG_BUFFER_SIZE*4);
      memset((uint16_t*)PING_OUT, 0, PING_PONG_BUFFER_SIZE*4);
      memset((uint16_t*)PONG_OUT, 0, PING_PONG_BUFFER_SIZE*4);
      BSP_AUDIO_IN_MultiBufferRecord((uint16_t*)PING_IN, (uint16_t*)PONG_IN, PING_PONG_BUFFER_SIZE*2);
      BSP_AUDIO_OUT_MultiBufferPlay((uint16_t*)PING_OUT, (uint16_t*)PONG_OUT, PING_PONG_BUFFER_SIZE*2);
      break;
   
   case IO_METHOD_INTR:
      BSP_AUDIO_IN_OUT_Init_SAIinterrupt(select_input, select_output, fs);
      wm8994_SetVolume(AUDIO_I2C_ADDRESS, headphone_gain, line_in_gain, dmic_gain);
      BSP_AUDIO_OUT_SetAudioFrameSlot(CODEC_AUDIOFRAME_SLOT_02); // may be redundant for intr version DSR 29 May 2017 NO!! 19 June
      BSP_AUDIO_SAI_INTERRUPT_INIT(&rx_sample_L, &rx_sample_R, &tx_sample_L, &tx_sample_R);
      break;
	 
	 default:
		  break;
  } 
}
