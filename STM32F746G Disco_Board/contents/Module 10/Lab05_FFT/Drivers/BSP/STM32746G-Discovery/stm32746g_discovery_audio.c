/**
  ******************************************************************************
  * @file    stm32f746g_discovery_audio.c
  * @author  MCD Application Team
  * Adapted for use as part of ARM University DSP Education Kit May 2017
  * @version V2.0.0
  * @date    30-December-2016
  * @brief   This file provides the Audio driver for the STM32746G-Discovery board.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 

#include "stm32746g_discovery_audio.h"
#include <math.h>

SAI_HandleTypeDef         haudio_out_sai={0};
SAI_HandleTypeDef         haudio_in_sai={0};

// uint16_t __IO AudioInVolume = DEFAULT_AUDIO_IN_VOLUME;
    
static void SAIx_In_Init(uint32_t SaiOutMode, uint32_t SlotActive, uint32_t AudioFreq);
static void SAIx_In_Init_SAIinterrupt(uint32_t SaiOutMode, uint32_t SlotActive, uint32_t AudioFreq);
static void SAIx_In_DeInit(void);
static void SAIx_Out_DeInit(void);

/**
  * Modified 12 May 2017 DSR for multibuffer operation
  * @brief  Starts playing audio stream from a data buffer for a determined size. 
  * @param  pBuffer: Pointer to the buffer 
  * @param  Size: Number of audio data in BYTES unit.
  *         In memory, first element is for left channel, second element is for right channel
  * @retval AUDIO_OK if correct communication, else wrong communication
  */
uint8_t BSP_AUDIO_OUT_MultiBufferPlay(uint16_t* pBufferM0, uint16_t* pBufferM1, uint32_t Size)
{
  {
    HAL_SAI_Transmit_DMA_MultiBuffer(&haudio_out_sai, (uint8_t*) pBufferM0, (uint8_t*) pBufferM1, Size);
    
    return AUDIO_OK;
  }
}

void BSP_AUDIO_OUT_SetAudioFrameSlot(uint32_t AudioFrameSlot)
{ 
  /* Disable SAI peripheral to allow access to SAI internal registers */
  __HAL_SAI_DISABLE(&haudio_out_sai);
  
  /* Update the SAI audio frame slot configuration */
  haudio_out_sai.SlotInit.SlotActive = AudioFrameSlot;
  HAL_SAI_Init(&haudio_out_sai);
  
  /* Enable SAI peripheral to generate MCLK */
  __HAL_SAI_ENABLE(&haudio_out_sai);
}

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{
  /* Manage the remaining file size and new address offset: This function 
     should be coded by user (its prototype is already declared in stm32746g_discovery_audio.h) */
  BSP_AUDIO_OUT_TransferComplete_CallBack();
}

void HAL_SAI_TxM1CpltCallback(SAI_HandleTypeDef *hsai)
{
  /* Manage the remaining file size and new address offset: This function 
     should be coded by user (its prototype is already declared in stm32746g_discovery_audio.h) */
  BSP_AUDIO_OUT_TransferCompleteM1_CallBack();
}

void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai)
{
  HAL_SAI_StateTypeDef audio_out_state;
  HAL_SAI_StateTypeDef audio_in_state;

  audio_out_state = HAL_SAI_GetState(&haudio_out_sai);
  audio_in_state = HAL_SAI_GetState(&haudio_in_sai);

  /* Determines if it is an audio out or audio in error */
  if ((audio_out_state == HAL_SAI_STATE_BUSY) || (audio_out_state == HAL_SAI_STATE_BUSY_TX))
  {
    BSP_AUDIO_OUT_Error_CallBack();
  }

  if ((audio_in_state == HAL_SAI_STATE_BUSY) || (audio_in_state == HAL_SAI_STATE_BUSY_RX))
  {
    BSP_AUDIO_IN_Error_CallBack();
  }
}

__weak void BSP_AUDIO_OUT_TransferComplete_CallBack(void)
{
}

__weak void BSP_AUDIO_OUT_TransferCompleteM1_CallBack(void)
{
}

__weak void BSP_AUDIO_OUT_Error_CallBack(void)
{
}

__weak void BSP_AUDIO_OUT_MspInit(SAI_HandleTypeDef *hsai, void *Params)
{ 
  static DMA_HandleTypeDef hdma_sai_tx;
  GPIO_InitTypeDef  gpio_init_structure;  

  /* Enable SAI clock */
  AUDIO_OUT_SAIx_CLK_ENABLE();
  
  /* Enable GPIO clock */
  AUDIO_OUT_SAIx_MCLK_ENABLE();
  AUDIO_OUT_SAIx_SCK_SD_ENABLE();
  AUDIO_OUT_SAIx_FS_ENABLE();
  /* CODEC_SAI pins configuration: FS, SCK, MCK and SD pins ------------------*/
  gpio_init_structure.Pin = AUDIO_OUT_SAIx_FS_PIN;
  gpio_init_structure.Mode = GPIO_MODE_AF_PP;
  gpio_init_structure.Pull = GPIO_NOPULL;
  gpio_init_structure.Speed = GPIO_SPEED_HIGH;
  gpio_init_structure.Alternate = AUDIO_OUT_SAIx_FS_SD_MCLK_AF;
  HAL_GPIO_Init(AUDIO_OUT_SAIx_FS_GPIO_PORT, &gpio_init_structure);

  gpio_init_structure.Pin = AUDIO_OUT_SAIx_SCK_PIN;
  gpio_init_structure.Mode = GPIO_MODE_AF_PP;
  gpio_init_structure.Pull = GPIO_NOPULL;
  gpio_init_structure.Speed = GPIO_SPEED_HIGH;
  gpio_init_structure.Alternate = AUDIO_OUT_SAIx_SCK_AF;
  HAL_GPIO_Init(AUDIO_OUT_SAIx_SCK_SD_GPIO_PORT, &gpio_init_structure);

  gpio_init_structure.Pin =  AUDIO_OUT_SAIx_SD_PIN;
  gpio_init_structure.Mode = GPIO_MODE_AF_PP;
  gpio_init_structure.Pull = GPIO_NOPULL;
  gpio_init_structure.Speed = GPIO_SPEED_HIGH;
  gpio_init_structure.Alternate = AUDIO_OUT_SAIx_FS_SD_MCLK_AF;
  HAL_GPIO_Init(AUDIO_OUT_SAIx_SCK_SD_GPIO_PORT, &gpio_init_structure);

  gpio_init_structure.Pin = AUDIO_OUT_SAIx_MCLK_PIN;
  gpio_init_structure.Mode = GPIO_MODE_AF_PP;
  gpio_init_structure.Pull = GPIO_NOPULL;
  gpio_init_structure.Speed = GPIO_SPEED_HIGH;
  gpio_init_structure.Alternate = AUDIO_OUT_SAIx_FS_SD_MCLK_AF;
  HAL_GPIO_Init(AUDIO_OUT_SAIx_MCLK_GPIO_PORT, &gpio_init_structure);

  /* Enable the DMA clock */
  AUDIO_OUT_SAIx_DMAx_CLK_ENABLE();
    
  if(hsai->Instance == AUDIO_OUT_SAIx)
  {
    /* Configure the hdma_saiTx handle parameters */   
    hdma_sai_tx.Init.Channel             = AUDIO_OUT_SAIx_DMAx_CHANNEL;
    hdma_sai_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_sai_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_sai_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_sai_tx.Init.PeriphDataAlignment = AUDIO_OUT_SAIx_DMAx_PERIPH_DATA_SIZE;
    hdma_sai_tx.Init.MemDataAlignment    = AUDIO_OUT_SAIx_DMAx_MEM_DATA_SIZE;
    hdma_sai_tx.Init.Mode                = DMA_CIRCULAR;
    hdma_sai_tx.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_sai_tx.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;         
    hdma_sai_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma_sai_tx.Init.MemBurst            = DMA_MBURST_SINGLE;
    hdma_sai_tx.Init.PeriphBurst         = DMA_PBURST_SINGLE; 
    
    hdma_sai_tx.Instance = AUDIO_OUT_SAIx_DMAx_STREAM;
    
    /* Associate the DMA handle */
    __HAL_LINKDMA(hsai, hdmatx, hdma_sai_tx);
    
    /* Deinitialize the Stream for new transfer */
    HAL_DMA_DeInit(&hdma_sai_tx);
    
    /* Configure the DMA Stream */
    HAL_DMA_Init(&hdma_sai_tx);      
  }
  
  /* SAI DMA IRQ Channel configuration */
  HAL_NVIC_SetPriority(AUDIO_OUT_SAIx_DMAx_IRQ, AUDIO_OUT_IRQ_PREPRIO, 0);
  HAL_NVIC_EnableIRQ(AUDIO_OUT_SAIx_DMAx_IRQ); 
}

__weak void BSP_AUDIO_OUT_MspDeInit(SAI_HandleTypeDef *hsai, void *Params)
{
    GPIO_InitTypeDef  gpio_init_structure;

    /* SAI DMA IRQ Channel deactivation */
    HAL_NVIC_DisableIRQ(AUDIO_OUT_SAIx_DMAx_IRQ);

    if(hsai->Instance == AUDIO_OUT_SAIx)
    {
      /* Deinitialize the DMA stream */
      HAL_DMA_DeInit(hsai->hdmatx);
    }

    /* Disable SAI peripheral */
    __HAL_SAI_DISABLE(hsai);  

    /* Deactives CODEC_SAI pins FS, SCK, MCK and SD by putting them in input mode */
    gpio_init_structure.Pin = AUDIO_OUT_SAIx_FS_PIN;
    HAL_GPIO_DeInit(AUDIO_OUT_SAIx_FS_GPIO_PORT, gpio_init_structure.Pin);

    gpio_init_structure.Pin = AUDIO_OUT_SAIx_SCK_PIN;
    HAL_GPIO_DeInit(AUDIO_OUT_SAIx_SCK_SD_GPIO_PORT, gpio_init_structure.Pin);

    gpio_init_structure.Pin =  AUDIO_OUT_SAIx_SD_PIN;
    HAL_GPIO_DeInit(AUDIO_OUT_SAIx_SCK_SD_GPIO_PORT, gpio_init_structure.Pin);

    gpio_init_structure.Pin = AUDIO_OUT_SAIx_MCLK_PIN;
    HAL_GPIO_DeInit(AUDIO_OUT_SAIx_MCLK_GPIO_PORT, gpio_init_structure.Pin);
  
    /* Disable SAI clock */
    AUDIO_OUT_SAIx_CLK_DISABLE();

    /* GPIO pins clock and DMA clock can be shut down in the application
       by surcharging this __weak function */
}

__weak void BSP_AUDIO_OUT_ClockConfig(SAI_HandleTypeDef *hsai, uint32_t AudioFreq, void *Params)
{ 
  RCC_PeriphCLKInitTypeDef rcc_ex_clk_init_struct;

  HAL_RCCEx_GetPeriphCLKConfig(&rcc_ex_clk_init_struct);
  
  /* Set the PLL configuration according to the audio frequency */
  if((AudioFreq == AUDIO_FREQUENCY_11K) || (AudioFreq == AUDIO_FREQUENCY_22K) || (AudioFreq == AUDIO_FREQUENCY_44K))
  {
    /* Configure PLLI2S prescalers */
    /* PLLI2S_VCO: VCO_429M
    I2S_CLK(first level) = PLLI2S_VCO/PLLI2SQ = 429/2 = 214.5 Mhz
    I2S_CLK_x = I2S_CLK(first level)/PLLI2SDIVQ = 214.5/19 = 11.289 Mhz */
    rcc_ex_clk_init_struct.PeriphClockSelection = RCC_PERIPHCLK_SAI2;
    rcc_ex_clk_init_struct.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLI2S;
    rcc_ex_clk_init_struct.PLLI2S.PLLI2SN = 429;
    rcc_ex_clk_init_struct.PLLI2S.PLLI2SQ = 2;
    rcc_ex_clk_init_struct.PLLI2SDivQ = 19;
    
    HAL_RCCEx_PeriphCLKConfig(&rcc_ex_clk_init_struct);
    
  }
  else /* AUDIO_FREQUENCY_8K, AUDIO_FREQUENCY_16K, AUDIO_FREQUENCY_48K), AUDIO_FREQUENCY_96K */
  {
    /* I2S clock config
    PLLI2S_VCO: VCO_344M
    I2S_CLK(first level) = PLLI2S_VCO/PLLI2SQ = 344/7 = 49.142 Mhz
    I2S_CLK_x = I2S_CLK(first level)/PLLI2SDIVQ = 49.142/1 = 49.142 Mhz */
    rcc_ex_clk_init_struct.PeriphClockSelection = RCC_PERIPHCLK_SAI2;
    rcc_ex_clk_init_struct.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLI2S;
    rcc_ex_clk_init_struct.PLLI2S.PLLI2SN = 344;
    rcc_ex_clk_init_struct.PLLI2S.PLLI2SQ = 7;
    rcc_ex_clk_init_struct.PLLI2SDivQ = 1;
    
    HAL_RCCEx_PeriphCLKConfig(&rcc_ex_clk_init_struct);
  }
}


uint8_t BSP_AUDIO_IN_OUT_Init(uint16_t InputDevice, uint16_t OutputDevice, uint32_t AudioFreq)
{
  uint8_t ret = AUDIO_ERROR;
  uint32_t slot_active;

	if ((InputDevice != INPUT_DEVICE_INPUT_LINE_1) && (InputDevice != INPUT_DEVICE_DIGITAL_MICROPHONE_2))
  {
    ret = AUDIO_ERROR;
  }
  else
  {
    /* Disable SAI */
    SAIx_In_DeInit();
    SAIx_Out_DeInit();

    /* PLL clock is set depending on the AudioFreq (44.1khz vs 48khz groups) */
    BSP_AUDIO_OUT_ClockConfig(&haudio_in_sai, AudioFreq, NULL); /* Clock config is shared between AUDIO IN and OUT */

    /* SAI data transfer preparation:
    Prepare the Media to be used for the audio transfer from SAI peripheral to memory */
    haudio_in_sai.Instance = AUDIO_IN_SAIx;
    if(HAL_SAI_GetState(&haudio_in_sai) == HAL_SAI_STATE_RESET) // where was this set up?
    {
      /* Init the SAI MSP: this __weak function can be redefined by the application*/
      BSP_AUDIO_IN_MspInit(&haudio_in_sai, NULL);
    }

    /* SAI data transfer preparation:
    Prepare the Media to be used for the audio transfer from memory to SAI peripheral */
    haudio_out_sai.Instance = AUDIO_OUT_SAIx;
    if(HAL_SAI_GetState(&haudio_out_sai) == HAL_SAI_STATE_RESET)
    {
      /* Init the SAI MSP: this __weak function can be redefined by the application*/
      BSP_AUDIO_OUT_MspInit(&haudio_out_sai, NULL);
    }

    /* Configure SAI in master mode :
     *   - SAI2_block_A in master TX mode
     *   - SAI2_block_B in slave RX mode synchronous from SAI2_block_A
     */
    if (InputDevice == INPUT_DEVICE_DIGITAL_MICROPHONE_2)
    {
      slot_active = CODEC_AUDIOFRAME_SLOT_13;
    }
    else
    {
      slot_active = CODEC_AUDIOFRAME_SLOT_02;
    }

    SAIx_In_Init(SAI_MODEMASTER_TX, slot_active, AudioFreq);

      wm8994_Reset(AUDIO_I2C_ADDRESS);
      wm8994_Init(AUDIO_I2C_ADDRESS, InputDevice | OutputDevice, AudioFreq);
//  		wm8994_SetVolume(AUDIO_I2C_ADDRESS, headphone_gain, line_in_gain, dmic_gain);
	}
  return ret;
}


uint8_t  BSP_AUDIO_IN_MultiBufferRecord(uint16_t* pbufM0, uint16_t* pbufM1, uint32_t size)
{
  uint32_t ret = AUDIO_ERROR;
  
  /* Start the process receive DMA */
// size should be (PING_PONGBUFFER_SIZE*2), that is the number of 16-bit sample values
// (left and right) in one DMA transfer block
  HAL_SAI_Receive_DMA_MultiBuffer(&haudio_in_sai, (uint8_t*)pbufM0, (uint8_t*)pbufM1, size);
  
  /* Return AUDIO_OK when all operations are correctly done */
  ret = AUDIO_OK;
  
  return ret;
}



void BSP_AUDIO_IN_DeInit(void)
{
  SAIx_In_DeInit();
  /* DeInit the SAI MSP : this __weak function can be rewritten by the application */
  BSP_AUDIO_IN_MspDeInit(&haudio_in_sai, NULL);
}

void HAL_SAI_RxCpltCallback() // called in DMA mode from SAI_DMARxCplt() in stm32f7xx_hal_sai.c
{
// function defined in main.c (??? or in stm32f7_wm8994_init.c DSR)
// effectively implements ping-pong double buffering mechanism
  BSP_AUDIO_IN_TransferComplete_CallBack();
}

static void SAIx_Out_DeInit(void)
{
  /* Initialize the haudio_out_sai Instance parameter */
  haudio_out_sai.Instance = AUDIO_OUT_SAIx;

  /* Disable SAI peripheral */
  __HAL_SAI_DISABLE(&haudio_out_sai);

  HAL_SAI_DeInit(&haudio_out_sai);
}

__weak void BSP_AUDIO_SAI_Interrupt_CallBack(void)
{
//	UNUSED(void);
}

void HAL_SAI_RxCpltCallback_Interrupt() // called in interrupt mode from SAI_Receive_IT16Bit() in hal_sai.c DSR
{
  BSP_AUDIO_SAI_Interrupt_CallBack(); // DSP algorithm is there - sample values in global variables
}


uint8_t BSP_AUDIO_SAI_INTERRUPT_INIT(int16_t* rxLptr, int16_t* rxRptr, int16_t* txLptr, int16_t* txRptr)
{
// set up input and output sample pointers in SAI_HandleTypeDef structures
  haudio_in_sai.sample_left_ptr = rxLptr;
  haudio_in_sai.sample_right_ptr = rxRptr;
  haudio_out_sai.sample_left_ptr = txLptr;
  haudio_out_sai.sample_right_ptr = txRptr;

	HAL_SAI_Receive_IT_SAIinterrupt(&haudio_in_sai, &haudio_out_sai);

	return AUDIO_OK;
}

__weak void BSP_AUDIO_OUT_MspInit_SAIinterrupt(SAI_HandleTypeDef *hsai, void *Params)
{ 
  GPIO_InitTypeDef  gpio_init_structure;  

  /* Enable SAI clock */
  AUDIO_OUT_SAIx_CLK_ENABLE();
  
  /* Enable GPIO clock */
  AUDIO_OUT_SAIx_MCLK_ENABLE();
  AUDIO_OUT_SAIx_SCK_SD_ENABLE();
  AUDIO_OUT_SAIx_FS_ENABLE();
  /* CODEC_SAI pins configuration: FS, SCK, MCK and SD pins ------------------*/
  gpio_init_structure.Pin = AUDIO_OUT_SAIx_FS_PIN;
  gpio_init_structure.Mode = GPIO_MODE_AF_PP;
  gpio_init_structure.Pull = GPIO_NOPULL;
  gpio_init_structure.Speed = GPIO_SPEED_HIGH;
  gpio_init_structure.Alternate = AUDIO_OUT_SAIx_FS_SD_MCLK_AF;
  HAL_GPIO_Init(AUDIO_OUT_SAIx_FS_GPIO_PORT, &gpio_init_structure);

  gpio_init_structure.Pin = AUDIO_OUT_SAIx_SCK_PIN;
  gpio_init_structure.Mode = GPIO_MODE_AF_PP;
  gpio_init_structure.Pull = GPIO_NOPULL;
  gpio_init_structure.Speed = GPIO_SPEED_HIGH;
  gpio_init_structure.Alternate = AUDIO_OUT_SAIx_SCK_AF;
  HAL_GPIO_Init(AUDIO_OUT_SAIx_SCK_SD_GPIO_PORT, &gpio_init_structure);

  gpio_init_structure.Pin =  AUDIO_OUT_SAIx_SD_PIN;
  gpio_init_structure.Mode = GPIO_MODE_AF_PP;
  gpio_init_structure.Pull = GPIO_NOPULL;
  gpio_init_structure.Speed = GPIO_SPEED_HIGH;
  gpio_init_structure.Alternate = AUDIO_OUT_SAIx_FS_SD_MCLK_AF;
  HAL_GPIO_Init(AUDIO_OUT_SAIx_SCK_SD_GPIO_PORT, &gpio_init_structure);

  gpio_init_structure.Pin = AUDIO_OUT_SAIx_MCLK_PIN;
  gpio_init_structure.Mode = GPIO_MODE_AF_PP;
  gpio_init_structure.Pull = GPIO_NOPULL;
  gpio_init_structure.Speed = GPIO_SPEED_HIGH;
  gpio_init_structure.Alternate = AUDIO_OUT_SAIx_FS_SD_MCLK_AF;
  HAL_GPIO_Init(AUDIO_OUT_SAIx_MCLK_GPIO_PORT, &gpio_init_structure);
    
  // there is no interrupt associated with output TX SAI2 block A and so no
  // HAL_NVIC_SetPriority() or HAL_NVIC_EnableIRQ() calls here 
}


__weak void BSP_AUDIO_IN_MspInit_SAIinterrupt(SAI_HandleTypeDef *hsai, void *Params)
{
  GPIO_InitTypeDef  gpio_init_structure;  

  /* Enable SAI clock */
  AUDIO_IN_SAIx_CLK_ENABLE();
  
  /* Enable SD GPIO clock */
  AUDIO_IN_SAIx_SD_ENABLE();
  /* CODEC_SAI pin configuration: SD pin */
  gpio_init_structure.Pin = AUDIO_IN_SAIx_SD_PIN;
  gpio_init_structure.Mode = GPIO_MODE_AF_PP;
  gpio_init_structure.Pull = GPIO_NOPULL;
  gpio_init_structure.Speed = GPIO_SPEED_FAST;
  gpio_init_structure.Alternate = AUDIO_IN_SAIx_SD_AF;
  HAL_GPIO_Init(AUDIO_IN_SAIx_SD_GPIO_PORT, &gpio_init_structure);

  /* Enable Audio INT GPIO clock */
  AUDIO_IN_INT_GPIO_ENABLE();
  /* Audio INT pin configuration: input */
  gpio_init_structure.Pin = AUDIO_IN_INT_GPIO_PIN;
  gpio_init_structure.Mode = GPIO_MODE_INPUT;
  gpio_init_structure.Pull = GPIO_NOPULL;
  gpio_init_structure.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(AUDIO_IN_INT_GPIO_PORT, &gpio_init_structure);

  
  // SAI2 block B RX interrupt configuration - but actually this is for blocks A and B
	// we can control interrupt enables for individual blocks in SAI_xIM register
	
  HAL_NVIC_SetPriority(SAI2_IRQn, AUDIO_IN_IRQ_PREPRIO, 0); // SAI2_IRQn #defined as 91 in stm32f746xx.h
  HAL_NVIC_EnableIRQ(SAI2_IRQn); // stick with existing interrupt priority for DMA
}


uint8_t BSP_AUDIO_IN_OUT_Init_SAIinterrupt(uint16_t InputDevice, uint16_t OutputDevice, uint32_t AudioFreq)
{
  uint32_t slot_active;

  if ((InputDevice != INPUT_DEVICE_INPUT_LINE_1) && (InputDevice != INPUT_DEVICE_DIGITAL_MICROPHONE_2))
  {
    return  AUDIO_ERROR;
  }
  else
  {
    // disable SAI
    SAIx_In_DeInit();
    SAIx_Out_DeInit();

    // set PLL clock depending on the AudioFreq (44.1khz vs 48khz groups)
    BSP_AUDIO_OUT_ClockConfig(&haudio_in_sai, AudioFreq, NULL); // clock config is shared between AUDIO IN and OUT */

    // configure SAI2 block B (input slave in sync with block A)
    haudio_in_sai.Instance = AUDIO_IN_SAIx; // #defined in stm32746g_discovery_audio.h as SAI2 block B
    if(HAL_SAI_GetState(&haudio_in_sai) == HAL_SAI_STATE_RESET)
    {
      BSP_AUDIO_IN_MspInit_SAIinterrupt(&haudio_in_sai, NULL);
    }

    // configure SAI2 block A (output master)
    haudio_out_sai.Instance = AUDIO_OUT_SAIx; // #defined in stm32746g_discovery_audio.h as SAI2 block A
    if(HAL_SAI_GetState(&haudio_out_sai) == HAL_SAI_STATE_RESET)
    {
      BSP_AUDIO_OUT_MspInit_SAIinterrupt(&haudio_out_sai, NULL);
    }

    /* Configure SAI in master mode :
     *   - SAI2_block_A in master TX mode
     *   - SAI2_block_B in slave RX mode synchronous from SAI2_block_A
     */
    if (InputDevice == INPUT_DEVICE_DIGITAL_MICROPHONE_2)
    {
      slot_active = CODEC_AUDIOFRAME_SLOT_13;
    }
    else
    {
      slot_active = CODEC_AUDIOFRAME_SLOT_02;
    }

    SAIx_In_Init_SAIinterrupt(SAI_MODEMASTER_TX, slot_active, AudioFreq);

      wm8994_Reset(AUDIO_I2C_ADDRESS);

      /* Initialize the codec internal registers, i.e. call function wm8994_init() defined in wm8994.c */
      wm8994_Init(AUDIO_I2C_ADDRESS, InputDevice | OutputDevice, AudioFreq);
  }
  return AUDIO_OK;
}



void HAL_SAI_RxM1CpltCallback(SAI_HandleTypeDef *hsai)
{
  /* Call the record update function to get the next buffer to fill and its size (size is ignored) */
  BSP_AUDIO_IN_TransferCompleteM1_CallBack();
}


__weak void BSP_AUDIO_IN_TransferComplete_CallBack(void)
{
  /* This function should be implemented by the user application.
     It is called into this driver when the current buffer is filled
     to prepare the next buffer pointer and its size. */
}

__weak void BSP_AUDIO_IN_TransferCompleteM1_CallBack(void)
{
  /* This function should be implemented by the user application.
     It is called into this driver when the current buffer is filled
     to prepare the next buffer pointer and its size. */
}

__weak void BSP_AUDIO_IN_Error_CallBack(void)
{   
  /* This function is called when an Interrupt due to transfer error on or peripheral
     error occurs. */
}

__weak void BSP_AUDIO_IN_MspInit(SAI_HandleTypeDef *hsai, void *Params)
{
  static DMA_HandleTypeDef hdma_sai_rx;
  GPIO_InitTypeDef  gpio_init_structure;  

  /* Enable SAI clock */
  AUDIO_IN_SAIx_CLK_ENABLE();
  
  /* Enable SD GPIO clock */
  AUDIO_IN_SAIx_SD_ENABLE();
  /* CODEC_SAI pin configuration: SD pin */
  gpio_init_structure.Pin = AUDIO_IN_SAIx_SD_PIN;
  gpio_init_structure.Mode = GPIO_MODE_AF_PP;
  gpio_init_structure.Pull = GPIO_NOPULL;
  gpio_init_structure.Speed = GPIO_SPEED_FAST;
  gpio_init_structure.Alternate = AUDIO_IN_SAIx_SD_AF;
  HAL_GPIO_Init(AUDIO_IN_SAIx_SD_GPIO_PORT, &gpio_init_structure);

  /* Enable Audio INT GPIO clock */
  AUDIO_IN_INT_GPIO_ENABLE();
  /* Audio INT pin configuration: input */
  gpio_init_structure.Pin = AUDIO_IN_INT_GPIO_PIN;
  gpio_init_structure.Mode = GPIO_MODE_INPUT;
  gpio_init_structure.Pull = GPIO_NOPULL;
  gpio_init_structure.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(AUDIO_IN_INT_GPIO_PORT, &gpio_init_structure);

  /* Enable the DMA clock */
  AUDIO_IN_SAIx_DMAx_CLK_ENABLE();
    
  if(hsai->Instance == AUDIO_IN_SAIx)
  {
    /* Configure the hdma_sai_rx handle parameters */
    hdma_sai_rx.Init.Channel             = AUDIO_IN_SAIx_DMAx_CHANNEL;
    hdma_sai_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_sai_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_sai_rx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_sai_rx.Init.PeriphDataAlignment = AUDIO_IN_SAIx_DMAx_PERIPH_DATA_SIZE;
    hdma_sai_rx.Init.MemDataAlignment    = AUDIO_IN_SAIx_DMAx_MEM_DATA_SIZE;
    hdma_sai_rx.Init.Mode                = DMA_CIRCULAR;
    hdma_sai_rx.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_sai_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hdma_sai_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma_sai_rx.Init.MemBurst            = DMA_MBURST_SINGLE;
    hdma_sai_rx.Init.PeriphBurst         = DMA_MBURST_SINGLE;
    
    hdma_sai_rx.Instance = AUDIO_IN_SAIx_DMAx_STREAM;
    
    /* Associate the DMA handle */
    __HAL_LINKDMA(hsai, hdmarx, hdma_sai_rx);
    
    /* Deinitialize the Stream for new transfer */
    HAL_DMA_DeInit(&hdma_sai_rx);
    
    /* Configure the DMA Stream */
    HAL_DMA_Init(&hdma_sai_rx);
  }
  
  /* SAI DMA IRQ Channel configuration */
  HAL_NVIC_SetPriority(AUDIO_IN_SAIx_DMAx_IRQ, AUDIO_IN_IRQ_PREPRIO, 0);
  HAL_NVIC_EnableIRQ(AUDIO_IN_SAIx_DMAx_IRQ);

  /* Audio INT IRQ Channel configuration */
//  HAL_NVIC_SetPriority(AUDIO_IN_INT_IRQ, AUDIO_IN_IRQ_PREPRIO, 0);
//  HAL_NVIC_EnableIRQ(AUDIO_IN_INT_IRQ);
}


__weak void BSP_AUDIO_IN_MspDeInit(SAI_HandleTypeDef *hsai, void *Params)
{
  GPIO_InitTypeDef  gpio_init_structure;

  static DMA_HandleTypeDef hdma_sai_rx;

  /* SAI IN DMA IRQ Channel deactivation */
  HAL_NVIC_DisableIRQ(AUDIO_IN_SAIx_DMAx_IRQ);

  if(hsai->Instance == AUDIO_IN_SAIx)
  {
    /* Deinitialize the Stream for new transfer */
    HAL_DMA_DeInit(&hdma_sai_rx);
  }

 /* Disable SAI block */
  __HAL_SAI_DISABLE(hsai);

  /* Disable pin: SD pin */
  gpio_init_structure.Pin = AUDIO_IN_SAIx_SD_PIN;
  HAL_GPIO_DeInit(AUDIO_IN_SAIx_SD_GPIO_PORT, gpio_init_structure.Pin);

  /* Disable SAI clock */
  AUDIO_IN_SAIx_CLK_DISABLE();

  /* GPIO pins clock and DMA clock can be shut down in the application
     by surcharging this __weak function */
}


static void SAIx_In_Init(uint32_t SaiOutMode, uint32_t SlotActive, uint32_t AudioFreq)
{
  /* Initialize SAI2 block A in MASTER TX */
  /* Initialize the haudio_out_sai Instance parameter */
  haudio_out_sai.Instance = AUDIO_OUT_SAIx;

  /* Disable SAI peripheral to allow access to SAI internal registers */
  __HAL_SAI_DISABLE(&haudio_out_sai);

  /* Configure SAI_Block_x
  LSBFirst: Disabled
  DataSize: 16 */
  haudio_out_sai.Init.AudioFrequency = AudioFreq;
  haudio_out_sai.Init.AudioMode = SaiOutMode;
  haudio_out_sai.Init.NoDivider = SAI_MASTERDIVIDER_ENABLED;
  haudio_out_sai.Init.Protocol = SAI_FREE_PROTOCOL;
  haudio_out_sai.Init.DataSize = SAI_DATASIZE_16;
  haudio_out_sai.Init.FirstBit = SAI_FIRSTBIT_MSB;
  haudio_out_sai.Init.ClockStrobing = SAI_CLOCKSTROBING_RISINGEDGE;
  haudio_out_sai.Init.Synchro = SAI_ASYNCHRONOUS;
  haudio_out_sai.Init.OutputDrive = SAI_OUTPUTDRIVE_ENABLED;
  haudio_out_sai.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_1QF;

  /* Configure SAI_Block_x Frame
  Frame Length: 64
  Frame active Length: 32
  FS Definition: Start frame + Channel Side identification
  FS Polarity: FS active Low
  FS Offset: FS asserted one bit before the first bit of slot 0 */
  haudio_out_sai.FrameInit.FrameLength = 64;
  haudio_out_sai.FrameInit.ActiveFrameLength = 32;
  haudio_out_sai.FrameInit.FSDefinition = SAI_FS_CHANNEL_IDENTIFICATION;
  haudio_out_sai.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  haudio_out_sai.FrameInit.FSOffset = SAI_FS_BEFOREFIRSTBIT;

  /* Configure SAI Block_x Slot
  Slot First Bit Offset: 0
  Slot Size  : 16
  Slot Number: 4 */
  haudio_out_sai.SlotInit.FirstBitOffset = 0;
  haudio_out_sai.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
  haudio_out_sai.SlotInit.SlotNumber = 4;
  haudio_out_sai.SlotInit.SlotActive = CODEC_AUDIOFRAME_SLOT_02;

  HAL_SAI_Init(&haudio_out_sai);

  /* Initialize SAI2 block B in SLAVE RX synchronous from SAI2 block A */
  /* Initialize the haudio_in_sai Instance parameter */
  haudio_in_sai.Instance = AUDIO_IN_SAIx;
  
  /* Disable SAI peripheral to allow access to SAI internal registers */
  __HAL_SAI_DISABLE(&haudio_in_sai);
  
  /* Configure SAI_Block_x
  LSBFirst: Disabled
  DataSize: 16 */
  haudio_in_sai.Init.AudioFrequency = AudioFreq;
  haudio_in_sai.Init.AudioMode = SAI_MODESLAVE_RX;
  haudio_in_sai.Init.NoDivider = SAI_MASTERDIVIDER_ENABLED;
  haudio_in_sai.Init.Protocol = SAI_FREE_PROTOCOL;
  haudio_in_sai.Init.DataSize = SAI_DATASIZE_16;
  haudio_in_sai.Init.FirstBit = SAI_FIRSTBIT_MSB;
  haudio_in_sai.Init.ClockStrobing = SAI_CLOCKSTROBING_RISINGEDGE;
  haudio_in_sai.Init.Synchro = SAI_SYNCHRONOUS;
  haudio_in_sai.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLED;
  haudio_in_sai.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_1QF;
  
  /* Configure SAI_Block_x Frame
  Frame Length: 64
  Frame active Length: 32
  FS Definition: Start frame + Channel Side identification
  FS Polarity: FS active Low
  FS Offset: FS asserted one bit before the first bit of slot 0 */
  haudio_in_sai.FrameInit.FrameLength = 64;
  haudio_in_sai.FrameInit.ActiveFrameLength = 32;
  haudio_in_sai.FrameInit.FSDefinition = SAI_FS_CHANNEL_IDENTIFICATION;
  haudio_in_sai.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  haudio_in_sai.FrameInit.FSOffset = SAI_FS_BEFOREFIRSTBIT;
  
  /* Configure SAI Block_x Slot
  Slot First Bit Offset: 0
  Slot Size  : 16
  Slot Number: 4 */
  haudio_in_sai.SlotInit.FirstBitOffset = 0;
  haudio_in_sai.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
  haudio_in_sai.SlotInit.SlotNumber = 4;
  haudio_in_sai.SlotInit.SlotActive = SlotActive;

  HAL_SAI_Init(&haudio_in_sai);

  /* Enable SAI peripheral to generate MCLK */
  __HAL_SAI_ENABLE(&haudio_out_sai);

  /* Enable SAI peripheral */
  __HAL_SAI_ENABLE(&haudio_in_sai);
}



static void SAIx_In_Init_SAIinterrupt(uint32_t SaiOutMode, uint32_t SlotActive, uint32_t AudioFreq)
{
  /* Initialize SAI2 block A in MASTER TX */
  /* Initialize the haudio_out_sai Instance parameter */
  haudio_out_sai.Instance = AUDIO_OUT_SAIx; // as this not already (previously) done in IN_OUT_Init ?

  /* Disable SAI peripheral to allow access to SAI internal registers */
  __HAL_SAI_DISABLE(&haudio_out_sai);

  /* Configure SAI_Block_A
  LSBFirst: Disabled
  DataSize: 16 */
  haudio_out_sai.Init.AudioFrequency = AudioFreq;
  haudio_out_sai.Init.AudioMode = SaiOutMode; // not sure why SAI_MODEMASTER_TX is _passed_ to this function
  haudio_out_sai.Init.NoDivider = SAI_MASTERDIVIDER_ENABLED;
  haudio_out_sai.Init.Protocol = SAI_FREE_PROTOCOL;
  haudio_out_sai.Init.DataSize = SAI_DATASIZE_16;
  haudio_out_sai.Init.FirstBit = SAI_FIRSTBIT_MSB;
  haudio_out_sai.Init.ClockStrobing = SAI_CLOCKSTROBING_RISINGEDGE;
  haudio_out_sai.Init.Synchro = SAI_ASYNCHRONOUS;
  haudio_out_sai.Init.OutputDrive = SAI_OUTPUTDRIVE_ENABLED;
  haudio_out_sai.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY; // can we try empty later on? was 1QF

  /* Configure SAI_Block_A Frame
  Frame Length: 64
  Frame active Length: 32
  FS Definition: Start frame + Channel Side identification
  FS Polarity: FS active Low
  FS Offset: FS asserted one bit before the first bit of slot 0 */
  haudio_out_sai.FrameInit.FrameLength = 64;
  haudio_out_sai.FrameInit.ActiveFrameLength = 32;
  haudio_out_sai.FrameInit.FSDefinition = SAI_FS_CHANNEL_IDENTIFICATION;
  haudio_out_sai.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  haudio_out_sai.FrameInit.FSOffset = SAI_FS_BEFOREFIRSTBIT;

  /* Configure SAI Block_A Slot
  Slot First Bit Offset: 0
  Slot Size  : 16
  Slot Number: 4
  Slot Active: All slot actives */
  haudio_out_sai.SlotInit.FirstBitOffset = 0;
  haudio_out_sai.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
  haudio_out_sai.SlotInit.SlotNumber = 4;
  haudio_out_sai.SlotInit.SlotActive = CODEC_AUDIOFRAME_SLOT_02; // global variable SlotActive should be slots 0 and 2

  HAL_SAI_Init(&haudio_out_sai);

  /* Initialize SAI2 block B in SLAVE RX synchronous from SAI2 block A */
  /* Initialize the haudio_in_sai Instance parameter */
  haudio_in_sai.Instance = AUDIO_IN_SAIx; // again, hasn't this been done previously?
  
  /* Disable SAI peripheral to allow access to SAI internal registers */
  __HAL_SAI_DISABLE(&haudio_in_sai);
  
  /* Configure SAI_Block_B
  LSBFirst: Disabled
  DataSize: 16 */
  haudio_in_sai.Init.AudioFrequency = AudioFreq;
  haudio_in_sai.Init.AudioMode = SAI_MODESLAVE_RX;
  haudio_in_sai.Init.NoDivider = SAI_MASTERDIVIDER_ENABLED;
  haudio_in_sai.Init.Protocol = SAI_FREE_PROTOCOL;
  haudio_in_sai.Init.DataSize = SAI_DATASIZE_16;
  haudio_in_sai.Init.FirstBit = SAI_FIRSTBIT_MSB;
  haudio_in_sai.Init.ClockStrobing = SAI_CLOCKSTROBING_RISINGEDGE;
  haudio_in_sai.Init.Synchro = SAI_SYNCHRONOUS;
  haudio_in_sai.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLED;
  haudio_in_sai.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  
  /* Configure SAI_Block_B Frame
  Frame Length: 64
  Frame active Length: 32
  FS Definition: Start frame + Channel Side identification
  FS Polarity: FS active Low
  FS Offset: FS asserted one bit before the first bit of slot 0 */
  haudio_in_sai.FrameInit.FrameLength = 64;
  haudio_in_sai.FrameInit.ActiveFrameLength = 32;
  haudio_in_sai.FrameInit.FSDefinition = SAI_FS_CHANNEL_IDENTIFICATION;
  haudio_in_sai.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  haudio_in_sai.FrameInit.FSOffset = SAI_FS_BEFOREFIRSTBIT;
  
  /* Configure SAI Block_B Slot
  Slot First Bit Offset: 0
  Slot Size  : 16
  Slot Number: 4
  Slot Active: All slot active */
  haudio_in_sai.SlotInit.FirstBitOffset = 0;
  haudio_in_sai.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
  haudio_in_sai.SlotInit.SlotNumber = 4;
  haudio_in_sai.SlotInit.SlotActive = SlotActive;

  HAL_SAI_Init(&haudio_in_sai);
  /* Enable SAI peripheral */
  __HAL_SAI_ENABLE(&haudio_in_sai);

  /* Enable SAI peripheral to generate MCLK */
  __HAL_SAI_ENABLE(&haudio_out_sai);

}



static void SAIx_In_DeInit(void)
{
  /* Initialize the haudio_in_sai Instance parameter */
  haudio_in_sai.Instance = AUDIO_IN_SAIx;

  /* Disable SAI peripheral */
  __HAL_SAI_DISABLE(&haudio_in_sai);

  HAL_SAI_DeInit(&haudio_in_sai);
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
