/**
  ******************************************************************************
  * @file    stm32f7xx_hal_sai.c
  * @author  MCD Application Team
  * @version Modified 12 May 2017 DSR for multibuffer ping pong and again
	*          4 July 2017 to allow use with both DMA- and interrupt-based i/o
	*          Cut-down version doesn't have all of the options of original version
  * @date    30-December-2016
  * @brief   SAI HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the Serial Audio Interface (SAI) peripheral:
  *           + Initialization/de-initialization functions
  *           + I/O operation functions
  *           + Peripheral Control functions
  *           + Peripheral State functions
  *
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

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"


#ifdef HAL_SAI_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/

typedef enum {
  SAI_MODE_DMA,
  SAI_MODE_IT
}SAI_ModeTypedef;

/* Private define ------------------------------------------------------------*/

/** @defgroup SAI_Private_Constants  SAI Private Constants
  * @{
  */
// not sure whether this FIFO size is actually implemented - DSR 13 July
#define SAI_DEFAULT_TIMEOUT   4 /* 4ms */

static uint32_t SAI_InterruptFlag(SAI_HandleTypeDef *hsai, uint32_t mode);

static HAL_StatusTypeDef SAI_Disable(SAI_HandleTypeDef *hsai);
static void SAI_Receive_IT16Bit(SAI_HandleTypeDef *hsai_in, SAI_HandleTypeDef *hsai_out);

static void SAI_DMATxCplt(DMA_HandleTypeDef *hdma);
static void SAI_DMATxM1Cplt(DMA_HandleTypeDef *hdma);
static void SAI_DMARxCplt(DMA_HandleTypeDef *hdma);
static void SAI_DMARxM1Cplt(DMA_HandleTypeDef *hdma);
static void SAI_DMAError(DMA_HandleTypeDef *hdma);

/**
  * @brief  Initialize the SAI according to the specified parameters.
  *         in the SAI_InitTypeDef structure and initialize the associated handle.
  * @param  hsai: pointer to a SAI_HandleTypeDef structure that contains
  *                the configuration information for SAI module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SAI_Init(SAI_HandleTypeDef *hsai)
{
  uint32_t tmpregisterGCR = 0;
  uint32_t ckstr_bits = 0;
  uint32_t syncen_bits = 0;

  /* Check the SAI handle allocation */
  if(hsai == NULL)
  {
    return HAL_ERROR;
  }
  
  /* check the instance */
  assert_param(IS_SAI_ALL_INSTANCE(hsai->Instance));
  
  /* Check the SAI Block parameters */
  assert_param(IS_SAI_AUDIO_FREQUENCY(hsai->Init.AudioFrequency));
  assert_param(IS_SAI_BLOCK_PROTOCOL(hsai->Init.Protocol));
  assert_param(IS_SAI_BLOCK_MODE(hsai->Init.AudioMode));
  assert_param(IS_SAI_BLOCK_DATASIZE(hsai->Init.DataSize));
  assert_param(IS_SAI_BLOCK_FIRST_BIT(hsai->Init.FirstBit));
  assert_param(IS_SAI_BLOCK_CLOCK_STROBING(hsai->Init.ClockStrobing));
  assert_param(IS_SAI_BLOCK_SYNCHRO(hsai->Init.Synchro));
  assert_param(IS_SAI_BLOCK_OUTPUT_DRIVE(hsai->Init.OutputDrive));
  assert_param(IS_SAI_BLOCK_NODIVIDER(hsai->Init.NoDivider));
  assert_param(IS_SAI_BLOCK_FIFO_THRESHOLD(hsai->Init.FIFOThreshold));
  assert_param(IS_SAI_MONO_STEREO_MODE(hsai->Init.MonoStereoMode));
  assert_param(IS_SAI_BLOCK_COMPANDING_MODE(hsai->Init.CompandingMode));
  assert_param(IS_SAI_BLOCK_TRISTATE_MANAGEMENT(hsai->Init.TriState));
  assert_param(IS_SAI_BLOCK_SYNCEXT(hsai->Init.SynchroExt));
  
  /* Check the SAI Block Frame parameters */
  assert_param(IS_SAI_BLOCK_FRAME_LENGTH(hsai->FrameInit.FrameLength));
  assert_param(IS_SAI_BLOCK_ACTIVE_FRAME(hsai->FrameInit.ActiveFrameLength));
  assert_param(IS_SAI_BLOCK_FS_DEFINITION(hsai->FrameInit.FSDefinition));
  assert_param(IS_SAI_BLOCK_FS_POLARITY(hsai->FrameInit.FSPolarity));
  assert_param(IS_SAI_BLOCK_FS_OFFSET(hsai->FrameInit.FSOffset));
  
  /* Check the SAI Block Slot parameters */
  assert_param(IS_SAI_BLOCK_FIRSTBIT_OFFSET(hsai->SlotInit.FirstBitOffset));
  assert_param(IS_SAI_BLOCK_SLOT_SIZE(hsai->SlotInit.SlotSize));
  assert_param(IS_SAI_BLOCK_SLOT_NUMBER(hsai->SlotInit.SlotNumber));
  assert_param(IS_SAI_SLOT_ACTIVE(hsai->SlotInit.SlotActive));
  
  if(hsai->State == HAL_SAI_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    hsai->Lock = HAL_UNLOCKED;
    
    /* Init the low level hardware : GPIO, CLOCK, NVIC and DMA */
// function defined in this source file and does nothing - DSR 13 July
    HAL_SAI_MspInit(hsai);
  }
  
  hsai->State = HAL_SAI_STATE_BUSY;
  
  /* Disable the selected SAI peripheral */
  SAI_Disable(hsai);
  
  /* SAI Block Synchro Configuration -----------------------------------------*/
  /* This setting must be done with both audio block (A & B) disabled         */
  switch(hsai->Init.SynchroExt)
  {
    case SAI_SYNCEXT_DISABLE :
      tmpregisterGCR = 0;
      break;
    case SAI_SYNCEXT_OUTBLOCKA_ENABLE :
      tmpregisterGCR = SAI_GCR_SYNCOUT_0;
      break;
    case SAI_SYNCEXT_OUTBLOCKB_ENABLE :
      tmpregisterGCR = SAI_GCR_SYNCOUT_1;
      break;
  default:
    break;
  }
  
  switch(hsai->Init.Synchro)
  {
    case SAI_ASYNCHRONOUS :
      {
        syncen_bits = 0;
      }
      break;
    case SAI_SYNCHRONOUS :
      {
        syncen_bits = SAI_xCR1_SYNCEN_0;
      }
      break;
    case SAI_SYNCHRONOUS_EXT_SAI1 :
      {
        syncen_bits = SAI_xCR1_SYNCEN_1;
      }
      break;
    case SAI_SYNCHRONOUS_EXT_SAI2 :
      {
        syncen_bits = SAI_xCR1_SYNCEN_1;
        tmpregisterGCR |= SAI_GCR_SYNCIN_0;
      }
      break;
  default:
    break;      
  }

  if((hsai->Instance == SAI1_Block_A) || (hsai->Instance == SAI1_Block_B))
  {
    SAI1->GCR = tmpregisterGCR;
  }
  else 
  {
    SAI2->GCR = tmpregisterGCR;
  }

  if(hsai->Init.AudioFrequency != SAI_AUDIO_FREQUENCY_MCKDIV)
  {
    uint32_t freq = 0;
    uint32_t tmpval;

    if((hsai->Instance == SAI1_Block_A ) || (hsai->Instance == SAI1_Block_B ))
    {
      freq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SAI1);
    }
    if((hsai->Instance == SAI2_Block_A ) || (hsai->Instance == SAI2_Block_B ))
    {
      freq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SAI2);
    }
    
    /* Configure Master Clock using the following formula :
       MCLK_x = SAI_CK_x / (MCKDIV[3:0] * 2) with MCLK_x = 256 * FS
       FS = SAI_CK_x / (MCKDIV[3:0] * 2) * 256
       MCKDIV[3:0] = SAI_CK_x / FS * 512 */
    /* (freq x 10) to keep Significant digits */
    tmpval = (freq * 10) / (hsai->Init.AudioFrequency * 2 * 256);
    hsai->Init.Mckdiv = tmpval / 10;
    
    /* Round result to the nearest integer */
    if((tmpval % 10) > 8)
    {
      hsai->Init.Mckdiv+= 1;
    }
  }
  
  /* Compute CKSTR bits of SAI CR1 according ClockStrobing and AudioMode */
  if((hsai->Init.AudioMode == SAI_MODEMASTER_TX) || (hsai->Init.AudioMode == SAI_MODESLAVE_TX))
  { /* Transmit */
    ckstr_bits = (hsai->Init.ClockStrobing == SAI_CLOCKSTROBING_RISINGEDGE) ? 0 : SAI_xCR1_CKSTR;
  }
  else
  { /* Receive */
    ckstr_bits = (hsai->Init.ClockStrobing == SAI_CLOCKSTROBING_RISINGEDGE) ? SAI_xCR1_CKSTR : 0;
  }
  
  /* SAI Block Configuration -------------------------------------------------*/
  /* SAI CR1 Configuration */
  hsai->Instance->CR1&=~(SAI_xCR1_MODE | SAI_xCR1_PRTCFG |  SAI_xCR1_DS |      \
                         SAI_xCR1_LSBFIRST | SAI_xCR1_CKSTR | SAI_xCR1_SYNCEN |\
                         SAI_xCR1_MONO | SAI_xCR1_OUTDRIV  | SAI_xCR1_DMAEN |  \
                         SAI_xCR1_NODIV | SAI_xCR1_MCKDIV);
  
  hsai->Instance->CR1|=(hsai->Init.AudioMode | hsai->Init.Protocol |           \
                        hsai->Init.DataSize | hsai->Init.FirstBit  |           \
                        ckstr_bits | syncen_bits |                               \
                        hsai->Init.MonoStereoMode | hsai->Init.OutputDrive |   \
                        hsai->Init.NoDivider | (hsai->Init.Mckdiv << 20));
  
  /* SAI CR2 Configuration */
  hsai->Instance->CR2&= ~(SAI_xCR2_FTH | SAI_xCR2_FFLUSH | SAI_xCR2_COMP | SAI_xCR2_CPL);
  hsai->Instance->CR2|=  (hsai->Init.FIFOThreshold | hsai->Init.CompandingMode | hsai->Init.TriState);
  
  /* SAI Frame Configuration -----------------------------------------*/
  hsai->Instance->FRCR&=(~(SAI_xFRCR_FRL | SAI_xFRCR_FSALL | SAI_xFRCR_FSDEF | \
                           SAI_xFRCR_FSPOL | SAI_xFRCR_FSOFF));
  hsai->Instance->FRCR|=((hsai->FrameInit.FrameLength - 1) |
                          hsai->FrameInit.FSOffset |
                          hsai->FrameInit.FSDefinition |
                          hsai->FrameInit.FSPolarity   |
                          ((hsai->FrameInit.ActiveFrameLength - 1) << 8));
  
  /* SAI Block_x SLOT Configuration ------------------------------------------*/
  /* This register has no meaning in AC 97 and SPDIF audio protocol */
  hsai->Instance->SLOTR&= (~(SAI_xSLOTR_FBOFF | SAI_xSLOTR_SLOTSZ |            \
                             SAI_xSLOTR_NBSLOT | SAI_xSLOTR_SLOTEN ));
  
  hsai->Instance->SLOTR|=  hsai->SlotInit.FirstBitOffset |  hsai->SlotInit.SlotSize
                          | (hsai->SlotInit.SlotActive << 16) | ((hsai->SlotInit.SlotNumber - 1) <<  8);
  
  /* Initialize the error code */
  hsai->ErrorCode = HAL_SAI_ERROR_NONE;
  
  /* Initialize the SAI state */
  hsai->State= HAL_SAI_STATE_READY;
  
  /* Release Lock */
  __HAL_UNLOCK(hsai);
  
  return HAL_OK;
}

/**
  * @brief  DeInitialize the SAI peripheral.
  * @param  hsai: pointer to a SAI_HandleTypeDef structure that contains
  *                the configuration information for SAI module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SAI_DeInit(SAI_HandleTypeDef *hsai)
{
  /* Check the SAI handle allocation */
  if(hsai == NULL)
  {
    return HAL_ERROR;
  }

  hsai->State = HAL_SAI_STATE_BUSY;

  /* Disabled All interrupt and clear all the flag */
  hsai->Instance->IMR = 0;
  hsai->Instance->CLRFR = 0xFFFFFFFFU;
  
  /* Disable the SAI */
  SAI_Disable(hsai);

  /* Flush the fifo */
  SET_BIT(hsai->Instance->CR2, SAI_xCR2_FFLUSH);
  
  /* DeInit the low level hardware: GPIO, CLOCK, NVIC and DMA */
  HAL_SAI_MspDeInit(hsai);

  /* Initialize the error code */
  hsai->ErrorCode = HAL_SAI_ERROR_NONE;

  /* Initialize the SAI state */
  hsai->State = HAL_SAI_STATE_RESET;

  /* Release Lock */
  __HAL_UNLOCK(hsai);

  return HAL_OK;
}

/**
  * @brief Initialize the SAI MSP.
  * @param  hsai: pointer to a SAI_HandleTypeDef structure that contains
  *                the configuration information for SAI module.
  * @retval None
  */
__weak void HAL_SAI_MspInit(SAI_HandleTypeDef *hsai)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hsai);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_SAI_MspInit could be implemented in the user file
   */
}

/**
  * @brief DeInitialize the SAI MSP.
  * @param  hsai: pointer to a SAI_HandleTypeDef structure that contains
  *                the configuration information for SAI module.
  * @retval None
  */
__weak void HAL_SAI_MspDeInit(SAI_HandleTypeDef *hsai)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hsai);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_SAI_MspDeInit could be implemented in the user file
   */
}


/**
  * Modified 29 May 2017 DSR for interrupt-based operation
  * @brief  initialises SAI with interrupt service routine for RX block B and enables
  * @param  none 
  * @retval AUDIO_OK if correct communication, else wrong communication
  */
HAL_StatusTypeDef HAL_SAI_Receive_IT_SAIinterrupt(SAI_HandleTypeDef *hsai_in, SAI_HandleTypeDef *hsai_out)

{
  hsai_out->ErrorCode = HAL_SAI_ERROR_NONE;
  hsai_out->State = HAL_SAI_STATE_BUSY_TX;
  hsai_in->ErrorCode = HAL_SAI_ERROR_NONE;
  hsai_in->State = HAL_SAI_STATE_BUSY_RX;

  hsai_in->NewInterruptServiceRoutine = SAI_Receive_IT16Bit;
  hsai_out->NewInterruptServiceRoutine = SAI_Receive_IT16Bit;
  /* Enable FRQ and OVRUDR interrupts */
 	__HAL_SAI_ENABLE_IT(hsai_out, SAI_InterruptFlag(hsai_out, SAI_MODE_IT)); // temp DSR
  /* Enable SAI peripheral blocks */
  __HAL_SAI_ENABLE(hsai_in); // does this set SAIEN bit in CR??? DSR
  __HAL_SAI_ENABLE(hsai_out); // does this set SAIEN bit in CR??? DSR
  return HAL_OK;
}


/**
  * @brief  Handle SAI interrupt request.
  * @param  hsai: pointer to a SAI_HandleTypeDef structure that contains
  *                the configuration information for SAI module.
  * @retval None
  */
// function modified by DSR - June 2017
void HAL_SAI_IRQHandler(SAI_HandleTypeDef *hsai_in, SAI_HandleTypeDef *hsai_out)
{
// check out hsai_out (TX SAI2 Block A)
	if(hsai_out->State != HAL_SAI_STATE_RESET)
  {
    uint32_t itflags = hsai_out->Instance->SR;
    uint32_t itsources = hsai_out->Instance->IMR;
    uint32_t cr1config = hsai_out->Instance->CR1;    
    uint32_t tmperror;

    /* SAI Fifo request interrupt occured ------------------------------------*/
    if(((itflags & SAI_xSR_FREQ) == SAI_xSR_FREQ) && ((itsources & SAI_IT_FREQ) == SAI_IT_FREQ))
    {
      hsai_out->NewInterruptServiceRoutine(hsai_in, hsai_out);
    }
    /* SAI Overrun error interrupt occurred ----------------------------------*/
    else if(((itflags & SAI_FLAG_OVRUDR) == SAI_FLAG_OVRUDR) && ((itsources & SAI_IT_OVRUDR) == SAI_IT_OVRUDR))
    {
      /* Clear the SAI Overrun flag */
      __HAL_SAI_CLEAR_FLAG(hsai_out, SAI_FLAG_OVRUDR);
      /* Get the SAI error code */
      tmperror = ((hsai_out->State == HAL_SAI_STATE_BUSY_RX) ? HAL_SAI_ERROR_OVR : HAL_SAI_ERROR_UDR);      
      /* Change the SAI error code */
      hsai_out->ErrorCode |= tmperror;
      /* the transfer is not stopped, we will forward the information to the user and we let the user decide what needs to be done */
      HAL_SAI_ErrorCallback(hsai_out);
    }
    else
    {
      /* Nothing to do */
    }
  }
}


/*
  New function for multibuffer DSR 12 May 2017
*/
HAL_StatusTypeDef HAL_SAI_Transmit_DMA_MultiBuffer(SAI_HandleTypeDef *hsai, uint8_t *pDataM0, uint8_t *pDataM1, uint16_t Size)
{
  if((pDataM0 == NULL) || (Size == 0))
  {
    return  HAL_ERROR;
  }

  if(hsai->State == HAL_SAI_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hsai);

    hsai->pBuffPtr = pDataM0; // only M0 address entered into hsai SAI_HandleTypeDef structure field !!!
    hsai->XferSize = Size;
    hsai->XferCount = Size;
    hsai->ErrorCode = HAL_SAI_ERROR_NONE;
    hsai->State = HAL_SAI_STATE_BUSY_TX;

    /* Set the SAI Tx DMA Half transfer complete callback - possibly make this NULL DSR ??? */
//    hsai->hdmatx->XferHalfCpltCallback = SAI_DMATxHalfCplt;
    hsai->hdmatx->XferHalfCpltCallback = NULL; // i.e. do not use half complete event/interrupt/callback

    /* Set the SAI Tx M1 DMA Half transfer complete callback - possibly make this NULL DSR ??? */
//    hsai->hdmatx->XferM1HalfCpltCallback = SAI_DMATxHalfCplt;
    hsai->hdmatx->XferM1HalfCpltCallback = NULL; // i.e. do not use half complete event/interrupt/callback


    /* Set the SAI TxDMA transfer M0 complete callback */
    hsai->hdmatx->XferCpltCallback = SAI_DMATxCplt;

    /* Set the SAI TxDMA transfer M1 complete callback - added DSR */
    hsai->hdmatx->XferM1CpltCallback = SAI_DMATxM1Cplt;

    /* Set the DMA error callback */
    hsai->hdmatx->XferErrorCallback = SAI_DMAError;

    /* Set the DMA Tx abort callback */
    hsai->hdmatx->XferAbortCallback = NULL;

    /* Enable the Tx DMA Stream for multibuffer operation DSR */
    /* possible error passing 32-bit pointer pDataM1 ??? */
    if(HAL_DMAEx_MultiBufferStart_IT(hsai->hdmatx, (uint32_t)hsai->pBuffPtr, (uint32_t)&hsai->Instance->DR, (uint32_t)pDataM1, hsai->XferSize) != HAL_OK)
    {
      __HAL_UNLOCK(hsai);
      return  HAL_ERROR;
    }

    /* Check if the SAI is already enabled */
    if((hsai->Instance->CR1 & SAI_xCR1_SAIEN) == RESET)
    {
      /* Enable SAI peripheral */
      __HAL_SAI_ENABLE(hsai); // sets the CR1 SAIEN bit
    }

    /* Enable the interrupts for error handling */
    __HAL_SAI_ENABLE_IT(hsai, SAI_InterruptFlag(hsai, SAI_MODE_DMA));

    /* Enable SAI Tx DMA Request */
    hsai->Instance->CR1 |= SAI_xCR1_DMAEN;

    /* Process Unlocked */
    __HAL_UNLOCK(hsai);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * modified by DSR for multibuffer 12 May 2017
  * @brief  Receive an amount of data in non-blocking mode with DMA.
  * @param  hsai: pointer to a SAI_HandleTypeDef structure that contains
  *                the configuration information for SAI module.
  * @param  pData: Pointer to data buffer
  * @param  Size: Amount of data to be received
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SAI_Receive_DMA_MultiBuffer(SAI_HandleTypeDef *hsai, uint8_t *pDataM0, uint8_t *pDataM1, uint16_t Size)
{

  if((pDataM0 == NULL) || (Size == 0))
  {
    return  HAL_ERROR;
  }

  if(hsai->State == HAL_SAI_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hsai);

    hsai->pBuffPtr = pDataM0;
    hsai->XferSize = Size;
    hsai->XferCount = Size;
    hsai->ErrorCode = HAL_SAI_ERROR_NONE;
    hsai->State = HAL_SAI_STATE_BUSY_RX;

    /* Set the SAI Rx DMA Half transfer complete callback */
    /* set half transfer callbacks to NULL ??? DSR */
//    hsai->hdmarx->XferHalfCpltCallback = SAI_DMARxHalfCplt;
    hsai->hdmarx->XferHalfCpltCallback = NULL; // i.e. do not use half complete event/interrupt/callback

    /* Set the SAI Rx M1 DMA Half transfer complete callback */
    /* set half transfer callbacks to NULL ??? DSR */
//    hsai->hdmarx->XferM1HalfCpltCallback = SAI_DMARxHalfCplt;
    hsai->hdmarx->XferM1HalfCpltCallback = NULL; // i.e. do not use half complete event/interrupt/callback


    /* Set the SAI Rx DMA transfer complete callback */
    hsai->hdmarx->XferCpltCallback = SAI_DMARxCplt;

    /* Set the SAI Rx DMA transfer M1 complete callback - new DSR */
    hsai->hdmarx->XferM1CpltCallback = SAI_DMARxM1Cplt;

    /* Set the DMA error callback */
    hsai->hdmarx->XferErrorCallback = SAI_DMAError;

    /* Set the DMA Rx abort callback */
    hsai->hdmarx->XferAbortCallback = NULL;

    /* Enable the Rx DMA Stream for multibuffer operation DSR 17 May 2017 */
    /* possible problem with 32-bit pointer to pDataM1 ??? */
		// apparently not
		// XferSize goes to NDTR register, it is the number of items in DMA transfer block
		// an item is a 16-bit 'half-word' as defined  by MSIZE and PSIZE
    if(HAL_DMAEx_MultiBufferStart_IT(hsai->hdmarx, (uint32_t)&hsai->Instance->DR, (uint32_t)hsai->pBuffPtr, (uint32_t)pDataM1, hsai->XferSize) != HAL_OK)
    {
      __HAL_UNLOCK(hsai);
      return  HAL_ERROR;
    }

    /* Check if the SAI is already enabled */
    if((hsai->Instance->CR1 & SAI_xCR1_SAIEN) == RESET)
    {
      /* Enable SAI peripheral */
      __HAL_SAI_ENABLE(hsai); // sets the CR1 SAIEN bit
    }

    /* Enable the interrupts for error handling */
    __HAL_SAI_ENABLE_IT(hsai, SAI_InterruptFlag(hsai, SAI_MODE_DMA));

    /* Enable SAI Rx DMA Request */
    hsai->Instance->CR1 |= SAI_xCR1_DMAEN;

    /* Process Unlocked */
    __HAL_UNLOCK(hsai);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Handle SAI interrupt request.
  * @param  hsai: pointer to a SAI_HandleTypeDef structure that contains
  *                the configuration information for SAI module.
  * @retval None
  */

/**
  * @brief Tx Transfer completed callback.
  * @param  hsai: pointer to a SAI_HandleTypeDef structure that contains
  *                the configuration information for SAI module.
  * @retval None
  */
// function defined in stm32f746g_discovery_audio.c
__weak void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{
  UNUSED(hsai);
}

/**
  * added DSR for multibuffer DMA
  * @brief Tx Transfer completed callback.
  * @param  hsai: pointer to a SAI_HandleTypeDef structure that contains
  *                the configuration information for SAI module.
  * @retval None
  */
// function defined in stm32f746g_discovery_audio.c
__weak void HAL_SAI_TxM1CpltCallback(SAI_HandleTypeDef *hsai)
{
  UNUSED(hsai);
}

/**
  * @brief Rx Transfer completed callback.
  * @param  hsai: pointer to a SAI_HandleTypeDef structure that contains
  *                the configuration information for SAI module.
  * @retval None
  */
// function defined in stm32f746g_discovery_audio.c
__weak void HAL_SAI_RxCpltCallback_Interrupt()
{
}

/**
  * added DSR for multibuffer
  * @brief Rx Transfer completed callback.
  * @param  hsai: pointer to a SAI_HandleTypeDef structure that contains
  *                the configuration information for SAI module.
  * @retval None
  */
// function defined in stm32f746g_discovery_audio.c
__weak void HAL_SAI_RxM1CpltCallback(SAI_HandleTypeDef *hsai)
{
  UNUSED(hsai);
}

/**
  * @brief SAI error callback.
  * @param  hsai: pointer to a SAI_HandleTypeDef structure that contains
  *                the configuration information for SAI module.
  * @retval None
  */
__weak void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai)
{
  UNUSED(hsai);
}




/**
  * @brief  Return the SAI handle state.
  * @param  hsai: pointer to a SAI_HandleTypeDef structure that contains
  *                the configuration information for SAI module.
  * @retval HAL state
  */
HAL_SAI_StateTypeDef HAL_SAI_GetState(SAI_HandleTypeDef *hsai)
{
  return hsai->State;
}

/**
* @brief  Return the SAI error code.
* @param  hsai: pointer to a SAI_HandleTypeDef structure that contains
  *              the configuration information for the specified SAI Block.
* @retval SAI Error Code
*/
uint32_t HAL_SAI_GetError(SAI_HandleTypeDef *hsai)
{
  return hsai->ErrorCode;
}


/**
  * @brief  Return the interrupt flag to set according the SAI setup.
  * @param  hsai: pointer to a SAI_HandleTypeDef structure that contains
  *                the configuration information for SAI module.
  * @param  mode: SAI_MODE_DMA or SAI_MODE_IT
  * @retval the list of the IT flag to enable
 */
static uint32_t SAI_InterruptFlag(SAI_HandleTypeDef *hsai, uint32_t mode)
{
  uint32_t tmpIT = SAI_IT_OVRUDR;
  
  if(mode == SAI_MODE_IT)
  {
    tmpIT|= SAI_IT_FREQ;
  }

  if((hsai->Init.Protocol == SAI_AC97_PROTOCOL) &&
    ((hsai->Init.AudioMode == SAI_MODESLAVE_RX) || (hsai->Init.AudioMode == SAI_MODEMASTER_RX)))
  {
    tmpIT|= SAI_IT_CNRDY;
  }

  if((hsai->Init.AudioMode == SAI_MODESLAVE_RX) || (hsai->Init.AudioMode == SAI_MODESLAVE_TX))
  {
    tmpIT|= SAI_IT_AFSDET | SAI_IT_LFSDET;
  }
  else
  {
    /* hsai has been configured in master mode */
    tmpIT|= SAI_IT_WCKCFG;
  }
  return tmpIT;
}



/**
  * @brief  Disable the SAI and wait for the disabling.
  * @param  hsai : pointer to a SAI_HandleTypeDef structure that contains
  *                the configuration information for SAI module.
  * @retval None
  */
static HAL_StatusTypeDef SAI_Disable(SAI_HandleTypeDef *hsai)
{
  register uint32_t count = SAI_DEFAULT_TIMEOUT * (SystemCoreClock /7/1000);
  HAL_StatusTypeDef status = HAL_OK;

  /* Disable the SAI instance */
  __HAL_SAI_DISABLE(hsai);

  do 
  {
    /* Check for the Timeout */
    if (count-- == 0)
    {         
      /* Update error code */
      hsai->ErrorCode |= HAL_SAI_ERROR_TIMEOUT;
      status = HAL_TIMEOUT;
      break;
    }
  } while((hsai->Instance->CR1 & SAI_xCR1_SAIEN) != RESET);

  return status;
}


/**
  * @brief  Rx Handler for Receive in Interrupt mode for 16-Bit transfer.
  * @param  hsai: pointer to a SAI_HandleTypeDef structure that contains
  *                the configuration information for SAI module.
  * @retval None
  */
// function modified from that of same name DSR June 2017
// used in interrupt-based i/o and not for DMA
static void SAI_Receive_IT16Bit(SAI_HandleTypeDef *hsai_in, SAI_HandleTypeDef *hsai_out)
{
  /* Receive data and place in global variables pointed to from structure */
// read two 16-bit values from the DR in the RX block of the SAI 
  *hsai_in->sample_right_ptr = hsai_in->Instance->DR;
  *hsai_in->sample_left_ptr = hsai_in->Instance->DR;
  /* Disable FREQ and OVRUDR interrupts */
  __HAL_SAI_DISABLE_IT(hsai_out, SAI_InterruptFlag(hsai_out, SAI_MODE_IT)); // changed to TX 31 May
    /* Clear the SAI Overrun flag */
  __HAL_SAI_CLEAR_FLAG(hsai_out, SAI_FLAG_OVRUDR); // changed to TX 31 May
// this routine, defined ultimately in stm32f746g_discovery_audio.c, is effectively the ISR
// the Rx in the function name is possibly superfluous or even misleading since interrupts
// are generated by TX block in SAI but both TX and RX are handled in the ISR
  HAL_SAI_RxCpltCallback_Interrupt();
	
  /* transmit data held in global variables pointed to from structure */
// write two 16-bit values to the DR in the TX block of the SAI 
	hsai_out->Instance->DR = *hsai_out->sample_right_ptr;
  hsai_out->Instance->DR = *hsai_out->sample_left_ptr;

  hsai_in->State = HAL_SAI_STATE_READY; // does this have any meaning to us? DSR
  __HAL_SAI_ENABLE_IT(hsai_out, SAI_InterruptFlag(hsai_out, SAI_MODE_IT));
}


/**
  * @brief DMA SAI transmit process complete callback.
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
static void SAI_DMATxCplt(DMA_HandleTypeDef *hdma)
{
  SAI_HandleTypeDef* hsai = (SAI_HandleTypeDef*)((DMA_HandleTypeDef* )hdma)->Parent;

  if((hdma->Instance->CR & DMA_SxCR_CIRC) == 0)
  {
    hsai->XferCount = 0;
    
    /* Disable SAI Tx DMA Request */
    hsai->Instance->CR1 &= (uint32_t)(~SAI_xCR1_DMAEN);

    /* Stop the interrupts error handling */
    __HAL_SAI_DISABLE_IT(hsai, SAI_InterruptFlag(hsai, SAI_MODE_DMA));
    
    hsai->State= HAL_SAI_STATE_READY;
  }
  HAL_SAI_TxCpltCallback(hsai);
}

/**
  * added 17 May 2017 DSR for multibuffer
  * @brief DMA SAI transmit process complete callback.
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
static void SAI_DMATxM1Cplt(DMA_HandleTypeDef *hdma)
{
  SAI_HandleTypeDef* hsai = (SAI_HandleTypeDef*)((DMA_HandleTypeDef* )hdma)->Parent;

  if((hdma->Instance->CR & DMA_SxCR_CIRC) == 0)
  {
    hsai->XferCount = 0;
    
    /* Disable SAI Tx DMA Request */
    hsai->Instance->CR1 &= (uint32_t)(~SAI_xCR1_DMAEN);

    /* Stop the interrupts error handling */
    __HAL_SAI_DISABLE_IT(hsai, SAI_InterruptFlag(hsai, SAI_MODE_DMA));
    
    hsai->State= HAL_SAI_STATE_READY;
  }
  HAL_SAI_TxM1CpltCallback(hsai);
}


/**
  * @brief DMA SAI receive process complete callback.
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
static void SAI_DMARxCplt(DMA_HandleTypeDef *hdma)
{
  SAI_HandleTypeDef* hsai = ( SAI_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;
  if((hdma->Instance->CR & DMA_SxCR_CIRC) == 0)
  {
    /* Disable Rx DMA Request */
    hsai->Instance->CR1 &= (uint32_t)(~SAI_xCR1_DMAEN);
    hsai->XferCount = 0;

    /* Stop the interrupts error handling */
    __HAL_SAI_DISABLE_IT(hsai, SAI_InterruptFlag(hsai, SAI_MODE_DMA));
    
    hsai->State = HAL_SAI_STATE_READY;
  }
  HAL_SAI_RxCpltCallback();
}


/**
  * added 17 May 2017 DSR for multibuffer
  * @brief DMA SAI receive process complete callback.
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
static void SAI_DMARxM1Cplt(DMA_HandleTypeDef *hdma)
{
  SAI_HandleTypeDef* hsai = ( SAI_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;
  if((hdma->Instance->CR & DMA_SxCR_CIRC) == 0)
  {
    /* Disable Rx DMA Request */
    hsai->Instance->CR1 &= (uint32_t)(~SAI_xCR1_DMAEN);
    hsai->XferCount = 0;

    /* Stop the interrupts error handling */
    __HAL_SAI_DISABLE_IT(hsai, SAI_InterruptFlag(hsai, SAI_MODE_DMA));
    
    hsai->State = HAL_SAI_STATE_READY;
  }
  HAL_SAI_RxM1CpltCallback(hsai);
}

/**
  * @brief DMA SAI communication error callback.
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
static void SAI_DMAError(DMA_HandleTypeDef *hdma)
{
  SAI_HandleTypeDef* hsai = ( SAI_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;

  /* Set SAI error code */
  hsai->ErrorCode |= HAL_SAI_ERROR_DMA;

  if((hsai->hdmatx->ErrorCode == HAL_DMA_ERROR_TE) || (hsai->hdmarx->ErrorCode == HAL_DMA_ERROR_TE))
  {
    /* Disable the SAI DMA request */
    hsai->Instance->CR1 &= ~SAI_xCR1_DMAEN;

    /* Disable SAI peripheral */
    SAI_Disable(hsai);
    
    /* Set the SAI state ready to be able to start again the process */
    hsai->State = HAL_SAI_STATE_READY;

    /* Initialize XferCount */
    hsai->XferCount = 0U;
  }
  /* SAI error Callback */ 
  HAL_SAI_ErrorCallback(hsai);
}


#endif /* HAL_SAI_MODULE_ENABLED */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
