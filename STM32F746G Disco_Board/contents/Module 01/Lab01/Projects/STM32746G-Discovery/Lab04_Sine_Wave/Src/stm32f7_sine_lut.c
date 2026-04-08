/* Includes ------------------------------------------------------------------*/
#include "stm32f7_sine_lut.h"    // your board/HAL declarations

/* Private defines -----------------------------------------------------------*/
#define SOURCE_FILE_NAME     "stm32f7_sine_lut.c"
#define AUDIO_FREQ           8000u
#define LOOPLENGTH           8u

/* Private variables ---------------------------------------------------------*/
static __IO uint8_t  PlayComplete = 0;
static int16_t sine_table[LOOPLENGTH] = {0, 7071, 10000, 7071, 0, -7071, -10000, -7071};
static int16_t stereo_buf[LOOPLENGTH * 2];

/* Private function prototypes -----------------------------------------------*/
static void MPU_Config(void);
static void SystemClock_Config(void);
static void CPU_CACHE_Enable(void);
static void Error_Handler(void);

int main(void)
{
    /* Configure MPU, enable cache, HAL init, system clock */
    MPU_Config();
    CPU_CACHE_Enable();
    HAL_Init();
    SystemClock_Config();

    /* LCD feedback */
    stm32f7_LCD_init(AUDIO_FREQ, SOURCE_FILE_NAME, GRAPH);

    /* Plot the raw 8-sample LUT on the LCD */
    plotSamples(sine_table, LOOPLENGTH, 32);

    /* Build interleaved stereo buffer */
    for (uint32_t i = 0; i < LOOPLENGTH; i++){
        stereo_buf[2*i] = sine_table[i];  		// left slot
        stereo_buf[2*i + 1] = sine_table[i];  // right slot
    }

    /* Init audio out @8 kHz */
    if (BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_HEADPHONE, 70, AUDIO_FREQ) != AUDIO_OK)
        Error_Handler();

    /* Force 2-slot (mono/stereo) mode */
    BSP_AUDIO_OUT_SetAudioFrameSlot(CODEC_AUDIOFRAME_SLOT_02);

    /* Play the 16-sample (8�2) buffer = 8 frames ? 1 kHz tone */
    if (BSP_AUDIO_OUT_Play((uint16_t*)stereo_buf, LOOPLENGTH * 2 * sizeof(int16_t)) != AUDIO_OK){
			Error_Handler();
		}

    while (1){
			if (PlayComplete){ 
				PlayComplete = 0; 
			} 
		}
}

void BSP_AUDIO_OUT_TransferComplete_CallBack(void)
{
    PlayComplete = 1;
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 216000000
  *            HCLK(Hz)                       = 216000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 25
  *            PLL_N                          = 432
  *            PLL_P                          = 2
  *            PLL_Q                          = 9
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 7
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;  
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* activate the OverDrive to reach the 216 Mhz Frequency */
  if(HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/* Override the weak clock-config to add 8 kHz support */
void BSP_AUDIO_OUT_ClockConfig(SAI_HandleTypeDef *hsai, uint32_t AudioFreq, void *Params)
{
    RCC_PeriphCLKInitTypeDef clkcfg;
    HAL_RCCEx_GetPeriphCLKConfig(&clkcfg);

    /* Always drive SAI2 from PLLI2S */
    clkcfg.PeriphClockSelection = RCC_PERIPHCLK_SAI2;
    clkcfg.Sai2ClockSelection    = RCC_SAI2CLKSOURCE_PLLI2S;

    if (AudioFreq == AUDIO_FREQUENCY_8K)
    {
        /* Want MCLK = 8 kHz � 256 = 2.048 MHz
           PLLI2S VCO = (HSE/PLLM)�PLLI2SN = 25 MHz/25�256 = 256 MHz
           first-level  = VCO/PLLI2SQ     = 256/5   = 51.2 MHz
           final MCLK   = 51.2 MHz/25     = 2.048 MHz */
        clkcfg.PLLI2S.PLLI2SN    = 256;
        clkcfg.PLLI2S.PLLI2SQ    = 5;
        clkcfg.PLLI2SDivQ        = 25;
    }
    else if ((AudioFreq == AUDIO_FREQUENCY_11K) ||
             (AudioFreq == AUDIO_FREQUENCY_22K) ||
             (AudioFreq == AUDIO_FREQUENCY_44K))
    {
        /* ST defaults for 11/22/44 kHz */
        clkcfg.PLLI2S.PLLI2SN    = 429;
        clkcfg.PLLI2S.PLLI2SQ    = 2;
        clkcfg.PLLI2SDivQ        = 19;
    }

    HAL_RCCEx_PeriphCLKConfig(&clkcfg);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1)
  {
		BSP_LED_Toggle(LED1);
  }
}

/**
  * @brief  CPU L1-Cache enable.
  * @param  None
  * @retval None
  */
static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}


/**
  * @brief  Configure the MPU attributes
  * @param  None
  * @retval None
  */
static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct;

  /* Disable the MPU */
  HAL_MPU_Disable();

  /* Configure the MPU as Strongly ordered for not defined regions */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0x20010000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_256KB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Enable the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

#ifdef  USE_FULL_ASSERT

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
#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

