// stm32f7_wm8994_init.h

// much of this was main.h - I'm not sure that all of it is needed !!

#include "stdio.h"
#include "stm32746g_discovery.h"
#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_sdram.h"
#include "stm32746g_discovery_audio.h"


#include <stdio.h>
#include "string.h"

//#define ARM_MATH_CM7 // no need for this if defined in target options C/C++ 
#include "arm_math.h"
#include "arm_const_structs.h"


#define IO_METHOD_INTR 0
#define IO_METHOD_DMA 1

// the following are digital PGA gains and attenuations
// implemented within the AIF

// write these to registers 0x1C and 0x1D
#define WM8994_HP_OUT_ANALOG_GAIN_0DB 0x39
#define WM8994_HP_OUT_ANALOG_GAIN_3DB 0x3C
#define WM8994_HP_OUT_ANALOG_GAIN_6DB 0x3F
#define WM8994_HP_OUT_ANALOG_ATTEN_3DB 0x36
#define WM8994_HP_OUT_ANALOG_ATTEN_6DB 0x33
#define WM8994_HP_OUT_ANALOG_ATTEN_9DB 0x30

// write these to registers 0x402 and 0x403
#define WM8994_HP_OUT_GAIN_MUTE 0x00
#define WM8994_HP_OUT_GAIN_0DB 0xC0
#define WM8994_HP_OUT_GAIN_3DB 0xC8
#define WM8994_HP_OUT_GAIN_6DB 0xD0
#define WM8994_HP_OUT_GAIN_9DB 0xD8
#define WM8994_HP_OUT_ATTEN_3DB 0xB8
#define WM8994_HP_OUT_ATTEN_6DB 0xB0
#define WM8994_HP_OUT_ATTEN_9DB 0xA8

// write these to registers 0x404 and 0x405
#define WM8994_DMIC_GAIN_MUTE 0x00
#define WM8994_DMIC_GAIN_0DB 0xC0
#define WM8994_DMIC_GAIN_3DB 0xC8
#define WM8994_DMIC_GAIN_6DB 0xD0
#define WM8994_DMIC_GAIN_9DB 0xD8
#define WM8994_DMIC_GAIN_17DB 0xEF
#define WM8994_DMIC_ATTEN_3DB 0xB8
#define WM8994_DMIC_ATTEN_6DB 0xB0
#define WM8994_DMIC_ATTEN_9DB 0xA8

// write these to registers 0x400 and 0x401
#define WM8994_LINE_IN_GAIN_MUTE 0x00
#define WM8994_LINE_IN_GAIN_0DB 0xC0
#define WM8994_LINE_IN_GAIN_3DB 0xC8
#define WM8994_LINE_IN_GAIN_6DB 0xD0
#define WM8994_LINE_IN_GAIN_9DB 0xD8
#define WM8994_LINE_IN_ATTEN_3DB 0xB8
#define WM8994_LINE_IN_ATTEN_6DB 0xB0
#define WM8994_LINE_IN_ATTEN_9DB 0xA8

// the following are analogue PGA gains and attenuations
// currently, function wm8994_SetVolume() sets these to
// 0dB

// write these to registers 0x18 and 0x1A
#define WM8994_LINE_IN_ANALOG_GAIN_0DB 0x0B
#define WM8994_LINE_IN_ANALOG_GAIN_3DB 0x0D
#define WM8994_LINE_IN_ANALOG_GAIN_6DB 0x0F
#define WM8994_LINE_IN_ANALOG_GAIN_9DB 0x11
#define WM8994_LINE_IN_ANALOG_ATTEN_3DB 0x09
#define WM8994_LINE_IN_ANALOG_ATTEN_6DB 0x07
#define WM8994_LINE_IN_ANALOG_ATTEN_9DB 0x05

// write these to registers 0x19 and 0x1B
#define WM8994_DMIC_ANALOG_GAIN_0DB 0x0B
#define WM8994_DMIC_ANALOG_GAIN_3DB 0x0D
#define WM8994_DMIC_ANALOG_GAIN_6DB 0x0F
#define WM8994_DMIC_ANALOG_GAIN_9DB 0x11
#define WM8994_DMIC_ANALOG_ATTEN_3DB 0x09
#define WM8994_DMIC_ANALOG_ATTEN_6DB 0x07
#define WM8994_DMIC_ANALOG_ATTEN_9DB 0x05


// this is the size of each ping pong buffer - PING_IN, PING_OUT, PONG_IN and PONG_OUT in sample instants
// there are two samples (left and right) per sample instant
#define PING_PONG_BUFFER_SIZE ((uint32_t)256)

// buffers are placed in SDRAM - AUDIO_REC_START_ADDR is defined in stm32f7_wm8994_init.h
// this is the start address of the PING_IN buffer
#define PING_IN AUDIO_REC_START_ADDR

// length of each ping pong buffer in bytes is number of sample instants (PING_PONG_BUFFER_SIZE)
// multiplied by bytes per 16-bit sample (2) multiplied by samples per sample instant (L+R => 2) 
// start addresses of PING_OUT, PONG_IN and PONG_OUT buffers are offset from start of PING_IN
// in the STM32F4 version these were extern arrays - may need to move these #defines to stm32f7_wm8994_init.h
// since they may be used in functions that are defined in stm32f7_wm8994_init.c - or not?
// on the other hand, perhaps all of these 'global' scope variables and #defines might be moved to
// stm32f7_wm8994_init.h

#define PING_OUT (AUDIO_REC_START_ADDR + (PING_PONG_BUFFER_SIZE * 4))
#define PONG_IN (AUDIO_REC_START_ADDR + (PING_PONG_BUFFER_SIZE * 8))
#define PONG_OUT (AUDIO_REC_START_ADDR + (PING_PONG_BUFFER_SIZE * 12))

// this code provided by ST - do we need it? should we place it in another, copyright-headed file?
/* Macros --------------------------------------------------------------------*/
#ifdef USE_FULL_ASSERT
/* Assert activated */
#define ASSERT(__condition__)                do { if(__condition__) \
                                                   {  assert_failed(__FILE__, __LINE__); \
                                                      while(1);  \
                                                    } \
                                              }while(0)
#else
/* Assert not activated : macro has no effect */
#define ASSERT(__condition__)                  do { if(__condition__) \
                                                   {   \
                                                    } \
                                              }while(0)
#endif /* USE_FULL_ASSERT */

#define RGB565_BYTE_PER_PIXEL     2
#define ARBG8888_BYTE_PER_PIXEL   4

/* Camera have a max resolution of VGA : 640x480 */
#define CAMERA_RES_MAX_X          640
#define CAMERA_RES_MAX_Y          480

/**
  * @brief  LCD FB_StartAddress
  * LCD Frame buffer start address : starts at beginning of SDRAM
  */
#define LCD_FRAME_BUFFER          SDRAM_DEVICE_ADDR

/**
  * @brief  Camera frame buffer start address
  * Assuming LCD frame buffer is of size 480x800 and format ARGB8888 (32 bits per pixel).
  */
#define CAMERA_FRAME_BUFFER       ((uint32_t)(LCD_FRAME_BUFFER + (RK043FN48H_WIDTH * RK043FN48H_HEIGHT * ARBG8888_BYTE_PER_PIXEL)))

/**
  * @brief  SDRAM Write read buffer start address after CAM Frame buffer
  * Assuming Camera frame buffer is of size 640x480 and format RGB565 (16 bits per pixel).
  */
#define SDRAM_WRITE_READ_ADDR        ((uint32_t)(CAMERA_FRAME_BUFFER + (CAMERA_RES_MAX_X * CAMERA_RES_MAX_Y * RGB565_BYTE_PER_PIXEL)))

#define SDRAM_WRITE_READ_ADDR_OFFSET ((uint32_t)0x0800)
#define SRAM_WRITE_READ_ADDR_OFFSET  SDRAM_WRITE_READ_ADDR_OFFSET

#define AUDIO_REC_START_ADDR         SDRAM_WRITE_READ_ADDR

// added 16 May DSR
#define PING 0
#define PONG 1


/* Exported types ------------------------------------------------------------*/

typedef enum {
  AUDIO_ERROR_NONE = 0,
  AUDIO_ERROR_NOTREADY,
  AUDIO_ERROR_IO,
  AUDIO_ERROR_EOF,
}AUDIO_ErrorTypeDef;

/* Exported variables ---------------------------------------------------*/
extern uint8_t     NbLoop;
extern uint8_t     MfxExtiReceived;
#ifndef USE_FULL_ASSERT
extern uint32_t    ErrorCounter;
#endif
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

#define COUNT_OF_EXAMPLE(x)    (sizeof(x)/sizeof(BSP_DemoTypedef))
/* Exported functions ------------------------------------------------------- */
void AudioLoopback_demo (void);
uint8_t CheckForUserInput(void);
void BSP_LCD_DMA2D_IRQHandler(void);
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line);
#endif



int16_t prand(void);
short prbs(int16_t noise_level);
void BSP_AUDIO_IN_TransferComplete_CallBack(void);
void BSP_AUDIO_IN_TransferCompleteM1_CallBack(void);
void BSP_AUDIO_IN_Error_CallBack(void);
void BSP_AUDIO_OUT_TransferComplete_CallBack(void);
void BSP_AUDIO_OUT_TransferCompleteM1_CallBack(void);
void BSP_AUDIO_OUT_Error_CallBack(void);
static void SystemClock_Config(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
static void MPU_Config(void);
static void CPU_CACHE_Enable(void);
void assert_failed(uint8_t* file, uint32_t line);
void stm32f7_wm8994_init(uint32_t fs, int16_t io_method, int16_t select_input, int16_t select_output, int16_t headphone_gain, int16_t line_in_gain, int16_t dmic_gain, char *name, int graph);

void init_LCD(int16_t sample_frequency, char *name, int16_t io_method, int graph);

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
