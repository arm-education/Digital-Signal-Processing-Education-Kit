// stm32f7_display.h

// much of this was main.h - I'm not sure that all of it is needed !!

#include "stdio.h"
//#include "stm32746g_discovery.h"
#include "stm32746g_discovery_lcd.h"

#include <stdlib.h>
#include "string.h"

#include "arm_math.h"
//#include "arm_const_structs.h"

#include "armlogo.h"

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

#define HEADER_HEIGHT	20
#define FIRST_DATA_PIXEL 100
#define GRAPH_VER_END_PIXEL 260
#define GRAPH_HEIGHT GRAPH_VER_END_PIXEL-HEADER_HEIGHT
#define GRAPH_WIDTH 256

#define LMS 1
#define FFT 2
#define WAVE 3
#define LOGFFT 4

#define FFT_YCENTRE 260
#define GRAPH_YCENTRE 140
#define LOGFFT_YCENTRE 68

#define BACKGROUND_COLOUR LCD_COLOR_WHITE
#define GRAPH_COLOUR LCD_COLOR_BLUE
#define GRID_COLOUR LCD_COLOR_BLACK
#define TEXT_COLOUR LCD_COLOR_BLACK
#define IMAGINARY_COLOUR LCD_COLOR_RED

#define GRAPH 1
#define NOGRAPH 0

#define LIVE 1
#define STATIC 0

#define COMPLEX_STRUCT 1
#define ARRAY 0

#define AUTO_SCALING 1
#define NO_AUTO_SCALING 0
/* Exported functions ------------------------------------------------------- */

void init_LCD(int16_t sample_frequency, char *name, int16_t io_method, int graph);
void stm32f7_LCD_init(int16_t sample_frequency, char *name, int graph);
void clearScreen(void);
void plotWave(float32_t * data_buffer, int size, int live, int complex);
void plotWaveNoAutoScale(float32_t * data_buffer, int num_samples);
void plotSamples(int16_t * data_buffer, int num_samples, int num_plots);
void plotSamplesIntr(int16_t data_sample, int num_plots);
void plotFFT(float32_t * data_buffer, int size, int auto_scaling);
void plotLogFFT(float32_t * data_buffer, int size, int live);
void plotLMS(float32_t * data_buffer, int size, int live);
int checkButtonFlag(void);
void changeButtonFlag(int value);
void proceed_statement(void);

uint8_t CheckForUserInput(void);

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
