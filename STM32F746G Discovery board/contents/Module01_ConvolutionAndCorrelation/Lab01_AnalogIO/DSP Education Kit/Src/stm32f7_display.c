/**
  ******************************************************************************
  * @file    display.c 
  * @author  ARM University Program
  * @date    Summer 2017 
  * @brief   This file provides functions for drawing graphs on the LCD.
  *					 Must call init_LCD() before calling the other drawing functions.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f7_display.h"

int refresh_counter = 0;	//Control the refresh rate
//If the program is intr, this factor will be larger
//because the intr program will run faster than dma program
int refresh_counter_factor = 1;

int button_flag = 0;	//A flag to control the button state
int stop = 0;	//A flag to stop drawing the graph to make a static graph
int update_flag = 0; //A flag to indicate the screen is going to update

int fixymax, fixymin = 0; //A variable to save the ymax and ymin which use in drawAxes()

//Global variable to store the sampling frequency that get from the main program
//use it in drawAxes(), initialise to a random value
uint16_t frequency = 0;

//A temporary buffer to store data from the main
//use in plotSamplesIntr()
int16_t temp_buffer[256];
int16_t temp_buffer_ptr = 0;

/**
  * @brief  Check for user input.
  * @param  None
  * @retval Input state (1 : active / 0 : Inactive)
  */
 uint8_t CheckForUserInput(void)
{
  if (BSP_PB_GetState(BUTTON_KEY) != RESET)
  {
//    HAL_Delay(10);
    while (BSP_PB_GetState(BUTTON_KEY) != RESET);
    return 1 ;
  }
  return 0;
}

/**
  * @brief  Drawing the graph grid at the logo layer which won't have frequent change       
  * @param  name: Name of the main file
  * @retval none
  */
	
void drawGrid(char * name) {
	int i = 0;
	
	//Choose the logo layer
	BSP_LCD_SelectLayer(0);
	BSP_LCD_Clear(BACKGROUND_COLOUR);

	//Show the main file name at the top-right hand corner
	BSP_LCD_SetBackColor(BACKGROUND_COLOUR);	
	BSP_LCD_SetTextColor(GRID_COLOUR);	
	if(strlen(name) > 28) BSP_LCD_SetFont(&Font12);
	else BSP_LCD_SetFont(&Font16);
	BSP_LCD_DisplayStringAt(0, 0, (uint8_t *)name, RIGHT_MODE);		
	
	//Draw the grid area
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_DrawRect(FIRST_DATA_PIXEL, HEADER_HEIGHT, GRAPH_WIDTH, GRAPH_HEIGHT);

	//Drawing grid lines
	for(i = 0; i < 5; i++) {
		BSP_LCD_DrawVLine(FIRST_DATA_PIXEL+64*i, HEADER_HEIGHT, GRAPH_HEIGHT+5);	
		BSP_LCD_DrawHLine(FIRST_DATA_PIXEL, HEADER_HEIGHT+48*i, GRAPH_WIDTH);
	}
	
	//Go back to the graph layer for the remaining graph drawing
	BSP_LCD_SelectLayer(LTDC_ACTIVE_LAYER);
	BSP_LCD_Clear(BACKGROUND_COLOUR);
	BSP_LCD_SetTransparency(1, 200);
}

/**
  * @brief  Display the x and y axis values and labels       
  * @param  ycentre: pixel of the y centre line of the graph
  * @param  ymax: maximum data pixel y location
  * @param  ymin: minimum data pixel y location
  * @param  max: maximum data actual value
  * @param  min: minimum data actual value
  * @param  size: size of the displaying buffer
  * @param  xpos: last data in the buffer pixel x location
  * @param  mode: graph type WAVE/ FFT/ LOG FFT/ LMS
  * @retval none
  */

void drawAxes (int ycentre, int ymax, int ymin, float max, float min, float dB_per_divs, int size, int xpos, int type) {
	uint32_t axes_value [20];

	int i = 0;
	
	BSP_LCD_SetFont(&Font12);

	/*Drawing y axis*/
	
	//If the max/min value doesn't change or user didn't push the button, stop drawing the y-axis to prevent flickering on the axis
	if(fixymax != ymax || fixymin != ymin || update_flag == 1){
		fixymax = ymax;
		fixymin = ymin;

		//Clear the y-axis area
		BSP_LCD_SetTextColor(BACKGROUND_COLOUR);
		BSP_LCD_FillRect(0, HEADER_HEIGHT, FIRST_DATA_PIXEL, 272);
		BSP_LCD_SetTextColor(TEXT_COLOUR);
		BSP_LCD_SetBackColor(BACKGROUND_COLOUR);		

		if(dB_per_divs != 0) {
			sprintf((char*)axes_value, "%.3f dB/div", dB_per_divs);
			BSP_LCD_DisplayStringAt(0, 20, (uint8_t * ) &axes_value, LEFT_MODE);	
			
			if(ycentre != FFT_YCENTRE) {
				//Display 0
				sprintf((char*)axes_value, "%c", '0');
				BSP_LCD_DisplayStringAt(0, ycentre, (uint8_t * ) &axes_value, LEFT_MODE);					
			}
			
		} else {
			//Display 0
			
			sprintf((char*)axes_value, "%c", '0');
			BSP_LCD_DisplayStringAt(0, ycentre, (uint8_t * ) &axes_value, LEFT_MODE);	
			
			if((ycentre - ymax) > 5) { 
				sprintf((char*)axes_value, "%.3f", max);
				BSP_LCD_DisplayStringAt(0, ymax, (uint8_t * ) &axes_value, LEFT_MODE);
			}		
			
			if((ymin - ycentre) > 5) { 
				//BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);	//for debugging
				sprintf((char*)axes_value, "%.3f", min);
				BSP_LCD_DisplayStringAt(0, ymin, (uint8_t * ) &axes_value, LEFT_MODE);
			}
		}
		update_flag = 0;
	}	
	
	/*Drawing x axis*/	
	
	switch(type){
		
		case LMS:
			BSP_LCD_SetTextColor(TEXT_COLOUR);
			BSP_LCD_DisplayStringAt(0, 0, (uint8_t * ) "sample value", LEFT_MODE);		

			BSP_LCD_SetTextColor(TEXT_COLOUR);
			BSP_LCD_SetBackColor(BACKGROUND_COLOUR);
		
			BSP_LCD_SetTextColor(TEXT_COLOUR);
			BSP_LCD_DisplayStringAt(20, GRAPH_VER_END_PIXEL - 15, (uint8_t * ) "Time(s)", RIGHT_MODE);
			//BSP_LCD_DisplayStringAt(20, 50, (uint8_t * ) "LMS Graph", RIGHT_MODE); //Uncomment this if the graph name is needed on screen
			
			sprintf((char*)axes_value, "%c", '0');
			BSP_LCD_DisplayStringAt(FIRST_DATA_PIXEL+3, GRAPH_VER_END_PIXEL+2, (uint8_t * ) &axes_value, LEFT_MODE);
			
			for(i = 1; i < 5; i++) {
				sprintf((char*)axes_value, "%.3f", (float)64*i/frequency);
				BSP_LCD_DisplayStringAt(FIRST_DATA_PIXEL+64*i+3, GRAPH_VER_END_PIXEL+2, (uint8_t * ) &axes_value, LEFT_MODE);
			}
			break;
			
		case FFT:

			BSP_LCD_SetTextColor(TEXT_COLOUR);
			BSP_LCD_DisplayStringAt(0, 0, (uint8_t * ) "Magnitude", LEFT_MODE);
		
			BSP_LCD_SetTextColor(TEXT_COLOUR);
			BSP_LCD_DisplayStringAt(20, GRAPH_VER_END_PIXEL - 15, (uint8_t * ) "Frequency(Hz)", RIGHT_MODE);	
			//BSP_LCD_DisplayStringAt(20, 50, (uint8_t * ) "FFT Graph", RIGHT_MODE);	//Uncomment this if the graph name is needed on screen
		
			sprintf((char*)axes_value, "%c", '0');
			BSP_LCD_DisplayStringAt(FIRST_DATA_PIXEL, GRAPH_VER_END_PIXEL+2, (uint8_t * ) &axes_value, LEFT_MODE);		
		
			sprintf((char*)axes_value, "%d", frequency/2);
			BSP_LCD_DisplayStringAt(xpos, GRAPH_VER_END_PIXEL+2, (uint8_t * ) &axes_value, LEFT_MODE);
		
			break;
		
		case WAVE:
			BSP_LCD_SetTextColor(TEXT_COLOUR);
			BSP_LCD_DisplayStringAt(0, 0, (uint8_t * ) "sample value", LEFT_MODE);
		
			BSP_LCD_SetTextColor(TEXT_COLOUR);
			BSP_LCD_DisplayStringAt(20, GRAPH_VER_END_PIXEL - 15, (uint8_t * ) "sample number", RIGHT_MODE);	
			//BSP_LCD_DisplayStringAt(20, 50, (uint8_t * ) "Input signal", RIGHT_MODE);	//Uncomment this if the graph name is needed on screen
		
			sprintf((char*)axes_value, "%c", '0');
			BSP_LCD_DisplayStringAt(FIRST_DATA_PIXEL, GRAPH_VER_END_PIXEL+2, (uint8_t * ) &axes_value, LEFT_MODE);		
		
			sprintf((char*)axes_value, "%d", size);
			BSP_LCD_DisplayStringAt(xpos, GRAPH_VER_END_PIXEL+2, (uint8_t * ) &axes_value, LEFT_MODE);
			break;
		
		case LOGFFT:
			BSP_LCD_SetTextColor(TEXT_COLOUR);
			BSP_LCD_DisplayStringAt(0, 0, (uint8_t * ) "Magnitude(dB)", LEFT_MODE);
			//BSP_LCD_DisplayStringAt(20, 50, (uint8_t * ) "FFT Graph", RIGHT_MODE);	//Uncomment this if the graph name is needed on screen
		
			BSP_LCD_SetTextColor(TEXT_COLOUR);
			BSP_LCD_DisplayStringAt(20, GRAPH_VER_END_PIXEL - 15, (uint8_t * ) "Frequency(Hz)", RIGHT_MODE);	

			sprintf((char*)axes_value, "%c", '0');
			BSP_LCD_DisplayStringAt(FIRST_DATA_PIXEL, GRAPH_VER_END_PIXEL+2, (uint8_t * ) &axes_value, LEFT_MODE);		
		
			sprintf((char*)axes_value, "%d", frequency/2);
			BSP_LCD_DisplayStringAt(xpos, GRAPH_VER_END_PIXEL+2, (uint8_t * ) &axes_value, LEFT_MODE);			
			break;
	}
		
}

/**
  * @brief  Initialise the LCD screen, initialise 2 layers
	*					layer 0 is for the logo and unchange elements
	*					layer 1 (LTDC_ACTIVE_LAYER) is for graph data
	*					Display the start screen which have logo and title
	*					Draw the grid
  * @param  sample_frequency: get the sample frequency from the main and save in the global
	*														variable frequency
  * @param  name: get the program name from the main and use in
	*								the drawGrid()
  * @param  io_method: IO_METHOD_DMA = slower refresh rate,
	*										 IO_METHOD_INTR = faster refresh rate
  * @param  graph: GRAPH = display graph, NOGRAPH = display start screen only
  * @retval none
  */

void init_LCD(int16_t sample_frequency, char *name, int16_t io_method, int graph) {

	frequency = sample_frequency;

	// Set up the LCD
	BSP_LCD_Init();
	
	BSP_LCD_LayerDefaultInit(0, 0xC0400000); //Initialise the logo layer
	BSP_LCD_LayerDefaultInit(LTDC_ACTIVE_LAYER, LCD_FRAME_BUFFER);
	
	BSP_LCD_DisplayOn();
	
	//BSP_LCD_SetLayerVisible(0, ENABLE);
	BSP_LCD_SelectLayer(0);
	BSP_LCD_Clear(0xFF0090BF);
	
	BSP_LCD_SelectLayer(LTDC_ACTIVE_LAYER);
	BSP_LCD_Clear(BACKGROUND_COLOUR);
	
	BSP_LCD_SetTransparency(0, 255);
	BSP_LCD_SetTransparency(1, 150);
	
	BSP_LCD_SelectLayer(0);
	/* Draw Bitmap Logo*/
	BSP_LCD_DrawBitmap(50, 70, (uint8_t *)armlogo);

	BSP_LCD_SelectLayer(LTDC_ACTIVE_LAYER);	
	 /* Set the LCD Text Color */
	BSP_LCD_SetTextColor(TEXT_COLOUR);
	
	 /* Display LCD messages */
	BSP_LCD_DisplayStringAt(0, 10, (uint8_t *)"STM32F746G DSP Education Kit", CENTER_MODE);
	if(strlen(name) > 28) BSP_LCD_SetFont(&Font16);
	BSP_LCD_DisplayStringAt(0, 50, (uint8_t *)name, CENTER_MODE);
	BSP_LCD_SetFont(&Font24);
	if(io_method == 0) {
		refresh_counter_factor = 100;
	}
	if(graph == 1){
		BSP_LCD_DisplayStringAt(0, 200, (uint8_t *)"Press User Button to start", CENTER_MODE);
		while(CheckForUserInput() != 1){}	
			
		drawGrid(name);
	}
}

/**
  * @brief  Clear the graph area only, draw vertical lines from the start of
	*					the graph to the end of the graph
  * @param  none
  * @retval none
  */

void clearScreen () {
	unsigned i = 0;
	
	refresh_counter = 0;
	
	for(i = FIRST_DATA_PIXEL; i < GRAPH_WIDTH+FIRST_DATA_PIXEL; i++) {	
		BSP_LCD_SetTextColor(BACKGROUND_COLOUR);
		BSP_LCD_DrawVLine(i, 0, GRAPH_VER_END_PIXEL);	
	}
}

/**
  * @brief  For debug only, display the axes data on the screen
  * @param  ymax: maximum data pixel y location
  * @param  ymin: minimum data pixel y location
  * @param  max: maximum data actual value
  * @param  min: minimum data actual value
  * @param  yscalefactor: y scale factor
  * @retval none
  */

void debug_display(int ymax, int ymin, float max, float min, float biggestmag, float yscalefactor) {
	uint8_t axes_debug_value [20];

	BSP_LCD_SetFont(&Font12);
	
	sprintf((char*)axes_debug_value, "ymax: %d", ymax);
	BSP_LCD_DisplayStringAt(10, 20, (uint8_t * ) &axes_debug_value, RIGHT_MODE);
	sprintf((char*)axes_debug_value, "ymin: %d", ymin);
	BSP_LCD_DisplayStringAt(10, 30, (uint8_t * ) &axes_debug_value, RIGHT_MODE);
	sprintf((char*)axes_debug_value, "max: %g", max);
	BSP_LCD_DisplayStringAt(10, 40, (uint8_t * ) &axes_debug_value, RIGHT_MODE);
	sprintf((char*)axes_debug_value, "min: %g", min);
	BSP_LCD_DisplayStringAt(10, 50, (uint8_t * ) &axes_debug_value, RIGHT_MODE);
	sprintf((char*)axes_debug_value, "ycentre: %g", biggestmag);
	BSP_LCD_DisplayStringAt(10, 60, (uint8_t * ) &axes_debug_value, RIGHT_MODE);	
	sprintf((char*)axes_debug_value, "yscalefactor: %g", yscalefactor);
	BSP_LCD_DisplayStringAt(10, 70, (uint8_t * ) &axes_debug_value, RIGHT_MODE);	
}

/**
  * @brief  Draw bars according to the value inside data_buffer
  * @param  data_buffer: a pointer that points to the data that need to plot
  * @param  num_samples: number of the plotted data buffer
  * @param  live: static data = 0, live data = 1
  * @param  complex: complex data = 1, non-complex data = 0
  * @retval none
  */

void plotWave(float32_t * data_buffer, int num_samples, int live, int complex) {
	int16_t i;
	int16_t xvalue;              // x in pixels
	int16_t ymax = HEADER_HEIGHT;
	int16_t ymin = GRAPH_VER_END_PIXEL;                  // max and min y values in pixels
	float32_t max, min = data_buffer[0]; // max and min y values passed to function
	float32_t biggestmag, yscalefactor;
	float x_spacing = 1;
	int step = 1;
	
	if(complex) step = 2;
	
	x_spacing = GRAPH_WIDTH / (num_samples*step);
	
	// initialise some variables
	max = data_buffer[0];
	min = data_buffer[0];
	xvalue = FIRST_DATA_PIXEL;
	
	//Whenever user push the button or live data is needed, bar charts will be drawn,
	//Otherwise, the graph will only be drawn once
	if(stop == 0 || CheckForUserInput() == 1 || live == 1){
		//If static data is needed, the graph will only be drawn once
		stop = 1;

		// determine min and max values
		for(i = 0; i < num_samples*step; i++) {		
			if(min >= data_buffer[i])	min = data_buffer[i];
			if(max <= data_buffer[i]) max = data_buffer[i];
		}

		//Determine the largest value and limit the graph size by using yscalefactor
		if(max*max > min*min) biggestmag = max; else biggestmag = -min;
		if(biggestmag == 0) biggestmag = 1;
		
		yscalefactor = 100/(biggestmag); // 100 is +/- pixels from centre of screen
		ymin = GRAPH_YCENTRE - min*yscalefactor;
		ymax = GRAPH_YCENTRE - max*yscalefactor;		
		
		for(i = 0; i < num_samples*step; i++) {
			//Clear the previous bar before drawing the new bar on the screen
			BSP_LCD_SetTextColor(BACKGROUND_COLOUR);	
			BSP_LCD_DrawLine(xvalue, 0, xvalue, 272);
			
			//if the data is complex values, real values and imaginary values will
			//display in different colour
			if(complex) {
				if(i % 2 == 0)
					BSP_LCD_SetTextColor(GRAPH_COLOUR);		
				else
					BSP_LCD_SetTextColor(IMAGINARY_COLOUR);	
			} else
				BSP_LCD_SetTextColor(GRAPH_COLOUR);		
			
			//Draw the bars
			BSP_LCD_DrawLine(xvalue, GRAPH_YCENTRE, xvalue, GRAPH_YCENTRE - data_buffer[i]*yscalefactor);

			xvalue += x_spacing;
		}
		
		//debug_display(ymax,ymin,max,min,data_buffer[10],data_buffer[20]);
		
		//Draw the axes values and labels
		drawAxes (GRAPH_YCENTRE, ymax, ymin, max, min, 0, num_samples, xvalue, WAVE);
	}
}

/**
  * @brief  Draw bars according to the value inside data_buffer but without auto-scaling
  * @param  data_buffer: a pointer that points to the data that need to plot
  * @param  num_samples: number of the plotted data buffer
  * @retval none
  */

void plotWaveNoAutoScale(float32_t * data_buffer, int num_samples) {
	unsigned i = 0;
	int xcoor = FIRST_DATA_PIXEL;
	
	int ymax = 20;
	int ymin = GRAPH_VER_END_PIXEL;
	float max, min = data_buffer[0];
	
	float yscalefactor = 270;
	
	float x_spacing = 1;

	x_spacing = GRAPH_WIDTH / num_samples;	

		
	for(i = 0; i < num_samples; i++) {
		//Clear the previous bar before drawing the new bar on the screen
		BSP_LCD_SetTextColor(BACKGROUND_COLOUR);
		BSP_LCD_DrawLine(xcoor, 0, xcoor, 272);
			
		//Draw the bars
		BSP_LCD_SetTextColor(GRAPH_COLOUR);		
		BSP_LCD_DrawLine(xcoor, GRAPH_YCENTRE, xcoor, GRAPH_YCENTRE - data_buffer[i]/yscalefactor);
		
		// determine min and max values	to draw the y-axis
		if(min >= data_buffer[i]){
			ymin = GRAPH_YCENTRE - data_buffer[i]/yscalefactor;
			min = data_buffer[i];
		} 
		if(max <= data_buffer[i]){
			ymax = GRAPH_YCENTRE - data_buffer[i]/yscalefactor;
			max = data_buffer[i];
		}
		
		//Safety measure to avoid the bars go outside the grid area
		if(ymax < HEADER_HEIGHT) ymax = HEADER_HEIGHT;
		if(ymin > GRAPH_VER_END_PIXEL) ymin = GRAPH_VER_END_PIXEL;
		
		xcoor += x_spacing;
	}
	//debug_display(ymax,ymin,max,min,20,yscalefactor);
	drawAxes (GRAPH_YCENTRE, ymax, ymin, max, min, 0, num_samples, xcoor, WAVE);

}

/**
  * @brief  Draw bars according to the value inside data_buffer,
	*					if buffer size is smaller than the no of plots, repeat the graph
  * @param  data_buffer: a pointer that points to the data that need to plot
  * @param  num_samples: number of the plotted data buffer
  * @param  num_plots: how many data points needed to plot
  * @retval none
  */

void plotSamples(int16_t * data_buffer, int num_samples, int num_plots) {
	int16_t i;
	int16_t xvalue;              // x in pixels
	int16_t ymax = HEADER_HEIGHT;
	int16_t ymin = GRAPH_VER_END_PIXEL;                  // max and min y values in pixels
	float32_t max, min = data_buffer[0]; // max and min y values passed to function
	float32_t biggestmag, yscalefactor;
	int counter = 0;
	float x_spacing = 1;

	// initialise some variables
	max = data_buffer[0];
	min = data_buffer[0];
	xvalue = FIRST_DATA_PIXEL;

	x_spacing = GRAPH_WIDTH / num_plots;
	// determine min and max values
	for(i = 0; i < num_samples; i++) {		
		if(min >= data_buffer[i])	min = data_buffer[i];
		if(max <= data_buffer[i]) max = data_buffer[i];
	}

	//Determine the largest value and limit the graph size by using yscalefactor
	if(max*max > min*min) biggestmag = max; else biggestmag = -min;
		
	yscalefactor = 100/(biggestmag); // 100 is +/- pixels from centre of screen
	ymin = GRAPH_YCENTRE - min*yscalefactor;
	ymax = GRAPH_YCENTRE - max*yscalefactor;		
		
	for(i = 0; i < num_plots; i++) {
		//Draw the bars
		BSP_LCD_SetTextColor(GRAPH_COLOUR);		
		BSP_LCD_DrawLine(xvalue, GRAPH_YCENTRE, xvalue, GRAPH_YCENTRE - data_buffer[counter]*yscalefactor);

		if(counter >= num_samples - 1)
			counter = 0;
		else
			counter ++;

		xvalue += x_spacing;
	}
	//Draw the axes values and labels
	drawAxes (GRAPH_YCENTRE, ymax, ymin, max, min, 0, num_plots, xvalue, WAVE);
}

/**
  * @brief  Save data_sample to the temporary buffer
  * @brief  Draw the graph from the temporary buffer only when enough data points have got
  * @param  data_sumple: a pointer that points to the data that need to plot
  * @param  num_plots: how many data points needed to plot
  * @retval none
  */

void plotSamplesIntr(int16_t data_sample, int num_plots) {
	float x_spacing = 1;
	unsigned int i = 0;

	int xvalue = FIRST_DATA_PIXEL;
	int ymax = 20;
	int ymin = GRAPH_VER_END_PIXEL;
	float max, min = data_sample;
	float32_t biggestmag, yscalefactor;

	if(stop == 0){
		x_spacing = GRAPH_WIDTH / num_plots;
		temp_buffer[temp_buffer_ptr] = data_sample;
		temp_buffer_ptr++;
		if(temp_buffer_ptr >= num_plots){
			temp_buffer_ptr = 0;
			stop = 1;

			for(i = 0; i < num_plots; i++) {		
				if(min >= temp_buffer[i]) min = temp_buffer[i];
				if(max <= temp_buffer[i]) max = temp_buffer[i];
			}

			//Determine the largest value and limit the graph size by using yscalefactor
			if(max*max >= min*min) biggestmag = max; else biggestmag = -min;
					
			yscalefactor = 100/(biggestmag); // 100 is +/- pixels from centre of screen
			ymin = GRAPH_YCENTRE - min*yscalefactor;
			ymax = GRAPH_YCENTRE - max*yscalefactor;	
			
			BSP_LCD_SetTextColor(GRAPH_COLOUR);						
			for(i = 0; i < num_plots; i++) {
				//Draw the bars
				BSP_LCD_SetTextColor(GRAPH_COLOUR);		
				BSP_LCD_DrawLine(xvalue, GRAPH_YCENTRE, xvalue, GRAPH_YCENTRE - temp_buffer[i]*yscalefactor);		
				xvalue += x_spacing;			
			}
			
			drawAxes (GRAPH_YCENTRE, ymax, ymin, max, min, 0, num_plots, xvalue, WAVE);
		}
	}	
}

/**
  * @brief  Plot the FFT graph according to the data buffer
  * @param  data_buffer: a pointer that points to the data that need to plot
  * @param  num_samples: number of the plotted data buffer
	* @param  auto_scale: AUTO_SCALING: auto scaling of the y-axis,
												NO_AUTO_SCALING: no auto scaling of the y-axis
  * @retval none
  */

void plotFFT(float32_t * data_buffer, int num_samples, int auto_scale){	
	int16_t i;
	int16_t xvalue;              // x and y value in pixels
	int16_t ymax = HEADER_HEIGHT;				 // max and min y values in pixels
	int16_t	ymin = GRAPH_VER_END_PIXEL;          
	float32_t max, min; // max and min y values passed to function
	float32_t biggestmag, yscalefactor;

	float x_spacing = 1;

	x_spacing = GRAPH_WIDTH / num_samples;	
	
	// initialise some variables
	max = data_buffer[0];
	min = data_buffer[0];
	xvalue = FIRST_DATA_PIXEL;
	
	//Refresh the screen in a specific rate, larger the number, slower refresh rate
	if(refresh_counter > 40) {
		clearScreen();
		refresh_counter = 0;
	}
				
	// determine min and max values
	for(i = 0; i < num_samples; i++) {		
		if(min >= data_buffer[i])	min = data_buffer[i];
		if(max <= data_buffer[i]) max = data_buffer[i];
	}
	
	//If y-axis needed to auto-scaling, have to work out the yscalefactor to limit the
	//graph size. If no auto-scaling, the yscalefactor will set to a fix value
	switch(auto_scale){
		case AUTO_SCALING:
			
			//Determine the largest value and limit the graph size by using yscalefactor
			if(max*max > min*min) biggestmag = max; else biggestmag = -min;
			
			yscalefactor = 200/(biggestmag); // 200 is +/- pixels from centre of screen

			break;
		
		case NO_AUTO_SCALING:
			yscalefactor = 0.0075;
		
			break;

	}
	ymin = FFT_YCENTRE - min*yscalefactor;
	ymax = FFT_YCENTRE - max*yscalefactor;	
	
	BSP_LCD_SetTextColor(GRAPH_COLOUR);	
	
	//Safety measure to prevent the graph go off the screen	
	if(ymax < HEADER_HEIGHT)
		ymax = HEADER_HEIGHT;
	
	//Draw the bars, only draw half of the data buffer because the other half data is duplicated
	for(i = 0; i < num_samples/2; i++) {		
		BSP_LCD_DrawLine(xvalue, FFT_YCENTRE, xvalue, FFT_YCENTRE - data_buffer[i]*yscalefactor);
		BSP_LCD_DrawLine(xvalue, FFT_YCENTRE, xvalue, FFT_YCENTRE - data_buffer[i]*yscalefactor);

		xvalue += x_spacing*2;
	}
	
	//debug_display(ymax,ymin,max,min,biggestmag,yscalefactor);
	//Draw the axes values and labels	
	drawAxes (FFT_YCENTRE, ymax, ymin, max, min, 0, num_samples, xvalue, FFT);
	refresh_counter ++;

}

/**
  * @brief  Plot the FFT graph in decibel according to the data buffer
  * @param  data_buffer: a pointer that points to the data that need to plot
  * @param  num_samples: number of the plotted data buffer
  * @param  live: LIVE = plot the graph live
	*								STATIC = will only draw the graph once
  * @retval none
  */

void plotLogFFT(float32_t *(data_buffer), int num_samples, int live){	
	int16_t i;
	int16_t xvalue = FIRST_DATA_PIXEL;
	int16_t yvalue;              // x and y value in pixels
	int16_t ymax = HEADER_HEIGHT;
	int16_t ymin = GRAPH_VER_END_PIXEL;                // max and min y values in pixels
	float32_t max, min, dB_per_divs;                  // max and min y values passed to function
	float32_t biggestmag, yscalefactor;
	int16_t ycentre = FFT_YCENTRE;
	int negative = 0;
	int pixels_from_centre = 180;

	float x_spacing = 1;

	x_spacing = GRAPH_WIDTH / num_samples;	
	
	//the data can't be zero as it has to be used in log calculation afterwards
	if(data_buffer[0] == 0)
		return;

	if(stop == 0) {
		if(live == 0){
			stop = 1;
		}	else
			stop = 0;
		//Refresh the screen in a specific rate, larger the number, slower refresh rate	
		if(refresh_counter > 100*refresh_counter_factor) {
			clearScreen();
			refresh_counter = 0;
		}
		
		if(refresh_counter == 0){
			
			// initialise some variables
			max = 20 * log10(data_buffer[0]);
			min = 20 * log10(data_buffer[0]);

			for(i = 0; i < num_samples; i++) {		
				if(data_buffer[i] == 0)
					return;
				//Calculate the decibel of the data
				data_buffer[i] = 20 * log10(data_buffer[i]);
				
				//Determine where is the y-axis centre
				if(data_buffer[i] < 0) negative++;
				
				if(min >= data_buffer[i])	min = data_buffer[i];
				if(max <= data_buffer[i]) max = data_buffer[i];
			}
			if(max*max > min*min) biggestmag = max; else biggestmag = -min;
			
			if(negative != num_samples && negative != 0) {
				pixels_from_centre = 100;
				ycentre = GRAPH_YCENTRE;
			} else if (negative == num_samples){
				pixels_from_centre = 150;
				ycentre = LOGFFT_YCENTRE;
			} else {
				pixels_from_centre = 200;
				ycentre = FFT_YCENTRE;
			}

			yscalefactor = pixels_from_centre/(biggestmag); // pixels_from_centre is +/- pixels from centre of screen
			ymin = ycentre - min*yscalefactor;
			ymax = ycentre - max*yscalefactor;	

			dB_per_divs = (max - min)*48/abs(ymin - ymax);
			
			BSP_LCD_SetTextColor(GRAPH_COLOUR);	
			
			for(i = 0; i < num_samples/2; i++) {		
				yvalue = ycentre - data_buffer[i]*yscalefactor;
				
				//Determine if the plotting out of the graph area, if yes plot it at the bottom of the graph
				if(yvalue > GRAPH_VER_END_PIXEL){
					yvalue = GRAPH_VER_END_PIXEL;
				}	else if (yvalue < HEADER_HEIGHT) {
					yvalue = HEADER_HEIGHT;
				}
				
				BSP_LCD_DrawLine(xvalue, GRAPH_VER_END_PIXEL, xvalue, yvalue);

				xvalue += x_spacing*2;
			}

			//Determine the largest value and limit the graph size by using yscalefactor	
/*			if(max*max > min*min) biggestmag = max; else biggestmag = -min;
			
			//As the ycenter move up so less pixels from centre of screen
			if(ycenter == LOGFFT_YCENTRE) yscalefactor = 40/(biggestmag); // 40 is +/- pixels from centre of screen
			else yscalefactor = 200/(biggestmag); // 200 is +/- pixels from centre of screen
			ymin = ycenter - min*yscalefactor;
			ymax = ycenter - max*yscalefactor;	

			dB_per_divs = (max - min)*48/abs(ymin - ymax);
			BSP_LCD_SetTextColor(GRAPH_COLOUR);	
			
			for(i = 0; i < num_samples/2; i++) {		
				yvalue = ycenter - data_buffer[i]*yscalefactor;
				
				//Determine if the plotting out of the graph area, if yes plot it at the bottom of the graph
				if(yvalue > GRAPH_VER_END_PIXEL){
					yvalue = GRAPH_VER_END_PIXEL;
				}	else if (yvalue < HEADER_HEIGHT) {
					yvalue = HEADER_HEIGHT;
				}
				
				BSP_LCD_DrawLine(xvalue, GRAPH_VER_END_PIXEL, xvalue, yvalue);

				xvalue += x_spacing*2;
			}*/

			//debug_display(ymax, ymin, max, min, ycentre, yscalefactor);	
			drawAxes (ycentre, ymax, ymin, max, min, dB_per_divs, num_samples, xvalue, LOGFFT);
			negative = 0;
		}
		refresh_counter ++;
	}
}

/**
  * @brief  Plot the LMS graph according to the data buffer
  * @param  data_buffer: a pointer that points to the data that need to plot
  * @param  num_samples: number of the plotted data buffer
  * @param  live: LIVE = plot the graph live
	*								STATIC = will only draw the graph once
  * @retval none
  */

void plotLMS(float32_t * data_buffer, int num_samples, int live) {
	int16_t i;
	int16_t xvalue, yvalue;              // x and y value in pixels
	int16_t ymax, ymin;                  // max and min y values in pixels
	float32_t max, min;                  // max and min y values passed to function
	float32_t biggestmag, yscalefactor;

	float x_spacing = 1;

	x_spacing = GRAPH_WIDTH / num_samples;	
	
	if(stop == 0){	
		if(live == 0)
			stop = 1;
		else
			stop = 0;
		
		// initialise some variables
		max = data_buffer[0];
		min = data_buffer[0];
		xvalue = FIRST_DATA_PIXEL;
		
			
		// determine min and max values
		for(i = 0; i < num_samples; i++) {		
			if(min >= data_buffer[num_samples - 1 - i])	min = data_buffer[num_samples - 1 - i];
			if(max <= data_buffer[num_samples - 1 - i]) max = data_buffer[num_samples - 1 - i];
		}
		
		//Determine the largest value and limit the graph size by using yscalefactor	
		if(max*max > min*min) biggestmag = max; else biggestmag = -min;

		yscalefactor = (GRAPH_YCENTRE-60)/(biggestmag); // 100 is +/- pixels from centre of screen
		ymin = GRAPH_YCENTRE - min*yscalefactor;
		ymax = GRAPH_YCENTRE - max*yscalefactor;
		
		BSP_LCD_SetTextColor(GRAPH_COLOUR);
		for(i = 0; i < num_samples; i++) {		
			//truncates the decimal value from floating point value and returns integer value
			yvalue = GRAPH_YCENTRE - trunc(data_buffer[num_samples - 1 - i]*yscalefactor);
			
			//Determine if the plotting out of the graph area, if yes plot it at the limit of the graph area
			if(yvalue > GRAPH_VER_END_PIXEL){
				yvalue = GRAPH_VER_END_PIXEL;
			}	else if (yvalue < HEADER_HEIGHT) {
				yvalue = HEADER_HEIGHT;
			}
			
			BSP_LCD_DrawLine(xvalue, GRAPH_YCENTRE, xvalue, yvalue);
			xvalue += x_spacing;
		}
		
		drawAxes (GRAPH_YCENTRE, ymax, ymin, max, min, 0, num_samples, xvalue, LMS);

		//Refresh the screen in a specific rate, larger the number, slower refresh rate	
		if(refresh_counter > 100) {
			clearScreen();
			refresh_counter = 0;
		} 

		refresh_counter ++;
	}
}

/**
  * @brief  Change the global variable button_flag to a specific value
  * @brief  Button_flag control 3 states only
  * @param  value: No effect if value smaller than 0 or greater than 3
  * @retval none
  */

void changeButtonFlag(int value) {
	button_flag = value;
}

/**
  * @brief  Get the value of the global variable button_flag and check the user button status
  * @brief  Button_flag control 3 states only
  * @param  none
	* @retval button_flag: return the state
  */

int checkButtonFlag() {

	//If user press the button, the whole screen will be cleared and change the state
	if(CheckForUserInput() == 1) {
		refresh_counter = 0;
		stop = 0;
		BSP_LCD_Clear(BACKGROUND_COLOUR);
		button_flag++;
	} 

	
	//If button flag greater than 2, the button flag will go the first state and reset everything
	if(button_flag >= 2) {
		stop = 0;
		button_flag=0;	
	}

	return button_flag;
}

/**
  * @brief  To configure the system clock. There's an exactly the same function in
	*					stm32f7_wm8994_init.c This function will only use when stm32f7_wm8994_init
	*					didn't call in the main
  * @param  none
  * @retval none
  */

static void SystemClock_Config_LCD(void){
  HAL_StatusTypeDef ret = HAL_OK;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  
  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  ASSERT(ret != HAL_OK);

  /* activate the OverDrive */
  ret = HAL_PWREx_ActivateOverDrive();
  ASSERT(ret != HAL_OK);

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
  ASSERT(ret != HAL_OK);
}

/**
  * @brief  To initialise the LCD.
	*					This function will only use when stm32f7_wm8994_init
	*					didn't call in the main
  * @param  sample_frequency: get the sample frequency from the main and 
	*														pass it to init_LCD()
  * @param  name: get the program name from the main and pass it to
	*								init_LCD()
  * @param  graph: GRAPH = display graph, NOGRAPH = display start screen only
  * @retval none
  */

void stm32f7_LCD_init(int16_t sample_frequency, char *name, int graph){	

	
	//Enable I-Cache
  SCB_EnableICache();
	
  //Enable D-Cache
  SCB_EnableDCache();

	SystemClock_Config_LCD(); // configure the system clock to 200 Mhz
	BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_GPIO); // configure the  blue user pushbutton in GPIO mode
	BSP_LED_Init(LED1);   // initialise LED on GPIO pin P   (also accessible on arduino header)
	BSP_SDRAM_Init();
	init_LCD(sample_frequency, name, 0, graph);
}

/**
  * @brief  Display a statement on the top of the screen also reset the stop global variable.
	*					This global variable is used to draw the static graph
	*	@brief	Can add a paramenter to display custom string, change the default string to the
	*					paramenter's name, type of the parameter is char *
  * @param  none
  * @retval none
  */

void proceed_statement(){
	stop = 0;
	update_flag = 1;
	
	BSP_LCD_SetFont(&Font12);
	
	BSP_LCD_DisplayStringAt(100, 10, (uint8_t * ) "Push button to next screen", LEFT_MODE);
	
	while(CheckForUserInput() != 1){}	
}
