// stm32f7_adaptive.c

#include "stm32f7_wm8994_init.h"
#include "stm32f7_display.h"

#define SOURCE_FILE_NAME "stm32f7_adaptive.c"

#define BETA 0.01f            // learning rate
#define N 21                  // number of filter coeffs
#define NUM_ITERS 64          // number of iterations

float32_t desired[NUM_ITERS]; // storage for results
float32_t y_out[NUM_ITERS];
float32_t error[NUM_ITERS];
float32_t w[N+1] = {0.0};       // adaptive filter weights
float32_t x[N+1] = {0.0};       // adaptive filter delay line
int i, t;
float32_t d, y, e;

int main()
{
  for (t = 0; t < NUM_ITERS; t++)
  {
    x[0] = sin(2*PI*t/8);        // get new input sample
    d = cos(2*PI*t/8);           // get new desired output
    y = 0;                       // compute filter output
    for (i = 0; i <= N; i++)
      y += (w[i]*x[i]);
    e = d - y;                   // compute error
    for (i = N; i >= 0; i--)
    {
      w[i] += (BETA*e*x[i]);     // update filter weights
      if (i != 0)
      x[i] = x[i-1];             // shift data in delay line
    }
    desired[t] = d;              // store results
    y_out[t] = y;
    error[t] = e;
  }
  stm32f7_LCD_init(0, SOURCE_FILE_NAME, GRAPH);

  while(1)
  {
    plotWave(desired, NUM_ITERS, 0, 0);
    proceed_statement();
    plotWave(y_out, NUM_ITERS, 0, 0);
    proceed_statement();
    plotWave(error, NUM_ITERS, 0, 0);
    proceed_statement();
  }
}
