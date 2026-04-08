#include "stm32f7_prbs.h"

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
