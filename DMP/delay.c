#include "delay.h"

void delay_ms(uint32_t Delay)
{
  uint32_t tickstart = OSTimeGet();
  uint32_t wait = OS_TICKS_PER_SEC * ((INT32U)Delay + 500uL / OS_TICKS_PER_SEC) / 1000uL;

//   /* Add a freq to guarantee minimum wait */
//   if (wait < 0xFFFFFFFFU)
//   {
//     wait += (uint32_t)(uwTickFreq);
//   }

  while((OSTimeGet() - tickstart) < wait)
  {
  }
}