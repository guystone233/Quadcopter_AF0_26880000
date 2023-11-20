#ifndef __LED_H
#define __LED_H

#include "stm32f4xx.h"
void InitLED(void);
void LED_On(void);
void LED_Off(void);
void FlashLED(void);

#endif /* LED */