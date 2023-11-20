#ifndef __LED_H
#define __LED_H

#include "stm32f4xx.h"
void LED_Init(void);
void LED_On(void);
void LED_Off(void);
void FlashLED(void);

#endif /* LED */