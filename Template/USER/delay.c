#include "stm32f4xx.h"                  // Device header

void mydelay(uint16_t ms)
{
	int i=0;
	while(ms--)
	{	
		i=12000;
		while(i--);
	}
}
