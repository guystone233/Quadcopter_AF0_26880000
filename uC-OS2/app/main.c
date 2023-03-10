#include "led.h"
#include "stm32f4xx.h"
#include "ucos_ii.h"
#define TASK_PRIO 1
#define TASK_STK_SIZE 128
static OS_STK task_stk[TASK_STK_SIZE]; // 定义栈

int main(void)
{
	InitLED();
	OSInit();
	OSTaskCreate(FlashLED, (void *)0,
				 &task_stk[TASK_STK_SIZE - 1], TASK_PRIO);
	OSStart();
	return 0;
}