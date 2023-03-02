#include "led.h"
#include "stm32f4xx.h"
#include "ucos_ii.h"
#define STARTUP_TASK_PRIO 1
#define STARTUP_TASK_STK_SIZE 128
static OS_STK startup_task_stk[STARTUP_TASK_STK_SIZE];		  //定义栈
  
int main(void)
{
	OSInit();
	OSTaskCreate(FlashLED,(void *)0,
	&startup_task_stk[STARTUP_TASK_STK_SIZE-1], STARTUP_TASK_PRIO);
 
	OSStart();
    return 0;
 }
