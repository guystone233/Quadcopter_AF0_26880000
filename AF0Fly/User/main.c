#include "stm32f4xx.h"
#include "system_stm32f4xx.h"

#include "ucos_ii.h"
#include "os_cpu.h"

#include "tasks.h"
#include "ekf.h"


extern uint32_t SystemCoreClock;

OS_STK InitTaskStk[800];
void InitTask(void *p_arg);


int main(void)
{
	OS_CPU_SysTickInitFreq(84000000);

	
	USARTInit();
	OSInit();
	
	OSTaskCreate(InitTask, NULL, &InitTaskStk[799], 1);
	
	OSStart();
	
	return 0;
}


void InitTask(void *p_arg)
{
	
	InitLED();
	Tim_Init();
	MPU6050_Init();
	OLED_Init();
//	USART1_printf("TEST114514\r\n");
	
	OSTaskCreate(TimTask, NULL, &TimTaskStk[99], 4);
	// OSTaskCreate(MPU6050Task, NULL, &GY86TaskStk[99], 3);
	OSTaskCreate(OledTask, NULL, &OledTaskStk[99], 5);
	OSTaskCreate(TestTask1, NULL, &TestTaskStk1[99], 7);
	OSTaskCreate(TestTask2, NULL, &TestTaskStk2[99], 8);
	OSTaskCreate(TestTask3, NULL, &TestTaskStk3[19999], 2);
	OSTaskDel(1);
}

