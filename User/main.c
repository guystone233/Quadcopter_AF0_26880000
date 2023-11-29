#include "stm32f4xx.h"
#include "system_stm32f4xx.h"

#include "ucos_ii.h"
#include "os_cpu.h"

#include "tasks.h"

extern uint32_t SystemCoreClock;

OS_STK InitTaskStk[300];
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
	Tim_Init();

	GY86_Init();

	OLED_Init();
	MotorInit();
	LED_Init();
	ekf_init();

	int16_t accx_read_list[100] = {0}, accy_read_list[100] = {0}, accz_read_list[100] = {0};
	int16_t gyrox_read_list[100] = {0}, gyroy_read_list[100] = {0}, gyroz_read_list[100] = {0};
	for (int i = 0; i < 100; i++) {
		GY86Task();
		accx_read_list[i] = -accx_read;
		accy_read_list[i] = -accy_read;
		accz_read_list[i] = -(accz_read-2048);
		gyrox_read_list[i] = gyrox_read;
		gyroy_read_list[i] = gyroy_read;
		gyroz_read_list[i] = gyroz_read;
	}
	// arm_mean_q15(accx_read_list, 100, &ACC_OFFSET_X);
	// arm_mean_q15(accy_read_list, 100, &ACC_OFFSET_Y);
	// arm_mean_q15(accz_read_list, 100, &ACC_OFFSET_Z);
	arm_mean_q15(gyrox_read_list, 100, &GYRO_OFFSET_X);
	arm_mean_q15(gyroy_read_list, 100, &GYRO_OFFSET_Y);
	arm_mean_q15(gyroz_read_list, 100, &GYRO_OFFSET_Z);
	// ACC_OFFSET_Z = 0;



	OSTaskCreate(Task1000HZ, NULL, &Task1000HZStk[599], 2);
	// OSTaskCreate(GY86Task, NULL, &GY86TaskStk[99], 2);
	// OSTaskCreate(KalmanTask, NULL, &KalmanTaskStk[299], 3);
	// OSTaskCreate(SendTask, NULL, &SendTaskStk[99], 4);
	// OSTaskCreate(OLEDTask, NULL, &OLEDTaskStk[99], 5);

	OSTaskCreate(Task500HZ, NULL, &Task500HZStk[199], 3);
	// OSTaskCreate(InnerLoopTask, NULL, &InnerLoopTaskStk[99], 6);
	// OSTaskCreate(MotorTask, NULL, &MotorTaskStk[99], 7);

	OSTaskCreate(Task250HZ, NULL, &Task250HZStk[199], 4);
	// OSTaskCreate(OuterLoopTask, NULL, &OuterLoopTaskStk[99], 8);
	// OSTaskCreate(BlinkTask, NULL, &BlinkTaskStk[99], 9);

	OSTaskDel(1);
}
