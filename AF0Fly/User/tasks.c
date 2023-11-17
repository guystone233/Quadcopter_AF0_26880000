#include "tasks.h"

uint16_t accx = 0, accy = 0, accz = 0;
uint16_t gyrox = 0, gyroy = 0, gyroz = 0;
uint16_t magx = 0, magy = 0, magz = 0;

INT32U gy86_time = 0;


OS_STK OledTaskStk[100];
OS_STK TimTaskStk[100];
OS_STK MotorTaskStk[100];
OS_STK GY86TaskStk[100];

OS_STK TestTaskStk1[100];
OS_STK TestTaskStk2[100];
OS_STK TestTaskStk3[20000];
OS_STK TestTaskStk4[100];

uint16_t data;


void TestTask1(void *p_arg)
{
	while(1){
		LED_On();
		delay_ms(100000000);
		OSTimeDly(10);
	}
}

void TestTask2(void *p_arg)
{
	while(1){
		LED_Off();
		OSTimeDly(10);
	}
}


void TestTask3(void *p_arg)
{
	ekf_init();
	OSTimeDly(30);
	while(1){
		ekf_calculate();
		OSTimeDly(1);
	}
	
}


void TestTask4(void *p_arg)
{
	while(1){
		OLED_ShowChar(2,2,ReadByte());
		for(int i=0;i<10000000;i++);
		OLED_Clear();
		OSTimeDly(100);
	}
}


void OledTask(void *p_arg)
{
	while(1){
		OLED_ShowString(1,1,"T:");
		OLED_ShowNum(1,3,OSTime,5);
		
		OLED_ShowString(1,9,"COS:");
		OLED_ShowNum(1,13,gy86_time,4);

		OLED_ShowString(2,1,"X:");
		OLED_ShowNum(2,3,accx,4);
		OLED_ShowNum(2,8,gyrox,4);
		OLED_ShowNum(2,13,magx,4);

		OLED_ShowString(3,1,"Y:");
		OLED_ShowNum(3,3,accy,4);
		OLED_ShowNum(3,8,gyroy,4);
		OLED_ShowNum(3,13,magy,4);

		OLED_ShowString(4,1,"Z:");
		OLED_ShowNum(4,3,accz,4);
		OLED_ShowNum(4,8,gyroz,4);
		OLED_ShowNum(4,13,magz,4);

		OSTimeDly(30);
	}
}


void TimTask(void *p_arg)
{
	while(1){
		PWM_output();
		OSTimeDly(15);
	}
	
}


void MotorInitTask(void *p_arg)
{
	
	setPWMDutyCycle(TIM1, 1,  80);
	setPWMDutyCycle(TIM1, 2,  80);
	setPWMDutyCycle(TIM1, 3,  80);
	setPWMDutyCycle(TIM1, 4,  80);
	delay_ms(1000000000);
	delay_ms(1000000000);
	delay_ms(1000000000);
	
	setPWMDutyCycle(TIM1, 1,  60);
	setPWMDutyCycle(TIM1, 2,  60);
	setPWMDutyCycle(TIM1, 3,  60);
	setPWMDutyCycle(TIM1, 4,  60);
	delay_ms(1000000000/2);
	delay_ms(1000000000);
	OSTaskDel(1);
}


void MPU6050Task(void *p_arg)
{
	while(1){
		INT32U tick1 = OSTimeGet();
		accx = I2C1_GetMPU6050X();
		accy = I2C1_GetMPU6050Y();
		accz = I2C1_GetMPU6050Z();
		gyrox = I2C1_GetGyroX();
		gyroy = I2C1_GetGyroY();
		gyroz = I2C1_GetGyroZ();
		magx = I2C1_GetHMC5883X();
		magy = I2C1_GetHMC5883Y();
		magz = I2C1_GetHMC5883Z();
		INT32U tick2 = OSTimeGet();
		gy86_time = tick2 - tick1;
		OSTimeDly(30);
	}
}