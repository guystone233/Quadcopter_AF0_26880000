#ifndef __TASKS_H
#define __TASKS_H

#include "stm32f4xx.h"

#include "ucos_ii.h"
#include "os_cpu.h"

#include "led.h"
#include "delay.h"
#include "oled.h"
#include "usart.h"
#include "tim.h"
#include "gy86.h"
#include "i2c.h"

#include "arm_math.h"
#include <stdio.h>
#include "ekf.h"

extern int dutyCycleArray[6];

extern OS_STK OledTaskStk[];
extern OS_STK TimTaskStk[];
extern OS_STK MotorTaskStk[];
extern OS_STK GY86TaskStk[];

extern OS_STK TestTaskStk1[];
extern OS_STK TestTaskStk2[];
extern OS_STK TestTaskStk3[];
extern OS_STK TestTaskStk4[];

extern INT8U OSPrioCur;
extern INT8U OSPrioHighRdy;  

void OledTask(void *p_arg);
void TimTask(void *p_arg);
void MotorInitTask(void *p_arg);
void MPU6050Task(void *p_arg);

void TestTask1(void *p_arg);
void TestTask2(void *p_arg);
void TestTask3(void *p_arg);
void TestTask4(void *p_arg);

#endif /* TASKS */