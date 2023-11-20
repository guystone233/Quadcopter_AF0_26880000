#ifndef __TASKS_H
#define __TASKS_H

#include "stm32f4xx.h"

#include "ucos_ii.h"
#include "os_cpu.h"

#include "led.h"
#include "oled.h"
#include "usart.h"
#include "tim.h"
#include "gy86.h"
#include "i2c.h"
#include "ano.h"
#include "ekf.h"

extern int dutyCycleArray[6];

/* Task Stacks */
extern OS_STK Task1000HZStk[];
// extern OS_STK GY86TaskStk[];
// extern OS_STK KalmanTaskStk[];
// extern OS_STK SendTaskStk[];
// extern OS_STK OLEDTaskStk[];

extern OS_STK Task500HZStk[];
// extern OS_STK InnerLoopTaskStk[];
// extern OS_STK MotorTaskStk[];

extern OS_STK Task250HZStk[];
// extern OS_STK OuterLoopTaskStk[];
// extern OS_STK BlinkTaskStk[];
/* Task Stacks End */

extern INT8U OSPrioCur;
extern INT8U OSPrioHighRdy;

extern int16_t accx_read, accy_read, accz_read;
extern int16_t gyrox_read, gyroy_read, gyroz_read;
extern int16_t magx_read, magy_read, magz_read;

// 1000Hz Tasks
void Task1000HZ(void *p_arg);
void GY86Task();
void KalmanTask();
void SendTask();
void OLEDTask();
// void GY86Task(void *p_arg);
// void KalmanTask(void *p_arg);
// void SendTask(void *p_arg);
// void OLEDTask(void *p_arg);

// 500Hz Tasks
void Task500HZ(void *p_arg);
void InnerLoopTask();
void MotorTask();
// void InnerLoopTask(void *p_arg);
// void MotorTask(void *p_arg);

// 250Hz Tasks
void Task250HZ(void *p_arg);
void OuterLoopTask();
void BlinkTask();
// void OuterLoopTask(void *p_arg);
// void BlinkTask(void *p_arg);


#endif /* TASKS */