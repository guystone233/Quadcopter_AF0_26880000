#ifndef __TIM_H_
#define __TIM_H_

#include "stm32f4xx.h"
#include "os_cpu.h"
#include "ucos_ii.h"
#include "tasks.h"

void Tim_Init();
void TIM3_PPM_Init(void);
void TIM1_PWM_Init(void);
void MotorInit(void);
uint8_t CheckMotorLock(void);
void TIM3_IRQHandler(void);
uint16_t calculateDutyCycle(uint16_t period);
void setPWMDutyCycle(TIM_TypeDef *TIMx, uint16_t channel, uint16_t dutyCycle);
void PWM_output(void);
uint8_t CheckMotorInit(void);
void StoreDutyCycle(uint16_t *dutyCycleArray, uint16_t *ppm_CCR1data);

#endif /* TIM */