#ifndef __TIM_H_
#define __TIM_H_

#include "stm32f4xx.h"
#include "os_cpu.h"
#include "ucos_ii.h"

void Tim_Init();
void TIM3_PPM_Init(void);
void TIM1_PWM_Init(void);
void TIM3_IRQHandler(void);
int calculateDutyCycle(uint16_t pulseWidth, uint16_t period);
void setPWMDutyCycle(TIM_TypeDef *TIMx, uint32_t channel, uint16_t dutyCycle);
void PWM_output(void);
void storeDutyCycle(int dutyCycle1, int dutyCycle2, int dutyCycle3, int dutyCycle4, int dutyCycle5, int dutyCycle6);

#endif /* TIM */