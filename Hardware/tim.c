#include "tim.h"

// 定义全局变量，存储接收到的PPM数据
volatile uint16_t ppmData[7], ppm_CCR1data[7];
volatile int dutyCycleArray[6];
uint16_t dutyCycle1;
uint16_t dutyCycle2;
uint16_t dutyCycle3;
uint16_t dutyCycle4;
uint16_t dutyCycle5;
int dutyCycle6;

uint8_t psc = 84 - 1;
uint32_t arr = 0XFFFF;

void TIM3_IRQHandler(void)
{
	
    if (TIM3->SR & TIM_SR_CC1IF)
    {
        static uint8_t ppmChannel = 0;
        static uint32_t lastCapture = 0;
        uint32_t CCR_1;
        uint32_t CCR2;

        TIM3->SR &= ~TIM_SR_CC1IF;

        uint32_t timeout = 10000;

        if (ppmChannel < 7)
        {
            // 检测到PPM帧起始信号，重置计数器
            TIM3->CNT = 0;
            while ((TIM3->SR & TIM_SR_CC1IF) == 0)
            {
                if (--timeout == 0)
                {
                    return;
                }
            }
            if (TIM3->SR & TIM_SR_CC1IF)
            {
                TIM3->SR &= ~TIM_SR_CC1IF;
                CCR_1 = TIM3->CCR1;
                CCR2 = TIM3->CCR2;

                // 计算两次捕获之间的时间差，得到PPM脉冲宽度

                if (TIM3->CCR2 > 0x1500)
                {
                    TIM3->SR &= ~TIM_SR_CC1IF;
                    ppmChannel = 1;
                    ppm_CCR1data[ppmChannel - 1] = CCR_1;
                    ppmData[ppmChannel - 1] = CCR2;
                    TIM3->CNT = 0;
                    ppmChannel++;
                    while ((TIM3->SR & TIM_SR_CC1IF) == 0)
                        ;
                    for (int i = 0; i < 6; i++)
                    {
                        if (TIM3->CCR2 > 0x1500)
                            break;
                        while ((TIM3->SR & TIM_SR_CC1IF) == 0)
                            ;
                        TIM3->SR &= ~TIM_SR_CC1IF;
                        CCR_1 = TIM3->CCR1;
                        CCR2 = TIM3->CCR2;
                        ppm_CCR1data[ppmChannel - 1] = CCR_1;
                        ppmData[ppmChannel - 1] = CCR2;
                        TIM3->CNT = 0;
                        ppmChannel++;
                    }
                }
            }
        }
        ppmChannel++;
        if (ppmChannel > 7)
        {
            ppmChannel = 0;
        }
    }
}

// 配置TIM1用于PWM输出
void TIM1_PWM_Init(void)
{

    // 使能TIM1时钟
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    // 配置GPIO引脚
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    GPIOA->OSPEEDR |= 0x00FF0000;
    GPIOA->MODER |= (GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1 |
                     GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1);
    GPIOA->OTYPER |= 0x00AA0000;

    GPIOA->AFR[1] |= 0x1111;

    // 配置TIM1基本参数
    TIM1->PSC = 100 - 1;  // 84MHz时钟分频为84，得到1MHz计数频率
    TIM1->ARR = 3200 - 1; // PWM周期为20ms

    // 配置TIM1通道1~4为PWM输出模式
    // OC1~4设置PWM1模式
    TIM1->CCMR1 |= (6 << 4 | 6 << 12);
    TIM1->CCMR2 |= (6 << 4 | 6 << 12);
    // 使能预装载寄存器
    TIM1->CCMR1 |= 0x6868;
    TIM1->CCMR2 |= 0x6868;

    TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E |
                   TIM_CCER_CC3E | TIM_CCER_CC4E);
    //	TIM1->CCER|=~(1<<1);
    TIM1->BDTR |= 1 << 15;

    // 初始化所有寄存器
    TIM1->EGR |= 1 << 0;

    // 启动TIM1
    TIM1->CR1 |= TIM_CR1_CEN;
    TIM1->CR1 |= 1 << 7; // 自动重载预装载使能
}

// 配置TIM3用于PPM输入捕获
void TIM3_PPM_Init(void)
{
    // 使能TIM3和GPIOA时钟
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // 配置GPIO引脚
    GPIOA->MODER |= GPIO_MODER_MODER6_1;
    GPIOA->AFR[0] |= 0x2 << 6 * 4;

    // 配置TIM3基本参数
    TIM3->PSC = psc - 1; // 84MHz时钟分频为84，得到1MHz计数频率
    TIM3->ARR = arr - 1;

    // 递增计数，边沿对齐
    TIM3->CR1 &= ~(1 << 4);
    TIM3->CR1 &= ~(3 << 5);

    // 配置TIM3通道1为输入捕获模式
    // IC1->TI1
    TIM3->CCMR1 |= 1 << 0;
    TIM3->CCMR1 |= 6 << 4;
    // 上升沿触发
    TIM3->CCER &= ~(15 << 0);

    // 配置TIM3通道2为输入捕获模式
    // IC2->TI1
    TIM3->CCMR1 |= 1 << 9;
    // 下降沿触发
    TIM3->CCER |= 2 << 4;

    // 配置通道1为有效触发
    TIM3->SMCR |= 5 << 4;

    // 从模式配置为复位模式
    TIM3->SMCR |= 1 << 2;

    // 使能TIM3通道1,2捕获
    TIM3->CCER |= 0x0011;

    // 使能TIM3捕获中断
    TIM3->DIER |= TIM_DIER_CC1IE;

    // 配置中断优先级
    NVIC_SetPriority(TIM3_IRQn, 2);
    NVIC_EnableIRQ(TIM3_IRQn);

    // 启动TIM3，使能计数器
    TIM3->CR1 |= TIM_CR1_CEN;
}

// 计算占空比（以百分比表示）
int calculateDutyCycle(uint16_t pulseWidth, uint16_t period)
{
    int duty = (100 * pulseWidth) / period;
    ;
    duty = (5 * (duty - 60));
    return duty;
}

// 设置PWM输出占空比
void setPWMDutyCycle(TIM_TypeDef *TIMx, uint32_t channel, uint16_t dutyCycle)
{
    if (dutyCycle > 70)
    {
        dutyCycle = (25 * (dutyCycle - 70)) / 10 + 75;
    }
    else
        dutyCycle = 75 - ((25 * (70 - dutyCycle)) / 10);
    uint16_t pulse = (dutyCycle * (TIMx->ARR + 1)) / 100;

    switch (channel)
    {
    case 1:
        TIMx->CCR1 = pulse;
        break;
    case 2:
        TIMx->CCR2 = pulse;
        break;
    case 3:
        TIMx->CCR3 = pulse;
        break;
    case 4:
        TIMx->CCR4 = pulse;
        break;
    default:
        break;
    }
}

// 存储占空比数据到数组中
void storeDutyCycle(int dutyCycle1, int dutyCycle2, int dutyCycle3, int dutyCycle4, int dutyCycle5, int dutyCycle6)
{
    dutyCycleArray[0] = dutyCycle1;
    dutyCycleArray[1] = dutyCycle2;
    dutyCycleArray[2] = dutyCycle3;
    dutyCycleArray[3] = dutyCycle4;
    dutyCycleArray[4] = dutyCycle5;
    dutyCycleArray[5] = dutyCycle6;
}

void PWM_output(void)
{

    dutyCycle1 = calculateDutyCycle(ppmData[1], ppm_CCR1data[1]);
    dutyCycle2 = calculateDutyCycle(ppmData[2], ppm_CCR1data[2]);
    dutyCycle3 = calculateDutyCycle(ppmData[3], ppm_CCR1data[3]);
    dutyCycle4 = calculateDutyCycle(ppmData[4], ppm_CCR1data[4]);
    dutyCycle5 = calculateDutyCycle(ppmData[5], ppm_CCR1data[5]);
    dutyCycle6 = calculateDutyCycle(ppmData[6], ppm_CCR1data[6]);

    storeDutyCycle(dutyCycle1, dutyCycle2, dutyCycle3, dutyCycle4, dutyCycle5, dutyCycle6);

    setPWMDutyCycle(TIM1, 1, dutyCycleArray[0]);
    setPWMDutyCycle(TIM1, 2, dutyCycleArray[1]);
    setPWMDutyCycle(TIM1, 3, dutyCycleArray[2]);
    setPWMDutyCycle(TIM1, 4, dutyCycleArray[3]);
}

void Tim_Init()
{
    TIM1_PWM_Init();
    TIM3_PPM_Init();
}
