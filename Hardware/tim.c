#include "tim.h"

#define MotorLockOn 1
#define MotorLockOff 0
#define pulseWidth 20000
// 定义全局变量，存储接收到的PPM数据
 uint16_t ppmData[7], ppm_CCR1data[7];
 uint16_t dutyCycleArray[7];

uint8_t LockStatus = 0;
uint8_t psc = 84 - 1;
uint32_t arr = 0XFFFF;
uint32_t CCR_1;
uint32_t CCR2;

/*标准遥控器接收机的频率均为50MHz,即周期为20ms,ppm将一个周期划分为10个channel,1个channel对应2ms的长度       */
/*电调油门行程为1100us~1940us高电平,一个周期的长度为2000us,对应的占空比为5%~10%                            */
/*我们使用的FS-i6遥控器及FS-iA6B,理论来讲有一共10个channel,这里我们对应的固件只输入了6个channel             */
/*目前的遥控器Channel分布:                                                                                */
/*低油门:摇杆向下或向左，旋钮逆时针旋转                                                                   */
/*高油门:摇杆向上或向右，旋钮顺时针旋转                                                                   */
/*CH1:右手摇杆左右                                                                                      */
/*CH2:右手摇杆上下                                                                                      */
/*CH3:左手摇杆上下                                                                                      */
/*CH4:左手摇杆左右                                                                                      */
/*CH5:顶部左旋钮                                                                                        */
/*CH6:顶部右旋钮                                                                                        */

void TIM3_IRQHandler(void)
{

    if (TIM3->SR & TIM_SR_CC1IF)
    {
        uint8_t ppmChannel = 0;
        uint32_t lastCapture = 0;

        TIM3->SR &= ~TIM_SR_CC1IF;


				// 检测到PPM帧起始信号，重置计数器
                // 计算两次捕获之间的时间差，得到PPM脉冲宽度

                if (TIM3->CCR1 > 0x1000)
                {
//                    TIM3->SR &= ~TIM_SR_CC1IF;
                    ppm_CCR1data[ppmChannel] = TIM3->CCR1;
                    ppmData[ppmChannel] = TIM3->CCR2;
                    TIM3->CNT = 0;

                    for (int i = 1; i < 7; i++)
                    {

                        while ((TIM3->SR & TIM_SR_CC1IF) == 0)
                            ;
                        TIM3->SR &= ~TIM_SR_CC1IF;
                        CCR_1 = TIM3->CCR1;
                        CCR2 = TIM3->CCR2;
                        ppm_CCR1data[i] = TIM3->CCR1;
                        ppmData[i] = TIM3->CCR2;
                        TIM3->CNT = 0;
                        ppmChannel++;
                    }
                }

//        }
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

//配置TIM2用于PWM输出，引脚为PA0
void TIM2_PWM_Init(void)
{
    // 使能TIM2时钟
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // 配置GPIO引脚
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    GPIOA->OSPEEDR |= 0x00000003;
    GPIOA->MODER |= GPIO_MODER_MODER0_1;
    GPIOA->OTYPER |= 0x00000001;

    GPIOA->AFR[0] |= 0x00000001;

    // 配置TIM2基本参数
    TIM2->PSC = 100 - 1;  // 84MHz时钟分频为84，得到1MHz计数频率
    TIM2->ARR = 3200 - 1; // PWM周期为20ms

    // 配置TIM2通道1为PWM输出模式
    // OC1设置PWM1模式
    TIM2->CCMR1 |= 6 << 4;
    // 使能预装载寄存器
    TIM2->CCMR1 |= 0x6868;

    TIM2->CCER |= TIM_CCER_CC1E;
    //	TIM1->CCER|=~(1<<1);

    // 初始化所有寄存器
    TIM2->EGR |= 1 << 0;

    // 启动TIM2
    TIM2->CR1 |= TIM_CR1_CEN;
    TIM2->CR1 |= 1 << 7; // 自动重载预装载使能
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
uint16_t calculateDutyCycle(uint16_t period)
{
    uint16_t duty = (100 * (period+1)) / pulseWidth;
    ;
    return duty;
}

// 设置PWM输出占空比
void setPWMDutyCycle(TIM_TypeDef *TIMx, uint16_t channel, uint16_t CCR)
{
    switch (channel)
    {
    case 1:
        TIMx->CCR1 = CCR;
        break;
    case 2:
        TIMx->CCR2 = CCR;
        break;
    case 3:
        TIMx->CCR3 = CCR;
        break;
    case 4:
        TIMx->CCR4 = CCR;
        break;
    default:
        break;
    }
}

// 存储占空比数据到数组中


void PWM_output(void)
{
		LockStatus = CheckMotorLock();
#ifdef MOTOR_INIT
		MotorInit();
#endif
		
    StoreDutyCycle(dutyCycleArray, ppm_CCR1data);
    if(!LockStatus)
    {
        for(uint8_t i = 1; i < 5; i++)
        {
            setPWMDutyCycle(TIM1, i, ppm_CCR1data[i]);
        }
    }
}

/*用来初始化电机上电后校准步骤*/
void MotorInit(void)
{
	if(LockStatus == 1)
	{
		if(CheckMotorInit())
		{
			for(uint8_t i = 1; i < 5; i++)
			{
					setPWMDutyCycle(TIM1, i, 1940);
			}
		}
		else
		{
			for(uint8_t i = 1; i < 5; i++)
			{
					setPWMDutyCycle(TIM1, i, 1100);
			}
		}
	}
}

void StoreDutyCycle(uint16_t *dutyCycleArray, uint16_t *ppm_CCR1data)
{
    for(uint8_t i = 1; i < 7; i++)
    {
        dutyCycleArray[i] = calculateDutyCycle(ppm_CCR1data[i]);
    }
}

uint8_t CheckMotorLock(void)
{
    if(dutyCycleArray[5] > 5)
		{
			LockStatus = 1;
			return MotorLockOff;
		}
    if((dutyCycleArray[5] == 5))
		{
				LockStatus = 0;
				return MotorLockOn;
		}
}

uint8_t CheckMotorInit(void)
{
	  if(dutyCycleArray[6] != 5)
		{
			return 1;
		}
		else return 0;
}
void Tim_Init()
{
    TIM1_PWM_Init();
    TIM2_PWM_Init();
    TIM3_PPM_Init();
    
}

    