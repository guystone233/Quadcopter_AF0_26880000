#include "stm32f4xx.h" 
#include "string.h" 
#include "stdlib.h"
#include "OLED.h"
#include "usart.h"
#include "i2c.h"
#include "MPU6050.h"
#include "delay.h"

void flash_light(void);
typedef struct
{
	uint32_t SYSCLK_Frequency;
	uint32_t HCLK_Frequency;
	uint32_t PCLK1_Frequency;
	uint32_t PCLK2_Frequency;
} MyClocksTypeDef;

static __I uint8_t APBAHBPrescTable[16] = {0, 0, 0, 0, 1, 2, 3, 4, 1, 2, 3, 4, 6, 7, 8, 9};

void GetMyClocks(MyClocksTypeDef* MyClocks) {

	uint32_t tmp = 0, presc = 0, pllvco = 0, pllp = 2, pllsource = 0, pllm = 2;
	tmp = RCC->CFGR & RCC_CFGR_SWS;
  
	switch (tmp)
	{
	case 0x00:  /* HSI used as system clock source */
		MyClocks->SYSCLK_Frequency = HSI_VALUE;
		break;
	case 0x04:  /* HSE used as system clock  source */
		MyClocks->SYSCLK_Frequency = HSE_VALUE;
		break;
	case 0x08:  /* PLL P used as system clock  source */ 
		/* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLLM) * PLLN SYSCLK = PLL_VCO / PLLP */    
		pllsource = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) >> 22;
		pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;
		
		if (pllsource != 0)
		{
		  /* HSE used as PLL clock source */
		  pllvco = (HSE_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
		}
		else
		{
		  /* HSI used as PLL clock source */
		  pllvco = (HSI_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);      
		}
		
		pllp = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >>16) + 1 ) *2;
		MyClocks->SYSCLK_Frequency = pllvco/pllp;
		break;
	default:
		MyClocks->SYSCLK_Frequency = HSI_VALUE;
		break;
	}
	/* Compute HCLK, PCLK1 and PCLK2 clocks frequencies ------------------------*/
  
	/* Get HCLK prescaler */
	tmp = RCC->CFGR & RCC_CFGR_HPRE;
	tmp = tmp >> 4;
	presc = APBAHBPrescTable[tmp];
	/* HCLK clock frequency */
	MyClocks->HCLK_Frequency = MyClocks->SYSCLK_Frequency >> presc;

	/* Get PCLK1 prescaler */
	tmp = RCC->CFGR & RCC_CFGR_PPRE1;
	tmp = tmp >> 10;
	presc = APBAHBPrescTable[tmp];
	/* PCLK1 clock frequency */
	MyClocks->PCLK1_Frequency = MyClocks->HCLK_Frequency >> presc;

	/* Get PCLK2 prescaler */
	tmp = RCC->CFGR & RCC_CFGR_PPRE2;
	tmp = tmp >> 13;
	presc = APBAHBPrescTable[tmp];
	/* PCLK2 clock frequency */
	MyClocks->PCLK2_Frequency = MyClocks->HCLK_Frequency >> presc;
}

void AutoBaudRate(void) {
	uint32_t MyBaudRate = 9600;
	
	uint32_t tmpreg = 0x00, apbclock = 0x00;
	uint32_t integerdivider = 0x00;
	uint32_t fractionaldivider = 0x00;
	
	MyClocksTypeDef MyClocksStatus;
	GetMyClocks(&MyClocksStatus);
	
	USART_TypeDef* USARTx = USART1;

	apbclock = MyClocksStatus.PCLK2_Frequency;
	  
	/* Determine the integer part */
	if ((USART1->CR1 & USART_CR1_OVER8) != 0)
	{
	  /* Integer part computing in case Oversampling mode is 8 Samples */
	  integerdivider = ((25 * apbclock) / (2 * (MyBaudRate)));    
	}
	else /* if ((USARTx->CR1 & USART_CR1_OVER8) == 0) */
	{
	  /* Integer part computing in case Oversampling mode is 16 Samples */
	  integerdivider = ((25 * apbclock) / (4 * (MyBaudRate)));    
	}
	tmpreg = (integerdivider / 100) << 4;

	/* Determine the fractional part */
	fractionaldivider = integerdivider - (100 * (tmpreg >> 4));

	/* Implement the fractional part in the register */
	if ((USART1->CR1 & USART_CR1_OVER8) != 0)
	{
	  tmpreg |= ((((fractionaldivider * 8) + 50) / 100)) & ((uint8_t)0x07);
	}
	else /* if ((USARTx->CR1 & USART_CR1_OVER8) == 0) */
	{
	  tmpreg |= ((((fractionaldivider * 16) + 50) / 100)) & ((uint8_t)0x0F);
	}
	  
	/* Write to USART BRR register */
	USART1->BRR = (uint16_t)tmpreg;
	
}
int main()
{		
	USARTInit();
	OLED_Init();
	I2C_Configuration();
	SendString("114514\r\n");
	MPU_Init2();
	OLED_ShowNum(2,0,SystemCoreClock,16);
	char a[6],b[6],c[6];
	uint16_t x,y,z;
	SendString("114514\r\n");
	//flash_light();
	
	x = MPU_dataout(0x3B,0x3C);
	y = MPU_dataout(0x3D,0x3E);
	z = MPU_dataout(0x3F,0x40);
	OLED_ShowString(1,1,"G");
	OLED_ShowNum(1,2,9,1);
	OLED_ShowString(2,1,"AF0_26880000");
	while(1){
		x = MPU_dataout(0x3B,0x3C);
		y = MPU_dataout(0x3D,0x3E);
		z = MPU_dataout(0x3F,0x40);
		sprintf(a,"%d\r\n",x);
		sprintf(b,"%d\r\n",y);
		sprintf(c,"%d\r\n",z);
		SendString("x:");
		SendString(a);
		SendString("y:");
		SendString(b);
		SendString("z:");
		SendString(c);
		OLED_ShowNum(3,1,MPU_dataout(0x43,0x44),4);//陀螺仪
		OLED_ShowNum(3,6,MPU_dataout(0x45,0x46),4);
		OLED_ShowNum(3,11,MPU_dataout(0x47,0x48),4);
		
		OLED_ShowNum(4,1,MPU_dataout(0x3B,0x3C),4);//加速度
		OLED_ShowNum(4,6,MPU_dataout(0x3D,0x3E),4);
		OLED_ShowNum(4,11,MPU_dataout(0x3F,0x40),4);
	}
}

	//OLED_ShowBinNum(2,4,GPIOB->PUPDR,14);
	//OLED_ShowNum(2,0,GPIOB->PUPDR,16);
	//OLED_ShowHexNum(2,0,6,16);
	//OLED_ShowNum(2,0,SystemCoreClock,16);
	//SendString("\r\n");
	//SendString(ToString(RCC->AHB1ENR));
	//SendString("\r\n");
	//SendString(ToString(GPIOB->MODER));
	//SendString("\r\n");
	//SendString(ToString(GPIOB->OSPEEDR));
	//SendString("\r\n");
	//SendString(ToString(GPIOB->OTYPER));
	//SendString("\r\n");
	//SendString(ToString(GPIOB->PUPDR));
	//SendString("\r\n");
	//SendString(ToString(USART1->BRR));
	//SendString("\r\n");
	
	//flash_light();	
	
	//SystemCoreClockUpdate();
	//USARTInit();
	//AutoBaudRate();

void flash_light()
{
	RCC->AHB1ENR |= 0x1 << 0; /* GPIOA 时钟外设使能 6.3.12 */
	
	GPIOA->MODER   &= ~(0x3 << 2*5);  /* GPIO 端口模式寄存器 7.4.1 */
	GPIOA->MODER   |=   0x1 << 2*5;
	
	GPIOA->OTYPER  &=   0x0 << 5;  /* GPIO 端口输出类型寄存器 7.4.2 */
	
	GPIOA->OSPEEDR &= ~(0x3 << 2*5); /* GPIO 端口输出速度寄存器 7.4.3 */
	GPIOA->OSPEEDR |=   0x2 << 2*5;
	
	GPIOA->PUPDR   &= ~(0x3 << 2*5); /* GPIO 端口上拉/下拉寄存器 7.4.4 */
	
	
	while(1)
	{
		GPIOA->ODR |= 0x1 << 5;
        int temp_cnt=1000000;
        while(temp_cnt--);
        GPIOA->ODR &= ~(0x1 << 5);
        temp_cnt=1000000;
        while(temp_cnt--);
	}
}
