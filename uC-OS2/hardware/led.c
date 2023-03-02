#include "led.h"
void InitLED(void){
	RCC->AHB1ENR |= 0x1 << 0; /* GPIOA 时钟外设使能 6.3.12 */
	
	GPIOA->MODER   &= ~(0x3 << 2*5);  /* GPIO 端口模式寄存器 7.4.1 */
	GPIOA->MODER   |=   0x1 << 2*5;
	
	GPIOA->OTYPER  &=   0x0 << 5;  /* GPIO 端口输出类型寄存器 7.4.2 */
	
	GPIOA->OSPEEDR &= ~(0x3 << 2*5); /* GPIO 端口输出速度寄存器 7.4.3 */
	GPIOA->OSPEEDR |=   0x2 << 2*5;
	
	GPIOA->PUPDR   &= ~(0x3 << 2*5); /* GPIO 端口上拉/下拉寄存器 7.4.4 */
}
void LED_On(void){
	GPIOA->ODR |= 0x1 << 5;
}
void LED_Off(void){
	GPIOA->ODR &= ~(0x1 << 5);
}
void FlashLED(void)
{
	while(1)
	{
		LED_On();
        int temp_cnt=1000000;
        while(temp_cnt--);
        LED_Off();
        temp_cnt=1000000;
        while(temp_cnt--);
	}
}
