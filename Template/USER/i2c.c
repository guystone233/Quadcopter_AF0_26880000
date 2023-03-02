#include "stm32f4xx.h"                  // Device header

void I2C_Configuration()
{
	RCC->AHB1ENR |= 0x01<<1;//打开GPIO
	RCC->APB1ENR |= 0x01<<21;//打开IIC
	
	GPIOB->MODER&=~(0x03<<(2*8));
	GPIOB->MODER&=~(0x03<<(2*9));
	GPIOB->MODER|=(0x02<<(2*8));
	GPIOB->MODER|=(0x02<<(2*9));//配置模式成复用模式
	
	//开漏输出
	GPIOB->OTYPER|=0x01<<8;
	GPIOB->OTYPER|=0x01<<9;
	//速率：100MHz
	GPIOB->OSPEEDR&=~(0x03<<(2*8));
	GPIOB->OSPEEDR&=~(0x03<<(2*9));
	GPIOB->OSPEEDR|=(0x03<<(2*8));
	GPIOB->OSPEEDR|=(0x03<<(2*9));
	//不需要上拉或下拉 置为00
	GPIOB->PUPDR=0;
	//配置复用 AFRH AF4(IIC1)(GPIOPB8和9)
	GPIOB->AFR[1]&=~(0xF);
	GPIOB->AFR[1]&=~(0xF<<4);
	GPIOB->AFR[1]|=0x04;
	GPIOB->AFR[1]|=0x04<<4;
	
	
//配置IIC
	//选择IIC模式
	I2C1->CR1&=~((0x01)<<1);
	//设置频率为42MHz
	I2C1->CR2|=0x2A;
	//设置自身地址 不用跟其他地址冲突就行
	I2C1->OAR1|=0x0A;
	//设置TRISE
	I2C1->TRISE|=I2C1->CR2+1;
	//设置总线速度
	I2C1->CCR |= 0xD2;
	//上电
	I2C1->CR1|=0x01;
	//设置应答位,开启应答
	I2C1->CR1|=0x01<<10;
}

//检测函数EVx
uint8_t MY_CHECK(uint16_t SR1,uint16_t SR2)
{
	//若和我们期望一致则返回1，否则返回0
	if(I2C1->SR1&0x0002||!(I2C1->SR1&0x0010))//检测ADDR是否为1和检测STOPF是否清零
	{
		if((I2C1->SR1&SR1)==SR1&&(I2C1->SR2&SR2)==SR2)//SR1和SR2都符合我们期望
		{
			return 1;
		}
		else{
			return 0;
		}
	}
	else{
		return 0;
	}
	
}

//发送一个字节数据
void I2C_Send_Byte(uint8_t slavead,uint8_t writead,uint8_t data)//需要发送从设备地址，从设备内寄存器地址，data
{
	//设置起始位
	I2C1->CR1|=(0x01)<<8;
	//EV5
	while(!(MY_CHECK(0x0001,0x0003)));
	//将slavead写入DR寄存器
	I2C1->DR=slavead;
	//EV6EV8
	//while(!(MY_CHECK(0x0080,0x0007)));
	while(!(MY_CHECK(0x0082,0x0007)));//TRA被置1（发送器）
	//发送寄存器地址
	I2C1->DR=writead;
	//while(!(MY_CHECK(0x0080,0x0007)));
	while(!(MY_CHECK(0x0084,0x0007)));
	//发送数据
	I2C1->DR=data;
	//结束数据
	//while(!(MY_CHECK(0x0080,0x0007)));
	while(!(MY_CHECK(0x0084,0x0007)));
	I2C1->CR1|=0x01<<9;
	
}
//读一个字节
uint8_t I2C_Read_Byte(uint8_t slavead,uint8_t readad)
{
	uint8_t data;
	//设置起始位
	I2C1->CR1|=(0x01)<<8;
	//EV5
	while(!(MY_CHECK(0x0001,0x0003)));
	//将slavead写入DR寄存器
	I2C1->DR=slavead;
	//EV6EV8
	//while(!(MY_CHECK(0x0080,0x0007)));
	while(!(MY_CHECK(0x0082,0x0007)));
	
	//发送寄存器地址
	I2C1->DR=readad;
	//while(!(MY_CHECK(0x0080,0x0007)));
	while(!(MY_CHECK(0x0084,0x0007)));
	
	//重新设置起始位
	I2C1->CR1|=(0x01)<<8;
	while(!(MY_CHECK(0x0001,0x0003)));
	//将slavead写入DR寄存器
	I2C1->DR=slavead|(1);
	//while(!(MY_CHECK(0x0040,0x0003)));
	while(!(MY_CHECK(0x0002,0x0003)));
	I2C1->CR1&=~(0x01<<10);//ACK=0
	I2C1->CR1|=0x01<<9;
	while(!(MY_CHECK(0x0040,0x0003)));
	data = I2C1->DR;
	return data;
}






