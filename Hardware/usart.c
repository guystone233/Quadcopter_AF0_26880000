#include "usart.h"

// char USART1_RX_BUF[USART1_RX_BUF_SIZE]; // 接收缓冲,最大USART1_RX_BUF_SIZE个字节.
char USART1_TX_BUF[USART1_TX_BUF_SIZE]; // 发送缓冲,最大USART1_TX_BUF_SIZE字节.

void DMA2_Stream7_IRQHandler(void){
	if(DMA_GetITStatus(DMA2_Stream7,DMA_IT_TCIF7) != RESET)   
    {  
        /* 清除标志位 */
        DMA_ClearFlag(DMA2_Stream7,DMA_IT_TCIF7);  
        /* 关闭DMA */
        DMA_Cmd(DMA2_Stream7,DISABLE);
        /* 打开发送完成中断,确保最后一个字节发送成功 */
        USART_ITConfig(USART1,USART_IT_TC,ENABLE);  
    }  
}

void USART1_IRQHandler(void){
    if(USART_GetITStatus(USART1, USART_IT_TXE) == RESET)  
    {  
        /* 关闭发送完成中断  */ 
        USART_ITConfig(USART1,USART_IT_TC,DISABLE);  
    }   
}

void DMA_USART1_Send(char *data,int size)
{
    memcpy(USART1_TX_BUF,data,size);                                                                            //复制数据到DMA发送缓存区
    while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE);                                                //确保DMA可以被设置
    DMA_SetCurrDataCounter(DMA2_Stream7,size);                                                                //设置数据传输长度
    DMA_Cmd(DMA2_Stream7,ENABLE);                                                                                            //打开DMA数据流，开始发送
}

void USART1_printf(char *format, ...)
{
	//VA_LIST 是在C语言中解决变参问题的一组宏，所在头文件：#include <stdarg.h>，用于获取不确定个数的参数。
	va_list arg_ptr;														//实例化可变长参数列表
	
	while(DMA_GetCmdStatus(DMA2_Stream7) != DISABLE);													//等待上一次发送完成（USART2_TX_FLAG为1即还在发送数据）
	
	va_start(arg_ptr, format); 												//初始化可变参数列表，设置format为可变长列表的起始点（第一个元素）
	
	// USART2_MAX_TX_LEN+1可接受的最大字符数(非字节数，UNICODE一个字符两个字节), 防止产生数组越界
	vsnprintf((char*)USART1_TX_BUF, USART1_TX_BUF_SIZE+1, format, arg_ptr);	//从USART2_TX_BUF的首地址开始拼合，拼合format内容；USART2_MAX_TX_LEN+1限制长度，防止产生数组越界
	
	va_end(arg_ptr);														//注意必须关闭
 
	DMA_USART1_Send(USART1_TX_BUF,strlen((const char*)USART1_TX_BUF));	//发送USART2_TX_BUF内容
}

void SendByte(char ch)
{
	USART1->DR = ch;
	while ((USART1->SR & (1 << 7)) == 0)
		; // 等待发送结束
}

void SendString(char *ch)
{
	for (int i = 0; ch[i] != '\0'; i++)
		SendByte(ch[i]);
}

char ReadByte(void)
{
	while ((USART1->SR & (1 << 5)) == 0)
		; // 查询是否受到数据
	char tmp = USART1->DR & (uint16_t)0x01FF;
	if (tmp != 0)
		return tmp;
	else
		return '*';
}

#define pin1 6
#define pin2 7

void USARTInit(void)
{
	/*** RCC ***/
	RCC->AHB1ENR |= 0x1 << 1; // Enable GPIOA 6.3.9
	RCC->APB2ENR |= 0x1 << 4; // Enable USART1 6.3.12

	/*** GPIO ***/
	GPIOB->AFR[0] &= (unsigned int)~(0xF << 4 * pin1); // AFRL PA9 Alternate 0000 8.4.9
	GPIOB->AFR[0] &= (unsigned int)~(0xF << 4 * pin2); // AFRL PA10 Alternate 0000 8.4.9

	GPIOB->AFR[0] |= 0x7 << 4 * pin1; // PA9 Alternate AF7-USART1..2: 0111 8.4.9 / refer: p151
	GPIOB->AFR[0] |= 0x7 << 4 * pin2; // PA10 Alternate AF7-USART1..2: 0111 8.4.9 / refer: p151

	GPIOB->MODER &= (unsigned int)~(0x3 << 2 * pin1); // GPIOA9 reset 00 8.4.1
	GPIOB->MODER &= (unsigned int)~(0x3 << 2 * pin2);

	GPIOB->MODER |= 0x2 << 2 * pin1; // GPIOA9 Altermate function mode 8.4.1
	GPIOB->MODER |= 0x2 << 2 * pin2; // GPIOA10 Altermate function mode 8.4.1

	GPIOB->OSPEEDR &= (unsigned int)~(0x3 << 2 * pin1);
	GPIOB->OSPEEDR &= (unsigned int)~(0x3 << 2 * pin2);

	GPIOB->OSPEEDR |= 0x2 << 2 * pin1; // GPIOA9 High speed 10 8.4.3
	GPIOB->OSPEEDR |= 0x2 << 2 * pin2; // GPIOA10 High speed 10

	GPIOB->OTYPER &= (unsigned int)~(0x1 << pin1); // GPIOA9 push-pull 00 8.4.2
	GPIOB->OTYPER &= (unsigned int)~(0x1 << pin2);

	GPIOB->PUPDR &= (unsigned int)~(0x3 << 2 * pin1);
	GPIOB->PUPDR &= (unsigned int)~(0x3 << 2 * pin2);

	GPIOB->PUPDR |= 0x0 << 2 * pin1; // GPIOA 2 No pull-up, pull-down 00 8.4.4
	GPIOB->PUPDR |= 0x0 << 2 * pin2;

	/*** USART ***/
	// USART1->BRR = 0x369d; // USARTDIV (Tx/Rx=fCK/16*USARTDIV) 19.6.3 / refer: p519
	// USART1->BRR = 0x682; // USARTDIV (Tx/Rx=fCK/16*USARTDIV) 19.6.3 / refer: p519
	USART1->BRR = 0x2d9; // 222e:9600 2d9:115200

	USART1->CR3 &= (unsigned int)~(0x1 << 9); // disable CTS hardware flow 0 19.6.6
	USART1->CR3 &= (unsigned int)~(0x1 << 8); // disable RTS hardware flow 0 19.6.6

	USART1->CR1 |= 0x1 << 3; // transmitter enable 1 19.6.4
	USART1->CR1 |= 0x1 << 2; // receiver enable 1 19.6.4

	USART1->CR1 &= (unsigned int)~(0x1 << 10); // parity control disable 0 19.6.4

	USART1->CR2 &= (unsigned int)~(0x3 << 2 * 6); // 1 stop bit 0 19.6.5

	USART1->CR1 &= (unsigned int)~(0x1 << 12); // 1-8-n word length 19.6.4

	USART1->CR1 |= 1 << 13; // USART enable 19.6.4

	/*** DMA ***/
	// 使用DMA2_Stream7 Channel4（USART1_TX）
	DMA_InitTypeDef  DMA_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE); // 使能DMA2时钟

	DMA_DeInit(DMA2_Stream7);												//复位DMA状态  清空数据
	while(DMA_GetCmdStatus(DMA2_Stream7) != 0);								//判断流内数据是否清零

	DMA_InitStructure.DMA_Channel = DMA_Channel_4;							//通道选择
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;		//外设地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)USART1_TX_BUF;		//内存地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;					//数据传输方向
	DMA_InitStructure.DMA_BufferSize = USART1_TX_BUF_SIZE;					//数据传输量
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		//外设地址不增加
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					//内存地址自增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//外设数据长度
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;			//内存数据长度
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;							//DMA模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;					//DMA优先级
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;					//DMA FIFO模式
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;		//DMA FIFO阈值
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;				//内存突发单次传输
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;		//外设突发单次传输

	DMA_Init(DMA2_Stream7, &DMA_InitStructure);								//初始化DMA

	DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);							//使能DMA传输完成中断
	
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);							//使能串口DMA发送

	// NVIC_InitStructure.NVIC_IRQChannel                   = DMA2_Stream7_IRQn;           
    // NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;          
    // NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1; 
    // NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    // NVIC_Init(&NVIC_InitStructure);

	NVIC_SetPriority(DMA2_Stream7_IRQn, 1);									//设置DMA中断优先级
	NVIC_EnableIRQ(DMA2_Stream7_IRQn);										//使能DMA中断

	DMA_Cmd(DMA2_Stream7, DISABLE); //关闭DMA，因为一开始并没有数据要发送

}

char *ToString(int iVal)
{
	char *ReadClock(void);
	char str[1024] = {
		'\0',
	};
	char *pos = 0;
	int sign = 0; // 正数 或者是 0
	int abs = iVal;

	pos = str + 1023; // 移动指针,指向堆栈底部
	*pos-- = '\0';	  // end

	if (iVal < 0)
	{
		sign = 1;
		abs = -abs;
	}
	int dit = 0;
	while (abs > 0)
	{
		dit = abs % 10;
		abs = abs / 10;
		*pos-- = (char)('0' + dit);
	}
	if (sign)
		*pos-- = '-';
	char *ret = (char *)malloc(1024 - (pos - str));
	if (iVal == 0) // 0的一个处理
		strcpy(ret, "0");
	else // iVal非0的拷贝
		strcpy(ret, pos + 1);

	return (ret);
}