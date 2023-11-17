#include "usart.h"

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
	USART1->BRR = 0x222E;

	USART1->CR3 &= (unsigned int)~(0x1 << 9); // disable CTS hardware flow 0 19.6.6
	USART1->CR3 &= (unsigned int)~(0x1 << 8); // disable RTS hardware flow 0 19.6.6

	USART1->CR1 |= 0x1 << 3; // transmitter enable 1 19.6.4
	USART1->CR1 |= 0x1 << 2; // receiver enable 1 19.6.4

	USART1->CR1 &= (unsigned int)~(0x1 << 10); // parity control disable 0 19.6.4

	USART1->CR2 &= (unsigned int)~(0x3 << 2 * 6); // 1 stop bit 0 19.6.5

	USART1->CR1 &= (unsigned int)~(0x1 << 12); // 1-8-n word length 19.6.4

	USART1->CR1 |= 1 << 13; // USART enable 19.6.4
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