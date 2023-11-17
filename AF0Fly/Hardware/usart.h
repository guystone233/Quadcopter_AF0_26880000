#ifndef __USART_H
#define __USART_H

#include "stm32f4xx.h"

void USARTInit(void);

void SendByte(char ch);
void SendString(char *ch);
char ReadByte(void);

char *ToString(int iVal);

#endif /* USART */