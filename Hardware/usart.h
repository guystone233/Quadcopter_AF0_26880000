#ifndef __USART_H
#define __USART_H

#include "stm32f4xx.h"
#include "string.h"
#include "os_cpu.h"
#include <stdarg.h>

void USART1_IRQHandler(void);
void DMA2_Stream7_IRQHandler(void);
void USARTInit(void);

void SendByte(char ch);
void SendString(char *ch);
char ReadByte(void);
void DMA_USART1_Send(char *data,int size);
void USART1_printf(char *format, ...);
void ReadString(char *ch);

char *ToString(int iVal);

#define USART1_RX_BUF_SIZE 256				// 接收缓冲区大小
extern char USART1_RX_BUF[USART1_RX_BUF_SIZE];


// #define USART1_RX_BUF_SIZE 256
#define USART1_TX_BUF_SIZE 256

#endif /* USART */