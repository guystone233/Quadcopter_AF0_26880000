#include "stm32f4xx.h" 

void USARTInit(void);

void SendByte(char ch);
void SendString(char *ch);
char ReadByte(void);

char* ToString(int iVal);