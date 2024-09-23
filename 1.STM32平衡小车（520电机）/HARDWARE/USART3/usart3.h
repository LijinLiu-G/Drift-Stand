#ifndef __USART3_H
#define __USART3_H

#include "sys.h" 



void USART3_Send_String(char *String);
void uart3_init(u32 bound);					//串口1初始化函数
void USART3_IRQHandler(void);     	//串口1中断服务程序
#endif
