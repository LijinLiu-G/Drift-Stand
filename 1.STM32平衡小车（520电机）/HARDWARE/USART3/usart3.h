#ifndef __USART3_H
#define __USART3_H

#include "sys.h" 



void USART3_Send_String(char *String);
void uart3_init(u32 bound);					//����1��ʼ������
void USART3_IRQHandler(void);     	//����1�жϷ������
#endif
