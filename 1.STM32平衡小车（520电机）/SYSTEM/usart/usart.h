#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 


void uart1_init(u32 bound);					//����1��ʼ������
void USART1_IRQHandler(void);     	//����1�жϷ������
#endif


