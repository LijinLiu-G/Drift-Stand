#include "usart3.h"	  
void uart3_init(u32 bound)
{
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//时钟GPIOB、USART3
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	
	//USART1_TX   PB10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	//USART1_RX	  PB11
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);  
	//USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//一般设置为9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);
	
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启中断
	USART_Cmd(USART3, ENABLE);                    //使能串口 
}

u8 Fore,Back,Left,Right;
void USART3_IRQHandler(void) 
{
	int Bluetooth_data;
	if(USART_GetITStatus(USART3,USART_IT_RXNE)!=RESET)//接收中断标志位拉高
	{
		Bluetooth_data=USART_ReceiveData(USART3);//保存接收的数据
		
		if(Bluetooth_data==0x00)Fore=0,Back=0,Left=0,Right=0;//刹
		else if(Bluetooth_data==0x01)Fore=1,Back=0,Left=0,Right=0;//前
		else if(Bluetooth_data==0x05)Fore=0,Back=1,Left=0,Right=0;//后
		else if(Bluetooth_data==0x07)Fore=0,Back=0,Left=0,Right=1;//右
		else if(Bluetooth_data==0x03)Fore=0,Back=0,Left=1,Right=0;//左
		else													Fore=0,Back=0,Left=0,Right=0;//刹
	}
}

//一个
void USART3_Send_Data(char data)
{
	USART_SendData(USART3,data);
	while(USART_GetFlagStatus(USART3,USART_FLAG_TC)!=1);
}

//一串
void USART3_Send_String(char *String)
{
	u16 len,j;
	
	len=strlen(String);
	for(j=0;j<len;j++)
	{
		USART3_Send_Data(*String++);
	}
}





