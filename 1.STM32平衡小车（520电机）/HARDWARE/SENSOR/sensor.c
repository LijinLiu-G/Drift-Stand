#include "sensor.h"
#include "stm32f10x.h"
#include "sys.h" 



int overcount=0;      //��¼��ʱ���������
void TIM3_IRQHandler(void)
{
		if (TIM_GetITStatus(TIM3,TIM_IT_Update)!= RESET) //����Ƿ���TIM3�ж�
		{
			TIM_ClearITPendingBit(TIM3, TIM_IT_Update );   //����жϸ��±�־
			overcount++;
		}
}
void TIM3_Int_Init()
{
		GPIO_InitTypeDef GPIO_InitStruct;
		TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
		NVIC_InitTypeDef NVIC_InitStructure;

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //ʱ��ʹ��
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
			
		
		GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;//����Ϊ�������
		GPIO_InitStruct.GPIO_Pin=GPIO_Pin_3;
		GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
		GPIO_Init(GPIOA,&GPIO_InitStruct);

		GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IN_FLOATING;
		GPIO_InitStruct.GPIO_Pin=GPIO_Pin_2;
		GPIO_Init(GPIOA,&GPIO_InitStruct);
	
		//��ʱ��3��ʼ��
		TIM_TimeBaseStructure.TIM_Period = 999; //ARR
		TIM_TimeBaseStructure.TIM_Prescaler =7199; //PSC
		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //���ϼ���ģʽ
		TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

		TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE );//ʹ���ƶ�TIM3�жϣ���������ж�
		//�ж�����
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
		NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

		TIM_Cmd(TIM3, DISABLE);
}

//��ຯ��
int Senor_Using() //��������������
{
		u16 sum=0;
		u16 tim;
		u16 i=0;
		u16 length;
		
		while(i!=3)        //����һ��ƽ��ֵ 3��
		{
			TRIG=1;          //�����źţ���Ϊ�����ź�
			delay_us(20);    //�ߵ�ƽ�źų���10us
			TRIG=0;          //�ȴ������ź�
			
			while(ECHO==0);  //�����źŵ�����������ʱ������
			TIM_Cmd(TIM3,ENABLE);
			
			i+=1;                     //ÿ�յ�һ�λ����ź�+1,�յ�5�ξͼ����ֵ
			
			while(ECHO==1);           //�����ź���ʧ
			TIM_Cmd(TIM3,DISABLE);    //�رն�ʱ��
			
			tim=TIM_GetCounter(TIM3);         //��ȡ��TIM3���Ĵ����еļ���ֵ
			length=(tim*100)/58.0; //ͨ�������źż������
			sum=length+sum;
			TIM3->CNT=0; //��TIM3�����Ĵ����ļ���ֵ����
			overcount=0; //�ж������������
			delay_ms(100);
		}
		length=sum/3;
		return length; //������Ϊ��������ֵ
}

