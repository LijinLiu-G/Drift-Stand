#ifndef __SENSOR_H_
#define __SENSOR_H_


#define TRIG PAout(3) //����˿�
#define ECHO PAin(2)  //����˿�
void TIM3_IRQHandler(void);
void TIM3_Int_Init();
int Senor_Using(); //��������������

#endif
