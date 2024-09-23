#include "control.h"
/*目的：引出二开接口，学习完本节后对二开有实质性帮助。对譬如平衡小车的巡线、追踪物体起到帮助作用。*/
/*

*/
float Med_Angle=0;		//机械中值。---在这里修改你的机械中值即可。
float Target_Speed=0;	//期望速度（俯仰）。---二次开发接口，用于控制小车前进后退及其速度。
float Turn_Speed=0;		//期望速度（偏航）

float 
	Vertical_Kp=-400,//直立环KP、KD
	Vertical_Kd=-1.92;//-1.7;
float 
	Velocity_Kp=-0.44,//速度环KP、KI
	Velocity_Ki=-0.0022;//0.01;
float 
	Turn_Kd=0.6,//转向环KP、KD
	Turn_Kp=20;
extern int length;
#define SPEED_Y 40   //俯仰(前后)最大设定速度
#define SPEED_Z 100//偏航(左右)最大设定速度 


int Vertical_out,Velocity_out,Turn_out;//直立环&速度环&转向环 的输出变量

int Vertical(float Med,float Angle,float gyro_Y);//函数声明
int Velocity(int Target,int encoder_left,int encoder_right);
int Turn(int gyro_Z,int RC); 

void EXTI9_5_IRQHandler(void)
{
	int PWM_out;
	if(EXTI_GetITStatus(EXTI_Line5)!=0)//一级判定
	{
		if(PBin(5)==0)//二级判定
		{
			EXTI_ClearITPendingBit(EXTI_Line5);//清除中断标志位
			
			//1、采集编码器数据&MPU6050角度信息。
			Encoder_Left=-Read_Speed(2);//电机是相对安装，刚好相差180度，为了编码器输出极性一致，就需要对其中一个取反。
			Encoder_Right=Read_Speed(4);
			
			mpu_dmp_get_data(&Roll,&Pitch,&Yaw);			//角度
			MPU_Get_Gyroscope(&gyroy,&gyrox,&gyroz);	//陀螺仪
			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//加速度

			//1.5、二开部分
			/*********************************************************************************************/
			/*前后*/
			if((Fore==0)&&(Back==0))Target_Speed=0;//未接受到前进后退指令-->速度清零，稳在原地
			if(Fore==1)
			{
				if(length<50)Target_Speed++;
				else
				Target_Speed--;
			}
			
			if(Back==1){Target_Speed++;}//
			Target_Speed=Target_Speed>SPEED_Y?SPEED_Y:(Target_Speed<-SPEED_Y?(-SPEED_Y):Target_Speed);//限幅
			
			/*左右*/
			if((Left==0)&&(Right==0))Turn_Speed=0;
			if(Left==1)Turn_Speed+=30;	//左转
			if(Right==1)Turn_Speed-=30;	//右转
			Turn_Speed=Turn_Speed>SPEED_Z?SPEED_Z:(Turn_Speed<-SPEED_Z?(-SPEED_Z):Turn_Speed);//限幅( (20*100) * 100   )
			
			/*转向约束*/
			if((Left==0)&&(Right==0))Turn_Kd=0.6;//若无左右转向指令，则开启转向约束
			else if((Left==1)||(Right==1))Turn_Kd=0;//若左右转向指令接收到，则去掉转向约束
			/*********************************************************************************************/
			
			//2、将数据压入闭环控制中，计算出控制输出量。
			Velocity_out=Velocity(Target_Speed,Encoder_Left,Encoder_Right);	//速度环
			Vertical_out=Vertical(Velocity_out+Med_Angle,Pitch,gyroy);			//直立环
			Turn_out=Turn(gyroz,Turn_Speed);																//转向环
			
			PWM_out=Vertical_out;//最终输出
			//3、把控制输出量加载到电机上，完成最终的的控制。
			MOTO1=PWM_out-Turn_out;//左电机
			MOTO2=PWM_out+Turn_out;//右电机
			Limit(&MOTO1,&MOTO2);	 //PWM限幅			
			Load(MOTO1,MOTO2);		 //加载到电机上。
			
			Stop(&Med_Angle,&Pitch);//安全检测
			
		}
	}
}




/*********************
直立环PD控制器：Kp*Ek+Kd*Ek_D

入口：期望角度、真实角度、真实角速度
出口：直立环输出
*********************/
int Vertical(float Med,float Angle,float gyro_Y)
{
	int PWM_out;
	
	PWM_out=Vertical_Kp*(Angle-Med)+Vertical_Kd*(gyro_Y-0);
	return PWM_out;
}


/*********************
速度环PI：Kp*Ek+Ki*Ek_S
*********************/
int Velocity(int Target,int encoder_left,int encoder_right)
{
	static int Encoder_S,EnC_Err_Lowout_last,PWM_out,Encoder_Err,EnC_Err_Lowout;
	float a=0.7;
	
	//1.计算速度偏差
	Encoder_Err=((encoder_left+encoder_right)-Target);//舍去误差--我的理解：能够让速度为"0"的角度，就是机械中值。
	//2.对速度偏差进行低通滤波
	//low_out=(1-a)*Ek+a*low_out_last;
	EnC_Err_Lowout=(1-a)*Encoder_Err+a*EnC_Err_Lowout_last;//使得波形更加平滑，滤除高频干扰，防止速度突变。
	EnC_Err_Lowout_last=EnC_Err_Lowout;//防止速度过大的影响直立环的正常工作。
	//3.对速度偏差积分，积分出位移
	Encoder_S+=EnC_Err_Lowout;
	//4.积分限幅
	Encoder_S=Encoder_S>20000?20000:(Encoder_S<(-20000)?(-20000):Encoder_S);
	
	if(stop==1)Encoder_S=0,stop=0;//清零积分量
	
	//5.速度环控制输出计算
	PWM_out=Velocity_Kp*EnC_Err_Lowout+Velocity_Ki*Encoder_S;
	return PWM_out;
}



/*********************
转向环：系数*Z轴角速度+系数*遥控数据
*********************/
int Turn(int gyro_Z,int RC)
{
	int PWM_out;
	//这不是一个严格的PD控制器，Kd针对的是转向的约束，但Kp针对的是遥控的转向。
	PWM_out=Turn_Kd*gyro_Z + Turn_Kp*RC;
	return PWM_out;
}

