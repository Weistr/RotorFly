#include "attcontrol.h"
#include "imu.h"
#include "pid.h"
#include "stdbool.h"
#include "filter.h"
#define motor_enable 1

const float dt = 0.003; //��������
uint8_t flight_sta=0x01; //�ɿ�״̬����control.h
RC_TYPE RC_Control={0,0,1500,1500,1500};//ң��������
extern bool att_offset_flag;//IMUУ׼��־λ
_st_AngE Angle;//��̬��
void beep(uint16_t per,uint8_t us, uint16_t ms);//T=per us, beep ms

PidObject *(pPidObject[])={&pidRateX,&pidRateY,&pidRateZ,&pidRoll,&pidPitch,&pidYaw };//pid�ṹ��ָ�룬������������

uint16_t pwm1=0;
uint16_t pwm2=0;
uint16_t pwm3=0;
uint16_t pwm4=0;
//�ɻ���̬΢��
int16_t roll_offset = 0;
int16_t pit_offset = 0;
int16_t yaw_offset = 0;

void TIM_SetTIM3Compare1(int16_t i){}
void TIM_SetTIM3Compare2(int16_t i){}
void TIM_SetTIM3Compare3(int16_t i){}
void TIM_SetTIM3Compare4(int16_t i){}

//���ֹͣ
void motor_stop()
{
		TIM_SetTIM3Compare1(0);
		TIM_SetTIM3Compare2(0);
		TIM_SetTIM3Compare3(0);
		TIM_SetTIM3Compare4(0);
	
}
//������ư���PID��PWM����
void motor_control()
{
	if(read_RC_timeout_flag)motor_stop();//ң�������ճ�ʱ���رյ��
	else
	{	
		//ң��������ת��
		/*
			if(RC_Control.ROLL<1700 && RC_Control.ROLL>1300)RC_Control.ROLL = 1500;
			if(RC_Control.PITCH<1700 && RC_Control.PITCH>1300)RC_Control.PITCH = 1500;
			if(RC_Control.YAW<1700 && RC_Control.YAW>1300)RC_Control.YAW = 1500;
*/
		pidRoll.desired = (float)(1500 -  RC_Control.ROLL+ roll_offset)/100.0f;
		pidPitch.desired = (float)(1500 - RC_Control.PITCH + pit_offset)/100.0f;
		
		//pid����		
		pidRateX.measured = MPU6050.gyroX * Gyro_G; 
		pidRateY.measured = MPU6050.gyroY * Gyro_G;
		pidRateZ.measured = MPU6050.gyroZ * Gyro_G;
	
		pidPitch.measured = Angle.pitch; 
		pidRoll.measured = Angle.roll;
		pidYaw.measured = Angle.yaw;
	
		pidUpdate(&pidRoll,dt);   //�ǶȻ�
		pidRateX.desired = pidRoll.out; 
		pidUpdate(&pidRateX,dt);  //���ٶȻ�

		pidUpdate(&pidPitch,dt);    
		pidRateY.desired = pidPitch.out; 
		pidUpdate(&pidRateY,dt); 
	
		pidUpdate(&pidYaw,dt);    
		pidRateZ.desired = pidYaw.out;  
		pidUpdate(&pidRateZ,dt); 
        //�����������	���Ŵ���100ʱ����
		if (RC_Control.THROTTLE > min_start_th && read_sys_flight_sta && read_user_flight_sta)
		{

		pidYaw.desired -= (float)(RC_Control.YAW - 1500 + yaw_offset)/4000.0f;
		if(pidYaw.desired>180)pidYaw.desired = 180;
		if(pidYaw.desired<-180)pidYaw.desired = -180;
			
			pwm1 = RC_Control.THROTTLE - pidRateX.out - pidRateY.out - pidRateZ.out;   
			pwm2 = RC_Control.THROTTLE + pidRateX.out - pidRateY.out + pidRateZ.out;
			pwm3 = RC_Control.THROTTLE + pidRateX.out + pidRateY.out - pidRateZ.out; 
			pwm4 = RC_Control.THROTTLE - pidRateX.out + pidRateY.out + pidRateZ.out;  
			if(motor_enable)
			{
				TIM_SetTIM3Compare1(pwm1);
				TIM_SetTIM3Compare2(pwm2);
				TIM_SetTIM3Compare3(pwm3);
				TIM_SetTIM3Compare4(pwm4);					
			}
			else
			{
        motor_stop();				
			}
				
		}
		else
		{ 
			pidRest(pPidObject,6); //������λPID���ݣ���ֹ�ϴ�����������Ӱ�챾�ο���
			Angle.yaw = pidYaw.desired =  pidYaw.measured = 0;   //���������
			motor_stop();//
		}				
	}	

}
void beep(uint16_t per,uint8_t us, uint16_t ms)//T=per(us), pwm-us, beep ms
{
	
//	HAL_TIM_Base_Stop_IT(&htim1);
//	htim3.Init.Prescaler = per;
//	htim3.Init.Period = 72;
//	HAL_TIM_PWM_Init(&htim3);
	TIM_SetTIM3Compare1(us);
	TIM_SetTIM3Compare2(us);
	TIM_SetTIM3Compare3(us);
	TIM_SetTIM3Compare4(us);	
//	HAL_Delay(ms);
			
//	htim3.Init.Prescaler = 3;
//	htim3.Init.Period = 3000;
//	HAL_TIM_PWM_Init(&htim3);
	TIM_SetTIM3Compare1(0);
	TIM_SetTIM3Compare2(0);
	TIM_SetTIM3Compare3(0);
	TIM_SetTIM3Compare4(0);	
//	HAL_TIM_Base_Start_IT(&htim1);
}
//�ɿ����ã�IMUУ׼���ͷɿ��쳣ʱ�رյ��
void fly_config()
{
	
	static uint16_t cnt = 10;
	static bool rc_flag=0;
	uint8_t i;
	if(!read_user_flight_sta)//�ɿ�û��ʹ��ʱ�������ã���ֹ
	{
		//IMUУ׼
		if(att_offset_flag)
		{
			att_offset_flag = 0;
	    MpuGetOffset();
	//		for(i=0;i<6;i++)flash_data[i+34]=MpuOffset[i];
	//		flash_save();
			beep(500,5,500);
			//HAL_Delay(100);
			beep(500,5,500);
		
		}
	//	if(pid_save_flag)
		{
	//		pid_save_flag = 0;
			//flash_save();
	//		HAL_Delay(500);
			beep(500,5,500);
	//		HAL_Delay(100);
			beep(500,5,500);
			
		}
	}
	if(read_RC_timeout_flag)
	{	 
		cnt++;
		if(cnt>20)
		{
		 cnt = 0;
			beep(500,2,110);
	//		HAL_Delay(120);
			beep(500,2,110);
		}
	}
	else if(read_user_flight_sta)
	{
		if(rc_flag ==0)
		{
			rc_flag = 1;
	//		HAL_Delay(500);
			beep(600,5,150);
			beep(500,5,150);
			beep(400,5,150);	
		}
	}
	else rc_flag = 0;
	if(//read_batt_low_flag ||  //��ص�����
		read_RC_timeout_flag    //���ճ�ʱ
		)
	sys_disable_flight;//�رշɿ�
	if(
		//(!read_batt_low_flag) && 
		(!read_RC_timeout_flag)  
	  )
	sys_enable_flight;
}








