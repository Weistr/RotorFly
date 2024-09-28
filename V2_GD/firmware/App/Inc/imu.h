#ifndef		__IMU_H
#define		__IMU_H
#include "main.h"
#include "mpu6050.h"
typedef struct
{
	int16_t X;
	int16_t Y;
	int16_t Z;
}INT16_XYZ;

typedef struct
{
	float X;
	float Y;
	float Z;
}FLOAT_XYZ;


typedef struct
{
	float YAW;
	float ROLL;
	float PITCH;
	float LIFT;	
}FLOAT_FLIGHT_POSE;
void mpu_gyr_offset(void);
void mpu_acc_offset(void);
//extern const float M_PI;
extern const float RtA;
extern const float AtR;
extern const float Gyro_G;	  	//�����ǳ�ʼ������+-2000��ÿ����1 / (65536 / 4000) = 0.03051756*2		
extern const float Gyro_Gr;     //������ÿ��,ת������ÿ���� 2*0.03051756	 * 0.0174533f = 0.0005326*2
extern _st_AngE Angle;    //��ǰ�Ƕ���ֵ̬
void GetAngle(const _st_Mpu *pMpu,_st_AngE *pAngE, float dt) ;

#endif
