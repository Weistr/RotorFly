#ifndef		__PID_H
#define		__PID_H
#include "main.h"
//PID�㷨�����ݽṹ 
typedef struct PID
{
  float P;         //����
  float I;
  float D;
  float Error;     //������
  float Integral;  //������
  float Differ;    //΢����
  float PreError;
  float PrePreError;
  float Ilimit; 
  float Irang;
  float Pout;
  float Iout;
  float Dout;
  float OutPut;   
  uint8_t Ilimit_flag;    //���ַ���	
}PID_TYPE;   

typedef volatile struct
{
	float desired;     //< set point
	float offset;      //
	float prevError;    //< previous error
	float integ;        //< integral
	float kp;           //< proportional gain
	float ki;           //< integral gain
	float kd;           //< derivative gain
	float ilimt;       //< integral limit
	float irange;
	float measured;
	float out;
	float OutLimitHigh;
	float OutLimitLow;
}PidObject;

void pidRest(PidObject **pid,const uint8_t len);
void pid_param_Init(void);
	void pidUpdate(PidObject* pid,const float dt);
void CascadePID(PidObject* pidRate,PidObject* pidAngE,const float dt);  //����PID

extern PidObject pidRateX; //�ڻ�PID����
extern PidObject pidRateY;
extern PidObject pidRateZ;

extern PidObject pidPitch; //�⻷PID����
extern PidObject pidRoll;
extern PidObject pidYaw;

#endif
