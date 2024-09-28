#ifndef		__PID_H
#define		__PID_H
#include "main.h"
//PID算法的数据结构 
typedef struct PID
{
  float P;         //参数
  float I;
  float D;
  float Error;     //比例项
  float Integral;  //积分项
  float Differ;    //微分项
  float PreError;
  float PrePreError;
  float Ilimit; 
  float Irang;
  float Pout;
  float Iout;
  float Dout;
  float OutPut;   
  uint8_t Ilimit_flag;    //积分分离	
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
void CascadePID(PidObject* pidRate,PidObject* pidAngE,const float dt);  //串级PID

extern PidObject pidRateX; //内环PID数据
extern PidObject pidRateY;
extern PidObject pidRateZ;

extern PidObject pidPitch; //外环PID数据
extern PidObject pidRoll;
extern PidObject pidYaw;

#endif
