#include "pid.h"

PidObject pidRateX; //�ڻ�PID����
PidObject pidRateY;
PidObject pidRateZ;

PidObject pidPitch; //�⻷PID����
PidObject pidRoll;
PidObject pidYaw;

/**************************************************************
 *������λPID����
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/	
void pidRest(PidObject **pid,const uint8_t len)
{
	uint8_t i;
	for(i=0;i<len;i++)
	{
	  	pid[i]->integ = 0;
	    pid[i]->prevError = 0;
	    pid[i]->out = 0;
		pid[i]->offset = 0;
	}
}

/**************************************************************
 * Update the PID parameters.
 *
 * @param[in] pid         A pointer to the pid object.
 * @param[in] measured    The measured value
 * @param[in] updateError Set to TRUE if error should be calculated.
 *                        Set to False if pidSetError() has been used.
 * @return PID algorithm output
 ***************************************************************/	
void pidUpdate(PidObject* pid,const float dt)
{
	 float error;
	 float deriv;
	
    error = pid->desired - pid->measured; //��ǰ�Ƕ���ʵ�ʽǶȵ����

    pid->integ += error * dt;	 //�������ۼ�ֵ
	
		if(pid->integ > pid->ilimt)//���л����޷�
			pid->integ = pid->ilimt;
		if(pid->integ < (-pid->ilimt))
			pid->integ = -(pid->ilimt);
		
    deriv = (error - pid->prevError)/dt;  //ǰ�����������΢��
	
    pid->out = pid->kp * error + pid->ki * pid->integ + pid->kd * deriv;//PID���
	
		
    pid->prevError = error;  //��У?��ϴε���?
		
}

/**************************************************************
1550
1233
2150
 *  CascadePID
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/	
void CascadePID(PidObject* pidRate,PidObject* pidAngE,const float dt)  //����PID
{	 
	pidUpdate(pidAngE,dt);    //�ȼ����⻷
	pidRate->desired = pidAngE->out;
	pidUpdate(pidRate,dt);    //�ټ����ڻ�	
}





void pid_param_Init(void)
{
//Ĭ��pid����/////////////////////////////////////////////	
	pidRateX.kp = 5.1f;
	pidRateY.kp = 5.1f;
	pidRateZ.kp = 9.0f;
	
	pidRateX.ki = 0.0f;
	pidRateY.ki = 0.0f;
	pidRateZ.ki = 0.0f;	
	
	pidRateX.kd = 0.24f;
	pidRateY.kd = 0.24f;
	pidRateZ.kd = 1.5f;	
	
	pidPitch.kp = 7.0f;
	pidRoll.kp = 7.0f;
	pidYaw.kp = 7.0f;	
	
	pidPitch.ki = 0.0f;
	pidRoll.ki = 0.0f;
	pidYaw.ki = 0.0f;	
	
/*	
	pidPitch.ki = 20.0f;
	pidRoll.ki = 20.0f;
	pidYaw.ki = 0.0f;	
	*/
	pidPitch.ilimt = 30.0f;
	pidRoll.ilimt = 30.0f;
	
	pidPitch.irange = 20.0f;
	pidRoll.irange = 20.0f;

	
////////////////////////////////////////////////////
	
//	pidRateX.kp = 2.0f;
//	pidRateY.kp = 2.0f;
//	pidRateZ.kp = 4.0f;
//	
//	pidRateX.ki = 0.0f;
//	pidRateY.ki = 0.0f;
//	pidRateZ.ki = 0.0f;	
//	
//	pidRateX.kd = 0.28f;
//	pidRateY.kd = 0.28f;
//	pidRateZ.kd = 0.4f;	
//	
//	pidPitch.kp = 7.0f;
//	pidRoll.kp = 7.0f;
//	pidYaw.kp = 7.0f;	

}

