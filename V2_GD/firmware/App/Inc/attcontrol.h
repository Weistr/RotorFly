#ifndef		__ATTCONTROL_H
#define		__ATTCONTROL_H
#include "main.h"
#include "stdbool.h"
#define RC_Data_length 10

#define sys_enable_flight (flight_sta|=0x01)//ϵͳʹ�ܷɿ�
#define sys_disable_flight (flight_sta&=~0x01)//ϵͳʧ�ܷɿ�
#define read_sys_flight_sta  (flight_sta&0x01)//��ȡϵͳ�ɿ�״̬��1ʹ�ܣ�0ʧ��

#define user_enable_flight  (flight_sta|=0x02)//�û�ʹ�ܷɿ�
#define user_disable_flight (flight_sta&=~0x02)//�û�ʧ�ܷɿ�
#define read_user_flight_sta (flight_sta&0x02)//��ȡ�û��ɿ�״̬��1ʹ�ܣ�0ʧ��

#define RC_timeout_set  (flight_sta|=0x04)//�������ݳ�ʱ
#define RC_timeout_reset  (flight_sta&=~0x04)//�������ݳ�ʱ��־λ��λ
#define read_RC_timeout_flag (flight_sta&0x04)//��ȡ�Ƿ�������ݳ�ʱ��1��ʱ��

#define batt_low_set (flight_sta|=0x08)//��ص�ѹ��
#define batt_low_reset (flight_sta&=~0x08)//��ص�ѹ�ͱ�־λ��λ
#define read_batt_low_flag (flight_sta&0x08)//��ȡ�Ƿ��ص�ѹ�ͣ�1��ص�ѹ�ͣ�0��

#define mpu_Faild_flag_set  (flight_sta|=0x10)//mpu6050��ʼ��ʧ��
#define mpu_Faild_flag_reset (flight_sta&=~0x10)//mpu6050��ʼ��ʧ�ܱ�־λ��λ
#define read_mpu_flag (flight_sta&0x10)//��ȡmpu��ʼ��״̬��1��ʼ��ʧ�ܣ�0��ʼ���ɹ�

#define min_start_th 100//��С����������ֵ��0-3000��

//ң���������ݽṹ 
typedef struct
{
	int16_t THROTTLE;//����
    int16_t LiftSpeed;
	int16_t YAW;
	int16_t ROLL;
	int16_t PITCH;
}RC_TYPE;
extern int16_t roll_offset;
extern int16_t pit_offset;
extern int16_t yaw_offset;
extern bool RC_success_flag;

extern uint8_t flight_sta;
void motor_control(void);
void fly_config(void);
#endif
