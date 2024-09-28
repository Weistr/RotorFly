#ifndef		__ATTCONTROL_H
#define		__ATTCONTROL_H
#include "main.h"
#include "stdbool.h"
#define RC_Data_length 10

#define sys_enable_flight (flight_sta|=0x01)//系统使能飞控
#define sys_disable_flight (flight_sta&=~0x01)//系统失能飞控
#define read_sys_flight_sta  (flight_sta&0x01)//读取系统飞控状态，1使能，0失能

#define user_enable_flight  (flight_sta|=0x02)//用户使能飞控
#define user_disable_flight (flight_sta&=~0x02)//用户失能飞控
#define read_user_flight_sta (flight_sta&0x02)//读取用户飞控状态，1使能，0失能

#define RC_timeout_set  (flight_sta|=0x04)//接收数据超时
#define RC_timeout_reset  (flight_sta&=~0x04)//接收数据超时标志位复位
#define read_RC_timeout_flag (flight_sta&0x04)//读取是否接收数据超时，1超时，

#define batt_low_set (flight_sta|=0x08)//电池电压低
#define batt_low_reset (flight_sta&=~0x08)//电池电压低标志位复位
#define read_batt_low_flag (flight_sta&0x08)//读取是否电池电压低，1电池电压低，0，

#define mpu_Faild_flag_set  (flight_sta|=0x10)//mpu6050初始化失败
#define mpu_Faild_flag_reset (flight_sta&=~0x10)//mpu6050初始化失败标志位复位
#define read_mpu_flag (flight_sta&0x10)//读取mpu初始化状态，1初始化失败，0初始化成功

#define min_start_th 100//最小启动油门数值（0-3000）

//遥控器的数据结构 
typedef struct
{
	int16_t THROTTLE;//油门
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
