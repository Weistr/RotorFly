#ifndef __MPU6050_H
#define __MPU6050_H


#include "main.h"


extern int16_t MpuOffset[6];

extern int8_t MpuInit(void);
extern void MpuGetData(void);
extern void MpuGetOffset(void);
typedef struct{
	int16_t accX;
	int16_t accY;
	int16_t accZ;
	int16_t gyroX;
	int16_t gyroY;
	int16_t gyroZ;
}_st_Mpu;

typedef struct{
	float roll;
	float pitch;
	float yaw;
}_st_AngE;


extern _st_Mpu MPU6050;   

#define FAILED 1
#define SUCCESS 0

#endif // __MPU6050_H__








