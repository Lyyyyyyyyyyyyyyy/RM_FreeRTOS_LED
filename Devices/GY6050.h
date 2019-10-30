#ifndef __GY_6050_H
#define __GY_6050_H

#include "user_common.h"

#define GY_IMU_PACKAGE_LENGTH 	18
#define GY6050_SENDID		        0x413
#define Conversion ratio

typedef union{
	struct{
		uint16_t yaw;
		int16_t gyro_z;
		int16_t pitch;
		int16_t gyro_x;
	};
	uint8_t dataBuff[8];
}Gyro_Data;

extern GY_IMU_t IMU_Cloud;
extern Gyro_Data Gyro_data[2];

void GY6050_getCloundInfo(CanRxMsg RxMessage);
void Gyro_Init(void);

#endif 

