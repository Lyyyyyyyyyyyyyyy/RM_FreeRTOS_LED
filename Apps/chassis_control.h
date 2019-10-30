#ifndef __CHIASSIS_CONTROL_H
#define __CHIASSIS_CONTROL_H

#include "user_common.h"

#define Chassis_LpfAttFactor 0.04f

#define WheelMaxSpeed        6600.0f		//单轮最大速度

#define Chassis_MaxSpeed_Y 	 6600.0f		//底盘前后最快速度
#define Chassis_MaxSpeed_X	 6600.0f		//底盘平移最快速度
#define Chassis_MaxSpeed_W	 6600.0f		//底盘自旋最快速度

#define Chassis_SpeedNormal	 4300.0f
#define Chassis_SpeedFast		 6600.0f
#define Chassis_SpeedSlow		 3000.0f

#define Chassis_SpeedPowerOverflow		3000.0f
#define Chassis_SpeedSupplyMode			  3800.0f

#define Chassis_SwingSpeed_Normal		3900
#define Chassis_SwingSpeed_Slow			2200

#define WheelCount 			4

#define Chassis_Mode_Normal			0
#define Chassis_Mode_Follow			1

#define Yaw_Center_Angle        6800  

#define DEG_TO_RAD      0.017453292519943295769236907684886f
#define RAD_TO_DEG      57.295779513082320876798154814105f

#define abs(x) ((x)>0?(x):-(x))

#define VAL_LIMIT(val, min, max) \
do {\
if((val) <= (min))\
{\
  (val) = (min);\
}\
else if((val) >= (max))\
{\
  (val) = (max);\
}\
} while(0)\

extern Chassis_t Chassis;
 
void Chassis_Init(void);
void MecanumCalculate(float Vx, float Vy, float VOmega, int16_t *Speed);
void Chassis_processing(float Vx, float Vy, float VOmega, uint8_t mode);
void Chassis_Follow_Cloud(float Vx, float Vy, float VOmega);
void Chassis_spinning(float Vx, float Vy, float VOmega);

#endif /* __CHIASSIS_CONTROL_H */
