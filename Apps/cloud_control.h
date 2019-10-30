#ifndef __CLOUD_CONTROL_H
#define __CLOUD_CONTROL_H

#include "user_common.h"

extern Cloud_t Cloud;

#define Cloud_LpfAttFactor    0.05f

#define Cloud_Yaw_Min			    4100.0f
#define Cloud_Yaw_Max			    5200.0f

#define M6020_mAngleRatio     22.7527f  //机械角度与真实角度的比率
#define PI                    3.1415926535897932384626433832795f

void Cloud_Init(void);
void Set_GM6020_TargetAngle(float angle_pitch,float angle_yaw); 
float Cloud_getYawAngleWithCenter(void);

 
#endif /* __CLOUD_CONTROL_H */
