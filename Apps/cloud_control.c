/**
  ******************************************************************************
  * @file    cloud_control.c
  * @author   
  * @version  
  * @date    
  * @brief   云台控制函数接口
  ******************************************************************************
  */
  
#include "cloud_control.h"

Cloud_t Cloud;

/**
  * @brief  云台初始化，配置参数
  * @param  None
  * @retval None
  */
	
void Cloud_Init(void){
	
    Cloud.LpfAttFactor = Cloud_LpfAttFactor;
	
  	M6020s[0].targetAngle = 6800.0f;
	  M6020s[1].targetAngle = 2700.0f;
	
/************************GM6020  保持启动时刻yaw和pitch轴的位置**************************/    
    //启动时yaw和pitch机械角度
//	  Cloud.targetPitchRaw = Cloud.targetPitchLPF = M6020s[0].totalAngle;
//	  Cloud.targetYawRaw = Cloud.targetYawLPF = M6020s[1].totalAngle;
	
	  //启动时陀螺仪yaw和pitch角度
//	  Cloud.IMUtargetYawRaw = Cloud.IMUtargetYawLPF = IMU_Cloud.totalYaw;
//	  Cloud.IMUtargetPitchRaw = Cloud.IMUtargetPitchLPF = IMU_Cloud.Eular.Pitch;
		
	  /*-----正常使用PID-----*/
	  //云台电机PID
    PositionPID_paraReset(&M6020s[0].pid_Speed, 30.0f, 0.0f, 6.0f, 27000, 800);
	  PositionPID_paraReset(&M6020s[0].pid_Angle, 13.0f, 0.0f, 2.0f, 8000, 1000);
	  
	  PositionPID_paraReset(&M6020s[1].pid_Speed, 70.0f, 0.0f, 0.8f, 27000, 800);
	  PositionPID_paraReset(&M6020s[1].pid_Angle, 1.0f, 0.0f, 0.8f, 8000, 800);
	
	  //陀螺仪双环PID
//	  PositionPID_paraReset(&Cloud.YawAttitude_pid, 0.0f, 0.0f, 0.0f, 8000, 800);
//	  PositionPID_paraReset(&Cloud.YawSpeed_pid, 0.0f, 0.0f, 0.0f, 27000, 1000);      //发送给电机的最大控制值

//	  PositionPID_paraReset(&Cloud.PitchAttitude_pid, 0.0f, 0.0f, 0.0f, 8000, 4000); 
//	  PositionPID_paraReset(&Cloud.PitchSpeed_pid, 0.0f, 0.0f, 0.0f, 27000, 1000);   
}

void Set_GM6020_TargetAngle(float angle_pitch,float angle_yaw)
{
	float YawOutCurrent = 0;
	
	//pitch云台限位
	if (M6020s[1].M6020InfoUpdateFlag == 1)
	{
		M6020s[1].targetAngle += angle_pitch;
		if (M6020s[1].targetAngle <= Cloud_Yaw_Min)
		{
			 M6020s[1].targetAngle = Cloud_Yaw_Min;
		}
		else if (M6020s[1].targetAngle >= Cloud_Yaw_Max)
		{
			 M6020s[1].targetAngle = Cloud_Yaw_Max;
		}
	}
	
//	M6020s[0].targetAngle += angle_yaw;

		//正常外环内环
		M6020s[1].outCurrent = Position_PID(&M6020s[1].pid_Angle, M6020s[1].targetAngle, M6020s[1].totalAngle);
	  M6020s[1].inneroutCurrent = Position_PID(&M6020s[1].pid_Speed, M6020s[1].outCurrent, M6020s[1].realSpeed);
		
	  M6020s[1].M6020InfoUpdateFlag = 0;
	
/************************GM6020  陀螺仪处理开始**************************/	 		
	if(abs(angle_yaw) > 0)
	{
    IMU_Cloud.targetYaw += angle_yaw;
	}
	//陀螺仪外环内环
    YawOutCurrent = Position_PID(&M6020s[0].pid_Angle, IMU_Cloud.targetYaw, IMU_Cloud.totalYaw); 
    M6020s[0].inneroutCurrent = Position_PID(&M6020s[0].pid_Speed, -YawOutCurrent, IMU_Cloud.Gyro.z); 

/************************GM6020  陀螺仪处理结束**************************/	 		
	  M6020_setCurrent(M6020s[0].inneroutCurrent, M6020s[1].inneroutCurrent, 0, 0);
}

/**
  * @brief      通过6020机械角度的方式获取云台Yaw旋转的角度（偏移车正前方的角度-中心点）
  * @param[in]  None
  * @retval     360度的角度值。
  */
float Cloud_getYawAngleWithCenter(void) {
	return (M6020s[0].totalAngle - 6800) / M6020_mAngleRatio;
}

