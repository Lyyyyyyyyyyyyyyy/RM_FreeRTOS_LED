/**
  ******************************************************************************
  * @file    cloud_control.c
  * @author   
  * @version  
  * @date    
  * @brief   ��̨���ƺ����ӿ�
  ******************************************************************************
  */
  
#include "cloud_control.h"

Cloud_t Cloud;

/**
  * @brief  ��̨��ʼ�������ò���
  * @param  None
  * @retval None
  */
	
void Cloud_Init(void){
	
    Cloud.LpfAttFactor = Cloud_LpfAttFactor;
	
  	M6020s[0].targetAngle = 6800.0f;
	  M6020s[1].targetAngle = 2700.0f;
	
/************************GM6020  ��������ʱ��yaw��pitch���λ��**************************/    
    //����ʱyaw��pitch��е�Ƕ�
//	  Cloud.targetPitchRaw = Cloud.targetPitchLPF = M6020s[0].totalAngle;
//	  Cloud.targetYawRaw = Cloud.targetYawLPF = M6020s[1].totalAngle;
	
	  //����ʱ������yaw��pitch�Ƕ�
//	  Cloud.IMUtargetYawRaw = Cloud.IMUtargetYawLPF = IMU_Cloud.totalYaw;
//	  Cloud.IMUtargetPitchRaw = Cloud.IMUtargetPitchLPF = IMU_Cloud.Eular.Pitch;
		
	  /*-----����ʹ��PID-----*/
	  //��̨���PID
    PositionPID_paraReset(&M6020s[0].pid_Speed, 30.0f, 0.0f, 6.0f, 27000, 800);
	  PositionPID_paraReset(&M6020s[0].pid_Angle, 13.0f, 0.0f, 2.0f, 8000, 1000);
	  
	  PositionPID_paraReset(&M6020s[1].pid_Speed, 70.0f, 0.0f, 0.8f, 27000, 800);
	  PositionPID_paraReset(&M6020s[1].pid_Angle, 1.0f, 0.0f, 0.8f, 8000, 800);
	
	  //������˫��PID
//	  PositionPID_paraReset(&Cloud.YawAttitude_pid, 0.0f, 0.0f, 0.0f, 8000, 800);
//	  PositionPID_paraReset(&Cloud.YawSpeed_pid, 0.0f, 0.0f, 0.0f, 27000, 1000);      //���͸������������ֵ

//	  PositionPID_paraReset(&Cloud.PitchAttitude_pid, 0.0f, 0.0f, 0.0f, 8000, 4000); 
//	  PositionPID_paraReset(&Cloud.PitchSpeed_pid, 0.0f, 0.0f, 0.0f, 27000, 1000);   
}

void Set_GM6020_TargetAngle(float angle_pitch,float angle_yaw)
{
	float YawOutCurrent = 0;
	
	//pitch��̨��λ
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

		//�����⻷�ڻ�
		M6020s[1].outCurrent = Position_PID(&M6020s[1].pid_Angle, M6020s[1].targetAngle, M6020s[1].totalAngle);
	  M6020s[1].inneroutCurrent = Position_PID(&M6020s[1].pid_Speed, M6020s[1].outCurrent, M6020s[1].realSpeed);
		
	  M6020s[1].M6020InfoUpdateFlag = 0;
	
/************************GM6020  �����Ǵ���ʼ**************************/	 		
	if(abs(angle_yaw) > 0)
	{
    IMU_Cloud.targetYaw += angle_yaw;
	}
	//�������⻷�ڻ�
    YawOutCurrent = Position_PID(&M6020s[0].pid_Angle, IMU_Cloud.targetYaw, IMU_Cloud.totalYaw); 
    M6020s[0].inneroutCurrent = Position_PID(&M6020s[0].pid_Speed, -YawOutCurrent, IMU_Cloud.Gyro.z); 

/************************GM6020  �����Ǵ������**************************/	 		
	  M6020_setCurrent(M6020s[0].inneroutCurrent, M6020s[1].inneroutCurrent, 0, 0);
}

/**
  * @brief      ͨ��6020��е�Ƕȵķ�ʽ��ȡ��̨Yaw��ת�ĽǶȣ�ƫ�Ƴ���ǰ���ĽǶ�-���ĵ㣩
  * @param[in]  None
  * @retval     360�ȵĽǶ�ֵ��
  */
float Cloud_getYawAngleWithCenter(void) {
	return (M6020s[0].totalAngle - 6800) / M6020_mAngleRatio;
}

