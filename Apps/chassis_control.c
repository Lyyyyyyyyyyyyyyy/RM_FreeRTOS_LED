/**
  ******************************************************************************
  * @file    Chassis_control.c
  * @author   
  * @version  
  * @date    
  * @brief   ���̿��ƺ����ӿ�
  ******************************************************************************
  */
  
#include "chassis_control.h"

Chassis_t Chassis;

/**
  * @brief  ���̳�ʼ�������ò���
  * @param  None
  * @retval None
  */
void Chassis_Init(void){
	
  Chassis.LpfAttFactor = Chassis_LpfAttFactor;
	Chassis.speedLimit = Chassis_SpeedNormal;
	Chassis.mode = Chassis_Mode_Normal;
	Chassis.PowerOverflowFlag = 1;
	Chassis.swingSpeed = Chassis_SwingSpeed_Normal;
	
	/*-----����ʹ��PID-----*/
	//���̵��PID
	for(int i = 0; i < 4; i++){
	IncrementalPID_paraReset(&M3508s[i].pid, 3.0f, 0.3f, 0.0f, 8000, 1000);
	}
	PositionPID_paraReset(&Chassis.FollowYawAttitude_pid, 4000.0f, 0.0f, 10.0f, 6000, 500);
}	


/**
  * @brief  �����ķ���ٶȽ���
  * @param[in]  Vx	  	x���ٶ�
  *				      Vy		  y���ٶ�
  *			      	VOmega	��ת�ٶ�
  * @param[out]	Speed	�ٶ�
  * @retval None
  */
void MecanumCalculate(float Vx, float Vy, float VOmega, int16_t *Speed)
{
  float tempSpeed[4];
	float MaxSpeed = 0.0f;
	float Param = 1.0f;
	
	//�ٶ�����
	VAL_LIMIT(Vx, -Chassis_MaxSpeed_X, Chassis_MaxSpeed_X);  
	VAL_LIMIT(Vy, -Chassis_MaxSpeed_Y, Chassis_MaxSpeed_Y);  
	VAL_LIMIT(VOmega, -Chassis_MaxSpeed_W, Chassis_MaxSpeed_W);  
	
	//�����ٶȷֽ�
	tempSpeed[0] = Vx - Vy + VOmega;
	tempSpeed[1] = Vx + Vy + VOmega;
	tempSpeed[2] = -Vx + Vy + VOmega;
	tempSpeed[3] = -Vx - Vy + VOmega;
	
    //Ѱ������ٶ�
    for(uint8_t i = 0; i < 4; i++)
    {
        if(abs(tempSpeed[i]) > MaxSpeed)
        {
            MaxSpeed = abs(tempSpeed[i]);
        }
    }
	
	
	//�ٶȷ���
    if(MaxSpeed > WheelMaxSpeed)
    {
        Param = (float)WheelMaxSpeed / MaxSpeed;
    }
	
	Speed[0] = tempSpeed[0] * Param;
	Speed[1] = tempSpeed[1] * Param;
	Speed[2] = tempSpeed[2] * Param;
	Speed[3] = tempSpeed[3] * Param;
	
}

/**
  * @brief  ������̨Ŀ��Ƕ��뵱ǰ�ǶȵĲ�ֵ����Vomega���и�ֵ
  * @param  ʹ�����˶�������̨�˶�
  * @retval None
  */							
float Chassis_followAngle(float target_angle,float real_angle)
{
	int16_t err;
	err = target_angle - real_angle;
  if (err > 4096)
  {
	  err -= 8191;
  }
  else if (err < -4096)
  {
	  err += 8191;
  }
		return err;
	
}

/**
  * @brief  ���̸�����̨
  * @param  ������̨Ŀ��ǶȲ�����ʵ�ǶȲ��Vomega��ֵ��ʹ�����ٶȲ��䣩
  * @retval None
  */
void Chassis_Follow_Cloud(float Vx, float Vy, float VOmega)
{
	int16_t speed[4];
	
	//���̸�����̨
	M6020s[0].countAngle = Chassis_followAngle(Yaw_Center_Angle , M6020s[0].totalAngle);
	
	if(abs(M6020s[0].countAngle) < 80)
	{
		M6020s[0].countAngle = 0;
//	M6020s[0].countAngle = M6020s[0].countAngle / (float)M6020_mAngleRatio;
//	M6020s[0].countAngle = M6020s[0].countAngle * (PI / 180);
	
//	if(abs(M6020s[0].totalAngle - Yaw_Center_Angle) > 30)
//	{
//		Position_PID(&Chassis.FollowYawAttitude_pid, M6020s[0].countAngle, 0.0f);
//		VOmega = VOmega + Chassis.FollowYawAttitude_pid.pwm;

	}
	/*******************************���̸��洦��ʼ*****************************/

	//ƽ������	
  Chassis.targetXLPF = Vx;
	Chassis.targetYLPF = Vy;
	Chassis.targetZLPF = VOmega = M6020s[0].countAngle * 1;
	
  Filter_IIRLPF(&Vx, &Chassis.targetXLPF,Chassis.LpfAttFactor);
	Filter_IIRLPF(&Vy, &Chassis.targetYLPF, Chassis.LpfAttFactor);
	Filter_IIRLPF(&VOmega, &Chassis.targetZLPF, Chassis.LpfAttFactor);

	MecanumCalculate(Chassis.targetXLPF, Chassis.targetYLPF, Chassis.targetZLPF, speed);

	for(int i = 0; i<4; i++)
	{
    if(M3508s[i].M3508InfoUpdateFlag == 1)
		{
			
		M3508s[i].targetSpeed = speed[i];
			//PID����
		M3508s[i].outCurrent = Incremental_PID(&M3508s[i].pid, 	M3508s[i].targetSpeed, 	M3508s[i].realSpeed); 
		 //���־λ
		M3508s[i].M3508InfoUpdateFlag = 0;
		}
	}
/*******************************���̸��洦�����*****************************/	
	  M3508_setCurrent(M3508s[0].outCurrent, M3508s[1].outCurrent, M3508s[2].outCurrent, M3508s[3].outCurrent);
	  Chassis.mode = Chassis_Mode_Follow;
}

/**
  * @brief  ���̿��ƴ���(4��)
  * @param[in]  Vx		x���ٶ�
  *				      Vy		y���ٶ�
  *				      Omega	ƫ���
  *				      mode	ģʽ - ��Status_ControlOFF�⣬������������
  * @retval None
  */

void Chassis_processing(float Vx, float Vy, float VOmega, uint8_t mode){
	
	if(mode == ControlMode_OFF){
		M3508_setCurrent(0, 0, 0, 0);
		return;
	}

	if(dr16_data.DR16OffLineFlag){
		Vx = Vy = VOmega = 0.0f;
	}
	
	//ƽ������
	Filter_IIRLPF(&Vx, &Chassis.targetXLPF, Chassis.LpfAttFactor);
	Filter_IIRLPF(&Vy, &Chassis.targetYLPF, Chassis.LpfAttFactor);
	Filter_IIRLPF(&VOmega, &Chassis.targetZLPF, Chassis.LpfAttFactor);
	
	int16_t speed[4];
	//���ֽ���
	MecanumCalculate(Chassis.targetXLPF, Chassis.targetYLPF, Chassis.targetZLPF, speed);
	
	for(int i = 0; i<4; i++){
    if(M3508s[i].M3508InfoUpdateFlag == 1){
			
			M3508s[i].targetSpeed = speed[i];
			//PID����
			M3508s[i].outCurrent = Incremental_PID(&M3508s[i].pid, M3508s[i].targetSpeed, M3508s[i].realSpeed); 
			//���־λ
			M3508s[i].M3508InfoUpdateFlag = 0;
		}
	}

	//�趨�������ֵ
	M3508_setCurrent(M3508s[0].outCurrent, M3508s[1].outCurrent, M3508s[2].outCurrent, M3508s[3].outCurrent);
}

/**
  * @brief      ���̿��ƴ���-����
  * @param[in]  angle		��е�Ƕ�
  *				      Radraw	����ƫ��
  *				      Vx    	x���ٶ�
  *             Vy      y���ٶ�
  * @retval     (VOmega������С�������ƶ������ٶȲ���)
  */
void Chassis_spinning(float Vx, float Vy, float VOmega){
	
	float RadRaw = 0.0f;
	float angle = 0.0f;
	float temp_Vx = 0.0f;
	
	angle = Cloud_getYawAngleWithCenter();//��е�Ƕ�ƫ��
	RadRaw = angle * DEG_TO_RAD;//����ƫ��
	
	/*Chassis_Vx = Chassis_Vx * cos(RadRaw) - Chassis_Vy * sin(RadRaw);
	Chassis_Vy = Chassis_Vy * cos(RadRaw) + temp_Vx * sin(RadRaw);*/

	temp_Vx = Vx;

	Vx = Vx * cos(RadRaw) - Vy * sin(RadRaw);
	Vy = Vy * cos(RadRaw) + temp_Vx * sin(RadRaw);
  VOmega = 2000;
	
	//ƽ������,����˲����ٶԵذ�����ϴ��Ħ����
	Filter_IIRLPF(&Vx, &Chassis.targetXLPF, Chassis.LpfAttFactor);
	Filter_IIRLPF(&Vy, &Chassis.targetYLPF, Chassis.LpfAttFactor);
	Filter_IIRLPF(&VOmega, &Chassis.targetZLPF, Chassis.LpfAttFactor);
	
	int16_t speed[4];
	
	//���ֽ���
	MecanumCalculate(Chassis.targetXLPF, Chassis.targetYLPF, Chassis.targetZLPF, speed);
	
	for(int i = 0; i<4; i++){
		//�����ڸ�ֵ��ζ�����Ŷ�ȡ��һ�����ݵ����
    if(M3508s[i].M3508InfoUpdateFlag == 1){
			M3508s[i].targetSpeed = speed[i];
			
			//PID����
			M3508s[i].outCurrent = Incremental_PID(&M3508s[i].pid, M3508s[i].targetSpeed, M3508s[i].realSpeed); 
			//M3508s[i].outCurrent = Position_PID(&M3508s[i].pid, M3508s[i].targetSpeed, M3508s[i].realSpeed); 
			
			//���־λ
			M3508s[i].M3508InfoUpdateFlag = 0;
		}
	}
	//�趨�������ֵ
	M3508_setCurrent(M3508s[0].outCurrent, M3508s[1].outCurrent, M3508s[2].outCurrent, M3508s[3].outCurrent);
}
