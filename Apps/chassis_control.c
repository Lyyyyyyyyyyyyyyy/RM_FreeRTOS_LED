/**
  ******************************************************************************
  * @file    Chassis_control.c
  * @author   
  * @version  
  * @date    
  * @brief   底盘控制函数接口
  ******************************************************************************
  */
  
#include "chassis_control.h"

Chassis_t Chassis;

/**
  * @brief  底盘初始化，配置参数
  * @param  None
  * @retval None
  */
void Chassis_Init(void){
	
  Chassis.LpfAttFactor = Chassis_LpfAttFactor;
	Chassis.speedLimit = Chassis_SpeedNormal;
	Chassis.mode = Chassis_Mode_Normal;
	Chassis.PowerOverflowFlag = 1;
	Chassis.swingSpeed = Chassis_SwingSpeed_Normal;
	
	/*-----正常使用PID-----*/
	//底盘电机PID
	for(int i = 0; i < 4; i++){
	IncrementalPID_paraReset(&M3508s[i].pid, 3.0f, 0.3f, 0.0f, 8000, 1000);
	}
	PositionPID_paraReset(&Chassis.FollowYawAttitude_pid, 4000.0f, 0.0f, 10.0f, 6000, 500);
}	


/**
  * @brief  麦克纳姆轮速度解算
  * @param[in]  Vx	  	x轴速度
  *				      Vy		  y轴速度
  *			      	VOmega	自转速度
  * @param[out]	Speed	速度
  * @retval None
  */
void MecanumCalculate(float Vx, float Vy, float VOmega, int16_t *Speed)
{
  float tempSpeed[4];
	float MaxSpeed = 0.0f;
	float Param = 1.0f;
	
	//速度限制
	VAL_LIMIT(Vx, -Chassis_MaxSpeed_X, Chassis_MaxSpeed_X);  
	VAL_LIMIT(Vy, -Chassis_MaxSpeed_Y, Chassis_MaxSpeed_Y);  
	VAL_LIMIT(VOmega, -Chassis_MaxSpeed_W, Chassis_MaxSpeed_W);  
	
	//四轮速度分解
	tempSpeed[0] = Vx - Vy + VOmega;
	tempSpeed[1] = Vx + Vy + VOmega;
	tempSpeed[2] = -Vx + Vy + VOmega;
	tempSpeed[3] = -Vx - Vy + VOmega;
	
    //寻找最大速度
    for(uint8_t i = 0; i < 4; i++)
    {
        if(abs(tempSpeed[i]) > MaxSpeed)
        {
            MaxSpeed = abs(tempSpeed[i]);
        }
    }
	
	
	//速度分配
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
  * @brief  计算云台目标角度与当前角度的差值，对Vomega进行赋值
  * @param  使底盘运动跟随云台运动
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
  * @brief  底盘跟随云台
  * @param  利用云台目标角度查与真实角度差，给Vomega赋值（使自旋速度不变）
  * @retval None
  */
void Chassis_Follow_Cloud(float Vx, float Vy, float VOmega)
{
	int16_t speed[4];
	
	//底盘跟随云台
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
	/*******************************底盘跟随处理开始*****************************/

	//平滑处理	
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
			//PID计算
		M3508s[i].outCurrent = Incremental_PID(&M3508s[i].pid, 	M3508s[i].targetSpeed, 	M3508s[i].realSpeed); 
		 //清标志位
		M3508s[i].M3508InfoUpdateFlag = 0;
		}
	}
/*******************************底盘跟随处理结束*****************************/	
	  M3508_setCurrent(M3508s[0].outCurrent, M3508s[1].outCurrent, M3508s[2].outCurrent, M3508s[3].outCurrent);
	  Chassis.mode = Chassis_Mode_Follow;
}

/**
  * @brief  底盘控制处理(4轮)
  * @param[in]  Vx		x轴速度
  *				      Vy		y轴速度
  *				      Omega	偏向角
  *				      mode	模式 - 除Status_ControlOFF外，其他正常控制
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
	
	//平滑处理
	Filter_IIRLPF(&Vx, &Chassis.targetXLPF, Chassis.LpfAttFactor);
	Filter_IIRLPF(&Vy, &Chassis.targetYLPF, Chassis.LpfAttFactor);
	Filter_IIRLPF(&VOmega, &Chassis.targetZLPF, Chassis.LpfAttFactor);
	
	int16_t speed[4];
	//麦轮解算
	MecanumCalculate(Chassis.targetXLPF, Chassis.targetYLPF, Chassis.targetZLPF, speed);
	
	for(int i = 0; i<4; i++){
    if(M3508s[i].M3508InfoUpdateFlag == 1){
			
			M3508s[i].targetSpeed = speed[i];
			//PID计算
			M3508s[i].outCurrent = Incremental_PID(&M3508s[i].pid, M3508s[i].targetSpeed, M3508s[i].realSpeed); 
			//清标志位
			M3508s[i].M3508InfoUpdateFlag = 0;
		}
	}

	//设定电机电流值
	M3508_setCurrent(M3508s[0].outCurrent, M3508s[1].outCurrent, M3508s[2].outCurrent, M3508s[3].outCurrent);
}

/**
  * @brief      底盘控制处理-自旋
  * @param[in]  angle		机械角度
  *				      Radraw	弧度偏差
  *				      Vx    	x轴速度
  *             Vy      y轴速度
  * @retval     (VOmega处理让小陀螺仪移动自旋速度不变)
  */
void Chassis_spinning(float Vx, float Vy, float VOmega){
	
	float RadRaw = 0.0f;
	float angle = 0.0f;
	float temp_Vx = 0.0f;
	
	angle = Cloud_getYawAngleWithCenter();//机械角度偏差
	RadRaw = angle * DEG_TO_RAD;//弧度偏差
	
	/*Chassis_Vx = Chassis_Vx * cos(RadRaw) - Chassis_Vy * sin(RadRaw);
	Chassis_Vy = Chassis_Vy * cos(RadRaw) + temp_Vx * sin(RadRaw);*/

	temp_Vx = Vx;

	Vx = Vx * cos(RadRaw) - Vy * sin(RadRaw);
	Vy = Vy * cos(RadRaw) + temp_Vx * sin(RadRaw);
  VOmega = 2000;
	
	//平滑处理,避免瞬间加速对地板产生较大的摩擦。
	Filter_IIRLPF(&Vx, &Chassis.targetXLPF, Chassis.LpfAttFactor);
	Filter_IIRLPF(&Vy, &Chassis.targetYLPF, Chassis.LpfAttFactor);
	Filter_IIRLPF(&VOmega, &Chassis.targetZLPF, Chassis.LpfAttFactor);
	
	int16_t speed[4];
	
	//麦轮解算
	MecanumCalculate(Chassis.targetXLPF, Chassis.targetYLPF, Chassis.targetZLPF, speed);
	
	for(int i = 0; i<4; i++){
		//避免在赋值多次而电机才读取了一次数据的情况
    if(M3508s[i].M3508InfoUpdateFlag == 1){
			M3508s[i].targetSpeed = speed[i];
			
			//PID计算
			M3508s[i].outCurrent = Incremental_PID(&M3508s[i].pid, M3508s[i].targetSpeed, M3508s[i].realSpeed); 
			//M3508s[i].outCurrent = Position_PID(&M3508s[i].pid, M3508s[i].targetSpeed, M3508s[i].realSpeed); 
			
			//清标志位
			M3508s[i].M3508InfoUpdateFlag = 0;
		}
	}
	//设定电机电流值
	M3508_setCurrent(M3508s[0].outCurrent, M3508s[1].outCurrent, M3508s[2].outCurrent, M3508s[3].outCurrent);
}
