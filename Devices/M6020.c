/**
  ******************************************************************************
  * @file    M6020.c
  * @author   
  * @version  
  * @date    
  * @brief   M6020无刷电机
	           M6020s[0]yaw
	           M6020s[1]pitch
  ******************************************************************************
  */

#include "M6020.h"

M6020s_t M6020s[2];

/**
  * @brief  设置M6020电机电流值（id号为1~4）
  * @param  iqx (x:1~4) 对应id号电机的电流值，范围-16384~0~16384
  * @retval 
  */
	
void M6020_setCurrent(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4){

	uint8_t data[8];
	
	//数据格式详见说明书P32
	data[0] = iq1 >> 8;
	data[1] = iq1;
	data[2] = iq2 >> 8;
	data[3] = iq2;
	data[4] = iq3 >> 8;
	data[5] = iq3;
	data[6] = iq4 >> 8;
	data[7] = iq4;
	
	CAN_SendData(CAN1, CAN_ID_STD, M6020_SENDID, data);
	
}	

/**
  * @brief  从CAN报文中获取M6020电机信息
  * @param  RxMessage 	CAN报文接收结构体
  * @retval None
  */

void M6020_getInfo(CanRxMsg RxMessage){
	//报文id确认
	if((RxMessage.StdId < M6020_READID_START) || (RxMessage.StdId > M6020_READID_END))
		return;
	uint32_t StdId;
	StdId = RxMessage.StdId - M6020_READID_START;
	
	//解包数据，数据格式详见C620电调说明书P33
	M6020s[StdId].realAngle = (uint16_t)(RxMessage.Data[0]<<8 | RxMessage.Data[1]);   //读回来的机械角度
	M6020s[StdId].realSpeed = (int16_t)(RxMessage.Data[2]<<8 | RxMessage.Data[3]);    //读回来的速度
	M6020s[StdId].realTorque = (int16_t)(RxMessage.Data[4]<<8 | RxMessage.Data[5]);   //读回来的实际转矩
	M6020s[StdId].temperture = RxMessage.Data[6];
	
	//过零处理
	    if(M6020s[StdId].realAngle - M6020s[StdId].lastAngle < -6000){
		    M6020s[StdId].turnCount++;
			}

			if(M6020s[StdId].lastAngle - M6020s[StdId].realAngle < -6000){
				M6020s[StdId].turnCount--;
			}

			M6020s[StdId].lastAngle =  M6020s[StdId].realAngle;

			M6020s[StdId].totalAngle = M6020s[StdId].realAngle + (8192*M6020s[StdId].turnCount);
 
	//清零处理
			if(	M6020s[StdId].totalAngle > 30000 || M6020s[StdId].totalAngle < -30000)
			{
				M6020s[StdId].totalAngle = M6020s[StdId].totalAngle-(8192*M6020s[StdId].turnCount);
			  M6020s[StdId].targetAngle = M6020s[StdId].targetAngle -(8192*M6020s[StdId].turnCount);
				M6020s[StdId].turnCount=0;
			}
			
	int count=0;
	
	//任何情况下云台使能不兜圈（陀螺仪）
	if(IMU_Cloud.totalYaw-IMU_Cloud.targetYaw >= 170)
	{
		count=(IMU_Cloud.totalYaw-IMU_Cloud.targetYaw)/200;
	  IMU_Cloud.targetYaw+=360*count;
	}
	if(IMU_Cloud.totalYaw-IMU_Cloud.targetYaw < 170)
	{
		count=(IMU_Cloud.totalYaw-IMU_Cloud.targetYaw)/200;
	  IMU_Cloud.targetYaw+=360*count;
	}
	
	//帧率统计，数据更新标志位
	M6020s[StdId].M6020InfoUpdateFrame++;
	M6020s[StdId].M6020InfoUpdateFlag = 1;
}	

