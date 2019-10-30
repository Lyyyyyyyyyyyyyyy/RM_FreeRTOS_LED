/**
  ******************************************************************************
  * @file    GY6050.c
  * @author   
  * @version V1.0
  * @date    2019.10.06
  * @brief   �����ǣ�Ϊ�����Խ���Ҹ����bug��
  ******************************************************************************
  */

#include "GY6050.h"
#include "string.h"

GY_IMU_t IMU_Cloud;

Gyro_Data Gyro_data[2];

/**
  * @brief  ��CAN�����л�ȡ��̨��������Ϣ
  * @param  RxMessage 	CAN���Ľ��սṹ��
  * @retval None
  */
void GY6050_getCloundInfo(CanRxMsg RxMessage) {
	
	//����idȷ�� 
	if (RxMessage.StdId != GY6050_SENDID)
	return;
	
		memcpy(Gyro_data[0].dataBuff, RxMessage.Data, sizeof(uint8_t[8]));
		
		IMU_Cloud.Eular.Yaw = (float)Gyro_data[0].yaw / 100.0f;
		IMU_Cloud.Gyro.z = Gyro_data[0].gyro_z / 16.0f;
		IMU_Cloud.Eular.Pitch = (float)Gyro_data[0].pitch / 100.0f;
		IMU_Cloud.Gyro.x = Gyro_data[0].gyro_x /16.0f;

		if (abs(IMU_Cloud.Gyro.z) < 2)
		{
			IMU_Cloud.Gyro.z = Gyro_data[0].gyro_z = 0;
		} 
		
  //�����������
		if (IMU_Cloud.Eular.Yaw - IMU_Cloud.lastYaw < -300) {
			IMU_Cloud.turnCount++;
		}
		if (IMU_Cloud.lastYaw - IMU_Cloud.Eular.Yaw < -300) {
			IMU_Cloud.turnCount--;
		}

		IMU_Cloud.totalYaw = IMU_Cloud.Eular.Yaw + (360 * IMU_Cloud.turnCount);

		IMU_Cloud.lastYaw = IMU_Cloud.Eular.Yaw;

		IMU_Cloud.ImuInfoUpdateFrame++;
		IMU_Cloud.ImuInfoUpdateFlag = 1;
}

/**
  * @brief  �����ǳ�ʼ��
  * @param  
  * @retval None
  */

void Gyro_Init(void)
{
	IMU_Cloud.turnCount = 0;
	IMU_Cloud.totalYaw = IMU_Cloud.Eular.Yaw;
	IMU_Cloud.targetYaw =IMU_Cloud.totalYaw; 
}
