/**
  ******************************************************************************
  * @file    DevicesMonitor.c
  * @author   
  * @version  
  * @date    
  * @brief   �ⲿ�豸֡�ʼ�⺯���ӿ�
  ******************************************************************************
  */
  
#include "DevicesMonitor.h"

#define GY_IMU_FRAMEMIN		5   //��С֡��

/**
  * @brief  ����豸֡�����㣬ÿ200ms����һ��
  * @param  None
  * @retval None
  */
	
void DevicesMonitor_update(void){

	for(int i = 0; i<4; i++){
		if(M3508s[i].M3508InfoUpdateFrame < 5){
			
			M3508s[i].M3508OffLineFlag = 1;
		}
		else{
			
			M3508s[i].M3508OffLineFlag = 0;
		}
		
		M3508s[i].M3508InfoUpdateFrame = 0;
	}

	if(dr16_data.DR16InfoUpdateFrame < 5){
		
		dr16_data.DR16OffLineFlag = 1;
	}
	else{
		
		dr16_data.DR16OffLineFlag = 0;
	}
	dr16_data.DR16InfoUpdateFrame = 0;
}
