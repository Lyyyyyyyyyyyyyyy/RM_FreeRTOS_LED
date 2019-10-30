/**
  ******************************************************************************
  * @file    DevicesMonitor.c
  * @author   
  * @version  
  * @date    
  * @brief   外部设备帧率检测函数接口
  ******************************************************************************
  */
  
#include "DevicesMonitor.h"

#define GY_IMU_FRAMEMIN		5   //最小帧率

/**
  * @brief  外接设备帧率清零，每200ms清零一次
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
