#ifndef __USERCODE_H
#define __USERCODE_H 	

#include "user_common.h"

extern TaskHandle_t StartTask_Handler;     //任务句柄
extern TaskHandle_t LED_GREENTask_Handler;

#define START_TASK_PRIO		    1         //任务优先级
#define START_STK_SIZE 		    128       //任务堆栈大小	

#define LED_GREEN_TASK_PRIO		2         //任务优先级
#define LED_GREEN_STK_SIZE 		50        //任务堆栈大小	

void start_task(void *pvParameters);//任务函数
void LED_GREEN_task(void *pvParameters);

void OS_Start(void);
void BSP_Init(void);
	
#endif /* __USERCODE_H */

