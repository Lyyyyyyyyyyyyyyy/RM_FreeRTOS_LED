#ifndef __USERCODE_H
#define __USERCODE_H 	

#include "user_common.h"

extern TaskHandle_t StartTask_Handler;     //������
extern TaskHandle_t LED_GREENTask_Handler;

#define START_TASK_PRIO		    1         //�������ȼ�
#define START_STK_SIZE 		    128       //�����ջ��С	

#define LED_GREEN_TASK_PRIO		2         //�������ȼ�
#define LED_GREEN_STK_SIZE 		50        //�����ջ��С	

void start_task(void *pvParameters);//������
void LED_GREEN_task(void *pvParameters);

void OS_Start(void);
void BSP_Init(void);
	
#endif /* __USERCODE_H */

