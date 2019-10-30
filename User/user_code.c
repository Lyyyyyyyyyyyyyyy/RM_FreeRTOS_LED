#include "user_code.h"

TaskHandle_t StartTask_Handler;     //任务句柄
TaskHandle_t LED_GREENTask_Handler;

/**
  * @brief  系统启动
  * @param  None
  * @retval None
  */
void OS_Start(void)
{
	//创建开始任务
	xTaskCreate((TaskFunction_t )start_task,            
							(const char*    )"start_task",          
							(uint16_t       )START_STK_SIZE,        
							(void*          )NULL,                  
							(UBaseType_t    )START_TASK_PRIO,       
							(TaskHandle_t*  )&StartTask_Handler);              
	vTaskStartScheduler();   
}

/**
  * @brief  底层硬件初始化
  * @param  None
  * @retval None
  */
void BSP_Init(void)
{	
  LED_Init();
}

/**
  * @brief  开始任务，所有的任务创建
  * @param  None
  * @retval None
  */
void start_task(void *pvParameters)
{
	taskENTER_CRITICAL();
							
	//创建LED_GREEN任务
    xTaskCreate((TaskFunction_t )LED_GREEN_task,     	
                (const char*    )"LED_GREEN_task",   	
                (uint16_t       )LED_GREEN_STK_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )LED_GREEN_TASK_PRIO,	
                (TaskHandle_t*  )&LED_GREENTask_Handler);   
								
	vTaskDelete(StartTask_Handler); 
	StartTask_Handler=NULL;
	taskEXIT_CRITICAL();
}

//LED_GREEN任务函数 
void LED_GREEN_task(void *pvParameters)
{
    while(1)
    {
        LED_GREEN_TOGGLE();
        vTaskDelay(500);
    }
}   

