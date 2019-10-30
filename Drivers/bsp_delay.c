/**
  ******************************************************************************
  * @file    bsp_delay.c
  * @author   
  * @version  
  * @date     
  * @brief   ��ʱӦ�ú����ӿ�
  ******************************************************************************
  */
  
  
#include "bsp_delay.h"

#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"				 
#include "task.h"
#endif

static u32 fac_us=0;							//us��ʱ������
static u16 fac_ms=0;				      //ms��ʱ������,��os��,����ÿ�����ĵ�ms��

/**
  * @brief  ��ʼ���ӳٺ���
  * @param  SYSCLK:ϵͳʱ��Ƶ��
  * @retval None
  */
void delay_init(void)
{
	u32 reload;					                            //ÿ���ӵļ������� ��λΪK	
	
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);//ѡ���ⲿʱ��HCLK
	
	fac_us=SystemCoreClock/1000000;			            //�����Ƿ�ʹ��OS,fac_us����Ҫʹ�� 
	
	reload=SystemCoreClock/1000000;                 //ÿ���ӵļ������� ��λΪM
	reload*=1000000/configTICK_RATE_HZ;		          //����configTICK_RATE_HZ�趨���ʱ��
	fac_ms=1000/configTICK_RATE_HZ;		            	//����OS������ʱ�����ٵ�λ
	
  SysTick->CTRL|=SysTick_CTRL_TICKINT_Msk;        //����SYSTICK�ж�
	SysTick->LOAD=reload; 					                //ÿ1/configTICK_RATE_HZ��һ��	
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk;         //����SYSTICK
}

/**
  * @brief  ���뼶��ʱ,�������������
  * @param  _ms ��ʱ������
  * @retval None
  */
void delay_ms(u32 _ms)
{	
	if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)//ϵͳ�Ѿ�����
	{		
		if(_ms>=fac_ms)						                           //��ʱ��ʱ�����OS������ʱ������ 
		{ 
   			vTaskDelay(_ms/fac_ms);	 		                     //FreeRTOS��ʱ
		}
		_ms%=fac_ms;					                               //OS�Ѿ��޷��ṩ��ôС����ʱ��,������ͨ��ʽ��ʱ    
	}
	delay_us((u32)(_ms*1000));				                     //��ͨ��ʽ��ʱ
}

/**
  * @brief  ΢�뼶��ʱ
  * @param  _us ��ʱ΢����
  * @retval None
  */
void delay_us(u32 _us)
{
	u32 ticks=_us*fac_us;						//��Ҫ�Ľ�����
	u32 told=SysTick->VAL;        	//�ս���ʱ�ļ�����ֵ
	u32 reload=SysTick->LOAD;				//װ��ֵ
	u32 tnow=SysTick->VAL;					//��ǰ������ֵ
	u32 tcnt=0;
	while(1)
	{
	  tnow=SysTick->VAL;	          //��ǰ������ֵ
		if(tnow!=told)
		{	    
			if(tnow<told)tcnt+=told-tnow;	
			else tcnt+=reload-tnow+told;	    
			told=tnow;
			if(tcnt>=ticks)break;			//ʱ�䳬��/����Ҫ�ӳٵ�ʱ��,���˳�.
		} 
	};
}


/**
  * @brief  ���뼶��ʱ,���������������
  * @param  nms ��ʱ������
  * @retval None
  */
void delay_xms(u32 nms)
{
	u32 i;
	for(i=0;i<nms;i++) delay_us(1000);
}
