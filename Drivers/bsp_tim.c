/**
  ******************************************************************************
  * @file    bsp_tim.c
  * @author   
  * @version  
  * @date    
  * @brief   TIMӦ�ú����ӿ�
  ******************************************************************************
  */

#include "bsp_tim.h"

/**
  * @brief  ������ʱ��TIM6��Ϊ����ģʽ��ʼ��
  * @param  prescaler 				ʱ��Ԥ��Ƶ
  *		    	period					  ��ʱ����װ�ؼĴ�����ֵ
  *			    ��ʱ����ʱƵ�ʣ�  ϵͳʱ��Ƶ��/Ԥ��Ƶ/��װ��ֵ
  * @retval None
  */
	
void TIM6_CounterMode(u16 prescaler, u16 period){
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);//ʹ�ܶ�ʱ��ʱ��
  
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;   	/* ����ʱ���ṹ�� */
	
	TIM_TimeBaseStructure.TIM_Prescaler = prescaler;    //���ö�ʱ��ʱ�ӷ�Ƶ
	TIM_TimeBaseStructure.TIM_Period = period;          //�����Զ���װ�ؼĴ�����ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;    //��������ʱ��������˲���Ƶ������Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//���ü���ģʽ�����ϼ���
	
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
	
	TIM_ARRPreloadConfig(TIM6, ENABLE);
	
	TIM_Cmd(TIM6, ENABLE);//ʹ�ܶ�ʱ��
	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
	NVIC_Config(TIM6_DAC_IRQn, 1, 0);
	
}



