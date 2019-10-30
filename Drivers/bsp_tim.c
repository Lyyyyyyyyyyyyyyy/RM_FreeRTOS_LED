/**
  ******************************************************************************
  * @file    bsp_tim.c
  * @author   
  * @version  
  * @date    
  * @brief   TIM应用函数接口
  ******************************************************************************
  */

#include "bsp_tim.h"

/**
  * @brief  基本定时器TIM6作为计数模式初始化
  * @param  prescaler 				时钟预分频
  *		    	period					  定时器重装载寄存器的值
  *			    定时器定时频率：  系统时钟频率/预分频/重装载值
  * @retval None
  */
	
void TIM6_CounterMode(u16 prescaler, u16 period){
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);//使能定时器时钟
  
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;   	/* 配置时基结构体 */
	
	TIM_TimeBaseStructure.TIM_Prescaler = prescaler;    //配置定时器时钟分频
	TIM_TimeBaseStructure.TIM_Period = period;          //配置自动重装载寄存器的值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;    //配置死区时间与采样滤波分频，不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//配置计数模式，向上计数
	
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
	
	TIM_ARRPreloadConfig(TIM6, ENABLE);
	
	TIM_Cmd(TIM6, ENABLE);//使能定时器
	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
	NVIC_Config(TIM6_DAC_IRQn, 1, 0);
	
}



