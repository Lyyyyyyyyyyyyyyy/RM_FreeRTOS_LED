#include "bsp_led.h"

/**
  * @brief  LEDµ∆≥ı ºªØ
  * @param  None
  * @retval None
  */
void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(LED_GREEN_GPIO_CLK|LED_RED_GPIO_CLK, ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;
	
	GPIO_InitStructure.GPIO_Pin = LED_GREEN_PIN;
	GPIO_Init(LED_GREEN_GPIO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = LED_RED_PIN;
	GPIO_Init(LED_RED_GPIO_PORT, &GPIO_InitStructure);

	GPIO_SetBits(LED_GREEN_GPIO_PORT, LED_GREEN_PIN);	
  GPIO_ResetBits(LED_RED_GPIO_PORT, LED_RED_PIN);	
}

