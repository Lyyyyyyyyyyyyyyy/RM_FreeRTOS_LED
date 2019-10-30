#ifndef __BSP_LED_H
#define __BSP_LED_H

#include "user_common.h"

#define LED_GREEN_GPIO_CLK    RCC_AHB1Periph_GPIOF
#define LED_GREEN_PIN         GPIO_Pin_14
#define LED_GREEN_GPIO_PORT   GPIOF

#define LED_RED_GPIO_CLK      RCC_AHB1Periph_GPIOE
#define LED_RED_PIN           GPIO_Pin_11
#define LED_RED_GPIO_PORT     GPIOE

#define LED_GREEN_TOGGLE() 		GPIO_ToggleBits(LED_GREEN_GPIO_PORT,LED_GREEN_PIN)
#define LED_RED_TOGGLE() 			GPIO_ToggleBits(LED_RED_GPIO_PORT, LED_RED_PIN)

void LED_Init(void);

#endif /* __BSP_LED_H */

