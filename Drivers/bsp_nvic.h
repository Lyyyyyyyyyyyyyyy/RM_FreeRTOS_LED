#ifndef __BSP_NVIC_H
#define __BSP_NVIC_H

#include "stm32f4xx.h"

void NVIC_Config(uint16_t IRQChannel, uint16_t MainPriority, uint16_t SubPriority);

#endif /*__BSP_NVIC_H*/
