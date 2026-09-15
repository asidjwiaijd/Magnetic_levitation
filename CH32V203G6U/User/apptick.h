/********************************** (C) COPYRIGHT *******************************
 * File Name          : apptick.h
 * Description        : TIM4 控制节拍 (CONTROL_HZ)
 *******************************************************************************/
#ifndef __APPTICK_H
#define __APPTICK_H

#include "board.h"

void     Tick_Init(void);
void     Tick_ISR(void);        /* 由 ch32v20x_it.c 的 TIM4_IRQHandler 调用 */
uint8_t  Tick_Take(void);       /* 有新节拍则消费掉并返回 1 */
uint32_t Tick_Count(void);
uint32_t Tick_Overruns(void);   /* 主循环没跟上节拍的次数, 调试用 */

#endif /* __APPTICK_H */
