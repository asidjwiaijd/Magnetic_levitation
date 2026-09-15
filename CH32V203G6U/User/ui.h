/********************************** (C) COPYRIGHT *******************************
 * File Name          : ui.h
 * Description        : WS2812 状态显示 —— 四颗灯对应四路线圈的驱动情况
 *******************************************************************************/
#ifndef __UI_H
#define __UI_H

#include "board.h"

void UI_Init(void);
void UI_Task(void);     /* 主循环按 LED_HZ 调用 */

#endif /* __UI_H */
