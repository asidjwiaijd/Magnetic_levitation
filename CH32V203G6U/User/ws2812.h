/********************************** (C) COPYRIGHT *******************************
 * File Name          : ws2812.h
 * Description        : WS2812B 驱动 (PB4 开漏位翻转), 4 颗菊花链
 *******************************************************************************/
#ifndef __WS2812_H
#define __WS2812_H

#include "board.h"

typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} RGB_t;

void WS2812_Init(void);
void WS2812_SetColor(uint8_t index, uint8_t r, uint8_t g, uint8_t b);
void WS2812_Update(void);       /* 刷新整条链, 期间关中断 ~180us */
void WS2812_Clear(void);

#endif /* __WS2812_H */
