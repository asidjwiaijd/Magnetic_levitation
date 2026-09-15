/********************************** (C) COPYRIGHT *******************************
 * File Name          : ui.c
 * Description        : WS2812 状态显示
 *
 * 悬浮时四颗灯一一对应四路线圈, 直接把驱动量显示出来:
 *      蓝色  = 正向电流 (IN1 侧为高), 亮度正比于 |指令|
 *      红色  = 反向电流
 *      暗    = 该路没出力
 *      叠加一点绿底 = 环路在正常悬浮
 * 这样一眼就能看出是哪一路在死撑、四路是否对称、有没有单路饱和。
 *
 * 其余状态用整条链的动效区分:
 *      标定中 = 蓝色流水     等待矿石 = 黄色呼吸     故障 = 红色快闪
 *
 * 亮度上限压到 160: LED 供电轨(网络名 +5V)实际由 U7 这颗 ME6211A33 供出,
 * 四颗全白会拉走可观的电流, 没必要为指示灯去挤 LDO 的余量。
 *******************************************************************************/
#include "ui.h"
#include "ws2812.h"
#include "coil.h"
#include "levitation.h"

#define BRIGHT_MAX      160

static const uint8_t led_coil[WS2812_LED_NUM] = LED_TO_COIL_INIT;
static uint16_t phase;      /* 动效相位, 每次 UI_Task 自增 */

void UI_Init(void)
{
    WS2812_Init();
    phase = 0;
}

/* |cmd|(0..COIL_CMD_MAX) -> 亮度(0..BRIGHT_MAX) */
static uint8_t cmd_to_bright(int16_t cmd)
{
    int32_t v = cmd < 0 ? -cmd : cmd;
    v = v * BRIGHT_MAX / COIL_CMD_MAX;
    if(v > BRIGHT_MAX) v = BRIGHT_MAX;
    return (uint8_t)v;
}

/* 0..255 的三角波, 用作呼吸 */
static uint8_t triangle(uint16_t x)
{
    x &= 0x1FF;
    return (x < 0x100) ? (uint8_t)x : (uint8_t)(0x1FF - x);
}

void UI_Task(void)
{
    const int16_t *out = Coil_GetOutputs();
    LevState_t st = Levitation_GetState();
    uint8_t i;

    phase++;

    switch(st)
    {
    case LEV_FAULT:
    {
        uint8_t on = (phase & 0x08) ? BRIGHT_MAX : 0;      /* ~3Hz 快闪 */
        for(i = 0; i < WS2812_LED_NUM; i++) WS2812_SetColor(i, on, 0, 0);
        break;
    }

    case LEV_BOOT:
    case LEV_CALIB:
    {
        uint8_t head = (uint8_t)((phase >> 3) % WS2812_LED_NUM);
        for(i = 0; i < WS2812_LED_NUM; i++)
            WS2812_SetColor(i, 0, 0, (i == head) ? BRIGHT_MAX : 12);
        break;
    }

    case LEV_IDLE:
    {
        uint8_t v = (uint8_t)((uint16_t)triangle(phase << 2) * BRIGHT_MAX / 255);
        for(i = 0; i < WS2812_LED_NUM; i++) WS2812_SetColor(i, v, v / 2, 0);
        break;
    }

    default:    /* LEV_RUN */
        for(i = 0; i < WS2812_LED_NUM; i++)
        {
            int16_t cmd = out[led_coil[i]];
            uint8_t b = cmd_to_bright(cmd);
            if(cmd > 0)      WS2812_SetColor(i, 0, 16, b);      /* 正向: 蓝 */
            else if(cmd < 0) WS2812_SetColor(i, b, 16, 0);      /* 反向: 红 */
            else             WS2812_SetColor(i, 0, 16, 0);      /* 未出力: 绿底 */
        }
        break;
    }

    WS2812_Update();
}
