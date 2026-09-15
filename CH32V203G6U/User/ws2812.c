/********************************** (C) COPYRIGHT *******************************
 * File Name          : ws2812.c
 * Description        : WS2812B 驱动 —— PB4 位翻转, 800kHz
 *
 * 两个必须保留的实现细节 (来自 maimai/button 参考工程的实测结论):
 *  1) PB4 必须配成开漏。DIN 网络经 R9(910R) 上拉到 LED 供电轨, 开漏时高电平
 *     由外部上拉给出, 电平才够 WS2812 的 0.7*VDD 阈值; 用推挽会和上拉打架,
 *     信号停在临界电平, 表现为第一颗亮、后面的锁存失败。
 *  2) 必须直接写 BSHR/BCR 寄存器。GPIO_SetBits/ResetBits 是非内联调用且带分支,
 *     96MHz 下这点开销会拉长每个 bit 的波形, 累积偏差同样会让菊花链后段失效。
 *******************************************************************************/
#include "ws2812.h"
#include "debug.h"

static RGB_t led[WS2812_LED_NUM];

#define WS2812_HIGH()  (GPIOB->BSHR = GPIO_Pin_4)
#define WS2812_LOW()   (GPIOB->BCR  = GPIO_Pin_4)

/* 96MHz 下 1 个 NOP ≈ 10.4ns */
#define NOP2   __NOP(); __NOP();
#define NOP5   __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
#define NOP10  NOP5 NOP5
#define NOP20  NOP10 NOP10
#define NOP40  NOP20 NOP20

static void WS2812_SendByte(uint8_t byte)
{
    uint8_t i;
    for(i = 0; i < 8; i++)
    {
        if(byte & 0x80)     /* 1 码: T1H≈0.8us, T1L≈0.45us */
        {
            WS2812_HIGH();
            NOP40 NOP20 NOP10 NOP5 NOP2
            WS2812_LOW();
            NOP40 NOP2 __NOP();
        }
        else                /* 0 码: T0H≈0.4us, T0L≈0.85us */
        {
            WS2812_HIGH();
            NOP20 NOP10 NOP5 NOP2 __NOP();
            WS2812_LOW();
            NOP40 NOP40 NOP2
        }
        byte <<= 1;
    }
}

void WS2812_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_4;
    gpio.GPIO_Mode = GPIO_Mode_Out_OD;      /* 见文件头说明 */
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);
    WS2812_LOW();

    WS2812_Clear();
}

void WS2812_SetColor(uint8_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if(index < WS2812_LED_NUM)
    {
        led[index].r = r;
        led[index].g = g;
        led[index].b = b;
    }
}

void WS2812_Update(void)
{
    uint8_t i;

    /* 位翻转时序不能被打断: 控制环的 TIM4 中断会把 bit 拉长导致锁存失败。
     * 4 颗 LED 共 96 bit, 关中断约 120us, 之后再等 60us 复位。 */
    __disable_irq();
    for(i = 0; i < WS2812_LED_NUM; i++)
    {
        WS2812_SendByte(led[i].g);
        WS2812_SendByte(led[i].r);
        WS2812_SendByte(led[i].b);
    }
    __enable_irq();

    WS2812_LOW();
    Delay_Us(60);
}

void WS2812_Clear(void)
{
    uint8_t i;
    for(i = 0; i < WS2812_LED_NUM; i++)
    {
        led[i].r = 0;
        led[i].g = 0;
        led[i].b = 0;
    }
    WS2812_Update();
}
