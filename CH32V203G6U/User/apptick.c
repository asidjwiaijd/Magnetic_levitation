/********************************** (C) COPYRIGHT *******************************
 * File Name          : apptick.c
 * Description        : TIM4 控制节拍
 *
 * 为什么不用 SysTick: debug.c 的 Delay_Us/Delay_Ms 是把 SysTick 当一次性
 * 定时器反复重配来实现的(每次调用都改 CMP 并重启计数), 再挂中断会互相破坏。
 *
 * 中断里只做计数, 真正的控制运算放在主循环 —— 控制拍里有阻塞式 I2C 读,
 * 放中断会把 USART 的收发挤掉。
 *******************************************************************************/
#include "apptick.h"

static volatile uint32_t tick_isr;
static uint32_t tick_taken;
static uint32_t overruns;

void Tick_Init(void)
{
    TIM_TimeBaseInitTypeDef tb = {0};
    NVIC_InitTypeDef nvic = {0};

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    /* APB1 = 48MHz, 但预分频不为 1 时定时器时钟翻倍 -> 96MHz。
     * 先分到 1MHz, 再按 CONTROL_HZ 定周期。 */
    tb.TIM_Prescaler = 96 - 1;
    tb.TIM_Period = (1000000 / CONTROL_HZ) - 1;
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    tb.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &tb);

    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

    nvic.NVIC_IRQChannel = TIM4_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    tick_isr = 0;
    tick_taken = 0;
    overruns = 0;

    TIM_Cmd(TIM4, ENABLE);
}

void Tick_ISR(void)
{
    tick_isr++;
}

uint8_t Tick_Take(void)
{
    uint32_t now = tick_isr;
    uint32_t behind = now - tick_taken;

    if(behind == 0) return 0;

    if(behind > 1)
    {
        /* 主循环这一拍没跑完, 记一次并直接对齐到最新, 不补跑 —— 补跑只会
         * 让积分项按错误的 dt 累加, 越补越落后。 */
        overruns++;
        tick_taken = now;
    }
    else
    {
        tick_taken++;
    }
    return 1;
}

uint32_t Tick_Count(void)    { return tick_isr; }
uint32_t Tick_Overruns(void) { return overruns; }
