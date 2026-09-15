/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Description        : 磁悬浮矿石底座 —— 主程序
 *
 * 硬件: CH32V203G6U6 @96MHz(8MHz 晶振) + 4x DRV8870 线圈 + TMAG5273B2 三维霍尔
 *       + 4x WS2812 状态灯 + USART1 接裁判系统
 *
 * 上电流程:
 *   Coil_Init 最先调用, 保证四路全桥在任何其它初始化之前就处于滑行状态;
 *   随后进入 LEV_CALIB 做零点与线圈串扰标定 —— 这一步要求台面上没有矿石。
 *
 * 调度: TIM4 给出 1kHz 节拍, 控制运算在主循环里跑; 上报 50Hz, 灯效 50Hz,
 *       两者错开在不同的拍上, 避开 WS2812 刷新时那 ~180us 的关中断窗口。
 *
 * 注意: 本工程不使用 printf。USART1 接的是裁判系统, 打印会插进协议帧里。
 *******************************************************************************/
#include "debug.h"
#include "board.h"
#include "apptick.h"
#include "coil.h"
#include "levitation.h"
#include "referee.h"
#include "tuning.h"
#include "ui.h"

int main(void)
{
    uint32_t n = 0;
    const uint32_t report_div = CONTROL_HZ / REPORT_HZ;
    const uint32_t led_div    = CONTROL_HZ / LED_HZ;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();

    /* 全桥优先进入安全状态 */
    Coil_Init();

    Referee_Init();
    Tuning_Init();          /* 把 COIL_SIGN_INIT 装进 coil.c, 必须在标定之前 */
    UI_Init();
    Levitation_Init();      /* 内含 I2C 与传感器初始化, 失败会直接进 LEV_FAULT */

    /* 标定要阻塞约 1s, 期间主循环进不来。先刷一次灯, 让指示灯在这一秒里
     * 显示「标定中」而不是全灭。 */
    UI_Task();

    Tick_Init();

    while(1)
    {
        if(!Tick_Take()) continue;

        /* 顺序有讲究: 先收命令, 再跑控制(开环测试生效时它会自动让路),
         * 最后由 Tuning_Task 驱动测试输出并发遥测。 */
        Referee_PollRx();
        Levitation_Task();
        Tuning_Task();
        n++;

        /* 上报和灯效都是 50Hz, 但错开半拍, 避免同一拍里既发帧又关中断刷灯 */
        if((n % report_div) == 0)                   Referee_Task();
        if((n % led_div) == (led_div / 2))          UI_Task();
    }
}
