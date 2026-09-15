/********************************** (C) COPYRIGHT *******************************
 * File Name          : referee.h
 * Description        : 裁判系统通信 (USART1, PA9/PA10, 115200 8N1)
 *******************************************************************************/
#ifndef __REFEREE_H
#define __REFEREE_H

#include "board.h"

#define REF_DOWN_HEAD       0xAA
#define REF_UP_HEAD         0xBB
#define REF_CMD_SET_HEIGHT  0x01
#define REF_STATUS_NORMAL   0x00
#define REF_STATUS_SETTLED  0x01

/* 上行帧长度。协议表格列出的是 Byte0~Byte3 共 4 字节, 且校验和定义为
 * 「Byte0~Byte2 累加和取低 8 位」, 自洽; 但同一份协议的标题写的是「6 字节」。
 * 以表格为准实现 4 字节。若裁判系统实际要 6 字节, 把这里改成 6, 多出的
 * Byte4/Byte5 会补 0x00 发出(校验和仍只覆盖 Byte0~Byte2)。 */
#define REF_UPLINK_LEN      4

void Referee_Init(void);
void Referee_PollRx(void);          /* 主循环按 CONTROL_HZ 调用, 分发收到的帧 */
void Referee_Task(void);            /* 主循环按 REPORT_HZ 调用, 负责上报 */
void Referee_IRQHandler(void);      /* 由 ch32v20x_it.c 的 USART1_IRQHandler 调用 */

/* 把任意字节压进发送队列 —— 供调参通道 (tuning.c) 复用这条串口 */
void Referee_Send(const uint8_t *data, uint8_t len);

/* 因下行队列满而丢掉的调试帧数 (饱和在 255)。丢帧本身是静默的, 只能靠这个数
 * 发现 —— 上位机把它显示在状态栏, 非 0 就说明命令没全部执行到。 */
uint8_t Referee_GetRxDrops(void);

#endif /* __REFEREE_H */
