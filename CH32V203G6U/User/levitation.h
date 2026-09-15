/********************************** (C) COPYRIGHT *******************************
 * File Name          : levitation.h
 * Description        : 悬浮控制器 —— 状态机 + 高度 PID + 横向 PD
 *******************************************************************************/
#ifndef __LEVITATION_H
#define __LEVITATION_H

#include "board.h"

typedef enum {
    LEV_BOOT = 0,       /* 上电自检 */
    LEV_CALIB,          /* 零点 + 线圈串扰标定 (此时不能放矿石) */
    LEV_IDLE,           /* 等待矿石进入 */
    LEV_RUN,            /* 悬浮中 */
    LEV_FAULT           /* 传感器失效 / 超温, 已切断输出 */
} LevState_t;

void        Levitation_Init(void);
void        Levitation_Task(void);              /* 由主循环按 CONTROL_HZ 调用 */

void        Levitation_SetTargetHeight(uint8_t h_mm);   /* 裁判系统下发, 单位 mm */
uint8_t     Levitation_GetHeight_mm(void);              /* 当前高度, 单位 mm */
LevState_t  Levitation_GetState(void);
uint8_t     Levitation_IsSettled(void);         /* 调整到位且稳定 -> 上报状态字 0x01 */
int16_t     Levitation_GetTemp(void);

/* ---- 调参通道用 ---- */
int16_t     Levitation_GetHeight_01mm(void);
int16_t     Levitation_GetTargetHeight_01mm(void);
/* 补偿后的三轴磁场 (0.01mT), 即控制器真正看到的量 */
void        Levitation_GetField(int16_t *bx, int16_t *by, int16_t *bz);
/* 读回标定时实测的每路正/反向场强 (L1 范数, 每 1000 指令)。
 * 两个方向都接近 0 = 线圈或驱动坏了; 只有一个接近 0 = 单 PWM 的快衰减死区。 */
void        Levitation_GetCalMag(uint8_t ch, float *pos, float *neg, uint8_t *sat);
/* 逐轴的串扰斜率 (每 1000 指令的磁场变化, 0.01mT), 取正反向的平均。
 * L1 范数看不出问题所在 —— Z 轴上的串扰只让高度读数偏一点, 而 X/Y 上的串扰会被
 * 当成横向位移直接进那个增益最高的环。判断「串扰大不大」必须看分轴值。 */
void        Levitation_GetCrosstalk(uint8_t ch, float *cx, float *cy, float *cz);
/* 只重测串扰斜率, 保留已标好的零点 —— 铁芯的磁化状态取决于总场, 空载标出来的
 * 斜率在矿石在场时并不成立。矿石必须用非磁性夹具固定在工作高度上再调用。 */
void        Levitation_RecalCrosstalk(void);
/* 标传感器倾斜: 矿石须用非磁性夹具摆在几何正中(那里真实 Bx/By 应为 0)。
 * 返回 0 = 成功, 1 = 矿石不在场或读数失败。 */
uint8_t     Levitation_CalibrateTilt(void);
/* 传感器实际更新率 (每秒出现多少个新的 Z 读数)。接近 CONTROL_HZ 说明跟得上;
 * 明显偏低意味着微分项在对一串重复值求导, 阻尼是假的。 */
uint16_t    Levitation_GetSensorRate(void);
/* 设定点自整定偏置 (磁场 counts)。收敛后 bx 恒等于 trim, 所以不单独看它就
 * 无法区分「矿石真偏了」和「trim 把传感器偏置吃掉了」。 */
void        Levitation_GetTrim(int16_t *tx, int16_t *ty);
/* 把当前读数抓成零点(写 trim_x/trim_y)。自整定积分的手动版本, 一步到位。
 * 要求 LEV_RUN。返回 0 = 成功。 */
uint8_t     Levitation_ZeroHere(void);
/* 按串扰标定的读数把四路强度拉齐(写 coil_gain[])。四路不等造成的是 X-Y 交叉
 * 耦合, 不是轴不对称。返回 0 = 成功, 1 = 还没标定过。 */
uint8_t     Levitation_BalanceCoils(void);
/* 原始三轴读数 (未扣零点与串扰)。判断有没有逼近满量程只能看这个。 */
void        Levitation_GetRawField(int16_t *bx, int16_t *by, int16_t *bz);
/* kz 被改写后重算目标磁场 */
void        Levitation_RefreshTarget(void);
/* 回到标定状态: 重做零点/串扰/分方向增益, 同时清故障 */
void        Levitation_Restart(void);

/* 一点法标定高度常数 K: 把矿石固定在已知高度 h_01mm(单位0.1mm) 处后调用。
 * 见 levitation.c 文件头的标定步骤说明。 */
void        Levitation_CalibrateHeightPoint(int16_t h_01mm);

#endif /* __LEVITATION_H */
