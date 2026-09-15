/********************************** (C) COPYRIGHT *******************************
 * File Name          : coil.h
 * Description        : 四路 DRV8870 线圈驱动
 *******************************************************************************/
#ifndef __COIL_H
#define __COIL_H

#include "board.h"

/* 初始化 TIM2/TIM3 PWM 与四个方向脚, 上电后所有线圈处于滑行(断电)状态 */
void Coil_Init(void);

/* 设置单路输出。cmd 范围 -COIL_CMD_MAX..+COIL_CMD_MAX,
 * 正值 = IN1 侧为高 (DRV8870 正转), 内部按 COIL_SIGN[] 归一化并做限幅。 */
void Coil_Set(uint8_t ch, int16_t cmd);

/* 一次写入四路 */
void Coil_SetAll(const int16_t *cmd);

/* 绕过分方向增益校正, 直接按原始指令驱动。
 * 开环硬件测试必须走这条路 —— 用标定出来的修正系数去测裸硬件是自相矛盾的,
 * 而且当某一侧是死区时那个系数本身就不可信。 */
void Coil_SetRaw(uint8_t ch, int16_t cmd);

/* 立即切断四路输出 (滑行, 输出 Hi-Z) —— 故障与急停路径 */
void Coil_DisableAll(void);

/* 读回当前实际生效的指令值 (校正前的请求值), 供 LED 显示与串扰补偿使用 */
const int16_t *Coil_GetOutputs(void);

/* 安装分方向增益校正。单 PWM 驱动下正反向的电流传递函数不同(见 coil.c),
 * 标定出两个方向的实际强度后调用本函数把强的一侧压到与弱的一侧齐平,
 * 使 Coil_Set() 的输入-输出在过零点两侧连续。
 * pos/neg 取值范围 (0,1], 其中至少一个应为 1.0。 */
void Coil_SetDirGain(uint8_t ch, float pos, float neg);

/* 设置单路电流极性 (+1 / -1)。上电由 COIL_SIGN_INIT 装入, 也可以由上位机在线
 * 改写 —— 调试时用来确认「正指令 = 更大升力」而不必重新编译。 */
void Coil_SetPolarity(uint8_t ch, int8_t sign);

#endif /* __COIL_H */
