/********************************** (C) COPYRIGHT *******************************
 * File Name          : tmag5273.h
 * Description        : TMAG5273B2QDBVR 三维霍尔传感器驱动 (I2C1, PB6/PB7)
 *                      改自 maimai/code/button 参考工程, 单传感器 + 温度读取
 *******************************************************************************/
#ifndef __TMAG5273_H
#define __TMAG5273_H

#include "board.h"

/* --- 寄存器 --- */
#define TMAG_REG_DEVICE_CONFIG_1        0x00
#define TMAG_REG_DEVICE_CONFIG_2        0x01
#define TMAG_REG_SENSOR_CONFIG_1        0x02
#define TMAG_REG_SENSOR_CONFIG_2        0x03
#define TMAG_REG_T_CONFIG               0x07
#define TMAG_REG_INT_CONFIG_1           0x08
#define TMAG_REG_DEVICE_ID              0x0D
#define TMAG_REG_MANUFACTURER_ID_LSB    0x0E
#define TMAG_REG_MANUFACTURER_ID_MSB    0x0F
#define TMAG_REG_T_MSB_RESULT           0x10
#define TMAG_REG_X_MSB_RESULT           0x12
#define TMAG_REG_CONV_STATUS            0x18
#define TMAG_REG_DEVICE_STATUS          0x1C

/* DEVICE_CONFIG_1: 片上平均次数。
 *
 * 【选这个值的依据是实测噪声, 不是"延迟越小越好"】
 * 实测本底 sigma = 11 counts (110uT), 而 1mm 横移只有 23 counts —— 位置噪声
 * 0.48mm。比例项无所谓, 但微分项要命: 白噪声下相邻两拍之差的 sigma 是
 * sqrt(2)*11 = 15.6, 除以 1ms 就是 15600 counts/s 的微分原始值, 过完 alpha=0.4
 * 的低通再乘 kd=0.064, 纯噪声就贡献 ±500 的线圈指令 —— 限幅才 ±850。
 * 于是 kd 加不上去, 而横向轴的阻尼只有 D 项一个来源, 加不上去就必发散。
 *
 * 片上平均对微分是【双重】收益, 因为本工程的微分是按真实采样间隔算的
 * (levitation.c 的 sample_dt): 4x 平均把每点噪声降到 1/2, 同时把差分间隔
 * 拉长到 4 倍, 微分噪声总共降到 1/8。
 * 代价只有约 20 度的相位滞后(@10Hz), 对一个 5Hz 带宽的环路可以接受。
 * 更高的档位收益递减而相位继续恶化, 不要再往上加。 */
#define TMAG_CONV_AVG_1                 (0 << 2)
#define TMAG_CONV_AVG_2                 (1 << 2)
#define TMAG_CONV_AVG_4                 (2 << 2)
#define TMAG_CONV_AVG_8                 (3 << 2)
#define TMAG_CONV_AVG_16                (4 << 2)
#define TMAG_CONV_AVG_32                (5 << 2)

/* DEVICE_CONFIG_2 */
#define TMAG_LP_LOWNOISE                (1 << 4)
#define TMAG_OPMODE_STANDBY             (0 << 0)
#define TMAG_OPMODE_CONTINUOUS          (2 << 0)

/* SENSOR_CONFIG_1: 使能 XYZ 三轴 */
#define TMAG_MAG_CH_EN_XYZ              (0x7 << 4)

/* SENSOR_CONFIG_2: bit1=X/Y 量程, bit0=Z 量程, 置 1 = 高量程。两轴互相独立。 */
#define TMAG_RANGE_XY_HIGH              (1 << 1)
#define TMAG_RANGE_Z_HIGH               (1 << 0)

/* T_CONFIG: bit0 = 温度通道使能 */
#define TMAG_T_CH_EN                    0x01

/* 版本 2 (B2) 的量程, 单位 mT */
#define TMAG_RANGE_LOW_MT               133
#define TMAG_RANGE_HIGH_MT              266

/* 厂商 ID */
#define TMAG_MANUF_ID_LSB               0x49
#define TMAG_MANUF_ID_MSB               0x54

typedef struct {
    int16_t  bx;            /* 磁场, 单位 0.01mT */
    int16_t  by;
    int16_t  bz;
    int16_t  temp_c;        /* 芯片温度, 单位 ℃ */
    uint16_t range_xy_mT;   /* X/Y 当前量程, 两轴可以不同 */
    uint16_t range_z_mT;    /* Z 当前量程 */
    uint8_t  addr;
} TMAG_t;

void    TMAG_I2C_Init(void);
uint8_t TMAG_Init(TMAG_t *s);           /* 0=成功 */
uint8_t TMAG_ReadXYZ(TMAG_t *s);        /* 0=成功, 失败时自动恢复总线 */
uint8_t TMAG_ReadTemp(TMAG_t *s);       /* 0=成功 */

#endif /* __TMAG5273_H */
