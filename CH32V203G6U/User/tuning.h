/********************************** (C) COPYRIGHT *******************************
 * File Name          : tuning.h
 * Description        : 在线调参通道 —— 与裁判系统复用同一路 USART1
 *
 * 帧格式 (与裁判协议用不同的帧头, 两者可以共存在一条线上):
 *      下行 (上位机 -> 装置):  0xA5 | LEN | CMD  | payload... | CKSUM
 *      上行 (装置 -> 上位机):  0x5B | LEN | TYPE | payload... | CKSUM
 *  LEN   = CMD/TYPE 加 payload 的总字节数 (不含帧头、LEN 本身、校验和)
 *  CKSUM = 前面所有字节累加和的低 8 位
 *
 * 遥测默认关闭, 必须由上位机显式打开。比赛时不发任何多余字节, 裁判系统的
 * 解析器不会看到 0x5B 开头的帧。
 *******************************************************************************/
#ifndef __TUNING_H
#define __TUNING_H

#include "board.h"

#define TUNE_DOWN_HEAD      0xA5
#define TUNE_UP_HEAD        0x5B

/* 下行命令 */
#define TUNE_CMD_PARAM_SET  0x01    /* id(1) + float(4, 小端) */
#define TUNE_CMD_PARAM_GET  0x02    /* id(1) */
#define TUNE_CMD_PARAM_LIST 0x03    /* 无参, 回一串 PARAM_INFO */
#define TUNE_CMD_STREAM     0x04    /* div(1): 0=关, 否则每 div 个控制拍发一帧 */
#define TUNE_CMD_ACTION     0x05    /* code(1) + arg(2, 小端 int16) */
#define TUNE_CMD_COIL_TEST  0x06    /* ch(1) + cmd(2, 小端 int16), 绕过分方向增益 */
#define TUNE_CMD_GET_CAL    0x07    /* 无参, 每路回一帧 CALMAG */

/* ACTION 的 code */
#define TUNE_ACT_RECALIB    0x01    /* 重做零点 + 串扰 + 分方向增益标定 */
#define TUNE_ACT_CAL_HEIGHT 0x02    /* arg = 当前已知高度(0.1mm), 反算 KZ */
#define TUNE_ACT_CLEAR_FAULT 0x03   /* 清故障, 重新标定 */
#define TUNE_ACT_SET_HEIGHT 0x04    /* arg = 目标高度(mm) */
#define TUNE_ACT_CAL_CT     0x05    /* 只重测串扰(带矿石), 保留零点 */
#define TUNE_ACT_CAL_TILT   0x06    /* 矿石摆正中时标传感器倾斜 */
#define TUNE_ACT_ZERO_HERE  0x07    /* 把当前读数抓成零点 (写入 trim_x/trim_y) */
#define TUNE_ACT_BAL_COILS  0x08    /* 按标定读数把四路强度拉齐 (写 coil_gain[]) */

/* 上行类型 */
#define TUNE_UP_TELEM       0x01
#define TUNE_UP_PARAM       0x02    /* id(1) + float(4) */
#define TUNE_UP_PARAM_INFO  0x03    /* id(1) + 名字(不定长, 无结尾 0) */
#define TUNE_UP_ACK         0x04    /* cmd(1) + result(1), result 0=ok */
#define TUNE_UP_CALMAG      0x05    /* ch(1)+正向(4)+反向(4)+饱和标志(1) */

/* ---- 可调参数 ----
 * 全部是 float 且顺序排列, 参数 ID 就是它在结构体里的下标。上位机靠
 * PARAM_LIST 拿到 ID 与名字的对应关系, 不需要两边硬编码同一张表。
 * 增删字段时必须同步改 param_names[] 和 TUNE_PARAM_COUNT。 */
typedef struct {
    float kp_z, ki_z, kd_z, ilim_z;     /* 0..3   高度环 */
    float kp_xy, kd_xy;                 /* 4,5    横向环 */
    float dlpf_z, dlpf_xy;              /* 6,7    微分低通 */
    float slew;                         /* 8      每拍输出变化上限 */
    float kff_z, h0;                    /* 9,10   高度前馈与被动平衡点 */
    float kz;                           /* 11     高度换算常数 */
    float lat_sign;                     /* 12     横向反馈总极性 */
    float coil_sign[COIL_NUM];          /* 13..16 每路电流极性 */
    float ct_lag;                       /* 17     串扰补偿的一阶滞后系数 */
    /* 传感器姿态校正 (飞线焊接不可能完全水平, 必须标)
     *   tilt_x/tilt_y: Z 轴漏进 X/Y 的比例。倾斜 5 度就会把 1754 counts 的 Bz
     *     漏出 153 counts 到 Bx, 按 80 counts/mm 算是 1.9mm 的假横移, 而且随
     *     高度变化, 零点标定吃不掉(标零点时矿石不在场)。
     *   yaw_cos/yaw_sin: 面内旋转。存 cos/sin 而不是角度, 省掉固件里的三角函数。
     *     由串扰标定自动算出(四路线圈方位已知), 不需要额外夹具。 */
    float tilt_x, tilt_y;               /* 18,19 */
    float yaw_cos, yaw_sin;             /* 20,21 */
    /* 移植自参考工程 magnetic-levitation, 详见 board.h 的说明
     *   xy_lpf:   横向读数在进入 P/D 之前的一阶低通, 1.0 = 不滤
     *   trim_k:    设定点自整定的积分增益, 被积量是线圈出力, 0 = 关闭
     *   trim_lim:  trim 的钳位
     *   trim_gate: 积分门限, |bx-trim| 超过它就停积分。必须明显小于 trim_lim,
     *              否则门限永远不会先于钳位动作(推导见 board.h) */
    float xy_lpf;                       /* 22 */
    float trim_k, trim_lim, trim_gate;  /* 23,24,25 */
    /* 零点偏置本身。放进参数表(而不是留成 levitation.c 的内部状态)是为了能【手动
     * 直接写】—— 自整定积分要求矿石能自由移动, 夹具场合或者快环还没调出来的时候
     * 它根本不可能收敛, 那些时候只能靠手动微调。两种方式共用同一个存储:
     * trim_k = 0 时这就是纯手动零点, trim_k != 0 时由积分器往里写。
     * 【不随 Restart 清零】—— 它是参数, 行为要和 kz 之类一致, 要清就显式写 0。*/
    float trim_x, trim_y;               /* 26,27 */
    /* ---- 增益修正 ----
     * 两件不同的事, 不要混:
     *
     *  gain_y: 【轴】比值, 只乘在 Y 通道上(X 恒为 1, 只需要相对值)。P 和 D 一起
     *      乘, 所以 kd/kp 比值不受影响 —— 这是做成一个乘子而不是拆成 kp_x/kp_y
     *      的理由: 拆开就得在两个轴上各维护一次阻尼比, 多一倍出错机会。
     *
     *  coil_gain[]: 【每路】强度。注意它【不能】造成轴不对称 ——
     *      X 有效增益 = Σ g_i·mix_x[i]² = Σ g_i,  Y 同理 = Σ g_i, 两者恒等。
     *      它真正修的是【交叉耦合】: 四路不等时 Σ g_i·mix_x[i]·mix_y[i] != 0,
     *      单路弱 δ 就产生 δ 的 X-Y 耦合, 环路推的方向与误差方向差一个角度,
     *      现象是"绕着转"或"朝某个固定方向飞"而不是简单的一轴偏软。
     *      乘在混合【之后】的整个 f 上(含 u_z): 线圈弱就该整体驱动得更狠。
     *      副作用是 Σ g_i·mix_x[i] != 0 会让横向纠偏漏出竖直分量 —— 而这正是
     *      把四路拉齐要解决的问题本身, 拉齐之后零和性质自动恢复。 */
    float gain_y;                       /* 28 */
    float coil_gain[COIL_NUM];          /* 29..32 */
} tune_t;

#define TUNE_PARAM_COUNT    33

extern tune_t g_tune;

void    Tuning_Init(void);
/* 收到一帧完整的调试下行 (不含帧头/LEN/校验和) */
void    Tuning_HandleFrame(const uint8_t *data, uint8_t len);
/* 由主循环按 CONTROL_HZ 调用: 发遥测 + 看管开环测试超时 */
void    Tuning_Task(void);
/* 开环单路测试是否生效 —— 生效期间控制器不介入 */
uint8_t Tuning_TestActive(void);

#endif /* __TUNING_H */
