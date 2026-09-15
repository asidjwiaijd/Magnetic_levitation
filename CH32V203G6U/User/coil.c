/********************************** (C) COPYRIGHT *******************************
 * File Name          : coil.c
 * Description        : 四路 DRV8870 线圈驱动
 *
 * 驱动方式由 board.h 的 BOARD_REWORK_DUAL_PWM 决定:
 *
 *   =1 (飞线改版后): 四路全双 PWM。每路的 IN1/IN2 都在同一个定时器上, 两个
 *      方向都走慢衰减, 对占空比线性, 没有死区。
 *
 *   =0 (原始板): C、D 双 PWM; A、B 单 PWM —— IN1_A=PA4 与 IN2_B=PA5 没有任何
 *      定时器通道(所有重映射组合都不含这两个脚), 只能当静态方向电平, 于是必有
 *      一个方向走快衰减。快衰减小占空比下电流断续, **实测比另一侧弱 31~403 倍**,
 *      也就是说 A、B 实质上是单向执行器, 可用方向由硬件固定为「方向脚为高」
 *      那一侧。这种状态下改 COIL_SIGN 修不了极性(只会切到死掉的那一侧),
 *      只能物理对调该路 H 排针的两根线。
 *
 * DRV8870 真值表:
 *   IN1 IN2 -> OUT
 *    0   0  -> 滑行 (Hi-Z)
 *    1   0  -> 正转
 *    0   1  -> 反转
 *    1   1  -> 刹车 (两端接地)
 *
 * 双 PWM (C/D): 一侧常高, 另一侧给 (1-幅值) 占空比 —— 在「刹车 / 驱动」之间
 *   切换, 两个方向都是慢衰减, 平均电压 = 幅值 x Vm, 线性。
 *
 * 单 PWM (A/B): 方向脚只有静态电平, 于是
 *   D=0, 占空比 m   -> 「驱动 / 滑行」快衰减。滑行时输出 Hi-Z, 电流经体二极管
 *                      回灌电源; 小占空比下电流断续(DCM), 平均电流约正比于 m^2。
 *   D=1, 占空比 1-m -> 「刹车 / 驱动」慢衰减, 线性。
 * 环形永磁体供升力后工作点就在零附近, 死区正好压在工作点上。
 *******************************************************************************/
#include "coil.h"

/* 每路的驱动资源。
 *   ch_dir != 0 -> 双 PWM: 两个输入脚都在同一个定时器上, 可以两个方向都慢衰减
 *   ch_dir == 0 -> 单 PWM: 另一个脚没有定时器通道, 只能当 GPIO 方向脚 */
typedef struct {
    TIM_TypeDef  *tim;
    uint8_t       ch_pwm;       /* eff>=0 时被驱动的那一侧 */
    uint8_t       ch_dir;       /* 双 PWM 时是另一侧的通道; 0 = 单 PWM */
    GPIO_TypeDef *dir_port;     /* 仅单 PWM 有效 */
    uint16_t      dir_pin;
} CoilMap_t;

static const CoilMap_t coil_map[COIL_NUM] = {
#if BOARD_REWORK_DUAL_PWM
    /* A: IN1_A=PA0 TIM2_CH1(飞线), IN2_A=PA3 TIM2_CH4 */
    { TIM2, 1, 4, 0, 0 },
    /* B: IN1_B=PA6 TIM3_CH1, IN2_B=PB1 TIM3_CH4(飞线) */
    { TIM3, 1, 4, 0, 0 },
#else
    /* A: 单 PWM。PWM=PA3 TIM2_CH4 (IN2_A), DIR=PA4 (IN1_A, 无定时器) */
    { TIM2, 4, 0, GPIOA, GPIO_Pin_4 },
    /* B: 单 PWM。PWM=PA6 TIM3_CH1 (IN1_B), DIR=PA5 (IN2_B, 无定时器) */
    { TIM3, 1, 0, GPIOA, GPIO_Pin_5 },
#endif
    /* C: 双 PWM。IN1_C=PA2 TIM2_CH3, IN2_C=PA1 TIM2_CH2 —— 同在 TIM2 */
    { TIM2, 3, 2, 0, 0 },
    /* D: 双 PWM。IN1_D=PB0 TIM3_CH3, IN2_D=PA7 TIM3_CH2 —— 同在 TIM3 */
    { TIM3, 3, 2, 0, 0 },
};

/* 上电默认值来自 COIL_SIGN_INIT, 但允许在线改写(见 Coil_SetPolarity) */
static int8_t coil_sign[COIL_NUM] = COIL_SIGN_INIT;
static int16_t coil_out[COIL_NUM];

/* 分方向增益校正, 标定前为 1.0 (直通) */
static float dir_gain_pos[COIL_NUM] = { 1.0f, 1.0f, 1.0f, 1.0f };
static float dir_gain_neg[COIL_NUM] = { 1.0f, 1.0f, 1.0f, 1.0f };

/* 均衡系数的下限, 低于它就放弃均衡。理由见 Coil_SetDirGain。 */
#define DIR_GAIN_FLOOR  0.5f

static void set_ccr(TIM_TypeDef *tim, uint8_t ch, uint16_t ccr)
{
    switch(ch)
    {
        case 1: TIM_SetCompare1(tim, ccr); break;
        case 2: TIM_SetCompare2(tim, ccr); break;
        case 3: TIM_SetCompare3(tim, ccr); break;
        default: TIM_SetCompare4(tim, ccr); break;
    }
}

static void Coil_TimerInit(TIM_TypeDef *tim)
{
    TIM_TimeBaseInitTypeDef tb = {0};
    TIM_OCInitTypeDef oc = {0};

    tb.TIM_Period = COIL_PWM_ARR;
    tb.TIM_Prescaler = 0;
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    tb.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(tim, &tb);

    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    oc.TIM_Pulse = 0;

    /* 四个通道一次配全。TIM2 实际用 CH2(IN2_C)/CH3(IN1_C)/CH4(IN2_A),
     * TIM3 用 CH1(IN1_B)/CH2(IN2_D)/CH3(IN1_D); 多配的通道没接出引脚, 不影响。*/
    TIM_OC1Init(tim, &oc);
    TIM_OC2Init(tim, &oc);
    TIM_OC3Init(tim, &oc);
    TIM_OC4Init(tim, &oc);
    TIM_OC1PreloadConfig(tim, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(tim, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(tim, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(tim, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(tim, ENABLE);
    TIM_Cmd(tim, ENABLE);
}

void Coil_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    uint8_t i;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3, ENABLE);

    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;

#if BOARD_REWORK_DUAL_PWM
    /* 八个输入脚全是定时器输出。PA4/PA5 的走线已被切断, 不再配置 ——
     * 保持复位后的浮空输入, 让那段死走线彻底没人驱动。 */
    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 |
                    GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOA, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;    /* PB0=IN1_D, PB1=IN2_B(飞线) */
    GPIO_Init(GPIOB, &gpio);
#else
    /* A/B 的方向脚(PA4/PA5)先置低再配置为输出, 避免上电瞬间出现 (1,1) 刹车 */
    GPIO_ResetBits(GPIOA, GPIO_Pin_4 | GPIO_Pin_5);
    gpio.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &gpio);

    /* 定时器输出脚。PA1(TIM2_CH2)/PA7(TIM3_CH2) 是 C/D 的第二路 PWM。 */
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOB, &gpio);
#endif

    Coil_TimerInit(TIM2);
    Coil_TimerInit(TIM3);

    for(i = 0; i < COIL_NUM; i++) coil_out[i] = 0;
    Coil_DisableAll();
}

static void coil_drive(uint8_t ch, int16_t cmd)
{
    const CoilMap_t *m = &coil_map[ch];
    int32_t eff;
    uint16_t full = COIL_PWM_ARR + 1;
    uint16_t mag;

    /* 归一化: eff >= 0 表示电流走 ch_pwm 那一侧 */
    eff = (int32_t)cmd * coil_sign[ch];

    if(m->ch_dir)
    {
        /* ---- 双 PWM: 两个方向都是「驱动 / 刹车」, 对占空比线性, 没有死区 ----
         * 一侧常高, 另一侧给 (1 - 幅值) 的占空比:
         *   该侧为高 -> (1,1) 刹车;  该侧为低 -> 驱动。
         * 幅值 0 时两侧都常高 = 刹车, 电流为零。 */
        mag = (uint16_t)((eff >= 0 ? eff : -eff) * full / COIL_CMD_MAX);
        if(eff >= 0)
        {
            set_ccr(m->tim, m->ch_pwm, full);
            set_ccr(m->tim, m->ch_dir, (uint16_t)(full - mag));
        }
        else
        {
            set_ccr(m->tim, m->ch_dir, full);
            set_ccr(m->tim, m->ch_pwm, (uint16_t)(full - mag));
        }
        return;
    }

    /* ---- 单 PWM: 方向脚只能给静态电平, 其中一个方向必然是快衰减 ---- */
    if(eff >= 0)
    {
        /* 驱动 / 滑行 —— 快衰减。小占空比时电流断续, 实测比另一侧弱几十倍。*/
        GPIO_ResetBits(m->dir_port, m->dir_pin);
        set_ccr(m->tim, m->ch_pwm, (uint16_t)(eff * full / COIL_CMD_MAX));
    }
    else
    {
        /* 刹车 / 驱动 —— 慢衰减, 线性。这是这一路唯一真正可用的方向。 */
        GPIO_SetBits(m->dir_port, m->dir_pin);
        set_ccr(m->tim, m->ch_pwm, (uint16_t)(full - (-eff) * full / COIL_CMD_MAX));
    }
}

static int16_t coil_clamp(int16_t cmd)
{
    if(cmd > COIL_CMD_LIMIT)  return COIL_CMD_LIMIT;
    if(cmd < -COIL_CMD_LIMIT) return -COIL_CMD_LIMIT;
    return cmd;
}

void Coil_Set(uint8_t ch, int16_t cmd)
{
    int16_t applied;

    if(ch >= COIL_NUM) return;
    cmd = coil_clamp(cmd);

    /* coil_out[] 存的是校正前的请求值: 串扰补偿的系数也是按请求值标定的,
     * 两边必须用同一个量纲。 */
    coil_out[ch] = cmd;

    /* 分方向增益校正, 让过零点两侧的输入-输出斜率一致 */
    if(cmd >= 0) applied = (int16_t)((float)cmd * dir_gain_pos[ch]);
    else         applied = (int16_t)((float)cmd * dir_gain_neg[ch]);

    coil_drive(ch, applied);
}

void Coil_SetRaw(uint8_t ch, int16_t cmd)
{
    if(ch >= COIL_NUM) return;
    cmd = coil_clamp(cmd);
    coil_out[ch] = cmd;
    coil_drive(ch, cmd);
}

void Coil_SetAll(const int16_t *cmd)
{
    uint8_t i;
    for(i = 0; i < COIL_NUM; i++) Coil_Set(i, cmd[i]);
}

void Coil_DisableAll(void)
{
    uint8_t i;
    for(i = 0; i < COIL_NUM; i++)
    {
        const CoilMap_t *m = &coil_map[i];
        /* 两个输入脚都拉低 => (0,0) => 输出 Hi-Z 滑行 */
        set_ccr(m->tim, m->ch_pwm, 0);
        if(m->ch_dir) set_ccr(m->tim, m->ch_dir, 0);
        else          GPIO_ResetBits(m->dir_port, m->dir_pin);
        coil_out[i] = 0;
    }
}

const int16_t *Coil_GetOutputs(void)
{
    return coil_out;
}

void Coil_SetPolarity(uint8_t ch, int8_t sign)
{
    if(ch >= COIL_NUM) return;

    /* 极性变了等于这一路的电流方向整个翻过来, 当前占空比立刻失去意义,
     * 先断电再接受新极性, 避免切换瞬间反向满电流。 */
    if(sign != coil_sign[ch])
    {
        Coil_Set(ch, 0);
        coil_sign[ch] = (sign >= 0) ? 1 : -1;
    }
}

void Coil_SetDirGain(uint8_t ch, float pos, float neg)
{
    if(ch >= COIL_NUM) return;

    /* 防呆: 增益只允许压低不允许放大, 标定异常时退回直通而不是把输出放飞。
     *
     * 下限 DIR_GAIN_FLOOR 是必须的。单 PWM 下快衰减一侧在零点附近是死区,
     * 实测两侧电流可以差 20 倍; 若照比例把好的一侧压到 0.05, 结果是两个方向
     * 一起失去驱动能力 —— 满量程的不对称远好过对称的零。差得过头就干脆不均衡,
     * 把不对称如实留给上层(和标定读数)去暴露。 */
    if(pos < DIR_GAIN_FLOOR || neg < DIR_GAIN_FLOOR) { pos = 1.0f; neg = 1.0f; }
    if(pos <= 0.0f || pos > 1.0f) pos = 1.0f;
    if(neg <= 0.0f || neg > 1.0f) neg = 1.0f;

    dir_gain_pos[ch] = pos;
    dir_gain_neg[ch] = neg;
}
