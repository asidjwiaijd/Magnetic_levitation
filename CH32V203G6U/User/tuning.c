/********************************** (C) COPYRIGHT *******************************
 * File Name          : tuning.c
 * Description        : 在线调参通道
 *
 * 安全约束:
 *   开环单路测试 (TUNE_CMD_COIL_TEST) 带 2s 看门狗。上位机掉线或者串口被拔掉时,
 *   线圈必须自己断电 —— 否则一路全桥会一直通着几百 mA 直到复位, 而这块板的
 *   温度上限只有 70℃。上位机想维持测试就得持续重发命令。
 *******************************************************************************/
#include "tuning.h"
#include "referee.h"
#include "levitation.h"
#include "coil.h"

#define TEST_TIMEOUT_TICKS  (2 * CONTROL_HZ)

tune_t g_tune = {
    KP_Z_DEF, KI_Z_DEF, KD_Z_DEF, I_LIMIT_Z_DEF,
    KP_XY_DEF, KD_XY_DEF,
    D_LPF_Z_DEF, D_LPF_XY_DEF,
    SLEW_LIMIT_DEF,
    KFF_Z, (float)H0_PASSIVE_01MM,
    KZ_DEF,
    (float)LATERAL_SIGN,
    { 0, 0, 0, 0 },                     /* 由 Tuning_Init 从 COIL_SIGN_INIT 填 */
    CT_LAG_DEF,
    TILT_X_DEF, TILT_Y_DEF,             /* 传感器倾角, 实测 */
    YAW_COS_DEF, YAW_SIN_DEF,           /* 面内旋转, 实测 */
    XY_LPF_DEF,
    TRIM_K_DEF, TRIM_LIM_DEF, TRIM_GATE_DEF,
    TRIM_X_DEF, TRIM_Y_DEF,             /* 零点偏置, 实测 */
    1.0f,                               /* gain_y: 轴比值 */
    { 1.0f, 1.0f, 1.0f, 1.0f }          /* coil_gain: 每路强度 */
};

/* 结构体必须是纯 float 连续排列, 参数 ID 才能当下标用。字段数对不上就编译不过。*/
typedef char tune_layout_check[(sizeof(tune_t) / sizeof(float) == TUNE_PARAM_COUNT) ? 1 : -1];

static const char * const param_names[TUNE_PARAM_COUNT] = {
    "kp_z", "ki_z", "kd_z", "ilim_z",
    "kp_xy", "kd_xy",
    "dlpf_z", "dlpf_xy",
    "slew",
    "kff_z", "h0",
    "kz",
    "lat_sign",
    "sign_a", "sign_b", "sign_c", "sign_d",
    "ct_lag",
    "tilt_x", "tilt_y", "yaw_cos", "yaw_sin",
    "xy_lpf", "trim_k", "trim_lim", "trim_gate",
    "trim_x", "trim_y",
    "gain_y",
    "gain_a", "gain_b", "gain_c", "gain_d"
};

static uint8_t  stream_div;         /* 0 = 关闭 */
static uint8_t  stream_cnt;
static uint16_t test_ticks;         /* 开环测试剩余拍数 */
static uint8_t  test_ch;
static int16_t  test_cmd;

/*********************************************************************
 * 发送
 *********************************************************************/
static void send_frame(uint8_t type, const uint8_t *payload, uint8_t n)
{
    uint8_t buf[40];
    uint8_t i, sum = 0;

    if((uint8_t)(n + 4) > sizeof(buf)) return;

    buf[0] = TUNE_UP_HEAD;
    buf[1] = (uint8_t)(n + 1);      /* TYPE + payload */
    buf[2] = type;
    for(i = 0; i < n; i++) buf[3 + i] = payload[i];
    for(i = 0; i < (uint8_t)(n + 3); i++) sum = (uint8_t)(sum + buf[i]);
    buf[3 + n] = sum;

    Referee_Send(buf, (uint8_t)(n + 4));
}

static void put_i16(uint8_t *p, int16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
}

static int16_t get_i16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void send_ack(uint8_t cmd, uint8_t result)
{
    uint8_t p[2];
    p[0] = cmd;
    p[1] = result;
    send_frame(TUNE_UP_ACK, p, 2);
}

static void send_param(uint8_t id)
{
    uint8_t p[5];
    const uint8_t *f;

    if(id >= TUNE_PARAM_COUNT) return;
    f = (const uint8_t *)((const float *)&g_tune + id);
    p[0] = id;
    p[1] = f[0]; p[2] = f[1]; p[3] = f[2]; p[4] = f[3];
    send_frame(TUNE_UP_PARAM, p, 5);
}

/*********************************************************************
 * 参数写入。有些参数不只是存个数, 还要推给别的模块。
 *********************************************************************/
static void param_apply(uint8_t id)
{
    uint8_t ch;

    /* 极性只取符号, 上位机传 ±1 之外的值也不会把输出放大 */
    for(ch = 0; ch < COIL_NUM; ch++)
    {
        if(id == (uint8_t)(13 + ch))
        {
            Coil_SetPolarity(ch, g_tune.coil_sign[ch] >= 0.0f ? 1 : -1);

            /* 极性一变, 已标定的串扰系数就全错了 —— 同一个指令值现在产生的是
             * 相反方向的电流, 补偿会按反号叠加, 比不补偿还糟(误差翻倍而不是
             * 抵消)。所以强制重标, 不给「改了极性忘了重标」留空间。 */
            Levitation_Restart();
            return;
        }
    }

    /* kz 改了之后目标磁场要跟着重算, 否则设定高度还按旧标度 */
    if(id == 11) Levitation_RefreshTarget();
}

void Tuning_Init(void)
{
    static const int8_t sign_init[COIL_NUM] = COIL_SIGN_INIT;
    uint8_t i;

    for(i = 0; i < COIL_NUM; i++)
    {
        g_tune.coil_sign[i] = (float)sign_init[i];
        Coil_SetPolarity(i, sign_init[i]);
    }

    stream_div = 0;
    stream_cnt = 0;
    test_ticks = 0;
    test_cmd = 0;
    test_ch = 0;
}

/*********************************************************************
 * 下行处理
 *********************************************************************/
void Tuning_HandleFrame(const uint8_t *data, uint8_t len)
{
    uint8_t cmd;

    if(len < 1) return;
    cmd = data[0];

    switch(cmd)
    {
    case TUNE_CMD_PARAM_SET:
        if(len >= 6 && data[1] < TUNE_PARAM_COUNT)
        {
            uint8_t *dst = (uint8_t *)((float *)&g_tune + data[1]);
            dst[0] = data[2]; dst[1] = data[3];
            dst[2] = data[4]; dst[3] = data[5];
            param_apply(data[1]);
            send_param(data[1]);        /* 回读, 让上位机确认真的写进去了 */
        }
        else send_ack(cmd, 1);
        break;

    case TUNE_CMD_PARAM_GET:
        if(len >= 2) send_param(data[1]);
        break;

    case TUNE_CMD_PARAM_LIST:
    {
        uint8_t i, n;
        uint8_t p[16];
        for(i = 0; i < TUNE_PARAM_COUNT; i++)
        {
            p[0] = i;
            for(n = 0; n < 14 && param_names[i][n]; n++) p[1 + n] = (uint8_t)param_names[i][n];
            send_frame(TUNE_UP_PARAM_INFO, p, (uint8_t)(n + 1));
        }
        break;
    }

    case TUNE_CMD_STREAM:
        if(len >= 2)
        {
            stream_div = data[1];
            stream_cnt = 0;
            send_ack(cmd, 0);
        }
        break;

    case TUNE_CMD_ACTION:
        if(len >= 4)
        {
            int16_t arg = get_i16(&data[2]);
            switch(data[1])
            {
            case TUNE_ACT_RECALIB:
            case TUNE_ACT_CLEAR_FAULT:
                Levitation_Restart();
                send_ack(cmd, 0);
                break;
            case TUNE_ACT_CAL_HEIGHT:
                Levitation_CalibrateHeightPoint(arg);
                send_param(11);         /* 把新的 kz 回读给上位机 */
                break;
            case TUNE_ACT_CAL_TILT:
                send_ack(cmd, Levitation_CalibrateTilt() ? 1 : 0);
                break;
            case TUNE_ACT_ZERO_HERE:
                /* 抓完把两个值回读给上位机, 否则参数表还显示旧值, 会让人
                 * 以为没生效 —— 而这个操作恰恰没有任何其它可见反馈。 */
                if(Levitation_ZeroHere() == 0)
                {
                    send_param(26);
                    send_param(27);
                }
                else send_ack(cmd, 1);
                break;
            case TUNE_ACT_BAL_COILS:
                if(Levitation_BalanceCoils() == 0)
                {
                    uint8_t k;
                    for(k = 0; k < COIL_NUM; k++) send_param((uint8_t)(29 + k));
                }
                else send_ack(cmd, 1);
                break;
            case TUNE_ACT_CAL_CT:
                Levitation_RecalCrosstalk();
                send_ack(cmd, 0);
                break;
            case TUNE_ACT_SET_HEIGHT:
                if(arg < 0) arg = 0;
                if(arg > 255) arg = 255;
                Levitation_SetTargetHeight((uint8_t)arg);
                send_ack(cmd, 0);
                break;
            default:
                send_ack(cmd, 1);
                break;
            }
        }
        break;

    case TUNE_CMD_COIL_TEST:
        if(len >= 4 && data[1] < COIL_NUM)
        {
            test_ch = data[1];
            test_cmd = get_i16(&data[2]);
            if(test_cmd == 0)
            {
                test_ticks = 0;
                Coil_DisableAll();
            }
            else
            {
                test_ticks = TEST_TIMEOUT_TICKS;
            }
            send_ack(cmd, 0);
        }
        else send_ack(cmd, 1);
        break;

    case TUNE_CMD_GET_CAL:
    {
        uint8_t i, sat, p[22];
        float mp, mn, cx, cy, cz;
        for(i = 0; i < COIL_NUM; i++)
        {
            Levitation_GetCalMag(i, &mp, &mn, &sat);
            Levitation_GetCrosstalk(i, &cx, &cy, &cz);
            p[0] = i;
            __builtin_memcpy(&p[1], &mp, 4);
            __builtin_memcpy(&p[5], &mn, 4);
            p[9] = sat;
            /* 分轴串扰: L1 范数看不出问题在哪个轴, 而只有 X/Y 那两个分量会被
             * 当成横向位移。上位机据此把串扰折算成等效毫米。 */
            __builtin_memcpy(&p[10], &cx, 4);
            __builtin_memcpy(&p[14], &cy, 4);
            __builtin_memcpy(&p[18], &cz, 4);
            send_frame(TUNE_UP_CALMAG, p, 22);
        }
        break;
    }

    default:
        send_ack(cmd, 1);
        break;
    }
}

uint8_t Tuning_TestActive(void)
{
    return test_ticks ? 1 : 0;
}

/*********************************************************************
 * 周期任务
 *********************************************************************/
void Tuning_Task(void)
{
    /* --- 开环测试看门狗 --- */
    if(test_ticks)
    {
        test_ticks--;
        if(test_ticks == 0)
        {
            /* 上位机没有续命, 断电。见文件头说明。 */
            Coil_DisableAll();
            test_cmd = 0;
        }
        else
        {
            uint8_t i;
            /* 必须用 Raw: 开环测的是裸硬件, 套上标定出来的分方向增益就成了
             * 自证循环 —— 而当某一侧本来就是死区时, 那个系数根本不可信。 */
            for(i = 0; i < COIL_NUM; i++)
                Coil_SetRaw(i, (i == test_ch) ? test_cmd : 0);
        }
    }

    /* --- 遥测 --- */
    if(stream_div == 0) return;
    if(++stream_cnt < stream_div) return;
    stream_cnt = 0;

    {
        uint8_t p[34];
        const int16_t *out = Coil_GetOutputs();
        int16_t bx, by, bz, rx, ry, rz, tx, ty;
        uint8_t i;

        Levitation_GetField(&bx, &by, &bz);
        Levitation_GetRawField(&rx, &ry, &rz);
        Levitation_GetTrim(&tx, &ty);

        p[0] = (uint8_t)Levitation_GetState();
        p[1] = (uint8_t)((Levitation_IsSettled() ? 0x01 : 0) |
                         (Tuning_TestActive()    ? 0x02 : 0));
        put_i16(&p[2],  bx);
        put_i16(&p[4],  by);
        put_i16(&p[6],  bz);
        put_i16(&p[8],  Levitation_GetHeight_01mm());
        put_i16(&p[10], Levitation_GetTargetHeight_01mm());
        for(i = 0; i < COIL_NUM; i++) put_i16(&p[12 + i * 2], out[i]);
        p[20] = (uint8_t)(int8_t)Levitation_GetTemp();
        /* 原始读数: 看有没有逼近满量程只能靠它, 补偿后的值恒在零附近 */
        put_i16(&p[21], rx);
        put_i16(&p[23], ry);
        put_i16(&p[25], rz);
        /* 下行丢帧计数: 命令没生效时唯一能区分「没发到」和「发到了没效果」的依据 */
        p[27] = Referee_GetRxDrops();
        /* 传感器实际更新率: 微分项是不是在对重复值求导, 只能靠它判断 */
        put_i16(&p[28], (int16_t)Levitation_GetSensorRate());
        /* 自整定偏置: 收敛后 bx 恒等于 trim, 不看它就分不清是矿石偏了还是
         * trim 把传感器偏置吃掉了 */
        put_i16(&p[30], tx);
        put_i16(&p[32], ty);

        send_frame(TUNE_UP_TELEM, p, 34);
    }
}
