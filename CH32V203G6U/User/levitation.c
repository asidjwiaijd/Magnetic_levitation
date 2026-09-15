/********************************** (C) COPYRIGHT *******************************
 * File Name          : levitation.c
 * Description        : 悬浮控制器
 *
 * 拓扑: 四个电磁铁外圈套一个环形永磁体。永磁体提供升力, 线圈只做修正。
 *
 * 【吸力式, 不是斥力式】矿石磁矩与环的场同向(能量极小), 被吸住而非被推开。
 * 实测依据见 CLAUDE.md, 三条独立证据: 平衡点落在环的轴向场峰下方; pitch/roll
 * 被动稳定(同向才是力矩极小点); LATERAL_SIGN 实测为 -1, 与按斥力推出的 +1 相反。
 *
 * 稳定性的方向 —— 这决定了两个环路的分工, 是整个文件的前提
 * ------------------------------------------------------------
 *   竖直: 被动稳定。平衡点在场峰【下方】: 下沉 -> dB/dz 变大 -> 吸力变大 -> 托回。
 *         => 慢环, 以前馈 + 积分为主, 不需要带宽。
 *         注意被动稳定窗口只有 8.5~41.5mm(磁中心, 环顶起算): 上界是场峰(越过
 *         就掉), 下界是梯度峰(低于它下沉反而吸力变小, 会一路砸下来)。
 *   倾斜: 被动稳定, 不需要控制。
 *   横向: 被动不稳定。场峰是鞍点。
 *         => 快环, 这是必须靠反馈撑住的那个轴。
 *   (Earnshaw 定理要求至少一个方向不稳定, 这里就是横向。
 *    k_x = -k_z/2 由 ∇²B = 0 保证, 与场源形状、吸力还是斥力都无关。)
 *
 * 测量链路
 * --------
 * 传感器在底座正中。矿石的偶极子场 |Bz| 随高度单调下降, 当作高度的代理量;
 * Bx/By 一阶正比于横向偏移。两者都要先扣掉两个干扰源:
 *   1) 环形永磁体的静态偏置 —— 开机标零点, 且空载时持续重新归零(见下)。
 *   2) 线圈自身的场 —— 传感器夹在四个线圈中间, 干扰与控制量成正比, 不补偿
 *      就是正反馈。开机逐路正反通电测斜率, 运行时按当前指令实时扣除。
 *
 * 为什么空载要持续归零
 * --------------------
 * 环形永磁体在传感器处的直流偏置可能有几十 mT, 而矿石信号只有 0.5~12mT。
 * NdFeB 温度系数约 -0.12%/℃, 底座允许升到 70℃ —— 开机标一次, 跑热了零点就
 * 漂掉了, 而漂移量在 5cm 处能占到信号的十几个百分点。所以只要判定矿石不在,
 * 就用 2s 时间常数持续把零点跟过去。
 *
 * 已知的结构性局限 (代码解决不了, 记在这里免得反复踩)
 * --------------------------------------------------
 *   单颗中心传感器给 3 个测量值, 而矿石有 5 个相关自由度(x,y,h + 两个倾角)。
 *   倾斜 θ 与横移 x 在读数上简并: 倾斜 θ ≡ 横移 -θ*h/3。h=3cm 时 5.7° 的倾斜
 *   就伪装成 1mm 的横移。横向又恰恰是唯一不稳定的轴, 所以这个简并压在关键
 *   路径上。根治要靠四颗传感器(TMAG5273 的 A/B/C/D 四个地址正好够挂一条 I2C),
 *   差分可以把倾角与平移分开, 还能顺带消掉线圈的共模串扰。
 *
 * 高度标定 (KZ 必须在实物上测一次, 下面的默认值只是数量级占位)
 * ------------------------------------------------------------
 *   1. 上电进入 LEV_IDLE 后, 用非磁性垫块把矿石垫到已知高度, 例如 30.0mm;
 *   2. 调用 Levitation_CalibrateHeightPoint(300);
 *   3. 函数按 KZ = |Bz| * (h + H_SENSOR_OFFSET)^3 反算并保存。
 *   本工程没有 Flash 存储层, KZ 掉电丢失。
 *******************************************************************************/
#include "levitation.h"
#include "coil.h"
#include "tmag5273.h"
#include "tuning.h"

/* 控制增益全部搬到 board.h 的 *_DEF 宏, 运行时值放在 tuning.h 的 g_tune,
 * 可由上位机在线改写。下面的代码一律读 g_tune, 不再引用宏。 */

/* 串扰标定用的测试指令幅度与每点稳定时间 */
#define CAL_CMD     400
#define CAL_SETTLE_TICKS    80      /* 80ms */
#define CAL_AVG_TICKS       40      /* 40ms 取平均 */

/* 输出持续顶在限幅上多久算失控。正常悬浮时线圈只做小幅修正, 四路同时长期
 * 满限幅只有两种可能: 环路发散, 或者根本没有矿石而在追一个零点误差造出来的
 * 幻影。两种都该立刻断电 —— 四路满电流撑不了多久就会顶到 70℃ 上限。 */
#define SAT_STALL_TICKS     (2 * CONTROL_HZ)

/* 到位判据 */
#define SETTLE_TOL_01MM     50      /* ±5.0mm, 留足余量给 ±0.5cm 的测量误差要求 */
#define SETTLE_TICKS        300     /* 连续 300ms 在容差内才算到位 */

static TMAG_t       sensor;
static LevState_t   state;

/* 零点。用 float 是因为空载重新归零的时间常数是 2000 拍, 整数会被截断成不动。 */
static float bx0, by0, bz0;

/* 串扰系数: 每 1000 指令引起的磁场变化, 单位 0.01mT。
 * 正反向分开存 —— 单 PWM 驱动下两个方向的电流传递函数不同(见 coil.c 文件头),
 * 用一个平均斜率会在两侧各留一半的残差, 而残差看起来跟横向位移一模一样。 */
static float ct_pos[3][COIL_NUM];   /* [0]=x [1]=y [2]=z */
static float ct_neg[3][COIL_NUM];

/* 标定时实测到的每路正/反向场强(L1 范数, 每 1000 指令)。
 * 这是判断「线圈/驱动是不是活的」最直接的证据: 两个方向都接近 0 就是硬件问题,
 * 只有一个方向接近 0 则是单 PWM 的快衰减死区。 */
static float cal_mag_p[COIL_NUM], cal_mag_n[COIL_NUM];

/* 线圈电流对指令的一阶滞后模型。
 *
 * 串扰系数是在直流下标的(每点稳 80ms 再平均), 但线圈是个 L/R 环节: 实测
 * L=4.1mH, R=4.2Ω -> tau = 0.98ms, 与 1ms 的控制周期同量级。指令跳变后第一拍
 * 电流只走到 64%, 而补偿若按稳态值立刻全扣掉, 就留下 36% 的残差 —— 残差在
 * 读数上跟横向位移长得一模一样, 于是被增益最高的那个环当成位移去追。
 *
 * 更糟的是残差与 u_x 同相, 等效于给横向环套了一圈正反馈:
 *      bx_读 = bx_真 / (1 - eps * k_c * kp_xy)
 * 分母归零就发散。按实测的串扰强度(每 1000 指令约 950, X 轴差分后 k_c≈0.6)
 * 与 eps=0.36, kp_xy 加到 4.6 就到极点 —— 这个上限跟被控对象无关, 纯粹是
 * 补偿模型欠了一个时间常数造出来的。
 *
 * 所以扣串扰之前先把指令过一遍同时间常数的低通, 让补偿量跟真实电流同步。 */
static float cmd_lag[COIL_NUM];

/* 把上一拍已经下发的指令朝电流推进一步。必须在 sense() 之前调用: 此刻
 * Coil_GetOutputs() 拿到的正是上一拍末尾下发的值, 而本拍采到的场就是它流了
 * 1ms 的结果, 两者配对正确。 */
static void update_cmd_lag(void)
{
    const int16_t *out = Coil_GetOutputs();
    float a = g_tune.ct_lag;
    uint8_t i;

    if(a > 1.0f)  a = 1.0f;
    if(a < 0.01f) a = 0.01f;    /* 0 会让滞后量永远停在原地, 等于完全不补偿 */

    for(i = 0; i < COIL_NUM; i++)
        cmd_lag[i] += a * ((float)out[i] - cmd_lag[i]);
}

static const int8_t mix_x[COIL_NUM] = COIL_MIX_X_INIT;
static const int8_t mix_y[COIL_NUM] = COIL_MIX_Y_INIT;

/* 目标 */
static int16_t target_h_01mm = H_DEFAULT_01MM;
static float   target_bz;
static float   ff_z;            /* 高度前馈, 随目标高度变化 */

/* PID 状态 */
static float i_z, d_z_filt, prev_bz;
static float d_x_filt, prev_bx;
static float d_y_filt, prev_by;
static float lat_norm_f = 1.0f;     /* 滤波后的横向增益归一化系数 */

/* 横向读数的输入低通。原来只滤微分不滤比例, 比例通道直接吃 LSB 抖动。
 * 参考工程是先滤读数再算误差, P 和 D 都用滤波值 —— 照搬。 */
static float bx_f, by_f;

/* 设定点自整定偏置, 见 board.h 的 TRIM_K_DEF。
 * 跨起浮保留(拿开矿石再放回不必重新收敛 3s), 只在显式 Restart/Init 时清零。 */
static float bx_trim, by_trim;

static int16_t out_cmd[COIL_NUM];

static uint8_t  fail_count;
static uint16_t settle_count;
static uint16_t sat_count;      /* 四路同时满限幅已持续的拍数 */
static uint8_t  cur_h_mm;
static int16_t  cur_h_01mm;
static uint32_t tick_count;

/* 补偿后的三轴磁场, 即控制器真正看到的量。只为遥测保存, 控制路径不读它。 */
static int16_t  last_bx, last_by, last_bz;
/* 原始读数(未扣零点与串扰)。判断有没有逼近满量程只能看这个。 */
static int16_t  raw_bx, raw_by, raw_bz;
/* 标定时的饱和标志, 每路 bit0=正向 bit1=反向 */
static uint8_t  cal_sat[COIL_NUM];

/* 传感器实际更新率 (每秒出现多少个新的 Z 读数)。
 *
 * 为什么要测: 控制环跑 1kHz, 但 TMAG5273 在连续转换下未必能跟上 —— 跟不上就会
 * 连续几拍读回完全相同的寄存器值。对比例项没影响, 但微分项会算出一串假的 0,
 * 中间夹一个大跳变, 等于把阻尼变成脉冲噪声。横向是唯一靠阻尼撑住的轴, 这件事
 * 必须有数, 不能靠数据手册猜。
 * 判据: 接近 CONTROL_HZ 就是跟得上; 明显偏低就要降控制频率或换采样策略。 */
static int16_t  prev_rx, prev_ry, prev_rz;
static uint16_t sens_new_cnt;       /* 本秒内出现的新读数个数 */
static uint16_t sens_rate_hz;       /* 上一秒的统计结果 */

/* 本拍传感器有没有出新值, 以及距上一个新值隔了多少拍。
 * 微分必须按【真实间隔】求导: 读数若隔 n 拍才更新一次, 而式子固定除以 1ms,
 * 输出就是 "0,0,0,0,n倍尖峰" —— 那不是阻尼, 是被放大了 n 倍的脉冲噪声,
 * 恰恰喂给唯一靠阻尼撑住的那个轴。按间隔求导之后, 无论传感器多快都正确。 */
static uint8_t  sample_new;         /* 本拍是否拿到新值 */
static uint16_t sample_gap;         /* 自上一个新值以来累计的拍数 */
static uint16_t sample_dt;          /* 本次新值与上一个新值之间的拍数 (>=1) */

/*********************************************************************
 * 立方根 —— 不拉 libm, 用经典的浮点指数近似作初值再 Newton 迭代。
 *********************************************************************/
static float cbrt_f(float x)
{
    union { float f; uint32_t i; } u;
    float y;
    uint8_t i;

    if(x <= 0.0f) return 0.0f;
    u.f = x;
    u.i = u.i / 3 + 709921077u;
    y = u.f;
    for(i = 0; i < 4; i++) y = (2.0f * y + x / (y * y)) / 3.0f;
    return y;
}

static float fabs_f(float x) { return x < 0.0f ? -x : x; }

/* 平方根 —— 只在标定里用一次, Newton 迭代足够 */
static float sqrt_f(float x)
{
    float y; uint8_t i;
    if(x <= 0.0f) return 0.0f;
    y = x > 1.0f ? x * 0.5f : 1.0f;
    for(i = 0; i < 12; i++) y = 0.5f * (y + x / y);
    return y;
}
static int16_t abs_i16(int16_t x) { return x < 0 ? (int16_t)(-x) : x; }

/* 当前读数是否逼近满量程。X/Y 与 Z 的量程可以不同, 阈值分开算。 */
static uint8_t sensor_saturated(void)
{
    int16_t sat_xy = (int16_t)(sensor.range_xy_mT * 95);    /* 95% 满量程 */
    int16_t sat_z  = (int16_t)(sensor.range_z_mT  * 95);
    return (abs_i16(sensor.bx) > sat_xy ||
            abs_i16(sensor.by) > sat_xy ||
            abs_i16(sensor.bz) > sat_z) ? 1 : 0;
}

/* |Bz| -> 高度(0.1mm) */
static int16_t bz_to_height(float bz_mag)
{
    float r;
    if(bz_mag < 1.0f) return H_MAX_01MM;
    r = cbrt_f(g_tune.kz / bz_mag) - (float)H_SENSOR_OFFSET;
    if(r < 0.0f) r = 0.0f;
    if(r > 1000.0f) r = 1000.0f;
    return (int16_t)r;
}

/* 高度(0.1mm) -> 目标 |Bz| */
static float height_to_bz(int16_t h_01mm)
{
    float r = (float)h_01mm + (float)H_SENSOR_OFFSET;
    return g_tune.kz / (r * r * r);
}

/* 设定点变更时重算目标磁场与前馈 */
static void apply_target(int16_t h_01mm)
{
    target_h_01mm = h_01mm;
    target_bz = height_to_bz(h_01mm);
    ff_z = g_tune.kff_z * ((float)h_01mm - g_tune.h0);
}

/*********************************************************************
 * 读一次传感器并扣除零点与线圈串扰
 *********************************************************************/
static uint8_t sense(float *bx, float *by, float *bz)
{
    float cx = 0.0f, cy = 0.0f, cz = 0.0f;
    uint8_t i;

    if(TMAG_ReadXYZ(&sensor))
    {
        if(fail_count < 255) fail_count++;
        return 1;
    }

    raw_bx = sensor.bx;
    raw_by = sensor.by;
    raw_bz = sensor.bz;

    /* 三个轴任何一个变了, 就说明发生过一次新转换。
     * 不能只看 Bz: Z 走高量程, 1 LSB = 8.12uT = 0.8 counts, 场安静时相邻两次
     * 真实转换本来就会读出完全相同的值, 只看 Z 会把「信号没动」误判成
     * 「传感器没出新值」。X/Y 走低量程(4.06uT/LSB), 噪声更容易跨过 1 LSB。
     * 即便如此, 这个数仍是「读数变化率」的下界估计 —— 完全静止时必然偏低,
     * 要在矿石有动作的时候读它。 */
    /* 间隔的累加与清零都放在这里, 不要放到控制分支里 —— IDLE / 故障 / 开环测试
     * 这些提前 return 的路径同样会走到 sense(), 清零点漏在后面的话计数会一直涨。*/
    if(sample_gap < 0xFFFF) sample_gap++;
    sample_new = 0;
    if(sensor.bx != prev_rx || sensor.by != prev_ry || sensor.bz != prev_rz)
    {
        prev_rx = sensor.bx; prev_ry = sensor.by; prev_rz = sensor.bz;
        if(sens_new_cnt < 0xFFFF) sens_new_cnt++;
        sample_new = 1;
        sample_dt  = sample_gap;
        sample_gap = 0;
    }

    /* 饱和检测。环形永磁体在传感器处的直流偏置可能有几十 mT, 一旦它加上矿石和
     * 线圈的场顶到满量程, 读数就被削平 —— 控制器拿着削平的数据照算, 表现是
     * 「参数怎么调都不对」而不是明确的故障。这里按失败计数处理: 偶发一次不latch
     * (大扰动时可能瞬时过量程), 持续饱和才进 FAULT, 提示要换高量程。 */
    if(sensor_saturated())
    {
        if(fail_count < 255) fail_count++;
        return 1;
    }

    fail_count = 0;

    /* 用滞后后的指令而不是当前指令 —— 见 cmd_lag 的说明。正反向系数按滞后量
     * 的符号选: 决定磁场的是此刻真实的电流方向, 不是刚发出去的那个指令。 */
    for(i = 0; i < COIL_NUM; i++)
    {
        float u = cmd_lag[i] * 0.001f;
        if(cmd_lag[i] >= 0.0f)
        {
            cx += ct_pos[0][i] * u;
            cy += ct_pos[1][i] * u;
            cz += ct_pos[2][i] * u;
        }
        else
        {
            cx += ct_neg[0][i] * u;
            cy += ct_neg[1][i] * u;
            cz += ct_neg[2][i] * u;
        }
    }

    {
        float x = (float)sensor.bx - bx0 - cx;
        float y = (float)sensor.by - by0 - cy;
        float z = (float)sensor.bz - bz0 - cz;
        float xr;

        /* 传感器姿态校正, 顺序不能反:
         * 1) 先扣 Z 漏项。飞线焊接不可能完全水平, 倾斜角 phi 会让 X 读到
         *    Bz*sin(phi) —— 矿石在场时 Bz 有上千 counts, 5 度就是 150+ counts 的
         *    假横移, 而且随高度变化, 空载标的零点扣不掉(标零点时矿石不在场)。
         *    这一项正比于 z, 所以减 tilt*z 就能连同它的高度依赖一起消掉。
         * 2) 再做面内旋转, 把传感器的 X/Y 转到线圈坐标系。串扰补偿必须在这之前
         *    完成 —— 它的系数是在传感器自己的坐标系里标的。 */
        x -= g_tune.tilt_x * z;
        y -= g_tune.tilt_y * z;

        xr = x * g_tune.yaw_cos + y * g_tune.yaw_sin;
        y  = -x * g_tune.yaw_sin + y * g_tune.yaw_cos;
        x  = xr;

        *bx = x; *by = y; *bz = z;
    }
    return 0;
}

static void reset_pid(void)
{
    i_z = 0.0f;
    d_z_filt = d_x_filt = d_y_filt = 0.0f;
    prev_bz = prev_bx = prev_by = 0.0f;
    bx_f = by_f = 0.0f;
    lat_norm_f = 1.0f;          /* 起浮瞬间不要带着上一次的系数 */
    settle_count = 0;
    sat_count = 0;
}

/* 取 CAL_AVG_TICKS 拍的三轴均值。返回 1 表示期间出现过饱和 —— 此时这一点的
 * 读数被削平, 算出来的斜率偏小, 不能用。标定这条路径不经过 sense(), 所以
 * 饱和检测必须在这里单独做一次, 否则会悄悄得到一组错的系数。 */
static uint8_t cal_average(int32_t *ax, int32_t *ay, int32_t *az)
{
    uint8_t n, sat = 0;
    int32_t sx = 0, sy = 0, sz = 0;

    for(n = 0; n < CAL_AVG_TICKS; n++)
    {
        TMAG_ReadXYZ(&sensor);
        if(sensor_saturated()) sat = 1;
        sx += sensor.bx; sy += sensor.by; sz += sensor.bz;
        Delay_Ms(1);
    }
    *ax = sx / CAL_AVG_TICKS;
    *ay = sy / CAL_AVG_TICKS;
    *az = sz / CAL_AVG_TICKS;
    return sat;
}

static void cal_settle(void)
{
    uint8_t n;
    for(n = 0; n < CAL_SETTLE_TICKS; n++) Delay_Ms(1);
}

/*********************************************************************
 * 零点 + 串扰 + 分方向增益 标定 (要求台面上没有矿石)
 *
 * 分两步:
 *   1) 增益校正置直通, 逐路测出正向斜率与反向斜率;
 *   2) 按两个方向的强度比把强的一侧压到弱的一侧, 装进 coil.c;
 *      校正之后两侧强度齐平, 串扰系数也跟着乘上各自的增益存回去。
 *********************************************************************/
/* 由串扰系数反算传感器的面内旋转角。
 *
 * 原理: 四路线圈的方位是已知的(COIL_MIX 那张象限表), 所以单独给某一路通电时,
 * 它在传感器处产生的面内磁场方向就是"这一路相对传感器的方位"。把实测方向与
 * 期望方位比一下, 差出来的就是传感器自己转了多少 —— 不需要任何额外夹具,
 * 标串扰时顺手就得到了。
 *
 * 求最优旋转不用 atan2: 设期望单位向量 e_i、实测单位向量 v_i, 最佳旋转满足
 *      cos(theta) ∝ Σ (e_i · v_i)        (点积)
 *      sin(theta) ∝ Σ (e_i × v_i)        (叉积的 z 分量)
 * 归一化即可, 只需要一次开方。
 *
 * 结果折到 ±90 度以内: 180 度旋转等价于 X/Y 同时取反, 那正是 LATERAL_SIGN 在做
 * 的事 —— 两边都纠一次就会互相抵消, 所以这里只管 ±90, 剩下的交给 LATERAL_SIGN。*/
static void solve_yaw(void)
{
    /* 期望方位, 与 COIL_MIX_X/Y_INIT 一致 (A,B,C,D), 未归一化不影响方向 */
    static const int8_t ex[COIL_NUM] = COIL_MIX_X_INIT;
    static const int8_t ey[COIL_NUM] = COIL_MIX_Y_INIT;
    float dot = 0.0f, crs = 0.0f, n;
    uint8_t ch;

    for(ch = 0; ch < COIL_NUM; ch++)
    {
        float vx = (ct_pos[0][ch] + ct_neg[0][ch]) * 0.5f;
        float vy = (ct_pos[1][ch] + ct_neg[1][ch]) * 0.5f;
        float m = sqrt_f(vx * vx + vy * vy);
        if(m < 1.0f) continue;              /* 这一路面内分量太弱, 方向不可信 */
        vx /= m; vy /= m;
        dot += (float)ex[ch] * vx + (float)ey[ch] * vy;
        crs += (float)ex[ch] * vy - (float)ey[ch] * vx;
    }

    n = sqrt_f(dot * dot + crs * crs);
    if(n < 0.5f) return;                    /* 四路互相矛盾, 保持原值不动 */
    dot /= n; crs /= n;

    if(dot < 0.0f) { dot = -dot; crs = -crs; }  /* 折到 ±90 度 */

    g_tune.yaw_cos = dot;
    g_tune.yaw_sin = crs;
}

/* 只测串扰斜率, 不动零点。
 *
 * 为什么要能单独跑: 铁芯的磁化状态取决于【总场】, 空载标出来的斜率在矿石在场时
 * 并不成立 —— 矿石离铁芯很近, 会把铁芯的工作点整个挪走。而残差 eps 一旦偏大,
 * 微分项就会对自己的输出求导形成自激(见 Levitation_Task 里横向环的说明)。
 * 所以: 零点空载测一次, 串扰带着矿石再测一次。
 *
 * 基线在函数内部就地取(线圈断电、矿石在场), 不写回 bx0/by0/bz0 —— 否则会把
 * 矿石自己的场吃进零点。
 *
 * 【使用要求】矿石必须用非磁性夹具固定在工作高度上。标定期间四路会轮流通 ±400,
 * 矿石若是自由悬浮会被推得到处跑, 测出来的斜率里混的是矿石位移而不是串扰。 */
static void measure_crosstalk(void)
{
    uint8_t ch, i;
    int32_t zx, zy, zz;
    int32_t px, py, pz, nx, ny, nz;
    float sp[3], sn[3];             /* 正/反向斜率, 每 1000 指令的磁场变化 */
    float mag_p, mag_n, gp, gn;

    Coil_DisableAll();
    for(ch = 0; ch < COIL_NUM; ch++)
        Coil_SetDirGain(ch, 1.0f, 1.0f);    /* 标定期间直通 */
    cal_settle();
    (void)cal_average(&zx, &zy, &zz);       /* 就地基线, 只用于算差分 */

    /* --- 逐路 --- */
    for(ch = 0; ch < COIL_NUM; ch++)
    {
        cal_sat[ch] = 0;

        Coil_Set(ch, CAL_CMD);
        cal_settle();
        if(cal_average(&px, &py, &pz)) cal_sat[ch] |= 0x01;

        Coil_Set(ch, -CAL_CMD);
        cal_settle();
        if(cal_average(&nx, &ny, &nz)) cal_sat[ch] |= 0x02;

        Coil_Set(ch, 0);

        /* 斜率 = (通电均值 - 零点) / 指令, 折算到每 1000 指令 */
        sp[0] = (float)(px - zx) * 1000.0f / (float)CAL_CMD;
        sp[1] = (float)(py - zy) * 1000.0f / (float)CAL_CMD;
        sp[2] = (float)(pz - zz) * 1000.0f / (float)CAL_CMD;

        sn[0] = (float)(nx - zx) * 1000.0f / (float)(-CAL_CMD);
        sn[1] = (float)(ny - zy) * 1000.0f / (float)(-CAL_CMD);
        sn[2] = (float)(nz - zz) * 1000.0f / (float)(-CAL_CMD);

        /* 用 L1 范数比较两个方向的强度 —— 只要比值, 不必开方 */
        mag_p = fabs_f(sp[0]) + fabs_f(sp[1]) + fabs_f(sp[2]);
        mag_n = fabs_f(sn[0]) + fabs_f(sn[1]) + fabs_f(sn[2]);
        cal_mag_p[ch] = mag_p;
        cal_mag_n[ch] = mag_n;

        if(mag_p > 1.0f && mag_n > 1.0f)
        {
            /* 把强的一侧压到弱的一侧 */
            if(mag_p > mag_n) { gp = mag_n / mag_p; gn = 1.0f; }
            else              { gp = 1.0f;          gn = mag_p / mag_n; }
        }
        else
        {
            /* 某个方向几乎没响应: 线圈开路 / 驱动坏了 / 标定被干扰。
             * 这时不做校正, 让问题以「四路不对称」的形式暴露出来,
             * 而不是被一个离谱的增益悄悄掩盖掉。 */
            gp = gn = 1.0f;
        }

        Coil_SetDirGain(ch, gp, gn);
        cmd_lag[ch] = 0.0f;             /* 每路测完都回零, 滞后量跟着清掉 */

        /* 串扰系数存校正后的值: sense() 里乘的是请求指令, 而请求指令经过
         * Coil_Set 的增益校正才变成实际输出。 */
        for(i = 0; i < 3; i++)
        {
            ct_pos[i][ch] = sp[i] * gp;
            ct_neg[i][ch] = sn[i] * gn;
        }
    }

    solve_yaw();
    Coil_DisableAll();
}

/* 完整标定: 空载零点 + 串扰。开机与 Levitation_Restart() 走这条。 */
static void calibrate_crosstalk(void)
{
    int32_t zx, zy, zz;
    uint8_t ch;

    Coil_DisableAll();
    for(ch = 0; ch < COIL_NUM; ch++) Coil_SetDirGain(ch, 1.0f, 1.0f);
    cal_settle();
    (void)cal_average(&zx, &zy, &zz);
    bx0 = (float)zx;
    by0 = (float)zy;
    bz0 = (float)zz;

    measure_crosstalk();
}

/* 只重测串扰, 保留已标好的零点。带矿石标定时用。 */
/* 标传感器倾斜。要求矿石用非磁性夹具【摆在几何正中】—— 那里真实的 Bx/By 应该
 * 是 0, 读到的非零值就是 Bz 漏过来的, 比值即 sin(倾角)。
 * 返回 0 = 成功。 */
uint8_t Levitation_CalibrateTilt(void)
{
    float bx, by, bz;
    float kc = g_tune.yaw_cos, ks = g_tune.yaw_sin;
    uint8_t bad;

    /* tilt 是在【传感器自己的坐标系】里定义的, 而 sense() 会先扣 tilt 再做 yaw
     * 旋转。所以标定期间两者都要暂时置成"不校正":
     *   tilt 不清零 -> 测到的是已校正后的残差, 会越标越偏;
     *   yaw  不置单位 -> 测到的 bx/by 是旋转后的分量, 存回去的比例就错位了。 */
    g_tune.tilt_x = 0.0f;
    g_tune.tilt_y = 0.0f;
    g_tune.yaw_cos = 1.0f;
    g_tune.yaw_sin = 0.0f;

    bad = sense(&bx, &by, &bz);
    if(!bad && fabs_f(bz) < (float)ORE_PRESENT_MIN_BZ) bad = 1;  /* 矿石不在场 */

    g_tune.yaw_cos = kc;
    g_tune.yaw_sin = ks;
    if(bad) return 1;

    g_tune.tilt_x = bx / bz;
    g_tune.tilt_y = by / bz;
    return 0;
}

void Levitation_RecalCrosstalk(void)
{
    measure_crosstalk();
    reset_pid();
}

void Levitation_Init(void)
{
    uint8_t i, j;

    for(i = 0; i < 3; i++)
        for(j = 0; j < COIL_NUM; j++)
        {
            ct_pos[i][j] = 0.0f;
            ct_neg[i][j] = 0.0f;
        }
    for(i = 0; i < COIL_NUM; i++) { cal_mag_p[i] = cal_mag_n[i] = 0.0f; cal_sat[i] = 0; }
    raw_bx = raw_by = raw_bz = 0;
    prev_rx = prev_ry = prev_rz = 0; sens_new_cnt = 0; sens_rate_hz = 0;
    sample_new = 0; sample_gap = 0; sample_dt = 1;
    for(i = 0; i < COIL_NUM; i++) { out_cmd[i] = 0; cmd_lag[i] = 0.0f; }

    bx0 = by0 = bz0 = 0.0f;
    bx_trim = by_trim = 0.0f;
    fail_count = 0;
    tick_count = 0;
    cur_h_mm = 0;
    cur_h_01mm = 0;
    last_bx = last_by = last_bz = 0;
    reset_pid();

    TMAG_I2C_Init();
    if(TMAG_Init(&sensor))
    {
        state = LEV_FAULT;
        return;
    }

    state = LEV_CALIB;
    apply_target(H_DEFAULT_01MM);
}

void Levitation_SetTargetHeight(uint8_t h_mm)
{
    int16_t h = (int16_t)h_mm * 10;         /* mm -> 0.1mm */

    if(h < H_MIN_01MM) h = H_MIN_01MM;
    if(h > H_MAX_01MM) h = H_MAX_01MM;

    if(h != target_h_01mm)
    {
        apply_target(h);
        settle_count = 0;                   /* 重新计时到位判据 */
    }
}

void Levitation_CalibrateHeightPoint(int16_t h_01mm)
{
    float bx, by, bz, r;

    if(sense(&bx, &by, &bz)) return;
    bz = fabs_f(bz);
    if(bz < 1.0f) return;

    r = (float)h_01mm + (float)H_SENSOR_OFFSET;
    g_tune.kz = bz * r * r * r;
    apply_target(target_h_01mm);             /* 用新的 kz 重算目标磁场 */
}

uint8_t    Levitation_GetHeight_mm(void) { return cur_h_mm; }
LevState_t Levitation_GetState(void)     { return state; }
int16_t    Levitation_GetTemp(void)      { return sensor.temp_c; }
uint8_t    Levitation_IsSettled(void)    { return (settle_count >= SETTLE_TICKS) ? 1 : 0; }

int16_t    Levitation_GetHeight_01mm(void)       { return cur_h_01mm; }
int16_t    Levitation_GetTargetHeight_01mm(void) { return target_h_01mm; }

void Levitation_GetField(int16_t *bx, int16_t *by, int16_t *bz)
{
    *bx = last_bx;
    *by = last_by;
    *bz = last_bz;
}

void Levitation_GetCalMag(uint8_t ch, float *pos, float *neg, uint8_t *sat)
{
    if(ch >= COIL_NUM) { *pos = 0.0f; *neg = 0.0f; *sat = 0; return; }
    *pos = cal_mag_p[ch];
    *neg = cal_mag_n[ch];
    *sat = cal_sat[ch];
}

void Levitation_GetCrosstalk(uint8_t ch, float *cx, float *cy, float *cz)
{
    if(ch >= COIL_NUM) { *cx = *cy = *cz = 0.0f; return; }
    /* 取正反向平均: 两个方向理应大小相等符号相同(都是"每 1000 指令"的斜率),
     * 差得多本身就说明驱动不对称。 */
    *cx = (ct_pos[0][ch] + ct_neg[0][ch]) * 0.5f;
    *cy = (ct_pos[1][ch] + ct_neg[1][ch]) * 0.5f;
    *cz = (ct_pos[2][ch] + ct_neg[2][ch]) * 0.5f;
}

uint16_t Levitation_GetSensorRate(void)
{
    return sens_rate_hz;
}

/* 自整定偏置。必须能看见 —— 否则无法区分"矿石真的偏了"和"trim 把偏置吃掉了",
 * 而这两者在 bx 上长得完全一样(收敛后 bx 恒等于 trim)。 */
void Levitation_GetTrim(int16_t *tx, int16_t *ty)
{
    *tx = (int16_t)bx_trim;
    *ty = (int16_t)by_trim;
}

void Levitation_GetRawField(int16_t *bx, int16_t *by, int16_t *bz)
{
    *bx = raw_bx;
    *by = raw_by;
    *bz = raw_bz;
}

void Levitation_RefreshTarget(void)
{
    apply_target(target_h_01mm);
}

void Levitation_Restart(void)
{
    Coil_DisableAll();
    reset_pid();
    bx_trim = by_trim = 0.0f;   /* 显式重启才清 trim, 见变量声明处 */
    fail_count = 0;
    cur_h_mm = 0;
    cur_h_01mm = 0;
    state = LEV_CALIB;
}

/*********************************************************************
 * 主控制拍, 由主循环按 CONTROL_HZ 调用
 *********************************************************************/
void Levitation_Task(void)
{
    float bx, by, bz, bz_mag;
    float err, d_raw, u_z, u_x, u_y, lat_norm, ex, ey;
    int16_t h, diff;
    int32_t slew;
    uint8_t i, all_sat = 1;

    tick_count++;
    update_cmd_lag();       /* 必须在 sense() 之前, 理由见函数说明 */

    /* 每秒结算一次传感器更新率。放在最前面, 保证 FAULT/IDLE 等提前 return 的
     * 路径也照常统计 —— 恰恰是"读不到新值"的时候最需要这个数。 */
    if((tick_count % CONTROL_HZ) == 0)
    {
        sens_rate_hz = sens_new_cnt;
        sens_new_cnt = 0;
    }

    /* 每 500ms 查一次温度 (整机要求全程 <70℃) */
    if((tick_count % (CONTROL_HZ / 2)) == 0)
    {
        if(TMAG_ReadTemp(&sensor) == 0 && sensor.temp_c > TEMP_LIMIT_C)
        {
            Coil_DisableAll();
            state = LEV_FAULT;
            return;
        }
    }

    switch(state)
    {
    case LEV_CALIB:
        calibrate_crosstalk();
        reset_pid();
        state = LEV_IDLE;
        return;

    case LEV_FAULT:
        Coil_DisableAll();
        return;

    default:
        break;
    }

    if(sense(&bx, &by, &bz))
    {
        if(fail_count >= SENSOR_FAIL_LIMIT)
        {
            Coil_DisableAll();
            state = LEV_FAULT;
        }
        return;
    }

    bz_mag = fabs_f(bz);

    /* 保存给遥测。放在这里而不是函数末尾, 是为了让提前 return 的几条路径
     * (开环测试、矿石不在) 也能在上位机上看到实时磁场。 */
    last_bx = (int16_t)bx;
    last_by = (int16_t)by;
    last_bz = (int16_t)bz;

    /* ---- 开环单路测试期间控制器完全让路 ---- */
    if(Tuning_TestActive())
    {
        cur_h_01mm = bz_to_height(bz_mag);
        cur_h_mm = (uint8_t)((cur_h_01mm + 5) / 10);
        return;                         /* 线圈由 Tuning_Task 驱动 */
    }

    /* ---- 矿石在不在? ---- */
    if(bz_mag < (float)ORE_PRESENT_MIN_BZ || bz_mag > (float)ORE_TOO_CLOSE_BZ)
    {
        /* 没检测到矿石, 或者矿石已经贴到底座上 —— 两种情况都不该继续通电 */
        Coil_DisableAll();
        for(i = 0; i < COIL_NUM; i++) out_cmd[i] = 0;
        if(state == LEV_RUN) reset_pid();
        state = LEV_IDLE;
        cur_h_mm = 0;
        cur_h_01mm = 0;

        /* 空载重新归零, 跟住环形永磁体的温漂。此时线圈已断电, 串扰为 0,
         * 所以直接把零点朝原始读数拉即可。
         * 只在磁场明显低于在场阈值时才做: 贴底(bz_mag 超上限)那一支绝不能
         * 归零, 否则会把矿石自己的场吃进零点。 */
        if(bz_mag < (float)REZERO_GUARD_BZ)
        {
            bx0 += ((float)sensor.bx - bx0) * REZERO_ALPHA;
            by0 += ((float)sensor.by - by0) * REZERO_ALPHA;
            bz0 += ((float)sensor.bz - bz0) * REZERO_ALPHA;
        }
        return;
    }

    if(state == LEV_IDLE)
    {
        /* 矿石进入量程, 起浮 */
        reset_pid();
        prev_bz = bz_mag;
        /* 输入低通与微分都从当前读数起步, 否则第一拍会有一个 bx 大小的假跳变
         * 直接喂进微分项 —— 而横向就是靠微分撑住的那个轴。 */
        bx_f = prev_bx = bx;
        by_f = prev_by = by;
        state = LEV_RUN;
    }

    /* ---- 高度换算。每拍只做这一次开立方 ----
     * 提到控制之前, 因为横向环要拿它做增益归一化(见下)。 */
    h = bz_to_height(bz_mag);
    cur_h_01mm = h;
    cur_h_mm = (uint8_t)((h + 5) / 10);     /* 0.1mm -> mm, 四舍五入 */

    /* ---- 高度: 慢环, 前馈 + 积分为主 ---- */
    err = bz_mag - target_bz;               /* >0: 偏低, 场太强, 需要更大推力 */

    i_z += err * g_tune.ki_z / (float)CONTROL_HZ;
    if(i_z >  g_tune.ilim_z) i_z =  g_tune.ilim_z;
    if(i_z < -g_tune.ilim_z) i_z = -g_tune.ilim_z;

    /* 微分取自测量而非误差, 避免设定值跳变时的微分冲击。
     * 只在有新采样时更新, 且按真实间隔求导 —— 见 sample_gap 的说明。 */
    if(sample_new)
    {
        d_raw = (bz_mag - prev_bz) * (float)CONTROL_HZ / (float)sample_dt;
        prev_bz = bz_mag;
        d_z_filt += g_tune.dlpf_z * (d_raw - d_z_filt);
    }

    u_z = ff_z + g_tune.kp_z * err + i_z + g_tune.kd_z * d_z_filt;

    /* ---- 横向: 快环, 这是唯一不稳定的轴 ---- */
    /* 极性见 board.h 的 LATERAL_SIGN: 线圈在象限上, 加大某一路会把矿石朝远离
     * 该线圈的方向推, 所以纠偏要加大偏移那一侧的线圈, u_x 与 bx 同号。 */

    /* 增益归一化 —— 不做这一步, 同一组参数在不同高度上根本不是同一个环路。
     *      bx = 1.5 · Bz · x / h,  而 Bz ∝ 1/h³   =>   bx ∝ x / h⁴
     * 也就是说"每毫米横移对应多少读数"随高度四次方变化。控制律直接拿 bx 当误差,
     * 于是从位移到力的开环增益也 ∝ 1/h⁴: 高度在 2.5~4.3cm 之间游走就是 8.8 倍的
     * 增益摆幅, 在一个高度上调到临界的参数, 掉下去一点必然发散 —— 现象正是
     * "先稳一阵, 然后越晃越大直到飞掉"。
     * 这里把读数折算成"标称高度下的等效读数", 环路增益就与高度无关了。
     * 标称高度处系数恰为 1, 所以 kp_xy 的含义不变, 不需要重调。 */
    lat_norm = ((float)h + (float)H_SENSOR_OFFSET) /
               ((float)target_h_01mm + (float)H_SENSOR_OFFSET);
    lat_norm = lat_norm * lat_norm * lat_norm * lat_norm;
    if(lat_norm > LAT_NORM_MAX) lat_norm = LAT_NORM_MAX;
    if(lat_norm < LAT_NORM_MIN) lat_norm = LAT_NORM_MIN;
    /* 先滤再用: 四次方放大了 Bz 的噪声, 而这个系数直接乘在输出上。 */
    lat_norm_f += LAT_NORM_LPF * (lat_norm - lat_norm_f);

    /* 归一化乘在【输出】上, 不是乘在 bx 上。乘在输入上的话微分项会把 lat_norm
     * 自己的抖动也微分一遍 —— 系数抖 0.5% 在 bx=100 时就是 60 的微分输出, 和
     * 比例项同量级, 等于往阻尼通道里灌噪声; 钳位切换时更是直接给一个阶跃。
     * 乘在输出上对恒定系数完全等价, 而微分只看原始读数。 */
    /* 输入低通。只在有新采样时推进 —— 传感器若慢于控制拍, 每拍都滤一次等于
     * 让滤波器跑在保持值上, 时间常数就不再是设计的那个。 */
    if(sample_new)
    {
        float a = g_tune.xy_lpf;
        if(a > 1.0f)  a = 1.0f;
        if(a < 0.01f) a = 0.01f;    /* 0 会让读数永远停在 0, 等于横向环失明 */
        bx_f += a * (bx - bx_f);
        by_f += a * (by - by_f);

        d_raw = (bx_f - prev_bx) * (float)CONTROL_HZ / (float)sample_dt;
        prev_bx = bx_f;
        d_x_filt += g_tune.dlpf_xy * (d_raw - d_x_filt);

        d_raw = (by_f - prev_by) * (float)CONTROL_HZ / (float)sample_dt;
        prev_by = by_f;
        d_y_filt += g_tune.dlpf_xy * (d_raw - d_y_filt);
    }

    /* 误差 = 滤波读数 - 自整定偏置 */
    ex = bx_f - bx_trim;
    ey = by_f - by_trim;

    u_x = g_tune.lat_sign * lat_norm_f *
          (g_tune.kp_xy * ex + g_tune.kd_xy * d_x_filt);
    u_y = g_tune.lat_sign * lat_norm_f *
          (g_tune.kp_xy * ey + g_tune.kd_xy * d_y_filt);

    /* ---- 设定点自整定 ---- 移植自参考工程, 推导见 board.h 的 TRIM_K_DEF。
     * 被积量是线圈出力, 收敛到 u = 0, 即矿石停在真几何中心, 而传感器的一切
     * 直流偏置被 trim 吸收。门限是唯一的保险: 快环没稳住时读数会冲过门限,
     * 积分随即停下, 不会跟着一起跑飞。 */
    if(g_tune.trim_k != 0.0f)
    {
        float lim = g_tune.trim_lim;
        if(lim < 0.0f) lim = 0.0f;

        if(fabs_f(ex) < lim)
        {
            bx_trim += g_tune.trim_k * u_x;
            if(bx_trim >  lim) bx_trim =  lim;
            if(bx_trim < -lim) bx_trim = -lim;
        }
        if(fabs_f(ey) < lim)
        {
            by_trim += g_tune.trim_k * u_y;
            if(by_trim >  lim) by_trim =  lim;
            if(by_trim < -lim) by_trim = -lim;
        }
    }

    /* ---- 混合 + 限斜率 ---- */
    /* slew <= 0 不是「限得很死」, 而是「输出永远不许变」: 下面两条分支都会把 v
     * 打回 out_cmd[i], 四路从此冻在当时的残值上, 再也不响应任何输入。而磁场、
     * 高度、状态照常更新, 现象酷似线圈或驱动坏掉, 极难往参数上想。这个取值没有
     * 任何有意义的用途, 直接当成不限斜率。 */
    slew = (int32_t)g_tune.slew;
    if(slew < 1) slew = COIL_CMD_LIMIT * 2;

    for(i = 0; i < COIL_NUM; i++)
    {
        float f = u_z + (float)mix_x[i] * u_x + (float)mix_y[i] * u_y;
        int32_t v;

        /* 按 COIL_CMD_LIMIT 而不是 COIL_CMD_MAX 限幅: Coil_Set 内部也是按
         * LIMIT 截的, 这里若放宽到 MAX, out_cmd[] 会记下一个实际没发出去的值,
         * 限斜率和串扰补偿就都跟真实输出对不上了。 */
        if(f >  (float)COIL_CMD_LIMIT) f =  (float)COIL_CMD_LIMIT;
        if(f < -(float)COIL_CMD_LIMIT) f = -(float)COIL_CMD_LIMIT;
        v = (int32_t)f;

        diff = (int16_t)(v - out_cmd[i]);
        if(diff >  slew) v = out_cmd[i] + slew;
        if(diff < -slew) v = out_cmd[i] - slew;

        out_cmd[i] = (int16_t)v;
        if(abs_i16(out_cmd[i]) < COIL_CMD_LIMIT) all_sat = 0;
    }
    Coil_SetAll(out_cmd);

    /* 失控保护, 见 SAT_STALL_TICKS 的说明 */
    if(all_sat)
    {
        if(++sat_count >= SAT_STALL_TICKS)
        {
            Coil_DisableAll();
            state = LEV_FAULT;
            return;
        }
    }
    else sat_count = 0;

    /* ---- 到位判据 (高度已在上面算过) ---- */
    diff = h - target_h_01mm;
    if(diff < 0) diff = -diff;
    if(diff <= SETTLE_TOL_01MM)
    {
        if(settle_count < SETTLE_TICKS) settle_count++;
    }
    else
    {
        settle_count = 0;
    }
}
