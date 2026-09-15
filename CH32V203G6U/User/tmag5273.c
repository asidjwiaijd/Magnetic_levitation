/********************************** (C) COPYRIGHT *******************************
 * File Name          : tmag5273.c
 * Description        : TMAG5273B2QDBVR 驱动 —— I2C1 PB6(SCL)/PB7(SDA), 地址 0x22
 *
 * 型号解码: TMAG5273 的字母决定 I2C 地址 (A=0x35 B=0x22 C=0x78 D=0x44),
 *           数字决定量程 (1 = ±40/±80mT, 2 = ±133/±266mT)。
 *           实装 B2 -> 地址 0x22, 量程 ±133/±266mT。
 *******************************************************************************/
#include "tmag5273.h"
#include "debug.h"

#define I2C_TIMEOUT     1000

static uint8_t I2C_WaitEvent(uint32_t event, uint32_t timeout_ms)
{
    uint32_t n = 0;
    while(!I2C_CheckEvent(I2C1, event))
    {
        if(++n > timeout_ms * 100) return 1;
    }
    return 0;
}

static uint8_t TMAG_WriteReg(uint8_t addr, uint8_t reg, uint8_t val)
{
    I2C_GenerateSTART(I2C1, ENABLE);
    if(I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT, I2C_TIMEOUT)) return 1;

    I2C_Send7bitAddress(I2C1, addr << 1, I2C_Direction_Transmitter);
    if(I2C_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED, I2C_TIMEOUT)) return 1;

    I2C_SendData(I2C1, reg);
    if(I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED, I2C_TIMEOUT)) return 1;

    I2C_SendData(I2C1, val);
    if(I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED, I2C_TIMEOUT)) return 1;

    I2C_GenerateSTOP(I2C1, ENABLE);
    return 0;
}

static uint8_t TMAG_ReadReg(uint8_t addr, uint8_t reg, uint8_t *buf)
{
    I2C_GenerateSTART(I2C1, ENABLE);
    if(I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT, I2C_TIMEOUT)) return 1;

    I2C_Send7bitAddress(I2C1, addr << 1, I2C_Direction_Transmitter);
    if(I2C_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED, I2C_TIMEOUT)) return 1;

    I2C_SendData(I2C1, reg);
    if(I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED, I2C_TIMEOUT)) return 1;

    I2C_GenerateSTART(I2C1, ENABLE);
    if(I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT, I2C_TIMEOUT)) return 1;

    I2C_Send7bitAddress(I2C1, addr << 1, I2C_Direction_Receiver);
    if(I2C_WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED, I2C_TIMEOUT)) return 1;

    I2C_AcknowledgeConfig(I2C1, DISABLE);
    I2C_GenerateSTOP(I2C1, ENABLE);

    if(I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED, I2C_TIMEOUT)) return 1;
    *buf = I2C_ReceiveData(I2C1);

    I2C_AcknowledgeConfig(I2C1, ENABLE);
    return 0;
}

static uint8_t TMAG_ReadBurst(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t i;

    I2C_GenerateSTART(I2C1, ENABLE);
    if(I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT, I2C_TIMEOUT)) return 1;

    I2C_Send7bitAddress(I2C1, addr << 1, I2C_Direction_Transmitter);
    if(I2C_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED, I2C_TIMEOUT)) return 1;

    I2C_SendData(I2C1, reg);
    if(I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED, I2C_TIMEOUT)) return 1;

    I2C_GenerateSTART(I2C1, ENABLE);
    if(I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT, I2C_TIMEOUT)) return 1;

    I2C_Send7bitAddress(I2C1, addr << 1, I2C_Direction_Receiver);
    if(I2C_WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED, I2C_TIMEOUT)) return 1;

    for(i = 0; i < len; i++)
    {
        if(i < len - 1)
        {
            I2C_AcknowledgeConfig(I2C1, ENABLE);
            if(I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED, I2C_TIMEOUT)) return 1;
        }
        else
        {
            I2C_AcknowledgeConfig(I2C1, DISABLE);
            I2C_GenerateSTOP(I2C1, ENABLE);
            if(I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED, I2C_TIMEOUT)) return 1;
        }
        buf[i] = I2C_ReceiveData(I2C1);
    }

    I2C_AcknowledgeConfig(I2C1, ENABLE);
    return 0;
}

/*********************************************************************
 * @brief   恢复挂死的 I2C 总线。控制环里每毫秒读一次传感器, 一旦某次传输
 *          超时, I2C 外设会停在 BUSY, 从机可能正把 SDA 拉低锁死总线。
 *          手动发最多 9 个 SCL 脉冲让从机吐完剩余位, 补一个 STOP, 再重init。
 *********************************************************************/
static void I2C_BusRecover(void)
{
    GPIO_InitTypeDef gpio = {0};
    uint8_t i;

    I2C_Cmd(I2C1, DISABLE);
    I2C_DeInit(I2C1);

    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_Out_OD;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);

    GPIO_SetBits(GPIOB, GPIO_Pin_6 | GPIO_Pin_7);
    Delay_Us(10);

    for(i = 0; i < 9; i++)
    {
        if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7) == Bit_SET) break;
        GPIO_ResetBits(GPIOB, GPIO_Pin_6);
        Delay_Us(5);
        GPIO_SetBits(GPIOB, GPIO_Pin_6);
        Delay_Us(5);
    }

    /* STOP: SCL 高时 SDA 由低变高 */
    GPIO_ResetBits(GPIOB, GPIO_Pin_7);
    Delay_Us(5);
    GPIO_SetBits(GPIOB, GPIO_Pin_6);
    Delay_Us(5);
    GPIO_SetBits(GPIOB, GPIO_Pin_7);
    Delay_Us(10);

    TMAG_I2C_Init();
}

void TMAG_I2C_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    I2C_InitTypeDef i2c = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    /* I2C1 默认脚位 PB6=SCL, PB7=SDA, 无需重映射 */
    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_AF_OD;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);

    I2C_DeInit(I2C1);
    i2c.I2C_Mode = I2C_Mode_I2C;
    /* 必须用 DutyCycle_2, 不能用 16_9。快速模式下 CCR 是整数分频, 两种占空比的
     * 公式不同, 在 PCLK1=48MHz 下差别很大:
     *   16_9: CCR = 48e6/(f*25). f=400k -> 4.8 截断成 4 -> 实际 480kHz(超规范 20%),
     *         而且可选频率只有 480/384/320 三档, 根本落不到 400。
     *   2   : CCR = 48e6/(f*3).  f=400k -> 40 整除 -> 实际 400kHz 精确。
     * 改时钟树(SYSCLK/PCLK1)之后要重算这里, 别想当然。 */
    i2c.I2C_DutyCycle = I2C_DutyCycle_2;
    i2c.I2C_OwnAddress1 = 0x00;
    i2c.I2C_Ack = I2C_Ack_Enable;
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    i2c.I2C_ClockSpeed = 400000;
    I2C_Init(I2C1, &i2c);

    I2C_Cmd(I2C1, ENABLE);
    I2C_AcknowledgeConfig(I2C1, ENABLE);
}

uint8_t TMAG_Init(TMAG_t *s)
{
    uint8_t v;
    uint8_t cfg2 = 0;

    s->addr = TMAG_I2C_ADDR;
    s->bx = s->by = s->bz = 0;
    s->temp_c = 25;

    /* 两轴量程独立组装, 见 board.h 里选「Z 高 / XY 低」的理由 */
#if TMAG_XY_HIGH_RANGE
    cfg2 |= TMAG_RANGE_XY_HIGH;
    s->range_xy_mT = TMAG_RANGE_HIGH_MT;
#else
    s->range_xy_mT = TMAG_RANGE_LOW_MT;
#endif
#if TMAG_Z_HIGH_RANGE
    cfg2 |= TMAG_RANGE_Z_HIGH;
    s->range_z_mT = TMAG_RANGE_HIGH_MT;
#else
    s->range_z_mT = TMAG_RANGE_LOW_MT;
#endif

    /* 校验厂商 ID, 顺便确认地址猜对了 */
    if(TMAG_ReadReg(s->addr, TMAG_REG_MANUFACTURER_ID_LSB, &v)) return 1;
    if(v != TMAG_MANUF_ID_LSB) return 1;
    if(TMAG_ReadReg(s->addr, TMAG_REG_MANUFACTURER_ID_MSB, &v)) return 1;
    if(v != TMAG_MANUF_ID_MSB) return 1;

    TMAG_WriteReg(s->addr, TMAG_REG_DEVICE_STATUS, 0x0F);   /* 清状态 */

    if(TMAG_WriteReg(s->addr, TMAG_REG_SENSOR_CONFIG_1, TMAG_MAG_CH_EN_XYZ)) return 1;

    if(TMAG_WriteReg(s->addr, TMAG_REG_SENSOR_CONFIG_2, cfg2)) return 1;

    /* 片上 4x 平均。曾经是 CONV_AVG_1 ("控制环要求最小延迟"), 实测证明那个
     * 取舍是反的: 噪声而不是延迟才是横向环的瓶颈。完整推导见 tmag5273.h。 */
    if(TMAG_WriteReg(s->addr, TMAG_REG_DEVICE_CONFIG_1, TMAG_CONV_AVG_4)) return 1;

    /* 连续转换 + 低噪声模式 */
    if(TMAG_WriteReg(s->addr, TMAG_REG_DEVICE_CONFIG_2,
                     TMAG_OPMODE_CONTINUOUS | TMAG_LP_LOWNOISE)) return 1;

    /* 温度通道: 用于整机温升监控 (要求全程 <70℃) */
    TMAG_WriteReg(s->addr, TMAG_REG_T_CONFIG, TMAG_T_CH_EN);

    /* INTB 未接线, 屏蔽掉 */
    TMAG_WriteReg(s->addr, TMAG_REG_INT_CONFIG_1, 0x01);

    return 0;
}

uint8_t TMAG_ReadXYZ(TMAG_t *s)
{
    uint8_t buf[6];
    int16_t raw_x, raw_y, raw_z;
    int32_t k_xy, k_z;

    if(TMAG_ReadBurst(s->addr, TMAG_REG_X_MSB_RESULT, buf, 6))
    {
        I2C_BusRecover();
        return 1;
    }

    raw_x = (int16_t)((buf[0] << 8) | buf[1]);
    raw_y = (int16_t)((buf[2] << 8) | buf[3]);
    raw_z = (int16_t)((buf[4] << 8) | buf[5]);

    /* 数据手册: B(mT) = raw / 65536 * 2 * range。这里输出 0.01mT:
     *   B = raw * range * 200 / 65536
     * X/Y 与 Z 的量程可以不同, 换算系数必须分开算。
     * 注意参考工程用的是 /32768, 结果是真实值的两倍 —— 它只取比值所以无所谓,
     * 这里要上报绝对高度, 必须用正确的标度。 */
    k_xy = (int32_t)s->range_xy_mT * 200;
    k_z  = (int32_t)s->range_z_mT  * 200;
    s->bx = (int16_t)((int32_t)raw_x * k_xy / 65536);
    s->by = (int16_t)((int32_t)raw_y * k_xy / 65536);
    s->bz = (int16_t)((int32_t)raw_z * k_z  / 65536);

    return 0;
}

uint8_t TMAG_ReadTemp(TMAG_t *s)
{
    uint8_t buf[2];
    int32_t raw;

    if(TMAG_ReadBurst(s->addr, TMAG_REG_T_MSB_RESULT, buf, 2)) return 1;

    raw = (int32_t)((uint16_t)((buf[0] << 8) | buf[1]));

    /* 数据手册: T(℃) = 25 + (T_ADC - 17508) / 60.1, 用整数近似 (x10/601) */
    s->temp_c = (int16_t)(25 + (raw - 17508) * 10 / 601);
    return 0;
}
