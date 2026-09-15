/********************************** (C) COPYRIGHT *******************************
 * File Name          : referee.c
 * Description        : USART1 收发 —— 裁判系统协议 + 调参通道共用一条线
 *
 * 下行 (裁判 -> 装置, 4 字节): AA | 命令字 | 高度(0.1cm) | 校验和
 * 上行 (装置 -> 裁判, 见 REF_UPLINK_LEN): BB | 状态字 | 高度(0.1cm) | 校验和
 * 校验和 = Byte0+Byte1+Byte2 的低 8 位。
 *
 * 调参通道用 0xA5 / 0x5B 帧头 (见 tuning.h), 不定长。两套协议靠帧头区分,
 * 互不干扰; 调参遥测默认关闭, 比赛时线上不会出现 0x5B 开头的字节。
 *
 * 发送走中断 + 环形缓冲: 一帧裁判报文在 115200 下要 347us, 遥测帧要 2.2ms,
 * 直接阻塞发会吃掉整个控制周期。
 *
 * 注意: USART1 就是模板里 printf 的出口。这条线现在接的是裁判系统/上位机,
 * 任何 printf 都会插进协议帧里把它冲烂, 所以整个工程不要用 printf。
 *******************************************************************************/
#include "referee.h"
#include "levitation.h"
#include "tuning.h"

/* PARAM_LIST 一次会连发 17 帧参数名(约 270 字节), 缓冲要能一口气吃下,
 * 否则上位机连上时参数表会缺项。 */
#define TXBUF_SIZE      512
#define TXBUF_MASK      (TXBUF_SIZE - 1)

#define DBG_MAX_LEN     32

static volatile uint8_t  txbuf[TXBUF_SIZE];
static volatile uint16_t tx_head, tx_tail;

/* 接收状态机: 两种帧头, 定长的裁判帧与不定长的调试帧 */
enum {
    RX_HEAD = 0,
    RX_REF_BODY,        /* 裁判帧, 还差 3 字节 */
    RX_DBG_LEN,
    RX_DBG_BODY,
    RX_DBG_CK
};
static volatile uint8_t rx_state;
static volatile uint8_t rx_idx;
static volatile uint8_t rx_len;
static volatile uint8_t rx_sum;
static volatile uint8_t rx_buf[DBG_MAX_LEN];

/* 裁判帧留一个槽位就够: 下行是人/裁判系统的节奏, 不会连发。 */
static volatile uint8_t ref_frame[4];
static volatile uint8_t ref_ready;

/* 调试帧必须排队。上位机拿到参数表后会对全部参数连发 GET, 115200 下一帧 5 字节
 * 只要 434us, 而 Referee_PollRx 每 1ms 才取一次 —— 单槽位会丢掉一多半, 现象是
 * 「参数表有的行有值、有的行是空的」「改参数时灵时不灵」, 而且完全没有提示。
 *
 * 深度 4 + 每拍把积压的全部取走 = 每毫秒可消化 4 帧, 高于最快 2.3 帧/ms 的到达
 * 速率, 留有余量。 */
#define DBG_QUEUE       4
static volatile uint8_t dbg_q[DBG_QUEUE][DBG_MAX_LEN];
static volatile uint8_t dbg_q_len[DBG_QUEUE];
static volatile uint8_t dbg_q_head, dbg_q_tail;
static volatile uint8_t dbg_drops;      /* 队列满而丢掉的帧数, 只增不减 */

void Referee_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    USART_InitTypeDef usart = {0};
    NVIC_InitTypeDef nvic = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);

    /* PA9 = TX */
    gpio.GPIO_Pin = GPIO_Pin_9;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    /* PA10 = RX */
    gpio.GPIO_Pin = GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);

    usart.USART_BaudRate = 115200;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &usart);

    nvic.NVIC_IRQChannel = USART1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART1, ENABLE);

    tx_head = tx_tail = 0;
    rx_state = RX_HEAD;
    rx_idx = 0;
    ref_ready = 0;
    dbg_q_head = dbg_q_tail = 0;
    dbg_drops = 0;
}

static void tx_push(uint8_t b)
{
    uint16_t next = (uint16_t)((tx_head + 1) & TXBUF_MASK);

    if(next == tx_tail) return;         /* 满了就丢, 下一拍还会再报 */
    txbuf[tx_head] = b;
    tx_head = next;
}

static void tx_start(void)
{
    if(tx_head != tx_tail) USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
}

void Referee_Send(const uint8_t *data, uint8_t len)
{
    uint8_t i;
    for(i = 0; i < len; i++) tx_push(data[i]);
    tx_start();
}

/*********************************************************************
 * 接收中断: 只做拆帧与校验, 语义处理交给主循环。
 *********************************************************************/
static void rx_byte(uint8_t b)
{
    switch(rx_state)
    {
    case RX_HEAD:
        if(b == REF_DOWN_HEAD)
        {
            rx_buf[0] = b;
            rx_idx = 1;
            rx_state = RX_REF_BODY;
        }
        else if(b == TUNE_DOWN_HEAD)
        {
            rx_sum = b;
            rx_state = RX_DBG_LEN;
        }
        /* 其它字节: 保持等待帧头, 天然完成重同步 */
        break;

    case RX_REF_BODY:
        rx_buf[rx_idx++] = b;
        if(rx_idx >= 4)
        {
            uint8_t sum = (uint8_t)(rx_buf[0] + rx_buf[1] + rx_buf[2]);
            if(sum == rx_buf[3] && !ref_ready)
            {
                ref_frame[0] = rx_buf[0]; ref_frame[1] = rx_buf[1];
                ref_frame[2] = rx_buf[2]; ref_frame[3] = rx_buf[3];
                ref_ready = 1;
            }
            rx_state = RX_HEAD;
        }
        break;

    case RX_DBG_LEN:
        if(b == 0 || b > DBG_MAX_LEN)
        {
            rx_state = RX_HEAD;         /* 长度不合法, 当噪声丢掉 */
            break;
        }
        rx_len = b;
        rx_sum = (uint8_t)(rx_sum + b);
        rx_idx = 0;
        rx_state = RX_DBG_BODY;
        break;

    case RX_DBG_BODY:
        rx_buf[rx_idx++] = b;
        rx_sum = (uint8_t)(rx_sum + b);
        if(rx_idx >= rx_len) rx_state = RX_DBG_CK;
        break;

    default:    /* RX_DBG_CK */
        if(b == rx_sum)
        {
            uint8_t next = (uint8_t)((dbg_q_head + 1) % DBG_QUEUE);
            if(next == dbg_q_tail)
            {
                if(dbg_drops < 255) dbg_drops++;     /* 满了, 记一笔再丢 */
            }
            else
            {
                uint8_t i;
                for(i = 0; i < rx_len; i++) dbg_q[dbg_q_head][i] = rx_buf[i];
                dbg_q_len[dbg_q_head] = rx_len;
                dbg_q_head = next;
            }
        }
        rx_state = RX_HEAD;
        break;
    }
}

void Referee_IRQHandler(void)
{
    /* 溢出优先处理。刷 WS2812 时要关中断约 180us, 而 115200 下一个字节只要
     * 87us —— 这个窗口足够丢字节并置上 ORE。不清 ORE 的话 RXNE 会一直进不来,
     * 整条下行链路就哑了。丢帧本身无所谓, 帧头会重新同步。 */
    if(USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET)
    {
        (void)USART_ReceiveData(USART1);        /* 读 DR 清 ORE */
        rx_state = RX_HEAD;
        return;
    }

    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        rx_byte((uint8_t)USART_ReceiveData(USART1));
    }

    if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
    {
        if(tx_head != tx_tail)
        {
            USART_SendData(USART1, txbuf[tx_tail]);
            tx_tail = (uint16_t)((tx_tail + 1) & TXBUF_MASK);
        }
        else
        {
            USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
        }
    }
}

/*********************************************************************
 * 由主循环按 CONTROL_HZ 调用: 分发收到的帧。
 * 放在控制拍上而不是上报拍上, 是为了让调参命令的响应延迟保持在 1ms,
 * 开环测试的续命包尤其不能等 20ms。
 *********************************************************************/
void Referee_PollRx(void)
{
    if(ref_ready)
    {
        if(ref_frame[1] == REF_CMD_SET_HEIGHT)
            Levitation_SetTargetHeight(ref_frame[2]);
        ref_ready = 0;
    }

    /* 把积压的调试帧一次全取走, 不是每拍只取一帧 —— 理由见 dbg_q 的说明。 */
    while(dbg_q_tail != dbg_q_head)
    {
        uint8_t buf[DBG_MAX_LEN];
        uint8_t n = dbg_q_len[dbg_q_tail], i;
        for(i = 0; i < n; i++) buf[i] = dbg_q[dbg_q_tail][i];
        dbg_q_tail = (uint8_t)((dbg_q_tail + 1) % DBG_QUEUE);
        Tuning_HandleFrame(buf, n);
    }
}

uint8_t Referee_GetRxDrops(void)
{
    return dbg_drops;
}

static void send_report(uint8_t status, uint8_t height_mm)
{
    uint8_t f[REF_UPLINK_LEN];
    uint8_t i;

    for(i = 0; i < REF_UPLINK_LEN; i++) f[i] = 0;
    f[0] = REF_UP_HEAD;
    f[1] = status;
    f[2] = height_mm;
    f[3] = (uint8_t)(f[0] + f[1] + f[2]);

    Referee_Send(f, REF_UPLINK_LEN);
}

void Referee_Task(void)
{
    /* 定期上报。状态字按「状态」语义给: 到位期间持续报 0x01, 未到位报 0x00,
     * 这样裁判系统丢一帧也不会错过到位通知。 */
    send_report(Levitation_IsSettled() ? REF_STATUS_SETTLED : REF_STATUS_NORMAL,
                Levitation_GetHeight_mm());
}
