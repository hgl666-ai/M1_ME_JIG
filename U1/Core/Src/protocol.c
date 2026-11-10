/**
  ******************************************************************************
  * @file    protocol.c
  * @brief   PC 协议收发状态机 + 命令分发
  *
  * 帧：A5 5B | CMD(2) | SEQ(2) | LEN(2) | DATA | CRC16(高字节在前)
  * 规则（IINFO 协议表）：
  *   - 收到合法帧必须回 ACK，ACK 的 SEQ 原样回填
  *   - CRC 错误只诊断不回 ACK
  *   - ACK 数据第 1 字节固定为状态码
  ******************************************************************************
  */
#include "protocol.h"
#include "protocol_frame.h"
#include "bsp_uart.h"
#include "usart.h"        /* huart1 */
#include "bsp_adc.h"
#include "bsp_sensor.h"
#include "app.h"

#if defined(BOARD_U2)
#include "bsp_motor.h"
#include "bsp_tmc2209.h"
#endif

#include <string.h>

/*==============================================================================
 * 内部状态
 *============================================================================*/
enum {
    S_HDR0 = 0, S_HDR1,
    S_CMD_H, S_CMD_L,
    S_SEQ_H, S_SEQ_L,
    S_LEN_H, S_LEN_L,
    S_DATA,
    S_CRC_H, S_CRC_L
};

static uint8_t  rx_state;
static uint8_t  rx_buf[FRAME_BUF_SIZE];
static uint16_t rx_pos;
static uint16_t rx_dlen;
static uint16_t req_seq;

/* 收帧缓冲与响应缓冲共用一块内存，减少 .bss（C6T6 只有 10KB RAM） */
static uint8_t  frame_mem[FRAME_BUF_SIZE];

static uint16_t crc_err_cnt;      /* CRC 错误计数（诊断） */
static uint16_t seq_warn_cnt;     /* 序号非递增计数（仅告警） */
static uint16_t last_seq;

/*==============================================================================
 * 小工具
 *============================================================================*/
static void put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)(v);
}

#if defined(BOARD_U2)   /* 32 位字段只在 U2 的电机/TMC 命令里用到 */
static void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)(v);
}
#endif

static uint16_t get_u16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

#if defined(BOARD_U2)
static uint32_t get_u32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8)  |  (uint32_t)p[3];
}
#endif

static void SendResponse(uint16_t cmd, const uint8_t *data, uint16_t len)
{
    uint16_t total = Frame_Build(frame_mem, cmd, req_seq, data, len);
    UART_SendArray(&huart1, frame_mem, total);
}

/*==============================================================================
 * 命令处理
 *============================================================================*/
static void HandlePing(void)
{
    uint8_t rsp = PROTO_OK;
    SendResponse(PROTO_CMD_PING, &rsp, 1u);
}

static void HandleGetVersion(void)
{
    uint8_t rsp[4] = { PROTO_OK, FW_VER_MAJOR, FW_VER_MINOR, FW_VER_PATCH };
    SendResponse(PROTO_CMD_GET_VERSION, rsp, sizeof(rsp));
}

static void HandleGetUid(void)
{
    /* STM32F103 的 96bit 唯一 ID 位于 0x1FFFF7E8 */
    const uint8_t *uid = (const uint8_t *)0x1FFFF7E8UL;
    uint8_t rsp[13];
    rsp[0] = PROTO_OK;
    memcpy(&rsp[1], uid, 12u);
    SendResponse(PROTO_CMD_GET_UID, rsp, sizeof(rsp));
}

/**
  * @brief  0x000E 测量 ADC
  * @param  mode 0=读最近一批快照，1=立即触发一轮
  */
static void HandleMeasureAdc(const uint8_t *data, uint16_t len)
{
    uint8_t mode = (len >= 1u) ? data[0] : 0u;
    uint8_t rsp[ADC_SNAPSHOT_BYTES];

    if (mode > 1u) {
        uint8_t e = PROTO_ERR_PARAM;
        SendResponse(PROTO_CMD_MEASURE_ADC, &e, 1u);
        return;
    }
    if (mode == 1u) {
        APP_RefreshAdc(ADC_SAMPLES_DEFAULT);
    }

    const APP_Snapshot_t *s = APP_GetSnapshot();

    rsp[0] = PROTO_OK;
    rsp[1] = (uint8_t)((s->valid ? 0x01u : 0x00u) | 0x04u);   /* bit2 = VDDA 已校准 */
    rsp[2] = (uint8_t)ADC_CH_NUM;
    put_u16(&rsp[3], s->round);
    put_u16(&rsp[5], s->interval_ms);
    put_u16(&rsp[7], s->vdda_mv);
    put_u16(&rsp[9], (uint16_t)(((uint16_t)0x05u << 8) | ADC_SAMPLES_DEFAULT)); /* 指纹 */

    uint8_t *p = &rsp[11];
    for (uint8_t ch = 0; ch < ADC_CH_NUM; ch++) {
        put_u16(p, s->ch[ch].raw);
        put_u16(&p[2], BSP_ADC_InputMv(ch, s->ch[ch].mv));
        p[4] = s->ch[ch].flags;
        p += 5;
    }

    SendResponse(PROTO_CMD_MEASURE_ADC, rsp, sizeof(rsp));
}

/**
  * @brief  0x000F 读对射状态
  * @param  mode 0=读最近状态，1=立即采一次
  */
static void HandleSensorRead(const uint8_t *data, uint16_t len)
{
    uint8_t mode = (len >= 1u) ? data[0] : 0u;

    if (mode > 1u) {
        uint8_t e = PROTO_ERR_PARAM;
        SendResponse(PROTO_CMD_SENSOR_READ, &e, 1u);
        return;
    }
    if (mode == 1u) {
        BSP_Sensor_ForcePoll();
    }

    uint8_t rsp[SENSOR_ACK_BYTES];
    rsp[0] = PROTO_OK;
    rsp[1] = BSP_Sensor_GetFlags();

    uint8_t *p = &rsp[2];
    for (uint8_t i = 0; i < SENSOR_NUM; i++) {
        const SENSOR_State_t *s = BSP_Sensor_Get(i);
        put_u16(p, s->raw);
        put_u16(&p[2], s->mv);
        put_u16(&p[4], s->block_mv);
        put_u16(&p[6], s->release_mv);
        p[8] = s->blocked;
        p += 9;
    }

    SendResponse(PROTO_CMD_SENSOR_READ, rsp, sizeof(rsp));
}

/**
  * @brief  0x0021 设置对射阈值：[ch][block u16][release u16]
  */
static void HandleSensorConfig(const uint8_t *data, uint16_t len)
{
    uint8_t e = PROTO_OK;

    if (len < 5u) {
        e = PROTO_ERR_PARAM;
    } else {
        uint8_t  ch      = data[0];
        uint16_t block   = get_u16(&data[1]);
        uint16_t release = get_u16(&data[3]);

        if (ch >= SENSOR_NUM || release >= block || block > 3300u) {
            e = PROTO_ERR_PARAM;
        } else {
            BSP_Sensor_SetThreshold(ch, block, release);
        }
    }

    SendResponse(PROTO_CMD_SENSOR_CONFIG, &e, 1u);
}

/**
  * @brief  0x0030 自测
  */
static void HandleSelfTest(void)
{
    uint8_t rsp[SELFTEST_BYTES];
    uint8_t mask = 0u;

    ADC_Value_t v[ADC_CH_NUM];
    BSP_ADC_ReadAll(v, 4u);

    /* bit0: 8 路全部采到 */
    uint8_t adc_ok = 1u;
    uint8_t rail   = 0u;
    for (uint8_t ch = 0; ch < ADC_CH_NUM; ch++) {
        if ((v[ch].flags & ADC_FLAG_OK) == 0u) adc_ok = 0u;
        if (v[ch].flags & ADC_FLAG_RAIL) rail |= (uint8_t)(1u << ch);
    }
    if (adc_ok) mask |= 0x01u;
    if (v[SENSOR_CH1].flags & ADC_FLAG_OK) mask |= 0x02u;
    if (v[SENSOR_CH2].flags & ADC_FLAG_OK) mask |= 0x04u;

    uint16_t vdda = BSP_ADC_GetVDDA();
    if (vdda >= 3000u && vdda <= 3600u) mask |= 0x08u;

#if defined(BOARD_U2)
    if (BSP_TMC_IsAlive() && BSP_TMC_CfgOk()) mask |= 0x10u;
    if (!BSP_Motor_IsBusy()) mask |= 0x20u;
#else
    mask |= 0x20u;      /* U1 无电机，恒空闲 */
#endif
    if (uart1_fifo.overflow == 0u) mask |= 0x40u;
    if (crc_err_cnt == 0u) mask |= 0x80u;

    uint8_t pass = (mask & 0x7Fu) == 0x7Fu;   /* 除 bit7 外全通过才算 OK */

    rsp[0] = pass ? PROTO_OK : PROTO_ERR_TEST;
    rsp[1] = mask;
    put_u16(&rsp[2], vdda);
    rsp[4] = rail;
    rsp[5] = (uint8_t)((crc_err_cnt > 255u) ? 255u : crc_err_cnt);

    SendResponse(PROTO_CMD_SELF_TEST, rsp, sizeof(rsp));
}

#if defined(BOARD_U2)
/**
  * @brief  0x0010 电机移动：[dir(1)][steps(4)][freq_hz(2)]
  */
static void HandleMotorMove(const uint8_t *data, uint16_t len)
{
    if (len < 7u) {
        uint8_t e = PROTO_ERR_PARAM;
        SendResponse(PROTO_CMD_MOTOR_MOVE, &e, 1u);
        return;
    }

    uint8_t  dir   = data[0];
    uint32_t steps = get_u32(&data[1]);
    uint16_t freq  = get_u16(&data[5]);
    uint8_t  e;

    if (dir > 1u || steps == 0u || steps > 100000000UL) {
        e = PROTO_ERR_PARAM;
    } else if (freq < MOTOR_FREQ_MIN_HZ || freq > MOTOR_FREQ_MAX_HZ) {
        e = PROTO_ERR_PARAM;
    } else if (BSP_Motor_IsBusy()) {
        e = PROTO_BUSY;
    } else if (!BSP_Motor_Move(dir, steps, freq)) {
        e = PROTO_ERR_PARAM;
    } else {
        e = PROTO_OK;
    }

    SendResponse(PROTO_CMD_MOTOR_MOVE, &e, 1u);
}

static void HandleMotorStop(void)
{
    BSP_Motor_Stop();
    uint8_t e = PROTO_OK;
    SendResponse(PROTO_CMD_MOTOR_STOP, &e, 1u);
}

static void HandleMotorStatus(void)
{
    uint8_t rsp[MOTOR_STATUS_BYTES];
    rsp[0] = PROTO_OK;
    rsp[1] = BSP_Motor_IsBusy();
    put_u32(&rsp[2], BSP_Motor_GetDone());
    put_u32(&rsp[6], BSP_Motor_GetTarget());
    SendResponse(PROTO_CMD_MOTOR_STATUS, rsp, sizeof(rsp));
}

static void HandleTmcQuery(void)
{
    uint8_t  rsp[TMC_QUERY_BYTES];
    uint32_t gconf = 0u, chop = 0u, drv = 0u, sg = 0u;

    /* 读 TMC 需要约 4×1.1ms 的位操作，会挤占 STEP 中断 → 运动中拒绝 */
    if (BSP_Motor_IsBusy()) {
        uint8_t e = PROTO_BUSY;
        SendResponse(PROTO_CMD_TMC_QUERY, &e, 1u);
        return;
    }

    rsp[0] = PROTO_OK;
    rsp[1] = BSP_TMC_IsAlive();
    rsp[2] = BSP_TMC_CfgOk();

    if (rsp[1]) (void)BSP_TMC_ReadReg(0x00u, &gconf);   /* GCONF */
    if (rsp[1]) (void)BSP_TMC_ReadReg(0x6Cu, &chop);    /* CHOPCONF */
    if (rsp[1]) (void)BSP_TMC_ReadReg(0x6Fu, &drv);     /* DRV_STATUS */
    if (rsp[1]) (void)BSP_TMC_ReadReg(0x41u, &sg);      /* SG_RESULT */

    put_u32(&rsp[3], gconf);
    put_u32(&rsp[7], chop);
    put_u32(&rsp[11], drv);
    put_u16(&rsp[15], (uint16_t)(sg & 0x3FFu));

    SendResponse(PROTO_CMD_TMC_QUERY, rsp, sizeof(rsp));
}

/**
  * @brief  0x0022 在线调整电流：[irun][ihold]，0..31
  */
static void HandleTmcSetIrun(const uint8_t *data, uint16_t len)
{
    uint8_t e = PROTO_OK;

    if (len < 2u || data[0] > 31u || data[1] > 31u || data[1] > data[0]) {
        e = PROTO_ERR_PARAM;
    } else if (BSP_Motor_IsBusy()) {
        e = PROTO_BUSY;
    } else {
        BSP_TMC_SetCurrent(data[0], data[1]);
    }

    SendResponse(PROTO_CMD_TMC_SET_IRUN, &e, 1u);
}
#endif /* BOARD_U2 */

/*==============================================================================
 * 分发
 *============================================================================*/
static void Dispatch(uint16_t cmd, const uint8_t *data, uint16_t len)
{
    switch (cmd) {
    case PROTO_CMD_PING:          HandlePing();                 break;
    case PROTO_CMD_GET_VERSION:   HandleGetVersion();           break;
    case PROTO_CMD_GET_UID:       HandleGetUid();               break;
    case PROTO_CMD_MEASURE_ADC:   HandleMeasureAdc(data, len);  break;
    case PROTO_CMD_SENSOR_READ:   HandleSensorRead(data, len);  break;
    case PROTO_CMD_SENSOR_CONFIG: HandleSensorConfig(data, len);break;
    case PROTO_CMD_SELF_TEST:     HandleSelfTest();             break;

#if defined(BOARD_U2)
    case PROTO_CMD_MOTOR_MOVE:    HandleMotorMove(data, len);   break;
    case PROTO_CMD_MOTOR_STOP:    HandleMotorStop();            break;
    case PROTO_CMD_MOTOR_STATUS:  HandleMotorStatus();          break;
    case PROTO_CMD_TMC_QUERY:     HandleTmcQuery();             break;
    case PROTO_CMD_TMC_SET_IRUN:  HandleTmcSetIrun(data, len);  break;
#endif

    default: {
        uint8_t e = PROTO_NIY;
        SendResponse(cmd, &e, 1u);
        break;
    }
    }
}

/*==============================================================================
 * 主循环入口
 *============================================================================*/
void PROTO_Init(void)
{
    rx_state     = S_HDR0;
    rx_pos       = 0u;
    rx_dlen      = 0u;
    req_seq      = 0u;
    crc_err_cnt  = 0u;
    seq_warn_cnt = 0u;
    last_seq     = 0u;
}

void PROTO_Run(void)
{
    uint8_t byte;

    while (UART_ReadByte(&uart1_fifo, &byte)) {
        switch (rx_state) {
        case S_HDR0:
            if (byte == FRAME_HDR0) { rx_buf[rx_pos++] = byte; rx_state = S_HDR1; }
            break;

        case S_HDR1:
            if (byte == FRAME_HDR1) {
                rx_buf[rx_pos++] = byte;
                rx_state = S_CMD_H;
            } else {
                rx_pos = 0u;
                rx_state = (byte == FRAME_HDR0) ? S_HDR1 : S_HDR0;
                if (byte == FRAME_HDR0) rx_buf[rx_pos++] = byte;
            }
            break;

        case S_CMD_H:  rx_buf[rx_pos++] = byte; rx_state = S_CMD_L;  break;
        case S_CMD_L:  rx_buf[rx_pos++] = byte; rx_state = S_SEQ_H;  break;
        case S_SEQ_H:  rx_buf[rx_pos++] = byte; rx_state = S_SEQ_L;  break;
        case S_SEQ_L:  rx_buf[rx_pos++] = byte; rx_state = S_LEN_H;  break;
        case S_LEN_H:  rx_buf[rx_pos++] = byte; rx_state = S_LEN_L;  break;

        case S_LEN_L: {
            rx_buf[rx_pos++] = byte;
            rx_dlen = (uint16_t)(((uint16_t)rx_buf[6] << 8) | byte);
            if (rx_dlen > FRAME_MAX_DATA) {          /* 长度非法：丢帧重来 */
                rx_pos = 0u;
                rx_state = S_HDR0;
            } else {
                rx_state = (rx_dlen > 0u) ? S_DATA : S_CRC_H;
            }
            break;
        }

        case S_DATA:
            rx_buf[rx_pos++] = byte;
            if (rx_pos >= (uint16_t)(8u + rx_dlen)) rx_state = S_CRC_H;
            break;

        case S_CRC_H:  rx_buf[rx_pos++] = byte; rx_state = S_CRC_L; break;

        case S_CRC_L: {
            rx_buf[rx_pos++] = byte;

            uint16_t cmd, seq, len;
            if (Frame_Parse(rx_buf, rx_pos, &cmd, &seq, frame_mem, &len)) {
                req_seq = seq;
                if (seq != 0u && last_seq != 0u && seq <= last_seq) {
                    seq_warn_cnt++;                  /* 仅告警，不拒绝 */
                }
                last_seq = seq;
                Dispatch(cmd, frame_mem, len);
            } else {
                crc_err_cnt++;                       /* CRC 错：不回 ACK */
            }

            rx_pos = 0u;
            rx_state = S_HDR0;
            break;
        }

        default:
            rx_pos = 0u;
            rx_state = S_HDR0;
            break;
        }
    }
}
