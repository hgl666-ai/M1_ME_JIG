/* 主机端协议层测试的桩实现（不参与固件编译） */
#include "main.h"
#include "bsp_uart.h"
#include "app.h"
#include "bsp_adc.h"
#include "bsp_sensor.h"

#if defined(BOARD_U2)
#include "bsp_motor.h"
#include "bsp_tmc2209.h"
#endif

UART_HandleTypeDef huart1;
static GPIO_TypeDef ga, gb, gc;
GPIO_TypeDef *const GPIOA_FAKE = &ga;
GPIO_TypeDef *const GPIOB_FAKE = &gb;
GPIO_TypeDef *const GPIOC_FAKE = &gc;

/* ---- 串口桩：从测试喂入缓冲读，输出写入捕获缓冲 ---- */
UART_FIFO_t uart1_fifo;

static const uint8_t *g_in;
static int            g_in_len;
static int            g_in_pos;

uint8_t g_out[4096];
int     g_out_len;

void test_feed(const uint8_t *buf, int len)
{
    g_in = buf; g_in_len = len; g_in_pos = 0;
}

void test_reset_out(void)
{
    g_out_len = 0;
}

uint8_t UART_ReadByte(UART_FIFO_t *fifo, uint8_t *pData)
{
    (void)fifo;
    if (g_in_pos >= g_in_len) return 0u;
    *pData = g_in[g_in_pos++];
    return 1u;
}

uint16_t UART_GetUnreadLen(UART_FIFO_t *fifo) { (void)fifo; return 0u; }
void UART_ClearBuffer(UART_FIFO_t *fifo) { (void)fifo; }
void BSP_UART_DMA_Sync(UART_FIFO_t *fifo) { (void)fifo; }

void UART_SendArray(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t len)
{
    (void)huart;
    for (uint16_t i = 0; i < len && g_out_len < (int)sizeof(g_out); i++) {
        g_out[g_out_len++] = pData[i];
    }
}

/* ---- ADC 桩：每通道固定 raw = 1000 + ch*100 ---- */
static uint16_t g_vdda = 3300u;

void BSP_ADC_Init(void) {}
uint16_t BSP_ADC_GetVDDA(void) { return g_vdda; }
uint8_t  BSP_ADC_ReadRaw(uint32_t channel, uint16_t *raw) { (void)channel; *raw = 1000u; return 1u; }

ADC_Value_t BSP_ADC_Read(uint8_t ch, uint8_t samples)
{
    (void)samples;
    ADC_Value_t v;
    v.raw = (uint16_t)(1000u + ch * 100u);
    v.mv  = (uint16_t)(v.raw * 3300u / 4095u);
    v.flags = ADC_FLAG_OK | ((ch == SENSOR_CH1 || ch == SENSOR_CH2) ? ADC_FLAG_SENSOR : 0u);
    return v;
}

void BSP_ADC_ReadAll(ADC_Value_t *out, uint8_t samples)
{
    for (uint8_t ch = 0; ch < ADC_CH_NUM; ch++) out[ch] = BSP_ADC_Read(ch, samples);
}

uint16_t BSP_ADC_InputMv(uint8_t ch, uint16_t pin_mv)
{
    return (uint16_t)(((uint32_t)pin_mv * ADC_CH_DIV_NUM(ch)) / ADC_CH_DIV_DEN(ch));
}

/* ---- 对射桩 ---- */
static SENSOR_State_t g_sensor[SENSOR_NUM];

void BSP_Sensor_Init(void) {}
void BSP_Sensor_Poll(void) {}
void BSP_Sensor_ForcePoll(void) {}

const SENSOR_State_t *BSP_Sensor_Get(uint8_t idx)
{
    if (idx >= SENSOR_NUM) return NULL;
    g_sensor[idx].raw = 2000u;
    g_sensor[idx].mv = 1610u;
    g_sensor[idx].block_mv = 2000u;
    g_sensor[idx].release_mv = 1700u;
    g_sensor[idx].blocked = idx;      /* ch0 未遮挡、ch1 遮挡，便于断言 */
    g_sensor[idx].valid = 1u;
    return &g_sensor[idx];
}

uint8_t BSP_Sensor_GetFlags(void) { return 0x05u; }   /* valid + ch2 blocked */
uint8_t BSP_Sensor_IsBlocked(uint8_t idx) { return (idx == 1u) ? 1u : 0u; }
void    BSP_Sensor_SetThreshold(uint8_t idx, uint16_t b, uint16_t r) { (void)idx; (void)b; (void)r; }

/* ---- APP 桩 ---- */
static APP_Snapshot_t g_snap;

void APP_Init(void)
{
    g_snap.round = 7u;
    g_snap.interval_ms = 12u;
    g_snap.vdda_mv = 3290u;
    g_snap.valid = 1u;
    for (uint8_t i = 0; i < ADC_CH_NUM; i++) g_snap.ch[i] = BSP_ADC_Read(i, 1u);
}

void APP_Poll(void) {}
void APP_RefreshAdc(uint8_t samples) { (void)samples; g_snap.round++; }
const APP_Snapshot_t *APP_GetSnapshot(void) { return &g_snap; }
uint32_t APP_GetUptimeMs(void) { return 0u; }

#if defined(BOARD_U2)
/* ---- 电机 / TMC 桩 ---- */
static uint8_t  g_busy;
static uint32_t g_done, g_target;
static uint8_t  g_irun = TMC_DEFAULT_IRUN, g_ihold = TMC_DEFAULT_IHOLD;

uint8_t  BSP_Motor_IsBusy(void) { return g_busy; }
uint32_t BSP_Motor_GetDone(void) { return g_done; }
uint32_t BSP_Motor_GetTarget(void) { return g_target; }
void     BSP_Motor_Stop(void) { g_busy = 0u; }
uint8_t  BSP_Motor_Move(uint8_t dir, uint32_t steps, uint32_t freq)
{
    (void)dir;
    if (g_busy) return 0u;
    g_done = 0u; g_target = steps; g_busy = 1u;
    (void)freq;
    return 1u;
}
void test_motor_set_busy(uint8_t b) { g_busy = b; }

uint8_t BSP_TMC_IsAlive(void) { return 1u; }
uint8_t BSP_TMC_CfgOk(void) { return 1u; }
uint8_t BSP_TMC_ReadReg(uint8_t reg, uint32_t *val) { *val = 0x11223344u + reg; return 1u; }
uint8_t BSP_TMC_WriteReg(uint8_t reg, uint32_t val) { (void)reg; (void)val; return 1u; }
void    BSP_TMC_SetCurrent(uint8_t irun, uint8_t ihold) { g_irun = irun; g_ihold = ihold; }
uint8_t BSP_TMC_GetIrun(void) { return g_irun; }
uint8_t BSP_TMC_GetIhold(void) { return g_ihold; }
#endif
