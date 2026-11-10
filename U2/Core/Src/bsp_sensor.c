/**
  ******************************************************************************
  * @file    bsp_sensor.c
  * @brief   对射检测：阈值 + 回差 + 连续 N 次去抖
  ******************************************************************************
  */
#include "bsp_sensor.h"
#include "bsp_adc.h"

static SENSOR_State_t sensor[SENSOR_NUM];
static uint32_t       last_poll_ms;

static const uint8_t sensor_ch[SENSOR_NUM] = { SENSOR_CH1, SENSOR_CH2 };

void BSP_Sensor_Init(void)
{
    for (uint8_t i = 0; i < SENSOR_NUM; i++) {
        sensor[i].raw        = 0u;
        sensor[i].mv         = 0u;
        sensor[i].block_mv   = SENSOR_BLOCK_MV_DEFAULT;
        sensor[i].release_mv = SENSOR_RELEASE_MV_DEFAULT;
        sensor[i].blocked    = 0u;
        sensor[i].valid      = 0u;
        sensor[i].debounce   = SENSOR_DEBOUNCE_DEFAULT;
        sensor[i].cnt        = 0u;
    }
    last_poll_ms = 0u;
}

/**
  * @brief  周期采样（10ms），带阈值回差与去抖
  * @note   单次采样约 24µs/通道，10ms 周期下 CPU 占用可忽略
  */
void BSP_Sensor_Poll(void)
{
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - last_poll_ms) < ADC_SENSOR_PERIOD_MS) return;
    last_poll_ms = now;

    for (uint8_t i = 0; i < SENSOR_NUM; i++) {
        ADC_Value_t v = BSP_ADC_Read(sensor_ch[i], 1u);
        if ((v.flags & ADC_FLAG_OK) == 0u) {
            sensor[i].valid = 0u;      /* 采样失败：保留上次结论，只清有效位 */
            continue;
        }

        sensor[i].raw   = v.raw;
        sensor[i].mv    = v.mv;
        sensor[i].valid = 1u;

        if (sensor[i].blocked == 0u) {
            /* 未遮挡 → 只有连续 N 次高于遮挡阈值才翻转 */
            if (v.mv >= sensor[i].block_mv) {
                if (sensor[i].cnt < 255u) sensor[i].cnt++;
                if (sensor[i].cnt >= sensor[i].debounce) {
                    sensor[i].blocked = 1u;
                    sensor[i].cnt = 0u;
                }
            } else {
                sensor[i].cnt = 0u;
            }
        } else {
            /* 已遮挡 → 只有连续 N 次低于释放阈值才翻转 */
            if (v.mv <= sensor[i].release_mv) {
                if (sensor[i].cnt < 255u) sensor[i].cnt++;
                if (sensor[i].cnt >= sensor[i].debounce) {
                    sensor[i].blocked = 0u;
                    sensor[i].cnt = 0u;
                }
            } else {
                sensor[i].cnt = 0u;
            }
        }
    }
}

/**
  * @brief  忽略 10ms 限速，立即采样一次（供协议 mode=1 使用）
  */
void BSP_Sensor_ForcePoll(void)
{
    last_poll_ms = HAL_GetTick() - ADC_SENSOR_PERIOD_MS;
    BSP_Sensor_Poll();
}

const SENSOR_State_t *BSP_Sensor_Get(uint8_t idx)
{
    if (idx >= SENSOR_NUM) return NULL;
    return &sensor[idx];
}

uint8_t BSP_Sensor_IsBlocked(uint8_t idx)
{
    if (idx >= SENSOR_NUM) return 0u;
    return sensor[idx].blocked;
}

/**
  * @brief  设置阈值（block > release，保证回差；非法参数返回忽略）
  */
void BSP_Sensor_SetThreshold(uint8_t idx, uint16_t block_mv, uint16_t release_mv)
{
    if (idx >= SENSOR_NUM) return;
    if (block_mv > 3300u || release_mv > 3300u) return;
    if (release_mv >= block_mv) return;         /* 回差必须为正 */
    sensor[idx].block_mv   = block_mv;
    sensor[idx].release_mv = release_mv;
    sensor[idx].cnt        = 0u;
}

uint8_t BSP_Sensor_GetFlags(void)
{
    uint8_t f = 0u;
    if (sensor[0].valid && sensor[1].valid) f |= SENSOR_FLAG_VALID;
    if (sensor[0].blocked) f |= 0x02u;
    if (sensor[1].blocked) f |= 0x04u;
    return f;
}
