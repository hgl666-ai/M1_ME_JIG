/**
  ******************************************************************************
  * @file    test_sensor.c
  * @brief   对射判定（bsp_sensor.c 真实代码）主机端回归测试
  *
  * 校验：遮挡/释放阈值回差、连续 3 次去抖、阈值参数保护。
  ******************************************************************************
  */
#include "main.h"
#include "bsp_sensor.h"
#include "bsp_adc.h"

#include <stdio.h>

static int g_fail;
static int g_checks;

#define CHECK(cond, msg) do { \
    g_checks++; \
    if (!(cond)) { g_fail++; printf("FAIL: %s (%d)\n", msg, __LINE__); } \
} while (0)

/* ---- HAL 桩 ---- */
static uint32_t g_tick;
uint32_t HAL_GetTick(void) { return g_tick; }

/* ---- ADC 桩：通道1/2 返回设定电压，其余 0 ---- */
static uint16_t g_mv_ch1;
static uint16_t g_mv_ch2;

ADC_Value_t BSP_ADC_Read(uint8_t ch, uint8_t samples)
{
    ADC_Value_t v;
    v.raw = 0u;
    v.mv = 0u;
    v.flags = 0u;
    (void)samples;

    if (ch == SENSOR_CH1)      { v.mv = g_mv_ch1; v.flags = ADC_FLAG_OK | ADC_FLAG_SENSOR; }
    else if (ch == SENSOR_CH2) { v.mv = g_mv_ch2; v.flags = ADC_FLAG_OK | ADC_FLAG_SENSOR; }
    else { return v; }
    v.raw = (uint16_t)((uint32_t)v.mv * 4095u / 3300u);
    return v;
}

static void feed(uint16_t mv, int n)
{
    g_mv_ch1 = mv;
    for (int i = 0; i < n; i++) {
        g_tick += 10u;
        BSP_Sensor_Poll();
    }
}

int main(void)
{
    printf("=== sensor hysteresis/debounce test ===\n");

    BSP_Sensor_Init();

    /* 默认阈值 2000/1700，去抖 3 次 */
    const SENSOR_State_t *s = BSP_Sensor_Get(0);
    CHECK(s != NULL, "get ch1");
    CHECK(s->block_mv == 2000u && s->release_mv == 1700u, "default thresholds");

    /* 低于遮挡阈值：不翻转 */
    feed(1900u, 5);
    CHECK(BSP_Sensor_IsBlocked(0) == 0u, "below block -> clear");

    /* 高于遮挡阈值 2 次：去抖未满，不翻转 */
    feed(2100u, 2);
    CHECK(BSP_Sensor_IsBlocked(0) == 0u, "2 high samples -> still clear");

    /* 第 3 次：翻转 */
    feed(2100u, 1);
    CHECK(BSP_Sensor_IsBlocked(0) == 1u, "3 high samples -> blocked");

    /* 回差区间（1800 介于 1700 和 2000）：保持遮挡 */
    feed(1800u, 5);
    CHECK(BSP_Sensor_IsBlocked(0) == 1u, "hysteresis keeps blocked");

    /* 低于释放阈值 2 次：仍遮挡 */
    feed(1600u, 2);
    CHECK(BSP_Sensor_IsBlocked(0) == 1u, "2 low samples -> still blocked");

    /* 第 3 次：恢复 */
    feed(1600u, 1);
    CHECK(BSP_Sensor_IsBlocked(0) == 0u, "3 low samples -> clear");

    /* 中间夹杂一次高电平会清零去抖计数 */
    feed(2100u, 2);
    feed(1000u, 1);          /* 打断 */
    feed(2100u, 2);
    CHECK(BSP_Sensor_IsBlocked(0) == 0u, "interrupted debounce resets");

    /* 通道标志 */
    CHECK((BSP_Sensor_GetFlags() & 0x03u) == 0x01u, "flags valid, ch1 clear");

    /* 阈值参数保护：release >= block 被拒绝 */
    BSP_Sensor_SetThreshold(0u, 1500u, 1800u);
    s = BSP_Sensor_Get(0);
    CHECK(s->block_mv == 2000u && s->release_mv == 1700u, "invalid threshold rejected");

    /* 合法阈值生效 */
    BSP_Sensor_SetThreshold(0u, 2500u, 2200u);
    CHECK(s->block_mv == 2500u && s->release_mv == 2200u, "valid threshold applied");

    /* 用新阈值重测：2100 不再触发遮挡 */
    feed(2100u, 5);
    CHECK(BSP_Sensor_IsBlocked(0) == 0u, "new threshold takes effect");

    feed(2600u, 3);
    CHECK(BSP_Sensor_IsBlocked(0) == 1u, "new threshold triggers");

    printf("%d checks, %d failed\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
