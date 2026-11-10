/**
  ******************************************************************************
  * @file    app.c
  * @brief   应用层实现：后台 ADC 快照 + 对射轮询 + 系统灯心跳
  ******************************************************************************
  */
#include "app.h"
#include "bsp_sensor.h"

static APP_Snapshot_t snapshot;
static uint32_t       last_adc_ms;
static uint32_t       last_led_ms;
static uint8_t        led_on;

/**
  * @brief  应用初始化（在 BSP_ADC_Init 之前调用 BSP_ADC_Init 自身在 main 中）
  */
void APP_Init(void)
{
    BSP_Sensor_Init();

    snapshot.round       = 0u;
    snapshot.interval_ms = 0u;
    snapshot.vdda_mv     = BSP_ADC_GetVDDA();
    snapshot.valid       = 0u;
    for (uint8_t i = 0; i < ADC_CH_NUM; i++) {
        snapshot.ch[i].raw   = 0u;
        snapshot.ch[i].mv    = 0u;
        snapshot.ch[i].flags = 0u;
    }

    last_adc_ms = HAL_GetTick();
    last_led_ms = HAL_GetTick();
    led_on      = 0u;
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);   /* 灭（低电平点亮） */

    APP_RefreshAdc(ADC_SAMPLES_DEFAULT);
}

/**
  * @brief  立即采一轮 8 路并更新快照
  */
void APP_RefreshAdc(uint8_t samples)
{
    uint32_t t0 = HAL_GetTick();

    for (uint8_t ch = 0; ch < ADC_CH_NUM; ch++) {
        snapshot.ch[ch] = BSP_ADC_Read(ch, samples);
    }

    snapshot.vdda_mv     = BSP_ADC_GetVDDA();
    snapshot.interval_ms = (uint16_t)(HAL_GetTick() - t0);
    snapshot.round++;
    snapshot.valid = 1u;
}

const APP_Snapshot_t *APP_GetSnapshot(void)
{
    return &snapshot;
}

uint32_t APP_GetUptimeMs(void)
{
    return HAL_GetTick();
}

/**
  * @brief  主循环轮询：对射采样(10ms) + ADC 快照(200ms) + 心跳灯(1Hz)
  */
void APP_Poll(void)
{
    uint32_t now = HAL_GetTick();

    BSP_Sensor_Poll();

    if ((uint32_t)(now - last_adc_ms) >= APP_ADC_REFRESH_MS) {
        last_adc_ms = now;
        APP_RefreshAdc(ADC_SAMPLES_DEFAULT);
    }

    /* 心跳：每 1s 亮 60ms，用于判断固件活着（PC13 低电平点亮） */
    if (led_on == 0u) {
        if ((uint32_t)(now - last_led_ms) >= 940u) {
            led_on = 1u;
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
        }
    } else {
        if ((uint32_t)(now - last_led_ms) >= 1000u) {
            led_on = 0u;
            last_led_ms = now;
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
        }
    }
}
