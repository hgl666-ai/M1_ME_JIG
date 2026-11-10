/**
  ******************************************************************************
  * @file    app.h
  * @brief   应用层：ADC 快照缓存、对射轮询、系统灯
  ******************************************************************************
  */
#ifndef __APP_H
#define __APP_H

#include "main.h"
#include "app_config.h"
#include "bsp_adc.h"

#define APP_ADC_REFRESH_MS   200u   /* 后台 ADC 快照刷新周期 */

typedef struct {
    ADC_Value_t ch[ADC_CH_NUM];
    uint16_t    round;          /* 回合号：每刷新一次 +1 */
    uint16_t    interval_ms;    /* 本次 8 路采集耗时 */
    uint16_t    vdda_mv;
    uint8_t     valid;          /* 1 = 至少完整采过一轮 */
} APP_Snapshot_t;

void                   APP_Init(void);
void                   APP_Poll(void);
void                   APP_RefreshAdc(uint8_t samples);
const APP_Snapshot_t  *APP_GetSnapshot(void);
uint32_t               APP_GetUptimeMs(void);

#endif /* __APP_H */
