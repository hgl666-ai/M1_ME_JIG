/**
  ******************************************************************************
  * @file    bsp_sensor.h
  * @brief   对射传感器检测（治具侧 PT334 光电三极管 → ADC4/ADC5）
  *
  * 硬件：PT334-6B 集电极经 20kΩ 上拉到 3.3V，发射极接地。
  *   有光（对射通路正常）→ 三极管导通 → 集电极电压低
  *   被遮挡            → 三极管截止 → 集电极电压接近 3.3V
  * 因此“遮挡/触发”判据是电压 **高于** 阈值；用回差 + 去抖防止临界抖动。
  ******************************************************************************
  */
#ifndef __BSP_SENSOR_H
#define __BSP_SENSOR_H

#include "main.h"
#include "app_config.h"

#define SENSOR_FLAG_VALID   0x01u
#define SENSOR_FLAG_BLOCKED 0x02u

typedef struct {
    uint16_t raw;
    uint16_t mv;
    uint16_t block_mv;
    uint16_t release_mv;
    uint8_t  blocked;      /* 1 = 光路被遮挡 */
    uint8_t  valid;
    uint8_t  debounce;
    uint8_t  cnt;
} SENSOR_State_t;

void                  BSP_Sensor_Init(void);
void                  BSP_Sensor_Poll(void);            /* 主循环调用，内部 10ms 限速 */
void                  BSP_Sensor_ForcePoll(void);       /* 忽略限速立即采一次（协议 mode=1） */
const SENSOR_State_t *BSP_Sensor_Get(uint8_t idx);      /* idx: 0..SENSOR_NUM-1 */
uint8_t               BSP_Sensor_IsBlocked(uint8_t idx);
void                  BSP_Sensor_SetThreshold(uint8_t idx, uint16_t block_mv, uint16_t release_mv);
uint8_t               BSP_Sensor_GetFlags(void);        /* bit0 valid, bit1 ch1 blocked, bit2 ch2 blocked */

#endif /* __BSP_SENSOR_H */
