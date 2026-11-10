/**
  ******************************************************************************
  * @file    bsp_adc.h
  * @brief   ADC1 八路测试点采集（含 VREFINT 反算 VDDA、分压还原）
  ******************************************************************************
  */
#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#include "main.h"
#include "app_config.h"

#define ADC_SAMPLES_MIN   1u
#define ADC_SAMPLES_MAX   32u

/* 通道标志 */
#define ADC_FLAG_OK       0x01u
#define ADC_FLAG_RAIL     0x02u   /* 贴边：raw ≤4 或 ≥4091（开路/短路/超量程） */
#define ADC_FLAG_SENSOR   0x04u   /* 该通道属于对射接收通道 */

typedef struct {
    uint16_t raw;
    uint16_t mv;        /* 引脚侧电压（按校准后的 VDDA 换算） */
    uint8_t  flags;
} ADC_Value_t;

void        BSP_ADC_Init(void);
uint16_t    BSP_ADC_GetVDDA(void);
uint8_t     BSP_ADC_ReadRaw(uint32_t channel, uint16_t *raw);
ADC_Value_t BSP_ADC_Read(uint8_t ch, uint8_t samples);
uint16_t    BSP_ADC_InputMv(uint8_t ch, uint16_t pin_mv);
void        BSP_ADC_ReadAll(ADC_Value_t *out, uint8_t samples);

#endif /* __BSP_ADC_H */
