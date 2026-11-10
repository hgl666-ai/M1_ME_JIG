/**
  ******************************************************************************
  * @file    bsp_adc.c
  * @brief   ADC1 单通道轮询采集（8 路测试点）+ VDDA 校准
  *
  * 与参考工程 diameter_JIG/U1 的 bsp_adc.c 思路一致：
  *   1) 置 TSVREFE 打开内部参考
  *   2) 执行 ADC 自校准
  *   3) 读 VREFINT 原始值，用出厂校准值反算真实 VDDA
  *
  * 通道映射来自原理图（见 app_config.h 顶部说明）：
  *   U1: ADC0..7 → PA3,PA4,PA5,PA6,PA7,PB0,PB1,PA2
  *   U2: ADC0..7 → PB0,PA7,PA6,PA5,PA4,PA3,PA2,PB1
  ******************************************************************************
  */
#include "bsp_adc.h"
#include "adc.h"          /* hadc1 */

#define VREFINT_CAL_ADDR   ((uint16_t *)0x1FFFF7BA)

/* 逻辑通道 → ADC1 通道号 */
#if defined(BOARD_U1)
static const uint32_t adc_ch_map[ADC_CH_NUM] = {
    ADC_CHANNEL_3,   /* ADC0 PA3  */
    ADC_CHANNEL_4,   /* ADC1 PA4  */
    ADC_CHANNEL_5,   /* ADC2 PA5  */
    ADC_CHANNEL_6,   /* ADC3 PA6  */
    ADC_CHANNEL_7,   /* ADC4 PA7  */
    ADC_CHANNEL_8,   /* ADC5 PB0  */
    ADC_CHANNEL_9,   /* ADC6 PB1  */
    ADC_CHANNEL_2    /* ADC7 PA2  */
};
#else
static const uint32_t adc_ch_map[ADC_CH_NUM] = {
    ADC_CHANNEL_8,   /* ADC0 PB0  */
    ADC_CHANNEL_7,   /* ADC1 PA7  */
    ADC_CHANNEL_6,   /* ADC2 PA6  */
    ADC_CHANNEL_5,   /* ADC3 PA5  */
    ADC_CHANNEL_4,   /* ADC4 PA4  */
    ADC_CHANNEL_3,   /* ADC5 PA3  */
    ADC_CHANNEL_2,   /* ADC6 PA2  */
    ADC_CHANNEL_9    /* ADC7 PB1  */
};
#endif

static uint16_t vdda_mv = 3300u;

/*==============================================================================
 * 内部：单次转换
 *============================================================================*/
uint8_t BSP_ADC_ReadRaw(uint32_t channel, uint16_t *raw)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    sConfig.Channel      = channel;
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) return 0u;

    if (HAL_ADC_Start(&hadc1) != HAL_OK) return 0u;

    if (HAL_ADC_PollForConversion(&hadc1, 10u) != HAL_OK) {
        (void)HAL_ADC_Stop(&hadc1);
        return 0u;
    }

    *raw = (uint16_t)HAL_ADC_GetValue(&hadc1);
    (void)HAL_ADC_Stop(&hadc1);
    return 1u;
}

/*==============================================================================
 * 公开接口
 *============================================================================*/

/**
  * @brief  ADC 初始化：打开 VREFINT、自校准、反算 VDDA
  * @note   在 MX_ADC1_Init() 之后调用
  */
void BSP_ADC_Init(void)
{
    SET_BIT(ADC1->CR2, ADC_CR2_TSVREFE);
    HAL_Delay(1u);
    (void)HAL_ADCEx_Calibration_Start(&hadc1);

    uint16_t vref_raw = 0u;
    if (BSP_ADC_ReadRaw(ADC_CHANNEL_VREFINT, &vref_raw) && vref_raw > 0u) {
        uint16_t vref_cal = *VREFINT_CAL_ADDR;     /* 3.3V 下的出厂校准值 */
        uint32_t v = (3300UL * (uint32_t)vref_cal) / (uint32_t)vref_raw;
        if (v >= 2500u && v <= 3700u) {            /* 合理性保护 */
            vdda_mv = (uint16_t)v;
        }
    }
}

uint16_t BSP_ADC_GetVDDA(void)
{
    return vdda_mv;
}

/**
  * @brief  读取一个逻辑通道（多点平均）
  * @param  ch:      0..7
  * @param  samples: 平均次数 1..32
  */
ADC_Value_t BSP_ADC_Read(uint8_t ch, uint8_t samples)
{
    ADC_Value_t v;
    v.raw = 0u;
    v.mv  = 0u;
    v.flags = 0u;

    if (ch >= ADC_CH_NUM) return v;
    if (samples < ADC_SAMPLES_MIN) samples = ADC_SAMPLES_MIN;
    if (samples > ADC_SAMPLES_MAX) samples = ADC_SAMPLES_MAX;

    uint32_t sum = 0u;
    uint16_t raw = 0u;
    for (uint8_t i = 0; i < samples; i++) {
        if (!BSP_ADC_ReadRaw(adc_ch_map[ch], &raw)) return v;  /* 任一次失败 → 无效 */
        sum += raw;
    }

    v.raw = (uint16_t)((sum + samples / 2u) / samples);
    v.mv  = (uint16_t)(((uint32_t)v.raw * vdda_mv) / 4095UL);
    v.flags = ADC_FLAG_OK;
    if (v.raw <= 4u || v.raw >= 4091u) v.flags |= ADC_FLAG_RAIL;
    if (ch == SENSOR_CH1 || ch == SENSOR_CH2) v.flags |= ADC_FLAG_SENSOR;
    return v;
}

/**
  * @brief  引脚侧 mV → 输入侧（还原分压）
  */
uint16_t BSP_ADC_InputMv(uint8_t ch, uint16_t pin_mv)
{
    if (ch >= ADC_CH_NUM) return pin_mv;
    return (uint16_t)(((uint32_t)pin_mv * ADC_CH_DIV_NUM(ch)) / ADC_CH_DIV_DEN(ch));
}

/**
  * @brief  一次采完 8 路
  */
void BSP_ADC_ReadAll(ADC_Value_t *out, uint8_t samples)
{
    for (uint8_t ch = 0; ch < ADC_CH_NUM; ch++) {
        out[ch] = BSP_ADC_Read(ch, samples);
    }
}
