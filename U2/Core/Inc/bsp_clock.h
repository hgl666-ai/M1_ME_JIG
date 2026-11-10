/**
  ******************************************************************************
  * @file    bsp_clock.h
  * @brief   时钟兜底：确保系统跑在 HSE 8MHz × 8 = 64MHz
  *
  * 背景：CubeMX 生成的 SystemClock_Config() 用的是 HSI/2×16 = 64MHz，
  *       而 460800 波特率对时钟精度敏感（HSI 只有 ±1%，HSE 晶振 ≈ ±20ppm）。
  *       本函数在 USER CODE SysInit 里调用：若当前不是 HSE+PLL 就切过去，
  *       晶振不起振则保持原样（不阻塞启动）。
  *       这样即使以后用 CubeMX 重新生成工程（会覆盖 SystemClock_Config），
  *       时钟仍然会被纠正回 HSE。
  ******************************************************************************
  */
#ifndef __BSP_CLOCK_H
#define __BSP_CLOCK_H

#include "main.h"

void BSP_Clock_Init(void);

#endif /* __BSP_CLOCK_H */
