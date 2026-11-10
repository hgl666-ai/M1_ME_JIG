/**
  ******************************************************************************
  * @file    bsp_motor.h
  * @brief   步进电机驱动（TMC2209 STEP/DIR/EN）—— 仅 U2
  *
  * 引脚：EN=PA0（低有效）、DIR=PB8、STEP=PB9。
  * 注意：STM32F103C6 属于小容量型号，只有 TIM1/TIM2/TIM3，**没有 TIM4**，
  *       而 PB9 只有 TIM4_CH4 一个复用功能 → 无法用硬件 PWM 发 STEP。
  *       因此采用 TIM2 定时中断 + GPIO 翻转的方式产生 STEP 脉冲，
  *       每个中断翻转一次，上升沿计一步（等价 50% 占空比 PWM）。
  ******************************************************************************
  */
#ifndef __BSP_MOTOR_H
#define __BSP_MOTOR_H

#include "main.h"
#include "app_config.h"

#define MOTOR_DIR_CW    0u
#define MOTOR_DIR_CCW   1u

void     BSP_Motor_Init(void);
uint8_t  BSP_Motor_Move(uint8_t dir, uint32_t steps, uint32_t freq_hz);
void     BSP_Motor_Stop(void);
uint8_t  BSP_Motor_IsBusy(void);
uint32_t BSP_Motor_GetDone(void);
uint32_t BSP_Motor_GetTarget(void);
void     BSP_Motor_SetEnable(uint8_t en);   /* 1=使能（EN 低），0=脱机 */

#endif /* __BSP_MOTOR_H */
