/**
  ******************************************************************************
  * @file    app_config.h
  * @brief   M1 治具 —— 板级配置 / 逻辑通道映射 / 协议常量
  *
  * 本文件与 protocol_frame / bsp_uart / bsp_adc / bsp_sensor / protocol / app
  * 一起在 U1、U2 两个工程中保持**逐字节一致**；两板的差异只允许出现在
  * 下面的 BOARD_U1 / BOARD_U2 条件编译块里。
  *
  * 硬件依据：IINFO/SCH_线材挤出机-入料传感器+磁编码器-测试治具_2026-11-05.pdf
  *   U1（主控1）：USART1 = PB6(TX)/PB7(RX) → H1；ADC0..ADC7 = PA2..PA7/PB0/PB1
  *   U2（主控2）：USART1 = PB6(TX)/PB7(RX) → H3；ADC0..ADC7 = PB0/PB1/PA2..PA7
  *                TMC2209：EN=PA0、DIR=PB8、STEP=PB9、单线 UART=PA9/PA10（软件 UART）
  ******************************************************************************
  */
#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

#include "main.h"

#if !defined(BOARD_U1) && !defined(BOARD_U2)
#error "必须在 Keil 工程 C/C++ Define 中定义 BOARD_U1 或 BOARD_U2"
#endif

/*===== 固件版本（协议 0x0002 查询）=====*/
#define FW_VER_MAJOR   1u
#define FW_VER_MINOR   0u
#define FW_VER_PATCH   0u

/*===== PC 链路 =====*/
#define PC_UART_BAUD   115200u    /* 默认 115200（与现有治具/串口助手统一）：协议表原写 460800，改这一行即可切回 */

/*===== 采样参数 =====*/
#define ADC_SAMPLES_DEFAULT   8u  /* 每通道默认平均次数 */
#define ADC_SENSOR_PERIOD_MS  10u /* 对射通道后台采样周期 */

/*===== 逻辑通道定义 =====
 * 8 路测试点（网络名 → 功能，见原理图“测试点功能”表）
 *   ADC0 : 板载 3.3V      （直连，无分压）
 *   ADC1 : 3.6V           （直连）
 *   ADC2 : 板载 5V        （R20=10k 下拉 / R24=15k 上拉 → 引脚 = 5V×10/25 = 0.4）
 *   ADC3 : 磁编码反馈
 *   ADC4 : 发光二极管检测1 （治具侧 PT334 光电三极管，对射通道 1）
 *   ADC5 : 发光二极管检测2 （治具侧 PT334 光电三极管，对射通道 2）
 *   ADC6 : 光电三极管检测1 （被测板光电三极管输出）
 *   ADC7 : 光电三极管检测2 （被测板光电三极管输出）
 */
enum {
    ADC_CH_3V3_BOARD = 0,
    ADC_CH_3V6,
    ADC_CH_5V_BOARD,
    ADC_CH_ENC,
    ADC_CH_OPTO1,
    ADC_CH_OPTO2,
    ADC_CH_PT1,
    ADC_CH_PT2,
    ADC_CH_NUM
};

/* 对射通道（治具侧接收管）在逻辑通道中的下标 */
#define SENSOR_CH1   ADC_CH_OPTO1
#define SENSOR_CH2   ADC_CH_OPTO2
#define SENSOR_NUM   2u

/* 输入侧还原系数：引脚 mV × num / den = 实际电压 mV（0x0E 报文里的“输入侧mV”）
 * 只有 ADC2 经过 10k/15k 分压，其余通道直连。 */
#define ADC_CH_DIV_NUM(ch)  (((ch) == ADC_CH_5V_BOARD) ? 25u : 1u)
#define ADC_CH_DIV_DEN(ch)  (((ch) == ADC_CH_5V_BOARD) ? 10u : 1u)

/* 对射判据默认值（mV，引脚侧）：光被遮挡 → 光电三极管截止 → 电压升高 */
#define SENSOR_BLOCK_MV_DEFAULT    2000u
#define SENSOR_RELEASE_MV_DEFAULT  1700u
#define SENSOR_DEBOUNCE_DEFAULT    3u      /* 连续 N 次一致才翻转 */

/*===== 板级引脚 =====*/
#define LED_PORT        GPIOC
#define LED_PIN         GPIO_PIN_13        /* PC13 = Ux_SYS-LED，低电平点亮 */

#if defined(BOARD_U2)
/* --- TMC2209（仅 U2）--- */
#define TMC_EN_PORT     GPIOA
#define TMC_EN_PIN      GPIO_PIN_0         /* EN2 → U6 pin2 ENN，低电平使能 */
#define TMC_DIR_PORT    GPIOB
#define TMC_DIR_PIN     GPIO_PIN_8        /* MCPWM_DIR2 */
#define TMC_STEP_PORT   GPIOB
#define TMC_STEP_PIN    GPIO_PIN_9        /* MCPWM_STEP2 */
#define TMC_TX_PORT     GPIOA
#define TMC_TX_PIN      GPIO_PIN_9        /* 网络 TM_RX：直连 PDN_UART，MCU 发送脚 */
#define TMC_RX_PORT     GPIOA
#define TMC_RX_PIN      GPIO_PIN_10       /* 网络 TM_TX：经 5.1k 接 PDN_UART，MCU 接收脚 */
#endif

/*===== 电机默认参数（仅 U2 使用）=====*/
#define MOTOR_F_START_HZ     500u        /* 起步频率 */
#define MOTOR_FREQ_MIN_HZ    500u
#define MOTOR_FREQ_MAX_HZ    20000u
#define MOTOR_ACCEL_STEPS    800u
#define MOTOR_DECEL_STEPS    800u
#define MOTOR_RAMP_GRAN      2u
#define MOTOR_MIN_RAMP_STEPS 4u
#define MOTOR_STEPS_PER_REV  3200u       /* 16 微步 × 200 步/圈 */

/* TMC2209 默认电流（0~31）：
 * 采样电阻 110mΩ、VFS=0.325V → 满量程 1.768A；IRUN=16 → (16+1)/32×1.768 ≈ 0.94A
 * M1 电机额定未知，先取保守值，可用协议 0x0022 在线调整。 */
#define TMC_DEFAULT_IRUN     16u
#define TMC_DEFAULT_IHOLD    8u

#endif /* __APP_CONFIG_H */
