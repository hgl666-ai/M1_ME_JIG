/**
  ******************************************************************************
  * @file    bsp_tmc2209.h
  * @brief   TMC2209 单线 UART（软件 UART）—— 仅 U2 使用
  *
  * 硬件（M1 原理图）：
  *   U6 PDN_UART(pin14) ← 网络 TM_RX → PA9    （MCU 发送脚，直连，推挽）
  *   U6 PDN_UART(pin14) ← 5.1k 串阻 ← TM_TX → PA10（MCU 接收脚）
  *   发送完成后必须把 TX 脚释放成输入，否则会与 TMC 的应答对顶。
  *
  * 实现参考 diameter_JIG/U7 的 bsp_tmc2209.c（硬件定时器做位定时 + 逐位
  * GPIO 翻转 + Trinamic CRC8），把定时器换成 TIM3、引脚换成 PA9/PA10，并增加：
  *   - 上电自动扫描从机地址（MS1/MS2 均上拉，默认地址应为 3，但不写死）
  *   - 上电自动换向探测（原理图网名 TM_RX/TM_TX 方向存在歧义）
  ******************************************************************************
  */
#ifndef __BSP_TMC2209_H
#define __BSP_TMC2209_H

#include "main.h"
#include "app_config.h"

#define TMC_SYNC_BYTE       0x05u

/* 寄存器地址 */
#define TMC_REG_GCONF       0x00u
#define TMC_REG_GSTAT       0x01u
#define TMC_REG_IFCNT       0x02u
#define TMC_REG_IOIN        0x06u
#define TMC_REG_IHOLD_IRUN  0x10u
#define TMC_REG_TPOWERDOWN  0x11u
#define TMC_REG_TSTEP       0x12u
#define TMC_REG_VACTUAL     0x22u
#define TMC_REG_SGTHRS      0x40u
#define TMC_REG_SG_RESULT   0x41u
#define TMC_REG_MSCNT       0x6Au
#define TMC_REG_CHOPCONF    0x6Cu
#define TMC_REG_DRV_STATUS  0x6Fu
#define TMC_REG_PWMCONF     0x70u

/* MS1/MS2 均 10k 上拉 → AD1:AD0 = 11 → 地址 3；probe 会扫描 0..3 */
#define TMC_ADDR_DEFAULT    3u

void     BSP_TMC_Init(void);
uint8_t  BSP_TMC_Probe(void);          /* 1=通信正常（扫地址 + 必要时换 TX/RX 引脚） */
uint8_t  BSP_TMC_Configure(void);      /* 写关键寄存器并读回校验 */
uint8_t  BSP_TMC_IsAlive(void);
uint8_t  BSP_TMC_CfgOk(void);
uint8_t  BSP_TMC_GetAddr(void);
uint8_t  BSP_TMC_ReadReg(uint8_t reg, uint32_t *val);
uint8_t  BSP_TMC_WriteReg(uint8_t reg, uint32_t val);
void     BSP_TMC_SetCurrent(uint8_t irun, uint8_t ihold);
uint8_t  BSP_TMC_GetIrun(void);
uint8_t  BSP_TMC_GetIhold(void);

#endif /* __BSP_TMC2209_H */
