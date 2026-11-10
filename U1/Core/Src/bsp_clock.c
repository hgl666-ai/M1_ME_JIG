/**
  ******************************************************************************
  * @file    bsp_clock.c
  * @brief   HSE 64MHz 兜底切换（直接操作 RCC 寄存器）
  *
  * 只在“当前不是 HSE+PLL”时动作：
  *   1) 先单独给 HSE 上电并等待起振（起振失败就放弃，保持现有时钟）
  *   2) HSI 作过渡时钟 → PLL 源切 HSE、倍频 ×8 → 切回 PLL
  * 全程 64MHz 不变，因此 SysTick 重装值/USART 波特率都不受影响。
  ******************************************************************************
  */
#include "bsp_clock.h"

#define HSE_START_TIMEOUT_LOOPS  0x50000UL   /* 约 3ms @64MHz 空循环 */

void BSP_Clock_Init(void)
{
    /* 已经是 HSE + PLL 就什么都不做 */
    if ((RCC->CR & RCC_CR_HSERDY) != 0u &&
        (RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL &&
        (RCC->CFGR & RCC_CFGR_PLLSRC) != 0u) {
        return;
    }

    /* 1) 先让 HSE 起振（不动当前时钟，失败可直接放弃） */
    RCC->CR |= RCC_CR_HSEON;
    uint32_t to = HSE_START_TIMEOUT_LOOPS;
    while ((RCC->CR & RCC_CR_HSERDY) == 0u) {
        if (--to == 0u) {
            RCC->CR &= ~RCC_CR_HSEON;     /* 无晶振/虚焊：保持 HSI 64MHz */
            return;
        }
    }

    /* 2) 切到 HSI 作为过渡 */
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSI;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI) { }

    /* 3) 关 PLL，改成 HSE ×8 = 64MHz */
    RCC->CR &= ~RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY) != 0u) { }

    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PLLMULL)
              | RCC_CFGR_PLLSRC
              | RCC_CFGR_PLLMULL8;

    RCC->CR |= RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY) == 0u) { }

    /* 4) 切回 PLL */
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) { }
}
