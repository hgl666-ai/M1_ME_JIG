/* 主机端测试用的假 main.h —— 只提供协议层需要的类型，不引入 STM32 HAL */
#ifndef __MAIN_H
#define __MAIN_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

typedef struct { int dummy; } UART_HandleTypeDef;
typedef struct { int dummy; } GPIO_TypeDef;
typedef struct { int dummy; } TIM_HandleTypeDef;
typedef struct { int dummy; } ADC_HandleTypeDef;

#define GPIO_PIN_0  (1u << 0)
#define GPIO_PIN_1  (1u << 1)
#define GPIO_PIN_8  (1u << 8)
#define GPIO_PIN_9  (1u << 9)
#define GPIO_PIN_10 (1u << 10)
#define GPIO_PIN_13 (1u << 13)

extern GPIO_TypeDef *const GPIOA_FAKE;
extern GPIO_TypeDef *const GPIOB_FAKE;
extern GPIO_TypeDef *const GPIOC_FAKE;
#define GPIOA GPIOA_FAKE
#define GPIOB GPIOB_FAKE
#define GPIOC GPIOC_FAKE

extern UART_HandleTypeDef huart1;

/* 主机端桩函数声明 */
uint32_t HAL_GetTick(void);
void     HAL_Delay(uint32_t ms);

#endif
