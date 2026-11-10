/**
  ******************************************************************************
  * @file    bsp_uart.h
  * @brief   PC 串口（USART1）驱动：DMA 环形接收 + FIFO，阻塞发送
  *
  * 460800bps 下 1 字节只有 21.7µs，用逐字节中断会吃掉大量 CPU，
  * 因此接收走 DMA1_Channel5 循环模式，主循环侧按 NDTR 差值同步到 FIFO。
  ******************************************************************************
  */
#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "main.h"

#define UART_FIFO_SIZE   768u    /* 主循环 FIFO，需容纳突发帧 */
#define UART_DMA_SIZE    128u    /* DMA 循环缓冲 */

typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t  *dma_buf;
    uint16_t  dma_size;
    uint16_t  last_dma_count;
    uint8_t   buffer[UART_FIFO_SIZE];
    uint16_t  head;
    uint16_t  tail;
    uint32_t  overflow;         /* FIFO 溢出计数（诊断用） */
} UART_FIFO_t;

extern UART_FIFO_t uart1_fifo;

void     BSP_UART_Init(void);                 /* 配置 460800 + 启动 DMA 接收 */
void     BSP_UART_DMA_Sync(UART_FIFO_t *fifo);
uint16_t UART_GetUnreadLen(UART_FIFO_t *fifo);
uint8_t  UART_ReadByte(UART_FIFO_t *fifo, uint8_t *pData);
void     UART_ClearBuffer(UART_FIFO_t *fifo);
void     UART_SendArray(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t len);

#endif /* __BSP_UART_H */
