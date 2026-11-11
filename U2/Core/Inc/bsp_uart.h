/**
  ******************************************************************************
  * @file    bsp_uart.h
  * @brief   PC 串口（USART1）驱动：DMA 环形接收 + FIFO，阻塞发送
  *
  * 115200bps 下 1 字节 86.8µs，逐字节中断也可行；这里仍用 DMA1_Channel5
  * 循环接收 + FIFO 同步，好处是主循环被长耗时命令（如读 TMC 寄存器）占住时也不丢字节。
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

void     BSP_UART_Init(void);                 /* 按 PC_UART_BAUD 配置 + 启动 DMA 接收 */
void     BSP_UART_DMA_Sync(UART_FIFO_t *fifo);
uint16_t UART_GetUnreadLen(UART_FIFO_t *fifo);
uint8_t  UART_ReadByte(UART_FIFO_t *fifo, uint8_t *pData);
void     UART_ClearBuffer(UART_FIFO_t *fifo);
void     UART_SendArray(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t len);

#endif /* __BSP_UART_H */
