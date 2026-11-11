/**
  ******************************************************************************
  * @file    bsp_uart.c
  * @brief   USART1（PB6/PB7）PC_UART_BAUD 8N1 + DMA 循环接收
  *
  * CubeMX 工程里没有配置 DMA（U1.ioc / U2.ioc 均无 DMA 通道），
  * 因此这里手工建立 DMA1_Channel5 → USART1_RX 的链接；
  * 这样即使以后重新生成 CubeMX 代码，本文件也不需要跟着改。
  ******************************************************************************
  */
#include "bsp_uart.h"
#include "usart.h"        /* huart1 */
#include "app_config.h"

static uint8_t u1_dma_buf[UART_DMA_SIZE];
static DMA_HandleTypeDef hdma_usart1_rx;

UART_FIFO_t uart1_fifo = {
    .huart         = &huart1,
    .dma_buf       = u1_dma_buf,
    .dma_size      = UART_DMA_SIZE,
    .last_dma_count = UART_DMA_SIZE,
    .head          = 0u,
    .tail          = 0u,
    .overflow      = 0u,
};

/*==============================================================================
 * DMA 配置（USART1_RX = DMA1 通道 5）
 *============================================================================*/
static void uart_dma_init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();

    hdma_usart1_rx.Instance                 = DMA1_Channel5;
    hdma_usart1_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode                = DMA_CIRCULAR;
    hdma_usart1_rx.Init.Priority            = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK) {
        Error_Handler();
    }

    __HAL_LINKDMA(&huart1, hdmarx, hdma_usart1_rx);

    /* DMA 完成/错误中断（循环模式下仅用于回调，不参与取数） */
    HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 1u, 0u);
    HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
}

/**
  * @brief  DMA 中断入口（HAL 要求存在，否则循环模式下 TC 中断会进默认死循环）
  */
void DMA1_Channel5_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart1_rx);
}

/*==============================================================================
 * 公开接口
 *============================================================================*/

/**
  * @brief  初始化：按 PC_UART_BAUD 重配 → 建 DMA → 启动循环接收
  * @note   在 MX_USART1_UART_Init() 之后调用
  */
void BSP_UART_Init(void)
{
    /* 波特率统一由 PC_UART_BAUD 决定；CubeMX 生成值会被这里覆盖，
     * 目的是让“重新生成工程”不会破坏波特率设置。 */
    huart1.Init.BaudRate = PC_UART_BAUD;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }

    uart_dma_init();

    UART_ClearBuffer(&uart1_fifo);
    if (HAL_UART_Receive_DMA(&huart1, u1_dma_buf, UART_DMA_SIZE) != HAL_OK) {
        Error_Handler();
    }
    uart1_fifo.last_dma_count = __HAL_DMA_GET_COUNTER(huart1.hdmarx);
}

/**
  * @brief  把 DMA 环形缓冲里“新到”的字节搬进 FIFO
  */
void BSP_UART_DMA_Sync(UART_FIFO_t *fifo)
{
    if (fifo == NULL || fifo->huart == NULL || fifo->huart->hdmarx == NULL) return;

    uint16_t cur = (uint16_t)__HAL_DMA_GET_COUNTER(fifo->huart->hdmarx);
    uint16_t delta;
    uint16_t src;

    if (fifo->last_dma_count >= cur) {
        delta = (uint16_t)(fifo->last_dma_count - cur);
    } else {
        delta = (uint16_t)(fifo->dma_size - cur + fifo->last_dma_count);
    }
    if (delta == 0u) return;
    if (delta > fifo->dma_size) delta = fifo->dma_size;

    src = (uint16_t)(fifo->dma_size - fifo->last_dma_count);
    if (src >= fifo->dma_size) src = 0u;

    while (delta--) {
        uint16_t next_head = (uint16_t)((fifo->head + 1u) % UART_FIFO_SIZE);
        if (next_head != fifo->tail) {
            fifo->buffer[fifo->head] = fifo->dma_buf[src];
            fifo->head = next_head;
        } else {
            fifo->overflow++;          /* FIFO 满：丢新字节，计数供诊断 */
        }
        src++;
        if (src >= fifo->dma_size) src = 0u;
    }

    fifo->last_dma_count = cur;
}

uint16_t UART_GetUnreadLen(UART_FIFO_t *fifo)
{
    BSP_UART_DMA_Sync(fifo);
    if (fifo->head >= fifo->tail) return (uint16_t)(fifo->head - fifo->tail);
    return (uint16_t)(UART_FIFO_SIZE - fifo->tail + fifo->head);
}

uint8_t UART_ReadByte(UART_FIFO_t *fifo, uint8_t *pData)
{
    if (fifo->head == fifo->tail) {
        BSP_UART_DMA_Sync(fifo);
        if (fifo->head == fifo->tail) return 0u;
    }
    *pData = fifo->buffer[fifo->tail];
    fifo->tail = (uint16_t)((fifo->tail + 1u) % UART_FIFO_SIZE);
    return 1u;
}

void UART_ClearBuffer(UART_FIFO_t *fifo)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    fifo->head = 0u;
    fifo->tail = 0u;
    if (fifo->huart != NULL && fifo->huart->hdmarx != NULL) {
        fifo->last_dma_count = (uint16_t)__HAL_DMA_GET_COUNTER(fifo->huart->hdmarx);
    }
    if (primask == 0u) __enable_irq();
}

/**
  * @brief  阻塞发送（1000ms 超时）
  */
void UART_SendArray(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t len)
{
    if (len == 0u) return;
    (void)HAL_UART_Transmit(huart, (uint8_t *)pData, len, 1000u);
}
