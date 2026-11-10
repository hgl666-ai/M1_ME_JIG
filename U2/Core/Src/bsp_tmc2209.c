/**
  ******************************************************************************
  * @file    bsp_tmc2209.c
  * @brief   TMC2209 单线 UART（软件 UART，TIM3 位定时）—— 仅 U2
  *
  * 时序：起始位 + 8 数据位(LSB first) + 停止位，115200 8N1。
  * 帧：写 8 字节 (0x05, addr, reg|0x80, d3..d0, crc8)
  *     读 4 字节请求 + 8 字节应答 (0x05, 0xFF, addr, d3..d0, crc8)
  * CRC8：Trinamic 专用（多项式 0x07，LSB first 的位处理）
  ******************************************************************************
  */
#include "bsp_tmc2209.h"

/* TIM3 时钟 = APB1×2 = 32MHz×2 = 64MHz */
#define TMC_TIM_CLK_HZ   64000000UL
#define TMC_BAUD         115200UL
#define TMC_BIT_TICKS    ((uint16_t)((TMC_TIM_CLK_HZ + (TMC_BAUD / 2u)) / TMC_BAUD))  /* 556 */
#define TMC_HALF_TICKS   ((uint16_t)(TMC_BIT_TICKS / 2u))

/* 运行时可切换的收发引脚（上电探测用） */
static GPIO_TypeDef *tx_port = TMC_TX_PORT;
static uint16_t      tx_pin  = TMC_TX_PIN;
static GPIO_TypeDef *rx_port = TMC_RX_PORT;
static uint16_t      rx_pin  = TMC_RX_PIN;

static uint8_t  tmc_addr   = TMC_ADDR_DEFAULT;
static uint8_t  tmc_alive  = 0u;
static uint8_t  tmc_cfg_ok = 0u;
static uint8_t  cur_irun   = TMC_DEFAULT_IRUN;
static uint8_t  cur_ihold  = TMC_DEFAULT_IHOLD;

#define TX_HIGH()   (tx_port->BSRR = tx_pin)
#define TX_LOW()    (tx_port->BRR  = tx_pin)
#define RX_LEVEL()  (((rx_port->IDR & rx_pin) != 0u) ? 1u : 0u)

/*==============================================================================
 * 位定时
 *============================================================================*/
static void tmc_tim_init(void)
{
    __HAL_RCC_TIM3_CLK_ENABLE();

    TIM3->CR1  = 0u;
    TIM3->PSC  = 0u;
    TIM3->ARR  = 0xFFFFu;
    TIM3->CNT  = 0u;
    TIM3->EGR  = TIM_EGR_UG;      /* 立即装载 PSC */
    TIM3->SR   = 0u;
    TIM3->CR1  = TIM_CR1_CEN;     /* 自由运行，64MHz */
}

static void tim_delay(uint16_t ticks)
{
    uint16_t start = TIM3->CNT;
    while ((uint16_t)(TIM3->CNT - start) < ticks) {
        /* 空等，精度 15.6ns */
    }
}

/*==============================================================================
 * 引脚角色
 *============================================================================*/
static void tmc_tx_output(void)
{
    GPIO_InitTypeDef g = {0};
    g.Pin   = tx_pin;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(tx_port, &g);
    TX_HIGH();                    /* 空闲高 */
}

static void tmc_tx_release(void)
{
    GPIO_InitTypeDef g = {0};
    g.Pin  = tx_pin;
    g.Mode = GPIO_MODE_INPUT;     /* 释放总线，交给 TMC 应答 */
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(tx_port, &g);
}

static void tmc_rx_input(void)
{
    GPIO_InitTypeDef g = {0};
    g.Pin  = rx_pin;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(rx_port, &g);
}

/*==============================================================================
 * CRC8（Trinamic 官方算法：CRC 的 MSB 与 data 的 LSB 异或）
 * 验证：CRC([0x05]) = 0x69
 *============================================================================*/
static uint8_t tmc_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0u;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t cur = data[i];
        for (uint8_t j = 0; j < 8u; j++) {
            if (((crc >> 7) ^ (cur & 0x01u)) != 0u) {
                crc = (uint8_t)((crc << 1) ^ 0x07u);
            } else {
                crc = (uint8_t)(crc << 1);
            }
            cur >>= 1;
        }
    }
    return crc;
}

/*==============================================================================
 * 收发
 *============================================================================*/
static void uart_tx_byte(uint8_t byte)
{
    TX_LOW();                                   /* 起始位 */
    tim_delay(TMC_BIT_TICKS);

    for (uint8_t i = 0; i < 8u; i++) {          /* D0..D7，LSB first */
        if ((byte & 0x01u) != 0u) TX_HIGH(); else TX_LOW();
        tim_delay(TMC_BIT_TICKS);
        byte >>= 1;
    }

    TX_HIGH();                                  /* 停止位 */
    tim_delay(TMC_BIT_TICKS);
}

/**
  * @brief  收一个字节（含起始位检测），超时返回 0
  * @note   等待阶段允许中断；一旦检出起始位，只在 8 个数据位 + 停止位
  *         采样期间关中断（约 78µs @115200），避免长时间阻塞电机中断。
  */
static uint8_t uart_rx_byte(uint8_t *out, uint16_t timeout_ms)
{
    uint32_t tout = (uint32_t)timeout_ms * (TMC_TIM_CLK_HZ / 1000u);
    uint32_t total = 0u;
    uint16_t last  = TIM3->CNT;
    uint16_t now;

    /* 阶段0：等总线回到高（滤掉释放引脚时的毛刺） */
    while (RX_LEVEL() == 0u) {
        now = TIM3->CNT;
        total += (uint16_t)(now - last);
        last = now;
        if (total > tout) return 0u;
    }

    /* 阶段1：等起始位下降沿 */
    total = 0u;
    while (RX_LEVEL() != 0u) {
        now = TIM3->CNT;
        total += (uint16_t)(now - last);
        last = now;
        if (total > tout) return 0u;
    }

    /* 阶段2：从起始位下降沿起按绝对时间点采样（D0 中心 = 1.5 bit） */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    uint16_t t_sample = (uint16_t)(TIM3->CNT + TMC_BIT_TICKS + TMC_HALF_TICKS);
    uint8_t  val = 0u;

    for (uint8_t i = 0; i < 8u; i++) {
        while ((uint16_t)(TIM3->CNT - t_sample) >= 0x8000u) {
            /* 等 CNT 到达采样点 */
        }
        if (RX_LEVEL() != 0u) val |= (uint8_t)(1u << i);
        t_sample = (uint16_t)(t_sample + TMC_BIT_TICKS);
    }

    /* 停止位 */
    while ((uint16_t)(TIM3->CNT - t_sample) >= 0x8000u) { }
    uint8_t stop_ok = (RX_LEVEL() != 0u) ? 1u : 0u;

    if (primask == 0u) __enable_irq();

    if (!stop_ok) return 0u;

    *out = val;
    return 1u;
}

/*==============================================================================
 * 数据报
 *============================================================================*/
static void build_write_datagram(uint8_t addr, uint8_t reg, uint32_t data, uint8_t *dg)
{
    dg[0] = TMC_SYNC_BYTE;
    dg[1] = addr;
    dg[2] = (uint8_t)(reg | 0x80u);
    dg[3] = (uint8_t)(data >> 24);
    dg[4] = (uint8_t)(data >> 16);
    dg[5] = (uint8_t)(data >> 8);
    dg[6] = (uint8_t)(data);
    dg[7] = tmc_crc8(dg, 7u);
}

static void build_read_datagram(uint8_t addr, uint8_t reg, uint8_t *dg)
{
    dg[0] = TMC_SYNC_BYTE;
    dg[1] = addr;
    dg[2] = reg;
    dg[3] = tmc_crc8(dg, 3u);
}

/*==============================================================================
 * 公开接口
 *============================================================================*/
void BSP_TMC_Init(void)
{
    tmc_tim_init();
    tmc_rx_input();
    tmc_tx_output();
    HAL_Delay(2u);        /* 等 TMC 上电复位完成（VCC_IO 稳定） */
}

uint8_t BSP_TMC_WriteReg(uint8_t reg, uint32_t val)
{
    uint8_t dg[8];
    build_write_datagram(tmc_addr, reg, val, dg);

    tmc_tx_output();

    /* 逐字节关中断（每字节约 87µs），把最长中断阻塞压到 1 个字节 */
    for (uint8_t i = 0; i < 8u; i++) {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        uart_tx_byte(dg[i]);
        if (primask == 0u) __enable_irq();
    }

    HAL_Delay(1u);        /* 帧间隔 */
    return 1u;
}

uint8_t BSP_TMC_ReadReg(uint8_t reg, uint32_t *val)
{
    uint8_t dg[4];
    uint8_t resp[8];
    build_read_datagram(tmc_addr, reg, dg);

    tmc_tx_output();

    for (uint8_t i = 0; i < 4u; i++) {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        uart_tx_byte(dg[i]);
        if (primask == 0u) __enable_irq();
    }
    tmc_tx_release();

    for (uint8_t i = 0; i < 8u; i++) {
        if (!uart_rx_byte(&resp[i], 10u)) {
            return 0u;
        }
    }

    if (resp[0] != TMC_SYNC_BYTE || resp[1] != 0xFFu) return 0u;
    if (tmc_crc8(resp, 7u) != resp[7]) return 0u;

    *val = ((uint32_t)resp[2] << 24) | ((uint32_t)resp[3] << 16) |
           ((uint32_t)resp[4] << 8)  |  (uint32_t)resp[5];
    tmc_alive = 1u;
    return 1u;
}

/**
  * @brief  扫描从机地址 / 交换收发引脚，直到读到 GCONF
  * @note   地址由 MS1/MS2 决定：本板两脚均 10k 上拉 → 地址 3。
  *         若原理图网名方向与实物相反，第二遍会用交换后的引脚重试。
  */
uint8_t BSP_TMC_Probe(void)
{
    static const uint8_t addr_list[4] = { TMC_ADDR_DEFAULT, 0u, 1u, 2u };
    uint8_t pass;

    for (pass = 0u; pass < 2u; pass++) {
        for (uint8_t i = 0; i < 4u; i++) {
            uint32_t v = 0u;
            tmc_addr = addr_list[i];
            if (BSP_TMC_ReadReg(TMC_REG_GCONF, &v)) {
                tmc_alive  = 1u;
                return 1u;
            }
        }

        /* 第一遍全失败 → 交换 TX/RX 引脚再试一次 */
        GPIO_TypeDef *p = tx_port; tx_port = rx_port; rx_port = p;
        uint16_t      n = tx_pin;  tx_pin  = rx_pin;  rx_pin  = n;
        tmc_rx_input();
        tmc_tx_output();
        HAL_Delay(2u);
    }

    /* 两遍都失败：还原默认方向，交给上层报错 */
    {
        GPIO_TypeDef *p = tx_port; tx_port = rx_port; rx_port = p;
        uint16_t      n = tx_pin;  tx_pin  = rx_pin;  rx_pin  = n;
        tmc_rx_input();
        tmc_tx_output();
    }
    tmc_alive = 0u;
    return 0u;
}

uint8_t BSP_TMC_IsAlive(void)
{
    return tmc_alive;
}

uint8_t BSP_TMC_CfgOk(void)
{
    return tmc_cfg_ok;
}

uint8_t BSP_TMC_GetAddr(void)
{
    return tmc_addr;
}

/**
  * @brief  写关键寄存器并读回校验
  * @retval 1 = 关键项全部生效
  * @note   GCONF/CHOPCONF/PWMCONF 可读回；IHOLD_IRUN、TPOWERDOWN 为只写，
  *         按手册不做回读校验（与 U7 参考工程一致）。
  */
uint8_t BSP_TMC_Configure(void)
{
    uint8_t  ok = 1u;
    uint32_t rb = 0u;

    /* GCONF: 内部基准、16 微步由寄存器控制、UART 控制使能、StealthChop */
    /* bit4 index_otpw | bit6 pdn_disable | bit7 mstep_reg_select | bit8 multistep_filt */
    uint32_t gconf = 0x000001D0UL;
    BSP_TMC_WriteReg(TMC_REG_GCONF, gconf);
    if (BSP_TMC_ReadReg(TMC_REG_GCONF, &rb)) {
        if ((rb & 0x000001C0UL) != (gconf & 0x000001C0UL)) ok = 0u;
    } else {
        ok = 0u;
    }

    /* IHOLD_IRUN: IRUN[12:8] IHOLD[4:0] IHOLDDELAY[19:16] */
    uint32_t ihold_irun = ((uint32_t)cur_irun << 8) | (uint32_t)cur_ihold | (3UL << 16);
    BSP_TMC_WriteReg(TMC_REG_IHOLD_IRUN, ihold_irun);

    /* TPOWERDOWN = 20 → 停下后约 0.44s 降到 IHOLD */
    BSP_TMC_WriteReg(TMC_REG_TPOWERDOWN, 20u);

    /* CHOPCONF: TOFF=3 HSTRT=4 HEND=1 TBL=2 VSENSE=0 MRES=4(16微步) INTPOL=1 */
    uint32_t chopconf = 0x140101C3UL;
    BSP_TMC_WriteReg(TMC_REG_CHOPCONF, chopconf);
    if (BSP_TMC_ReadReg(TMC_REG_CHOPCONF, &rb)) {
        if (rb != chopconf) ok = 0u;
    } else {
        ok = 0u;
    }

    /* PWMCONF: PWM_OFS=36 GRAD=14 FREQ=1 AUTOSCALE AUTOGRAO REG=8 LIM=12 */
    uint32_t pwmconf = 0xC80D0E24UL;
    BSP_TMC_WriteReg(TMC_REG_PWMCONF, pwmconf);
    if (BSP_TMC_ReadReg(TMC_REG_PWMCONF, &rb)) {
        if (rb != pwmconf) ok = 0u;
    } else {
        ok = 0u;
    }

    /* 清 GSTAT(写 1 清)：reset / drv_err */
    BSP_TMC_WriteReg(TMC_REG_GSTAT, 0x03u);

    tmc_cfg_ok = ok;
    return ok;
}

void BSP_TMC_SetCurrent(uint8_t irun, uint8_t ihold)
{
    if (irun > 31u || ihold > 31u || ihold > irun) return;
    cur_irun  = irun;
    cur_ihold = ihold;
    BSP_TMC_WriteReg(TMC_REG_IHOLD_IRUN,
                     ((uint32_t)irun << 8) | (uint32_t)ihold | (3UL << 16));
}

uint8_t BSP_TMC_GetIrun(void)  { return cur_irun;  }
uint8_t BSP_TMC_GetIhold(void) { return cur_ihold; }
