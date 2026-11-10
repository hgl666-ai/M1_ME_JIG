/**
  ******************************************************************************
  * @file    bsp_motor.c
  * @brief   步进电机：TIM2 定时翻转 STEP + 梯形加减速 + 非阻塞计步
  *
  * 时间基准：TIM2 时钟 64MHz，ARR 决定翻转周期：
  *   翻转频率 = 64MHz/(ARR+1)，STEP 频率 = 翻转频率/2
  *   → ARR = 64MHz/(2×f) - 1
  *
  * 说明：STM32F103C6 没有 TIM4，而 PB9 只有 TIM4_CH4 一个复用功能，
  *       所以 STEP 只能由定时中断 + GPIO 翻转产生（等价 50% 占空比）。
  *       这里直接用寄存器操作 TIM2，不依赖 HAL TIM 模块（CubeMX 工程里
  *       没有配置任何定时器，Hal 的 stm32f1xx_hal_tim.c 也没有被加入工程）。
  *
  * 加减速沿用 U7 参考工程的思路：全程在 ARR 域线性调节（ISR 内无除法），
  * 每 MOTOR_RAMP_GRAN 步调一档，到位只停脉冲、保持 EN 低（IHOLD 保持力矩）。
  ******************************************************************************
  */
#include "bsp_motor.h"

#define TIM2_CLK_HZ   64000000UL

static volatile uint32_t motor_target;
static volatile uint32_t motor_count;
static volatile uint8_t  motor_busy;
static volatile uint8_t  step_high;
static volatile uint8_t  motor_enabled;
static volatile uint8_t  motor_done_pending;   /* 已到目标步数，等下降沿补完整脉冲 */

/* 梯形加减速状态 */
enum { RAMP_ACCEL = 0, RAMP_CRUISE, RAMP_DECEL };

static volatile uint8_t  ramp_state;
static volatile uint16_t ramp_arr;
static uint16_t ramp_arr_start;
static uint16_t ramp_arr_apex;
static uint16_t ramp_step_a;
static uint16_t ramp_step_d;
static uint16_t ramp_decel_steps;
static uint8_t  ramp_tier_cnt;

/*==============================================================================
 * 底层
 *============================================================================*/
static void step_low(void)
{
    TMC_STEP_PORT->BRR = TMC_STEP_PIN;
}

static void motor_halt(void)
{
    TIM2->DIER &= ~TIM_DIER_UIE;      /* 先断中断源 */
    TIM2->CR1  &= ~TIM_CR1_CEN;       /* 再停计数 */
    step_low();
    step_high = 0u;
    motor_busy = 0u;
    motor_done_pending = 0u;
}

/*==============================================================================
 * 公开接口
 *============================================================================*/
void BSP_Motor_Init(void)
{
    GPIO_InitTypeDef g = {0};

    /* EN(PA0)：CubeMX 未配置，这里手工初始化；默认脱机 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    g.Pin   = TMC_EN_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(TMC_EN_PORT, &g);
    HAL_GPIO_WritePin(TMC_EN_PORT, TMC_EN_PIN, GPIO_PIN_SET);   /* EN 高 = 脱机 */

    /* DIR / STEP 输出低 */
    HAL_GPIO_WritePin(TMC_DIR_PORT, TMC_DIR_PIN, GPIO_PIN_RESET);
    step_low();

    /* TIM2：64MHz 自由计数，更新中断（不启动） */
    __HAL_RCC_TIM2_CLK_ENABLE();
    TIM2->CR1  = 0u;
    TIM2->PSC  = 0u;
    TIM2->ARR  = 63999u;               /* 500Hz：64M/(2×500)-1 */
    TIM2->CNT  = 0u;
    TIM2->CR1  = TIM_CR1_ARPE;
    TIM2->EGR  = TIM_EGR_UG;
    TIM2->SR   = 0u;
    TIM2->DIER = TIM_DIER_UIE;

    HAL_NVIC_SetPriority(TIM2_IRQn, 0u, 0u);   /* 最高优先级：脉冲不抖 */
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

    motor_target  = 0u;
    motor_count   = 0u;
    motor_busy    = 0u;
    motor_enabled = 0u;
    step_high     = 0u;
    motor_done_pending = 0u;
    ramp_state    = RAMP_CRUISE;
    ramp_arr      = 63999u;
}

void BSP_Motor_SetEnable(uint8_t en)
{
    motor_enabled = en ? 1u : 0u;
    /* ENN 低有效：en=1 → 拉低使能 */
    HAL_GPIO_WritePin(TMC_EN_PORT, TMC_EN_PIN,
                      en ? GPIO_PIN_RESET : GPIO_PIN_SET);
    if (!en) {
        motor_halt();
    }
}

/**
  * @brief  启动一段定长运动（非阻塞）
  * @param  dir      MOTOR_DIR_CW / MOTOR_DIR_CCW
  * @param  steps    步数（16 微步 → 3200 步/圈）
  * @param  freq_hz  巡航频率，自动夹到 [500, 20000]
  * @retval 1=已启动，0=拒绝（忙/参数非法）
  */
uint8_t BSP_Motor_Move(uint8_t dir, uint32_t steps, uint32_t freq_hz)
{
    uint32_t arr_start, arr_apex;
    uint32_t accel_steps, decel_steps, delta, a_t, d_t, step_a, step_d;
    uint8_t  armed;

    if (steps == 0u || freq_hz == 0u || dir > 1u) return 0u;
    if (motor_busy) return 0u;

    if (freq_hz < MOTOR_FREQ_MIN_HZ) freq_hz = MOTOR_FREQ_MIN_HZ;
    if (freq_hz > MOTOR_FREQ_MAX_HZ) freq_hz = MOTOR_FREQ_MAX_HZ;

    arr_start = TIM2_CLK_HZ / (2u * MOTOR_F_START_HZ) - 1u;   /* 63999 */
    arr_apex  = TIM2_CLK_HZ / (2u * freq_hz) - 1u;

    accel_steps = MOTOR_ACCEL_STEPS;
    decel_steps = MOTOR_DECEL_STEPS;

    if (steps < accel_steps + decel_steps) {
        /* 行程太短：三角曲线，半程加速、半程减速，避免高速急停 */
        accel_steps = steps / 2u;
        decel_steps = steps - accel_steps;
        arr_apex = arr_start - (arr_start - arr_apex) * accel_steps / MOTOR_ACCEL_STEPS;
        if (arr_apex < (TIM2_CLK_HZ / (2u * freq_hz) - 1u)) {
            arr_apex = TIM2_CLK_HZ / (2u * freq_hz) - 1u;
        }
    }

    delta = (arr_start > arr_apex) ? (arr_start - arr_apex) : 0u;
    armed = (steps >= MOTOR_MIN_RAMP_STEPS) && (delta >= 2u);
    if (!armed) {
        arr_apex = arr_start;
        delta = 0u;
    }

    a_t = accel_steps / MOTOR_RAMP_GRAN; if (a_t == 0u) a_t = 1u;
    d_t = decel_steps / MOTOR_RAMP_GRAN; if (d_t == 0u) d_t = 1u;
    step_a = delta / a_t; if (step_a == 0u) step_a = 1u;
    step_d = delta / d_t; if (step_d == 0u) step_d = 1u;

    ramp_arr_start   = (uint16_t)arr_start;
    ramp_arr_apex    = (uint16_t)arr_apex;
    ramp_step_a      = (uint16_t)step_a;
    ramp_step_d      = (uint16_t)step_d;
    ramp_decel_steps = (uint16_t)decel_steps;
    ramp_state       = (armed && accel_steps > 0u) ? RAMP_ACCEL : RAMP_CRUISE;
    ramp_tier_cnt    = 0u;
    ramp_arr         = (uint16_t)arr_start;

    motor_target = steps;
    motor_count  = 0u;

    /* 方向：低 = CW，高 = CCW */
    HAL_GPIO_WritePin(TMC_DIR_PORT, TMC_DIR_PIN,
                      (dir == MOTOR_DIR_CCW) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    if (!motor_enabled) {
        BSP_Motor_SetEnable(1u);
        HAL_Delay(2u);          /* 等电荷泵/线圈电流建立 */
    }

    /* 安全启动：停计数 → 清计数/标志 → 装载起步 ARR → 清挂起 → 启动 */
    TIM2->CR1 &= ~TIM_CR1_CEN;
    TIM2->CNT  = 0u;
    TIM2->ARR  = ramp_arr_start;
    TIM2->EGR  = TIM_EGR_UG;
    TIM2->SR   = 0u;
    NVIC_ClearPendingIRQ(TIM2_IRQn);
    step_high  = 0u;
    motor_done_pending = 0u;
    step_low();

    motor_busy = 1u;
    TIM2->CR1 |= TIM_CR1_CEN;
    return 1u;
}

void BSP_Motor_Stop(void)
{
    motor_halt();
    motor_target = motor_count;
    BSP_Motor_SetEnable(0u);        /* 脱机（释放保持力矩） */
}

uint8_t  BSP_Motor_IsBusy(void)   { return motor_busy; }
uint32_t BSP_Motor_GetDone(void)  { return motor_count; }
uint32_t BSP_Motor_GetTarget(void){ return motor_target; }

/*==============================================================================
 * 中断：每半个 STEP 周期进入一次
 *============================================================================*/
static void motor_tick(void)
{
    if (step_high == 0u) {
        /* 上升沿：计一步 */
        TMC_STEP_PORT->BSRR = TMC_STEP_PIN;
        step_high = 1u;
        motor_count++;

        if (motor_count >= motor_target) {
            /* 等这一拍的低电平把脉冲补完整，避免最后一步变成窄脉冲丢步 */
            motor_done_pending = 1u;
            return;
        }

        if ((motor_target - motor_count) <= ramp_decel_steps) {
            ramp_state = RAMP_DECEL;
        }

        if (++ramp_tier_cnt >= MOTOR_RAMP_GRAN) {
            ramp_tier_cnt = 0u;
            switch (ramp_state) {
            case RAMP_ACCEL:
                if ((uint32_t)ramp_arr <= (uint32_t)ramp_arr_apex + ramp_step_a) {
                    ramp_arr = ramp_arr_apex;
                    ramp_state = RAMP_CRUISE;
                } else {
                    ramp_arr = (uint16_t)(ramp_arr - ramp_step_a);
                }
                break;
            case RAMP_DECEL:
                if ((uint32_t)ramp_arr + ramp_step_d >= ramp_arr_start) {
                    ramp_arr = ramp_arr_start;
                } else {
                    ramp_arr = (uint16_t)(ramp_arr + ramp_step_d);
                }
                break;
            default:
                break;
            }
            TIM2->ARR = ramp_arr;   /* ARPE=1：下一个更新事件生效 */
        }
    } else {
        TMC_STEP_PORT->BRR = TMC_STEP_PIN;   /* 下降沿，脉宽 = 半个周期 */
        step_high = 0u;
        if (motor_done_pending) {
            motor_done_pending = 0u;
            motor_halt();
        }
    }
}

void TIM2_IRQHandler(void)
{
    if ((TIM2->SR & TIM_SR_UIF) != 0u) {
        TIM2->SR = (uint16_t)~TIM_SR_UIF;
        if (motor_busy) {
            motor_tick();
        }
    }
}
