/*
 * TMUX1108.h
 *
 * TMUX1108 极速驱动头文件（STM32G474 HAL）
 *
 * 设计目标：
 * 1) 地址切换在 100Hz 核心定时器中断内执行，必须极低延迟。
 * 2) 严禁使用 HAL_GPIO_WritePin，统一使用 GPIOx->BSRR 原子写。
 * 3) 对于同一端口内的多位地址线，组装 32 位掩码后一次写入，减少总线访问。
 */

#ifndef INC_TMUX1108_H_
#define INC_TMUX1108_H_

#include "main.h"

/*
 * 通道定义（对应 TMUX1108 的 A2/A1/A0 真值表）
 * S1: 000, S2: 001, ..., S8: 111
 */
typedef enum {
    TMUX_CH_S1 = 0U, /* A2=0, A1=0, A0=0 */
    TMUX_CH_S2 = 1U, /* A2=0, A1=0, A0=1 */
    TMUX_CH_S3 = 2U, /* A2=0, A1=1, A0=0 */
    TMUX_CH_S4 = 3U, /* A2=0, A1=1, A0=1 */
    TMUX_CH_S5 = 4U, /* A2=1, A1=0, A0=0 */
    TMUX_CH_S6 = 5U, /* A2=1, A1=0, A0=1 */
    TMUX_CH_S7 = 6U, /* A2=1, A1=1, A0=0 */
    TMUX_CH_S8 = 7U  /* A2=1, A1=1, A0=1 */
} TMUX_Channel_t;

/* 全局 GPIO 初始化（仅配置地址线，不接管 HRTIM 负责的 EN 管脚） */
void TMUX_Global_Init(void);

/*
 * @brief 根据某一位地址值生成 BSRR 掩码
 * @param bit_val 目标地址位（0 或 1）
 * @param pin_num GPIO 引脚号（0~15）
 * @return bit_val=1 时返回 (1U << pin_num)；bit_val=0 时返回 (1U << (pin_num + 16U))
 *
 * 说明：
 * - BSRR 低 16 位写 1 表示置位（输出高电平）。
 * - BSRR 高 16 位写 1 表示复位（输出低电平）。
 * - pin_num 取值仅为 0~15，因此 (pin_num + 16U) 最大为 31，32 位左移安全。
 */
static inline uint32_t TMUX_BSRR_BIT(uint32_t bit_val, uint32_t pin_num)
{
    return (bit_val != 0U) ? (1U << pin_num) : (1U << (pin_num + 16U));
}

/*
 * @brief 切换 KH 组通道（PB12=A0, PB13=A1, PB14=A2）
 *
 * 性能要点：
 * - 三根地址线均在 GPIOB，先组包后单次写 GPIOB->BSRR。
 * - 在中断中调用时仅一次端口写操作，切换时延最小。
 */
static inline void TMUX_KH_SetChannel(TMUX_Channel_t ch)
{
    const uint32_t bsrr_val =
        TMUX_BSRR_BIT(((uint32_t)ch >> 0U) & 0x1U, 12U) |
        TMUX_BSRR_BIT(((uint32_t)ch >> 1U) & 0x1U, 13U) |
        TMUX_BSRR_BIT(((uint32_t)ch >> 2U) & 0x1U, 14U);

    GPIOB->BSRR = bsrr_val;
}

/*
 * @brief 切换 KL 组通道（PD8=A0, PD9=A1, PD10=A2）
 *
 * 性能要点：
 * - 三根地址线均在 GPIOD，先组包后单次写 GPIOD->BSRR。
 */
static inline void TMUX_KL_SetChannel(TMUX_Channel_t ch)
{
    const uint32_t bsrr_val =
        TMUX_BSRR_BIT(((uint32_t)ch >> 0U) & 0x1U, 8U) |
        TMUX_BSRR_BIT(((uint32_t)ch >> 1U) & 0x1U, 9U) |
        TMUX_BSRR_BIT(((uint32_t)ch >> 2U) & 0x1U, 10U);

    GPIOD->BSRR = bsrr_val;
}

/*
 * @brief 切换 KB 组通道（PB15=A0, PC6=A1, PC7=A2）
 *
 * 端口跨域说明：
 * - A0 位于 GPIOB，需要写 GPIOB->BSRR。
 * - A1/A2 位于 GPIOC，可合并为一次 GPIOC->BSRR。
 * - 因跨端口，最少需要两次写操作（GPIOB 一次 + GPIOC 一次）。
 */
static inline void TMUX_KB_SetChannel(TMUX_Channel_t ch)
{
    const uint32_t bsrr_b = TMUX_BSRR_BIT(((uint32_t)ch >> 0U) & 0x1U, 15U);
    const uint32_t bsrr_c =
        TMUX_BSRR_BIT(((uint32_t)ch >> 1U) & 0x1U, 6U) |
        TMUX_BSRR_BIT(((uint32_t)ch >> 2U) & 0x1U, 7U);

    GPIOB->BSRR = bsrr_b;
    GPIOC->BSRR = bsrr_c;
}

#endif /* INC_TMUX1108_H_ */