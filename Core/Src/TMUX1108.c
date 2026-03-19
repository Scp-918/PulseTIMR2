/*
 * TMUX1108.c
 *
 * TMUX1108 极速驱动初始化实现（STM32G474 HAL）
 */

#include "TMUX1108.h"

/**
  * @brief 统一初始化 TMUX1108 三组地址线 GPIO
  *
  * 初始化范围：
  * 1) KH 组：PB12, PB13, PB14
  * 2) KL 组：PD8,  PD9,  PD10
  * 3) KB 组：PB15, PC6,  PC7
  *
  * 关键约束：
  * - 不初始化 PA8（KB 的 EN），该引脚由 HRTIM 硬件接管。
  * - 地址线统一配置为：推挽输出 + 无上下拉 + 超高速。
  * - 初始化完成后，三组地址线全部切换到 S1（A2/A1/A0 = 0/0/0）。
  */
void TMUX_Global_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 仅使能地址线所在端口时钟；GPIOA 不在本驱动初始化范围内。 */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* 通用输出参数：推挽、无上下拉、超高速。 */
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    /* KH 组：PB12/PB13/PB14。 */
    GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* KL 组：PD8/PD9/PD10。 */
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* KB 组 A0：PB15。 */
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* KB 组 A1/A2：PC6/PC7。 */
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*
     * 默认全部切到 S1（000）：
     * 使用头文件内联 BSRR 函数，避免 HAL_GPIO_WritePin 带来的额外开销。
     */
    TMUX_KH_SetChannel(TMUX_CH_S1);
    TMUX_KL_SetChannel(TMUX_CH_S1);
    TMUX_KB_SetChannel(TMUX_CH_S1);
}