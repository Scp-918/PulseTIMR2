/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    hrtim.c
  * @brief   This file provides code for the configuration
  *          of the HRTIM instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "hrtim.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

HRTIM_HandleTypeDef hhrtim1;

/* HRTIM1 init function */
void MX_HRTIM1_Init(void)
{

  /* USER CODE BEGIN HRTIM1_Init 0 */

  /* USER CODE END HRTIM1_Init 0 */

  HRTIM_TimeBaseCfgTypeDef pTimeBaseCfg = {0};
  HRTIM_TimerCfgTypeDef pTimerCfg = {0};
  HRTIM_CompareCfgTypeDef pCompareCfg = {0};
  HRTIM_TimerCtlTypeDef pTimerCtl = {0};
  HRTIM_OutputCfgTypeDef pOutputCfg = {0};

  /* USER CODE BEGIN HRTIM1_Init 1 */

  /* USER CODE END HRTIM1_Init 1 */
  hhrtim1.Instance = HRTIM1;
  hhrtim1.Init.HRTIMInterruptResquests = HRTIM_IT_NONE;
  hhrtim1.Init.SyncOptions = HRTIM_SYNCOPTION_SLAVE;
  hhrtim1.Init.SyncInputSource = HRTIM_SYNCINPUTSOURCE_INTERNALEVENT;
  if (HAL_HRTIM_Init(&hhrtim1) != HAL_OK)
  {
    Error_Handler();
  }
  /*
   * Master window is kept inside the 2.5ms TIM1 sync period.
   * With DIV4, one Master tick is 40ns; Period leaves 10 ticks after CMP4.
   */
  pTimeBaseCfg.Period = 30011;
  pTimeBaseCfg.RepetitionCounter = 0x00;
  pTimeBaseCfg.PrescalerRatio = HRTIM_PRESCALERRATIO_DIV4;
  pTimeBaseCfg.Mode = HRTIM_MODE_SINGLESHOT_RETRIGGERABLE;
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.InterruptRequests = HRTIM_MASTER_IT_MCMP4;
  pTimerCfg.DMARequests = HRTIM_MASTER_DMA_NONE;
  pTimerCfg.DMASrcAddress = 0x0000;
  pTimerCfg.DMADstAddress = 0x0000;
  pTimerCfg.DMASize = 0x1;
  pTimerCfg.HalfModeEnable = HRTIM_HALFMODE_DISABLED;
  pTimerCfg.InterleavedMode = HRTIM_INTERLEAVED_MODE_DISABLED;
  pTimerCfg.StartOnSync = HRTIM_SYNCSTART_ENABLED;
  pTimerCfg.ResetOnSync = HRTIM_SYNCRESET_DISABLED;
  pTimerCfg.DACSynchro = HRTIM_DACSYNC_NONE;
  pTimerCfg.PreloadEnable = HRTIM_PRELOAD_ENABLED;
  pTimerCfg.UpdateGating = HRTIM_UPDATEGATING_INDEPENDENT;
  pTimerCfg.BurstMode = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;
  pTimerCfg.RepetitionUpdate = HRTIM_UPDATEONREPETITION_ENABLED;
  pTimerCfg.ReSyncUpdate = HRTIM_TIMERESYNC_UPDATE_UNCONDITIONAL;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = 1;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  /* Master CMP2: 由 2us(50) 调整到 5us(125)。 */
  // pCompareCfg.CompareValue = 50;
  pCompareCfg.CompareValue = 200;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_2, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  /*
   * Master CMP3 后移 160us，给 34.5us 的 late CNV 脉冲组保留结束裕量。
   * 旧参数保留用于回滚对照。
   */
  // pCompareCfg.CompareValue = 24300;
  // pCompareCfg.CompareValue = 24750;
  pCompareCfg.CompareValue = 28500;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_3, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  /*
   * PA8/TA1 high time: (30001 - CMP1=1) * 40ns = 1.200ms.
   * TIM1 remains 400Hz, so each state is still 2.5ms.
   */
  pCompareCfg.CompareValue = 30001;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_4, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  /* Timer A：CNV 高电平 1.5us、低电平 15us，三个脉冲总长 34.5us。 */
  // pTimeBaseCfg.Period = 550;
  // pTimeBaseCfg.Period = 850;
  // pTimeBaseCfg.Period = 650;
  pTimeBaseCfg.Period = 3450;
  // pTimeBaseCfg.Period = 1650;
  // pTimeBaseCfg.Period = 350;
  // pTimeBaseCfg.Period = 450;
  // pTimeBaseCfg.Period = 750;
  // pTimeBaseCfg.Period = 3150;
  // pTimeBaseCfg.Period = 2050;
  pTimeBaseCfg.PrescalerRatio = HRTIM_PRESCALERRATIO_DIV1;
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCtl.UpDownMode = HRTIM_TIMERUPDOWNMODE_UP;
  pTimerCtl.TrigHalf = HRTIM_TIMERTRIGHALF_DISABLED;
  pTimerCtl.GreaterCMP3 = HRTIM_TIMERGTCMP3_EQUAL;
  pTimerCtl.GreaterCMP1 = HRTIM_TIMERGTCMP1_EQUAL;
  pTimerCtl.DualChannelDacEnable = HRTIM_TIMER_DCDE_DISABLED;
  if (HAL_HRTIM_WaveformTimerControl(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimerCtl) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.InterruptRequests = HRTIM_TIM_IT_NONE;
  pTimerCfg.DMARequests = HRTIM_TIM_DMA_NONE;
  pTimerCfg.StartOnSync = HRTIM_SYNCSTART_DISABLED;
  pTimerCfg.PushPull = HRTIM_TIMPUSHPULLMODE_DISABLED;
  pTimerCfg.FaultEnable = HRTIM_TIMFAULTENABLE_NONE;
  pTimerCfg.FaultLock = HRTIM_TIMFAULTLOCK_READWRITE;
  pTimerCfg.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_DISABLED;
  pTimerCfg.DelayedProtectionMode = HRTIM_TIMER_A_B_C_DELAYEDPROTECTION_DISABLED;
  pTimerCfg.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_NONE;
  /*
   * 当前生效配置：Timer A 由 Master CMP2 + CMP3 双触发复位，
   * 每个大周期生成两组 3 脉冲（总计 3+3）。
   */
  // pTimerCfg.ResetTrigger = HRTIM_TIMRESETTRIGGER_MASTER_CMP2;
  pTimerCfg.ResetTrigger = HRTIM_TIMRESETTRIGGER_MASTER_CMP2|HRTIM_TIMRESETTRIGGER_MASTER_CMP3;
  pTimerCfg.ResetUpdate = HRTIM_TIMUPDATEONRESET_DISABLED;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = 150;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  // pCompareCfg.CompareValue = 250;
  // pCompareCfg.CompareValue = 400;
  // pCompareCfg.CompareValue = 300;
  pCompareCfg.CompareValue = 1650;
  // pCompareCfg.CompareValue = 800;
  // pCompareCfg.CompareValue = 150;
  // pCompareCfg.CompareValue = 200;
  // pCompareCfg.CompareValue = 350;
  // pCompareCfg.CompareValue = 1550;
  // pCompareCfg.CompareValue = 1000;
  pCompareCfg.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
  pCompareCfg.AutoDelayedTimeout = 0x0000;

  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_2, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  // pCompareCfg.CompareValue = 300;
  // pCompareCfg.CompareValue = 450;
  // pCompareCfg.CompareValue = 350;
  pCompareCfg.CompareValue = 1800;
  // pCompareCfg.CompareValue = 850;
  // pCompareCfg.CompareValue = 200;
  // pCompareCfg.CompareValue = 250;
  // pCompareCfg.CompareValue = 400;
  // pCompareCfg.CompareValue = 1600;
  // pCompareCfg.CompareValue = 1050;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_3, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  // pCompareCfg.CompareValue = 500;
  // pCompareCfg.CompareValue = 800;
  // pCompareCfg.CompareValue = 600;
  pCompareCfg.CompareValue = 3300;
  // pCompareCfg.CompareValue = 1600;
  // pCompareCfg.CompareValue = 300;
  // pCompareCfg.CompareValue = 400;
  // pCompareCfg.CompareValue = 700;
  // pCompareCfg.CompareValue = 1000;

  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_4, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pOutputCfg.Polarity = HRTIM_OUTPUTPOLARITY_HIGH;
  /*
   * 事件1：TA1 在 Master CMP1 置位（而非周期起点），
   * 与“事件1拉高电桥输出”的时序定义保持一致。
   */
  // pOutputCfg.SetSource = HRTIM_OUTPUTSET_MASTERPER;
  pOutputCfg.SetSource = HRTIM_OUTPUTSET_MASTERCMP1;
  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_MASTERCMP4;
  pOutputCfg.IdleMode = HRTIM_OUTPUTIDLEMODE_NONE;
  pOutputCfg.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
  pOutputCfg.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_NONE;
  pOutputCfg.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
  pOutputCfg.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA1, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  /*
   * TA2 置位源恢复为 CMP2 + CMP3 双路径：
   * - CMP2 触发第一组 3 脉冲
   * - CMP3 触发第二组 3 脉冲
   */
  // pOutputCfg.SetSource = HRTIM_OUTPUTSET_MASTERCMP2
  //                             |HRTIM_OUTPUTSET_TIMCMP2|HRTIM_OUTPUTSET_TIMCMP4;
  pOutputCfg.SetSource = HRTIM_OUTPUTSET_MASTERCMP2|HRTIM_OUTPUTSET_MASTERCMP3
                              |HRTIM_OUTPUTSET_TIMCMP2|HRTIM_OUTPUTSET_TIMCMP4;
  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_TIMCMP1|HRTIM_OUTPUTRESET_TIMCMP3
                              |HRTIM_OUTPUTRESET_TIMPER;
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA2, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN HRTIM1_Init 2 */
  hhrtim1.Instance->sCommonRegs.CR2 |= (HRTIM_CR2_MSWU | HRTIM_CR2_TASWU);

  /* USER CODE END HRTIM1_Init 2 */
  HAL_HRTIM_MspPostInit(&hhrtim1);
}

void HAL_HRTIM_MspInit(HRTIM_HandleTypeDef* hrtimHandle)
{

  if(hrtimHandle->Instance==HRTIM1)
  {
  /* USER CODE BEGIN HRTIM1_MspInit 0 */

  /* USER CODE END HRTIM1_MspInit 0 */
    /* HRTIM1 clock enable */
    __HAL_RCC_HRTIM1_CLK_ENABLE();

    /*
     * HRTIM 中断优先级下调：
     * 采样场景下优先让 SPI3 DMA 完成中断先执行，减少 busy 跳过。
     */
    /* HRTIM1 interrupt Init */
    HAL_NVIC_SetPriority(HRTIM1_Master_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(HRTIM1_Master_IRQn);
    HAL_NVIC_SetPriority(HRTIM1_TIMA_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(HRTIM1_TIMA_IRQn);
  /* USER CODE BEGIN HRTIM1_MspInit 1 */

  /* USER CODE END HRTIM1_MspInit 1 */
  }
}

void HAL_HRTIM_MspPostInit(HRTIM_HandleTypeDef* hrtimHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hrtimHandle->Instance==HRTIM1)
  {
  /* USER CODE BEGIN HRTIM1_MspPostInit 0 */

  /* USER CODE END HRTIM1_MspPostInit 0 */

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**HRTIM1 GPIO Configuration
    PA8     ------> HRTIM1_CHA1
    PA9     ------> HRTIM1_CHA2
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF13_HRTIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN HRTIM1_MspPostInit 1 */

  /* USER CODE END HRTIM1_MspPostInit 1 */
  }

}

void HAL_HRTIM_MspDeInit(HRTIM_HandleTypeDef* hrtimHandle)
{

  if(hrtimHandle->Instance==HRTIM1)
  {
  /* USER CODE BEGIN HRTIM1_MspDeInit 0 */

  /* USER CODE END HRTIM1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_HRTIM1_CLK_DISABLE();

    /* HRTIM1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(HRTIM1_Master_IRQn);
    HAL_NVIC_DisableIRQ(HRTIM1_TIMA_IRQn);
  /* USER CODE BEGIN HRTIM1_MspDeInit 1 */

  /* USER CODE END HRTIM1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
