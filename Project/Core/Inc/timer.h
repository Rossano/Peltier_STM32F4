/*
 * timer.h
 *
 *  Created on: 23 août 2019
 *      Author: Ross
 */

#ifndef PROJECT_CORE_SRC_TIMER_H_
#define PROJECT_CORE_SRC_TIMER_H_

#include "stm32f4xx_hal.h"

/** @defgroup TIM_AutoReloadPreload TIM Auto-Reload Preload
  * @{
  */
#define TIM_AUTORELOAD_PRELOAD_DISABLE                0x00000000U               /*!< TIMx_ARR register is not buffered */
#define TIM_AUTORELOAD_PRELOAD_ENABLE                 TIM_CR1_ARPE              /*!< TIMx_ARR register is buffered */

extern TIM_HandleTypeDef htim4;		// 8
extern TIM_HandleTypeDef htim9;

void MX_TIM4_Init(void);			// 8
void MX_TIM9_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base);
void TIM4_IRQHandle();				// 8

#endif /* PROJECT_CORE_SRC_TIMER_H_ */
