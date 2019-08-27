/*
 * timer.c
 *
 *  Created on: 23 août 2019
 *      Author: Ross
 */

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "timer.h"
#include "main.h"

#include "FreeRTOS.h"
#include "PeltierApplication.h"

TIM_HandleTypeDef htim4;	// 8
TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef TimHandle;

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  *
  * ADC Sampling Timer:
  * TIMER4 72 MHz base
  * Prescaler 48000, count 500:
  * 72000000 / 48000 = 1500 Hz
  * 1500 / 3000 = 0.5Hz = 2 sec
  */
void MX_TIM4_Init(void)		// 8
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};	// New

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim4.Instance = TIM4;			// 8
  htim4.Init.Prescaler = 48000-1;	// 8
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;	// 8
  htim4.Init.Period = 300; //1750-1;		// 8
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;	// 8
  //htim8.Init.RepetitionCounter = 0;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;	// 8
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)		// 8
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)	// 8
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
	  Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)	// 8
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if(HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
	  Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 32;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 525;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @}
  */

//void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
//{
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
//  if(htim->Instance==TIM9)
//  {
//  /* USER CODE BEGIN TIM9_MspPostInit 0 */
//
//  /* USER CODE END TIM9_MspPostInit 0 */
//
//    __HAL_RCC_GPIOE_CLK_ENABLE();
//    /**TIM9 GPIO Configuration
//    PE6     ------> TIM9_CH2
//    */
//    GPIO_InitStruct.Pin = GPIO_PIN_6;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//    GPIO_InitStruct.Alternate = GPIO_AF3_TIM9;
//    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
//
//  /* USER CODE BEGIN TIM9_MspPostInit 1 */
//
//  /* USER CODE END TIM9_MspPostInit 1 */
//  }
//
//}

/**
* @brief TIM_Base MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param htim_base: TIM_Base handle pointer
* @retval None
*/
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim->Instance==TIM9)
  {
  /* USER CODE BEGIN TIM9_MspPostInit 0 */

  /* USER CODE END TIM9_MspPostInit 0 */

    __HAL_RCC_GPIOE_CLK_ENABLE();
    /**TIM9 GPIO Configuration
    PE6     ------> TIM9_CH2
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM9;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM9_MspPostInit 1 */

  /* USER CODE END TIM9_MspPostInit 1 */
  }

}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspDeInit 0 */

  /* USER CODE END TIM1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM1_CLK_DISABLE();

    /* TIM1 interrupt DeInit */
    HAL_NVIC_DisableIRQ(TIM1_TRG_COM_TIM11_IRQn);
  /* USER CODE BEGIN TIM1_MspDeInit 1 */

  /* USER CODE END TIM1_MspDeInit 1 */
  }
  else if(htim_base->Instance==TIM4)	// * 8
  {
  /* USER CODE BEGIN TIM8_MspDeInit 0 */

  /* USER CODE END TIM8_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM4_CLK_DISABLE();	// * 8
  /* USER CODE BEGIN TIM8_MspDeInit 1 */

  /* USER CODE END TIM8_MspDeInit 1 */
  }
  else if(htim_base->Instance==TIM9)
  {
  /* USER CODE BEGIN TIM9_MspDeInit 0 */

  /* USER CODE END TIM9_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM9_CLK_DISABLE();
  /* USER CODE BEGIN TIM9_MspDeInit 1 */

  /* USER CODE END TIM9_MspDeInit 1 */
  }

}

/**
 * TIMER4 Interrupt Hander
 *
 * Just unlock xSamplingTimerSemaphore for the default task
 */
void TIM4_IRQHandle()	// 8
{
	BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSamplingTimerSemaphore, &xHigherPriorityTaskWoken);
}

