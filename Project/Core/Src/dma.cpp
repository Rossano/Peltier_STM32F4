/*
 * dma.cpp
 *
 *  Created on: 26 août 2019
 *      Author: r.pantaleoni
 */

#include "stm32f4xx.h"
#include "adc.h"

DMA_HandleTypeDef hdma_adc1;

/**
  * Enable DMA controller clock
  */
void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

