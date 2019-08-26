/*
 * adc.h
 *
 *  Created on: 26 août 2019
 *      Author: r.pantaleoni
 */

#ifndef PROJECT_CORE_INC_ADC_H_
#define PROJECT_CORE_INC_ADC_H_

#include "stm32f4xx_hal.h"

#define ADC_FULL_SCALE		4095
#define ADC_CHANNELS		3
#define VSENSE				( 3.3f / ADC_FULL_SCALE )

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern uint16_t adcBuffer[ADC_CHANNELS];
float temperature;

void MX_ADC1_Init(void);
void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);

#endif /* PROJECT_CORE_INC_ADC_H_ */
