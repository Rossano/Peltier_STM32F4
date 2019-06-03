/*
 * PeltierApplication.h
 *
 *  Created on: 25 avr. 2019
 *      Author: r.pantaleoni
 */

#ifndef PROJECT_PELTIERAPPLICATION_H_
#define PROJECT_PELTIERAPPLICATION_H_

#include <touchgfx/hal/HAL.hpp>
#include <touchgfx/hal/BoardConfiguration.hpp>

using namespace touchgfx;

#include "stm32f4xx_hal.h"
//#include "stm32f4xx_hal_adc.h"
#include "cmsis_os.h"
/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "usb_device.h"

#include "shell.hpp"

#define USE_USBCDC

#define PWM0_AVAILABLE	false
#define PWM1_AVAILABLE	false
#define PWM2_AVAILABLE	false
#define PWM3_AVAILABLE	false
#define PWM4_AVAILABLE	false
#define PWM10_AVAILABLE	true

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* User can use this section to tailor ADCx instance used and associated
   resources */
/* Definition for ADCx clock resources */
#define ADCx                            ADC1
#define ADCx_CLK_ENABLE()               __HAL_RCC_ADC1_CLK_ENABLE()
#define DMAx_CLK_ENABLE()               __HAL_RCC_DMA2_CLK_ENABLE()
#define ADCx_CHANNEL_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOB_CLK_ENABLE()

#define ADCx_FORCE_RESET()              __HAL_RCC_ADC_FORCE_RESET()
#define ADCx_RELEASE_RESET()            __HAL_RCC_ADC_RELEASE_RESET()

/* Definition for ADCx Channel Pin */
#define ADCx_CHANNEL_PIN                GPIO_PIN_0
#define ADCx_CHANNEL_GPIO_PORT          GPIOB

/* Definition for ADCx's Channel */
#define ADCx_CHANNEL                    ADC_CHANNEL_8

/* Definition for ADCx's DMA */
#define ADCx_DMA_CHANNEL                DMA_CHANNEL_0
#define ADCx_DMA_STREAM                 DMA2_Stream0

/* Definition for ADCx's NVIC */
#define ADCx_DMA_IRQn                   DMA2_Stream0_IRQn
#define ADCx_DMA_IRQHandler             DMA2_Stream0_IRQHandler

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

extern TIM_HandleTypeDef htim10;

/* Exported macro ------------------------------------------------------------*/

void Error_Handler(void);

class PeltierApplication {
public:
	PeltierApplication();
	virtual ~PeltierApplication();

	//TIM_HandleTypeDef htim10;
	osThreadId defaultTaskHandle;
	osThreadId shellTaskHandle;
	osThreadId coreTaskHandle;
	osMessageQId uartRXQueueHandle;
	osSemaphoreId sampleBinarySemHandle;
	cShell *shell;
	void MX_ADC1_Init();
//	void MX_USB_DEVICE_Init();
	void StartDefaultTask(void const * argument);
	void StartShellTask(void const * argument);
	void StartCoreTask(void const * argument);
	void InitPWM0();
	void InitPWM1();
	void InitPWM2();
	void InitPWM3();
	void InitPWM4();
	void InitPWM10();
	void Pwm0_Start();
	void Pwm1_Start();
	void Pwm2_Start();
	void Pwm3_Start();
	void Pwm4_Start();
	void Pwm10_Start();
	void Pwm0_Stop();
	void Pwm1_Stop();
	void Pwm2_Stop();
	void Pwm3_Stop();
	void Pwm4_Stop();
	void Pwm10_Stop();
	void SetPWM0(uint16_t val);
	void SetPWM1(uint16_t val);
	void SetPWM2(uint16_t val);
	void SetPWM3(uint16_t val);
	void SetPWM4(uint16_t val);
	void SetPWM10(uint16_t val);
	uint16_t GetPWM10();
	uint16_t getPeltierTemperature();
	uint16_t getExtTemperature();
private:
	uint16_t pwm10;
	uint16_t uPeltierRaw;
	uint16_t uTemperatureRaw;
	void MX_GPIO_Init();
	void MX_TIM10_Init();
	//void MX_USB_DEVICE_Init();
	//void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

};

extern PeltierApplication *appli;

//	Module global variables
extern uint16_t pwm10_level;

//	Exported funcion prototypes
int myatoi(const char *string);
void vPwmInit(int argc, char *argv[]);
void vPwmStart(int argc, char *argv[]);
void vPwmStop(int argc, char *argv[]);
void vPwmSet(int argc, char *argv[]);
void ADCInit(int argc, char *argv[]);
void getADC_Result(int argc, char *argv[]);

void USB_ReceivedChar(uint8_t ch);

//void xStartDefaultTask(void * argument);

#endif /* PROJECT_PELTIERAPPLICATION_H_ */
