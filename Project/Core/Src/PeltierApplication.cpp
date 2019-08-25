/*
 * PeltierApplication.cpp
 *
 *  Created on: 25 avr. 2019
 *      Author: r.pantaleoni
 */

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_adc.h"
#include "stm32f4xx_hal_pcd.h"

#include "timer.h"
#include "usbd_desc.h"
#include "usbd_core.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"

#include "FreeRTOS.h"
#include "queue.h"

#include "main.h"
#include "PeltierApplication.h"

#include <stdlib.h>

#include <gui/mainscreen_screen/MainScreenView.hpp>
#include <gui\model\Model.hpp>


#define VSENSE									( 3.3f / 4095 )

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim6;
//TIM_HandleTypeDef htim8;
//TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;
SemaphoreHandle_t xSamplingTimerSemaphore = NULL;

uint16_t pwm10_level;

//uint8_t lineBuf[SHELL_MAX_LINE_LENGTH];
cdc_buf_t lineBuf;
bool bEOL = false;
uint8_t lenghtBuf = 0;
QueueHandle_t xLowLevelData;

extern uint16_t adcBuffer[ADC_CHANNELS];
uint16_t buffer[ADC_CHANNELS];
float temperature;

PeltierApplication::PeltierApplication() {
	// TODO Auto-generated constructor stub

	// Create Lowlevel queue
	xLowLevelData = xQueueCreate(3, sizeof(msg));

	/* USER CODE BEGIN RTOS_MUTEX */
	  /* add mutexes, ... */
	  /* USER CODE END RTOS_MUTEX */

	  /* Create the semaphores(s) */
	  /* definition and creation of sampleBinarySem */
//	  osSemaphoreDef(sampleBinarySem);
//	  sampleBinarySemHandle = osSemaphoreCreate(osSemaphore(sampleBinarySem), 1);

	  /* USER CODE BEGIN RTOS_SEMAPHORES */
	  /* add semaphores, ... */
	  /* USER CODE END RTOS_SEMAPHORES */

	  /* USER CODE BEGIN RTOS_TIMERS */
	  /* start timers, add new ones, ... */
	  /* USER CODE END RTOS_TIMERS */

	  /* Create the queue(s) */
	  /* definition and creation of uartRXQueue */
//	  osMessageQDef(uartRXQueue, 16, uint8_t);
//	  uartRXQueueHandle = osMessageCreate(osMessageQ(uartRXQueue), NULL);

	  /* USER CODE BEGIN RTOS_QUEUES */
	  /* add queues, ... */
	  /* USER CODE END RTOS_QUEUES */

	  /* Create the thread(s) */
	  /* definition and creation of defaultTask */
	  ///osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 4096);
	  ///defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

	  /* definition and creation of shellTask */
	  ///osThreadDef(shellTask, StartShellTask, osPriorityNormal, 0, 128);
	  ///shellTaskHandle = osThreadCreate(osThread(shellTask), NULL);

	  /* definition and creation of coreTask */
	  ///osThreadDef(coreTask, StartCoreTask, osPriorityNormal, 0, 128);
	  ///coreTaskHandle = osThreadCreate(osThread(coreTask), NULL);

	  /* USER CODE BEGIN RTOS_THREADS */
	  /* add threads, ... */
	  /* USER CODE END RTOS_THREADS */
	  cShell *shell = new cShell();
}

PeltierApplication::~PeltierApplication() {
	// TODO Auto-generated destructor stub
}




void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	while (true) ;
  /* USER CODE END Error_Handler_Debug */
}

/// <summary>
/// 16 bit clipping function
/// It returns the level of the input after clipping.
/// </summary>
/// <param name="level">Value to clip on 8 bits</param>
uint16_t clip16(uint16_t level)
{
	//	If the level value is higher than 65535 (2^16) it returns 65535 else it returns the level value
	if(level > 65535) return 65535;
	else return level;
}

/// <summary>
/// 10 bit clipping function
/// It returns the level of the input after clipping.
/// </summary>
/// <param name="level">Value to clip on 10 bits</param>
uint16_t clip10(uint16_t level)
{
	//	If the level value is higher than 1023 (2^10) it returns 1023 else it returns the level value
	if(level > 1023) return 1023;
	else return level;
}

/// <summary>
/// 8 bit clipping function
/// It returns the level of the input after clipping.
/// </summary>
/// <param name="level">Value to clip on 8 bits</param>
uint16_t clip8(uint16_t level)
{
	//	If the level value is higher than 1023 (2^10) it returns 1023 else it returns the level value
	if(level > 255) return 255;
	else return level;
}

/// <summary>
/// USB CDC Receive Callback function
/// It is called when data (chars) are received, it process the rx char
/// and create a new request for the shell task when a CR is received
/// </summary>
/// <param name='Buf">Buffer storing the received data</param>
/// <param name="Len">Lenght of the buffer</param>
void CDC_ReceiveCallback(uint8_t *Buf, uint32_t *Len)
{
	//CDC_Transmit_HS(Buf, 1);//(uint16_t *)Len);
	const TickType_t ticks = pdMS_TO_TICKS(10000); //portMAX_DELAY;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	///
	///	Checkif it is an EOL, in that case push the command in the rx FIFO for the shell
	///
	if(*Buf == 0 || *Buf == 13 || *Buf == 10 || lenghtBuf == SHELL_MAX_LINE_LENGTH)
	{
		bEOL = true;
		// Indication that new command is received
		CDC_Transmit_HS((uint8_t *)"\n\rNew Command:", 16);
		lineBuf[lenghtBuf] = 10;
		lineBuf[lenghtBuf+1] = 13;
		uint32_t foo = lenghtBuf + 2;
		CDC_Transmit_HS(lineBuf, foo);
		CDC_Transmit_HS((uint8_t *)"\r\nSTM32> ", 9);
		//lineBuf[lenghtBuf] = 0;
		//lineBuf[lenghtBuf+1] = 0;
		for(uint8_t i=lenghtBuf; i<SHELL_MAX_LINE_LENGTH; i++) lineBuf[i] = 0;
		//	Send the string into the rx FIFO
		BaseType_t xStatus;
		xStatus = xQueueSendToBackFromISR(xUsb_rx, lineBuf, &xHigherPriorityTaskWoken);
		if(xStatus != pdPASS)
		{
			/// Error
		}
		lenghtBuf = 0;
		for(uint8_t i=0; i<SHELL_MAX_LINE_LENGTH; i++) lineBuf[i] = 0;
	}
	else
	{
		lineBuf[lenghtBuf++] = *Buf;
		CDC_Transmit_HS(Buf, 1);
	}
}
/// <summary>
/// PWM10 Initialization function
/// </summary>
/// <param name="level">PWM10 new level value</param>
void SetPWM10(uint16_t level)
{
	#ifdef DEBUG
	iprintf("Entering SetPWM4 Set...\r\n");
	#endif // DEBUG

	//	Clip on 8 bits the level value before
	pwm10_level = clip16(level);
	//	Update the PWM level
	//analogWrite(PWM4_OUT_PIN, pwm4_level);
	TIM_OC_InitTypeDef sConfigOC = {0};
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 1000;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
}

/// <summary>
/// Shell PWM  Initialization command.
///  This is the implementation of the shell command pwm_init, it initializes a PWM channel
/// </summary>
/// <param name="argc">Number of parameters</param>
/// <param name="argv">List of parameters</param>
void vPwmInit(int argc, char *argv[])
{
	#ifdef DEBUG
	iprintf("Entering vPWMInit...\r\n");
	iprintf("argv[0] -> "); iprintf(argv[0]); iprintf(CR);
	iprintf("argv[1] -> "); iprintf(argv[1]); iprintf(CR);
	#endif // DEBUG

	//	If the number of arguments is not 1 report an error and show how to use the command
	//if (argc < 1)
	if (argc != 1)
	{
		vUsage((char *)"pwm_init <PWM Channel>");
	}
	else
	{
		//	Otherwise convert the first parameter from char to number
		uint8_t channel = atoi(argv[0]);
		if ((channel >= 0) && (channel <= 4) || channel == 10)
		{
			//	If the channel is between 0 and 4 decode it and launch the corresponding function, with an ACK at the end
			//	The function is launched only if the corresponding PWM channel is available
			switch(channel)
			{
				case 0: if (PWM0_AVAILABLE)
					{
						appli->InitPWM0();
						iprintf("OK\r\n");
					}
					break;
				case 1: if (PWM1_AVAILABLE)
					{
						appli->InitPWM1();
						iprintf("OK\r\n");
					}
					break;
				case 2: if (PWM2_AVAILABLE)
					{
						appli->InitPWM2();
						iprintf("OK\r\n");
					}
					break;
				case 3: if (PWM3_AVAILABLE)
					{
						appli->InitPWM3();
						iprintf("OK\r\n");
					}
					break;
				case 4: if (PWM4_AVAILABLE)
					{
						appli->InitPWM4();
						//iprintf("Passed to InitPWM4\r\n");
						iprintf("OK\r\n");
					}
					break;
				case 10: if (PWM10_AVAILABLE)
				{
					appli->InitPWM10();
					iprintf("OK\r\n");
				}
				break;
				default:
					//	Shouldn't pass here just in case report an error message
					iprintf("Error: Bad PWM channel!\r\n");
					vUsage((char *)"pwm_init <PWM Channel>");
					break;
			}
		}
		//	Found an invalid channel, report an error
		else iprintf("Error: Bad PWM Channel\r\n");
	}
}

/// <summary>
/// Shell PWM  Start command.
///  This is the implementation of the shell command pwm_start, it starts a PWM channel
/// </summary>
/// <param name="argc">Number of parameters</param>
/// <param name="argv">List of parameters</param>
void vPwmStart(int argc, char *argv[])
{
	#ifdef DEBUG
	iprintf("Entering vPwmStart...\r\b");
	#endif // DEBUG

	//	If the number or arguments is not 1 report an error message and show how to use the command
	if(argc != 1)
	//if (argc < 1)
	{
		vUsage((char *)"pwm_start <pwm channel>");
	}
	else
	{
		//	Otherwise convert the channel from char to numeric value
		uint8_t ch = (uint8_t)atoi(argv[0]);
		//	Decode the channel value, and launch the corresponding PWM Start funcion
		switch (ch)
		{
			case 0: if(PWM0_AVAILABLE)
				{
					appli->Pwm0_Start();
					iprintf("OK\r\n");
				}
				break;
			case 1: if (PWM1_AVAILABLE)
				{
					appli->Pwm1_Start();
					iprintf("OK\r\n");
				}
				break;
			case 2: if(PWM2_AVAILABLE)
				{
					appli->Pwm2_Start();
					iprintf("OK\r\n");
				}
				break;
			case 3: if(PWM3_AVAILABLE)
				{
					appli->Pwm3_Start();
					iprintf("OK\r\n");
				}
				break;
			case 4: if (PWM4_AVAILABLE)
				{
					appli->Pwm4_Start();
					iprintf("OK\r\n");
				}
				break;
			case 10: if(PWM10_AVAILABLE)
			{
				appli->Pwm10_Start();
				iprintf("OK\r\n");
			}
			break;
				//	If a channel is not between 0 and 4 it reports an error message
			default: iprintf("Error: PWM Channel not valid\r\n");
				break;
		}
	}
	//iprintf("OK\r\n");
}

/// <summary>
/// Shell PWM  Stop command.
///  This is the implementation of the shell command pwm_stop, it stops a PWM channel
/// </summary>
/// <param name="argc">Number of parameters</param>
/// <param name="argv">List of parameters</param>
void vPwmStop(int argc, char *argv[])
{
	#ifdef DEBUG
	iprintf("Entering vPwmStop...\r\b");
	#endif // DEBUG

	//	If the number of arguments is not 1 report an error message a displayt how to use the command
	if(argc != 1)
	//if (argc < 1)
	{
		vUsage((char *)"pwm_stop <pwm channel>");
	}
	else
	{
		//	Otherwise convert the channel from char to numeric value
		uint8_t ch = atoi(argv[0]);
		//	Decode the channel value and launch the corresponding stop function if the channel is available
		switch(ch)
		{
			case 0: if (PWM0_AVAILABLE)
				{
					appli->Pwm0_Stop();
					iprintf("OK\r\n");
				}
				break;
			case 1: if(PWM1_AVAILABLE)
				{
					appli->Pwm1_Stop();
					iprintf("OK\r\n");
				}
				break;
			case 2: if (PWM2_AVAILABLE)
				{
					appli->Pwm2_Stop();
					iprintf("OK\r\n");
				}
				break;
			case 3: if (PWM3_AVAILABLE)
				{
					appli->Pwm3_Stop();
					iprintf("OK\r\n");
				}
				break;
			case 4: if (PWM4_AVAILABLE)
				{
					appli->Pwm4_Stop();
					iprintf("OK\r\n");
				}
				break;
			case 10: if(PWM10_AVAILABLE)
			{
				appli->Pwm10_Stop();
				iprintf("OK\r\n");
			}
			break;
				//	If none of the above report an error message
			default: iprintf("Error: invalid PWM Channel\r\n"); break;
		}
	}
}

/// <summary>
/// Shell PWM  Setting command.
///  This is the implementation of the shell command pwm_set, it sets a PWM channel to a new value
/// </summary>
/// <param name="argc">Number of parameters</param>
/// <param name="argv">List of parameters</param>
void vPwmSet(int argc, char *argv[])
{
	#ifdef DEBUG
	iprintf("Entering vPWMSet...\r\n");
	#endif // DEBUG

	//	If the number of parameters is different than 2 there is an error, then display the usage
	if (argc != 2)
	{
		vUsage((char *)"pwm_set <PWM Channel> <value>");
	}
	else
	{
		//	Checks if the first parameters is a valid PWM channel (between 0 and 4)
		if ((*argv[1] >= '0') && (*argv[1] <= '4'))
		{
			//	If yes convert it first from char to numeric value
			uint8_t channel = atoi(argv[0]);
			uint16_t level = atoi(argv[1]);
			//	Then select the PWM channel and launch the corresponding action for that channel
			switch(channel)
			{
				case 0: if (PWM0_AVAILABLE)
				{
					appli->SetPWM0(level);
					iprintf("OK\r\n");
				}
				break;
				case 1: if(PWM1_AVAILABLE)
				{
					appli->SetPWM1(level);
					iprintf("OK\r\n");
				}
				break;
				case 2: if(PWM2_AVAILABLE)
				{
					appli->SetPWM2(level);
					iprintf("OK\r\n");
				}
				break;
				case 3: if(PWM3_AVAILABLE)
				{
					appli->SetPWM3(level);
					iprintf("OK\r\n");
				}
				break;
				case 4: if(PWM4_AVAILABLE)
				{
					appli->SetPWM4(level);
					iprintf("OK\r\n");
				}
				break;
				case 10: if(PWM10_AVAILABLE)
				{
					appli->SetPWM10(level);
					iprintf("OK\r\n");
				}
				break;
				default:
				//	Shouldn't go there, in case report an error to the shell
				iprintf("Error: Bad PWM channel!\r\n");
				vUsage((char *)"pwm_init <PWM Channel>");
				break;
			}
		}
	}
	//	Display an ACK to the shell that everything is fine
	iprintf("OK\r\n");
}

void ADCInit(int argc, char *argv[])
{
	// Configure the ADC 1
//	appli->MX_ADC1_Init();
#if 0
	MX_ADC1_Init();
#endif
	// Configure the GPIO to supply the temperature sensor

}

void getADC_Result(int argc, char *argv[])
{
	//	If the number of parameters is different than 2 there is an error, then display the usage
	if (argc != 2)
	{
		vUsage((char *)"adc_get <ADC Channel>");
	}
	else
	{
		//	Convert first the channel from char to numeric value
		uint8_t channel = atoi(argv[0]);
		//	Then select the ADC channel and launch the corresponding conversion for that channel
		switch(channel)
		{
			case 5: iprintf("Peltier T: %d\r\n", appli->getPeltierTemperature());
			break;
			case 13: iprintf("Ext T: %d\r\n", appli->getExtTemperature());
			break;
			default: iprintf("Channel not available\r\n");
		}
	}
}

/// <summary>
/// Set PWM 10 value.
/// Wrap-up to set the value of the private member pwm10
/// </summary>
/// <param name="val">New pwm10 value</param>
void PeltierApplication::SetPWM10(uint16_t val)
{
	pwm10 = val;
}

/// <summary>
/// Get PWM 10 value
/// Wrap-up to get the value of the private member pwm10
/// </summary>
/// <param name="return">PWM10 value</param>
uint16_t PeltierApplication::GetPWM10()
{
	return pwm10;
}

/// <summary>
/// Initialize the PWM on the TIMER 10.
/// </summary>
void PeltierApplication::InitPWM10()
{
//	MX_TIM10_Init();
}

/// <summary>
/// Start the PWM on the TIMER 10.
/// </summary>
void PeltierApplication::Pwm10_Start()
{
	/*if(HAL_TIM_Base_Start_IT(&htim10) != HAL_OK)
	{
		Error_Handler();
	}
	while (1) ;*/
    __HAL_RCC_TIM10_CLK_ENABLE();
}

/// <summary>
/// Stop the PWM on the TIMER 10.
/// </summary>
void PeltierApplication::Pwm10_Stop()
{
	/*if (HAL_TIM_Base_Stop(&htim10) != HAL_OK)
	{
		Error_Handler();
	}
	while (1) ;*/
	//HAL_TIM_MspeInit(&htim10);
    __HAL_RCC_TIM10_CLK_DISABLE();
}

uint16_t PeltierApplication::getPeltierTemperature()
{
	ADC_ChannelConfTypeDef sConfig = {0};
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	*/
	sConfig.Channel = ADC_CHANNEL_5;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	// Check if there is a pending conversion
	if(HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK)
	{
		// End of conversion flag not set on time
		Error_Handler();
	}
	// Check if the conversion is finished
	if((HAL_ADC_GetState(&hadc1) & HAL_ADC_STATE_EOC_REG) == HAL_ADC_STATE_EOC_REG)
	{
		// Get the converted value
		uPeltierRaw = HAL_ADC_GetValue(&hadc1);
	}
}

uint16_t PeltierApplication::getExtTemperature()
{
	ADC_ChannelConfTypeDef sConfig = {0};
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	*/
	sConfig.Channel = ADC_CHANNEL_13;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	// Check if there is a pending conversion
	if(HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK)
	{
		// End of conversion flag not set on time
		Error_Handler();
	}
	// Check if the conversion is finished
	if((HAL_ADC_GetState(&hadc1) & HAL_ADC_STATE_EOC_REG) == HAL_ADC_STATE_EOC_REG)
	{
		// Get the converted value
		uTemperatureRaw = HAL_ADC_GetValue(&hadc1);
	}
}

void USB_ReceivedChar(uint8_t ch)
{
	if(ch == '\r' || ch == '\n')
	{
		bEOL = true;
		lineBuf[lenghtBuf] = 0;
	}
	else if(lenghtBuf < SHELL_MAX_LINE_LENGTH)
	{
		lineBuf[lenghtBuf++] = ch;
	}
}

static void xStartDefaultTask(void * argument)
{
  /* init code for USB_HOST */
//  MX_USB_HOST_Init();

  /* USER CODE BEGIN 5 */
	  /* init code for USB_DEVICE */
//	  MX_USB_DEVICE_Init();
  /* Infinite loop */
	  uint8_t myBuf[20] = "Ciao Belo\r\n";
	  float temperature;
	  xSamplingTimerSemaphore = xSemaphoreCreateBinary();
	  HAL_TIM_Base_Start_IT(&htim4);		// 8

  for(;;)
  {
    //osDelay(1);
//	  CDC_Transmit_HS(myBuf, sizeof(myBuf));
//	  HAL_Delay(500);
	  /** HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcBuffer, ADC_CHANNELS);*/
	  if(xSemaphoreTake(xSamplingTimerSemaphore, 0xffff))
	  {

	  }

	  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)buffer, ADC_CHANNELS);
	  temperature  = ((float) adcBuffer[0] / ADC_FULL_SCALE * 3300);
//	  model->setPeltierTemperature((temperature - 760) / 2.5 + 25);
	  msg.peltier = temperature;
	  temperature  = ((float) adcBuffer[2] / ADC_FULL_SCALE * 3300);
	  msg.ext = temperature;
	  msg.pwm = 128;
	  BaseType_t status;
	  	  if((status = xQueueSend(xLowLevelData, &msg, 0)) == pdTRUE)
	  {

	  }
	  //osDelay(1500);
	 // HAL_Delay(1500);

  }
  /* USER CODE END 5 */
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    temperature = (((adcBuffer[0]*VSENSE)-.76)/.0025)+25;
}
