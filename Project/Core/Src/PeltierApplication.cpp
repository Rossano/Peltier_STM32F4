/*
 * PeltierApplication.cpp
 *
 *  Created on: 25 avr. 2019
 *      Author: r.pantaleoni
 */

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_adc.h"
#include "stm32f4xx_hal_pcd.h"

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


/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;

uint16_t pwm10_level;

//uint8_t lineBuf[SHELL_MAX_LINE_LENGTH];
cdc_buf_t lineBuf;
bool bEOL = false;
uint8_t lenghtBuf = 0;
QueueHandle_t xLowLevelData;

PeltierApplication::PeltierApplication() {
	// TODO Auto-generated constructor stub

	MX_GPIO_Init();
	MX_ADC1_Init();
	MX_TIM10_Init();

	// Create Lowlevel queue
	xQueueCreate(3, sizeof(msg));

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

void PeltierApplication::MX_GPIO_Init()
{
//	 GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//	  /* GPIO Ports Clock Enable */
//	  __HAL_RCC_GPIOE_CLK_ENABLE();
//	  __HAL_RCC_GPIOC_CLK_ENABLE();
//	  __HAL_RCC_GPIOF_CLK_ENABLE();
//	  __HAL_RCC_GPIOH_CLK_ENABLE();
//	  __HAL_RCC_GPIOA_CLK_ENABLE();
//	  __HAL_RCC_GPIOB_CLK_ENABLE();
//	  __HAL_RCC_GPIOG_CLK_ENABLE();
//	  __HAL_RCC_GPIOD_CLK_ENABLE();
//
//	  /*Configure GPIO pin Output Level */
//	  HAL_GPIO_WritePin(GPIOE, Peltier_VDD_Pin|Ext_VDD_Pin, GPIO_PIN_RESET);
//
//	  /*Configure GPIO pin Output Level */
//	  HAL_GPIO_WritePin(GPIOC, NCS_MEMS_SPI_Pin|CSX_Pin|OTG_FS_PSO_Pin, GPIO_PIN_RESET);
//
//	  /*Configure GPIO pin Output Level */
//	  HAL_GPIO_WritePin(ACP_RST_GPIO_Port, ACP_RST_Pin, GPIO_PIN_RESET);
//
//	  /*Configure GPIO pin Output Level */
//	  HAL_GPIO_WritePin(GPIOD, RDX_Pin|WRX_DCX_Pin, GPIO_PIN_RESET);
//
//	  /*Configure GPIO pin Output Level */
//	  HAL_GPIO_WritePin(GPIOG, LD3_Pin|LD4_Pin, GPIO_PIN_RESET);
//
//	  /*Configure GPIO pins : Peltier_VDD_Pin Ext_VDD_Pin */
//	  GPIO_InitStruct.Pin = Peltier_VDD_Pin|Ext_VDD_Pin;
//	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//	  GPIO_InitStruct.Pull = GPIO_NOPULL;
//	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//	  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
//
//	  /*Configure GPIO pins : NCS_MEMS_SPI_Pin CSX_Pin OTG_FS_PSO_Pin */
//	  GPIO_InitStruct.Pin = NCS_MEMS_SPI_Pin|CSX_Pin|OTG_FS_PSO_Pin;
//	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//	  GPIO_InitStruct.Pull = GPIO_NOPULL;
//	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
//
//	  /*Configure GPIO pins : B1_Pin MEMS_INT1_Pin MEMS_INT2_Pin TP_INT1_Pin */
//	  GPIO_InitStruct.Pin = B1_Pin|MEMS_INT1_Pin|MEMS_INT2_Pin|TP_INT1_Pin;
//	  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
//	  GPIO_InitStruct.Pull = GPIO_NOPULL;
//	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//
//	  /*Configure GPIO pin : ACP_RST_Pin */
//	  GPIO_InitStruct.Pin = ACP_RST_Pin;
//	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//	  GPIO_InitStruct.Pull = GPIO_NOPULL;
//	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//	  HAL_GPIO_Init(ACP_RST_GPIO_Port, &GPIO_InitStruct);
//
//	  /*Configure GPIO pin : OTG_FS_OC_Pin */
//	  GPIO_InitStruct.Pin = OTG_FS_OC_Pin;
//	  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
//	  GPIO_InitStruct.Pull = GPIO_NOPULL;
//	  HAL_GPIO_Init(OTG_FS_OC_GPIO_Port, &GPIO_InitStruct);
//
//	  /*Configure GPIO pin : BOOT1_Pin */
//	  GPIO_InitStruct.Pin = BOOT1_Pin;
//	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//	  GPIO_InitStruct.Pull = GPIO_NOPULL;
//	  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);
//
//	  /*Configure GPIO pin : TE_Pin */
//	  GPIO_InitStruct.Pin = TE_Pin;
//	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//	  GPIO_InitStruct.Pull = GPIO_NOPULL;
//	  HAL_GPIO_Init(TE_GPIO_Port, &GPIO_InitStruct);
//
//	  /*Configure GPIO pins : RDX_Pin WRX_DCX_Pin */
//	  GPIO_InitStruct.Pin = RDX_Pin|WRX_DCX_Pin;
//	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//	  GPIO_InitStruct.Pull = GPIO_NOPULL;
//	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//	  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
//
//	  /*Configure GPIO pins : LD3_Pin LD4_Pin */
//	  GPIO_InitStruct.Pin = LD3_Pin|LD4_Pin;
//	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//	  GPIO_InitStruct.Pull = GPIO_NOPULL;
//	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//	  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	  GPIO_InitTypeDef GPIO_InitStruct = {0};

	  /* GPIO Ports Clock Enable */
	  __HAL_RCC_GPIOC_CLK_ENABLE();
	  __HAL_RCC_GPIOF_CLK_ENABLE();
	  __HAL_RCC_GPIOH_CLK_ENABLE();
	  __HAL_RCC_GPIOA_CLK_ENABLE();
	  __HAL_RCC_GPIOB_CLK_ENABLE();
	  __HAL_RCC_GPIOG_CLK_ENABLE();
	  __HAL_RCC_GPIOE_CLK_ENABLE();
	  __HAL_RCC_GPIOD_CLK_ENABLE();

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOC, NCS_MEMS_SPI_Pin|CSX_Pin|OTG_FS_PSO_Pin, GPIO_PIN_RESET);

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(ACP_RST_GPIO_Port, ACP_RST_Pin, GPIO_PIN_RESET);

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOD, RDX_Pin|WRX_DCX_Pin, GPIO_PIN_RESET);

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOG, LD3_Pin|LD4_Pin, GPIO_PIN_RESET);

	  /*Configure GPIO pins : PCPin PCPin PCPin */
	  GPIO_InitStruct.Pin = NCS_MEMS_SPI_Pin|CSX_Pin|OTG_FS_PSO_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	  /*Configure GPIO pins : PAPin PAPin PAPin PAPin */
	  GPIO_InitStruct.Pin = B1_Pin|MEMS_INT1_Pin|MEMS_INT2_Pin|TP_INT1_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /*Configure GPIO pin : PtPin */
	  GPIO_InitStruct.Pin = ACP_RST_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(ACP_RST_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pin : PtPin */
	  GPIO_InitStruct.Pin = OTG_FS_OC_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(OTG_FS_OC_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pin : PtPin */
	  GPIO_InitStruct.Pin = BOOT1_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pin : PtPin */
	  GPIO_InitStruct.Pin = TE_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(TE_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pins : PDPin PDPin */
	  GPIO_InitStruct.Pin = RDX_Pin|WRX_DCX_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	  /*Configure GPIO pins : PGPin PGPin */
	  GPIO_InitStruct.Pin = LD3_Pin|LD4_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

void PeltierApplication::MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
void PeltierApplication::MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 16;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 10499;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  __HAL_TIM_SET_AUTORELOAD(&htim10, 10499);

  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */
  //HAL_TIM_MspPostInit(&htim10);

}

#if 0
void PeltierApplication::MX_USB_DEVICE_Init()
{
	  /* USER CODE BEGIN USB_DEVICE_Init_PreTreatment */

	  /* USER CODE END USB_DEVICE_Init_PreTreatment */

	  /* Init Device Library, add supported class and start the library. */
	  USBD_Init(&hUsbDeviceHS, &HS_Desc, DEVICE_HS);

	  USBD_RegisterClass(&hUsbDeviceHS, &USBD_CDC);

	  USBD_CDC_RegisterInterface(&hUsbDeviceHS, &USBD_Interface_fops_HS);

	  USBD_Start(&hUsbDeviceHS);

	  /* USER CODE BEGIN USB_DEVICE_Init_PostTreatment */

	  /* USER CODE END USB_DEVICE_Init_PostTreatment */
}
#endif

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void PeltierApplication::StartDefaultTask(void const * argument)
{
  /* init code for USB_HOST */
//  MX_USB_HOST_Init();

  /* USER CODE BEGIN 5 */
	  /* init code for USB_DEVICE */
	  MX_USB_DEVICE_Init();
  /* Infinite loop */
	  uint8_t myBuf[20] = "Ciao Belo\r\n";
	  float temperature;
  for(;;)
  {
    //osDelay(1);
//	  CDC_Transmit_HS(myBuf, sizeof(myBuf));
//	  HAL_Delay(500);
	  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcBuffer, ADC_CHANNELS);
	  temperature  = ((float) adcBuffer[3] / ADC_FULL_SCALE * 3300);
//	  model->setPeltierTemperature((temperature - 760) / 2.5 + 25);
	  msg.peltier = temperature;
	  temperature  = ((float) adcBuffer[2] / ADC_FULL_SCALE * 3300);
	  msg.ext = temperature;
	  msg.pwm = 128;
	  BaseType_t status;
	  if((status = xQueueSend(xLowLevelData, &msg, 0)) == pdTRUE)
	  {

	  }
	  osDelay(500);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartShellTask */
/**
* @brief Function implementing the shellTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartShellTask */
void PeltierApplication::StartShellTask(void const * argument)
{
  /* USER CODE BEGIN StartShellTask */
//  char line[SHELL_MAX_LINE_LENGTH];
  // chRegSetThreadName("shell");

  iprintf("%s STM32 Shell;%S",CR,CR);
  /* Infinite loop */
  for(;;)
  {
	  // Display the prompt
	  iprintf(SHELL_PROMPT);
#ifdef USE_USBCDC
	  if(bEOL)
	  {
		  //shell->ShellTask((void *)argument, line);
		  bEOL = false;
	  }
#else
	  // Get the command line from stdin
	  gets(line);
	  // Calls the shell task
	  shell->ShellTask((void *)argument, line);
#endif
    osDelay(1);
  }
  /* USER CODE END StartShellTask */
}

/* USER CODE BEGIN Header_StartCoreTask */
/**
* @brief Function implementing the coreTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCoreTask */
void PeltierApplication::StartCoreTask(void const * argument)
{
  /* USER CODE BEGIN StartCoreTask */
  /* Infinite loop */
  for(;;)
  {
	iprintf("Tick%s", CR);
    osDelay(1000);
  }
  /* USER CODE END StartCoreTask */
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
	appli->MX_ADC1_Init();

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
	MX_TIM10_Init();
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
  for(;;)
  {
    //osDelay(1);
	  CDC_Transmit_HS(myBuf, sizeof(myBuf));
	  HAL_Delay(500);
  }
  /* USER CODE END 5 */
}
