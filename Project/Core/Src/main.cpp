/******************************************************************************
 *
 * @brief     This file is part of the TouchGFX 4.8.0 evaluation distribution.
 *
 * @author    Draupner Graphics A/S <http://www.touchgfx.com>
 *
 ******************************************************************************
 *
 * @section Copyright
 *
 * This file is free software and is provided for example purposes. You may
 * use, copy, and modify within the terms and conditions of the license
 * agreement.
 *
 * This is licensed software for evaluation use, any use must strictly comply
 * with the evaluation license agreement provided with delivery of the
 * TouchGFX software.
 *
 * The evaluation license agreement can be seen on www.touchgfx.com
 *
 * @section Disclaimer
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Draupner Graphics A/S has
 * no obligation to support this software. Draupner Graphics A/S is providing
 * the software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Draupner Graphics A/S can not be held liable for any consequential,
 * incidental, or special damages, or any other relief, or for any claim by
 * any third party, arising from your use of this software.
 *
 *****************************************************************************/

#include <touchgfx/hal/HAL.hpp>
#include <touchgfx/hal/BoardConfiguration.hpp>

using namespace touchgfx;

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "usbd_cdc_if.h"
#include "PeltierApplication.h"
#include "shell.hpp"

#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "usbd_conf.h"
#include "usbd_desc.h"
#include "core_cm4.h"

#include <stdio.h>
/**
 * Define the FreeRTOS task priorities and stack sizes
 */
#define configGUI_TASK_PRIORITY                 ( tskIDLE_PRIORITY + 3 )

#define configGUI_TASK_STK_SIZE                 ( 1024 )

#define USE_USB

/*uint8_t usb_cdc_rx_char;
cdc_buf_t usb_cdc_buffer;
uint8_t newCmd;
uint8_t new_char_on_usb; */
QueueHandle_t xUsb_tx, xUsb_rx;

static void GUITask(void* params)
{
    touchgfx::HAL::getInstance()->taskEntry();
}

static void DefaultTask(void *p)
{
	void const *foo;
	appli->StartDefaultTask(foo);
}

static void ShellTask(void *p)
{
	appli->shell->vShellThread((void *)ShellCommand);
/*	cdc_buf_t line;
	for(uint8_t i=0; i<sizeof(cdc_buf_t); i++) line[i] = 0;
	while (true)
	{
		BaseType_t xStatus;
		xStatus = xQueueReceive(xUsb_rx, &line, 100);
		if(xStatus != pdPASS)
		{
			// ERROR
		}
		// Check if there was a message awaiting or it was just a timeout
		if(line[0] != 0)
		{
			// Calls the shell task
			appli->shell->ShellTask(p, (char *)line);
		}
	}*/
}

static void xStartDefaultTask(void * argument)
{
  /* init code for USB_HOST */
//  MX_USB_HOST_Init();

  uint8_t pos = 0;
  const TickType_t ticks = pdMS_TO_TICKS(50);
  cdc_buf_t buffer;
  cdc_buf_t data_to_send;
/*  xQueueCreate(3, sizeof(cdc_buf_t));
  xQueueCreate(3, sizeof(cdc_buf_t)); */

  /* USER CODE BEGIN 5 */
	  /* init code for USB_DEVICE */
  /* Infinite loop */
//	  uint8_t myBuf[20] = "Ciao Belo\r\n";
  ///
  ///	Print the PROMPT
  ///
  CDC_Transmit_HS((uint8_t*)"\r\nSTM32> ", 9);
  for(;;)
  {
    //osDelay(1);
#ifdef USE_USB
	  ///
	  /// Task only manage to send the outcoming messages, incoming messages are managed by callback
	  ///

//	  CDC_Transmit_HS(myBuf, sizeof(myBuf));
	  ///
	  /// Handle a new char from USB
	  ///
/*	  if(new_char_on_usb)
	  {
		  for(uint8_t i=0; i < usb_cdc_rx_char; i++)
		  {
			  if(usb_cdc_buffer[i] == '\r' || usb_cdc_buffer[i] == '\n' || usb_cdc_buffer[i] == '\0' || pos < MAX_BUF_LENGTH)
			  {
				  buffer[pos++] = '0';
				  xQueueSendToBack(xUsb_tx, &buffer, 0);
				  newCmd = true;
				  CDC_Transmit_HS(buffer, pos);
				  break;
			  }
			  else {
				  buffer[pos++] = usb_cdc_buffer[i];
			  }
		  }
		  new_char_on_usb = false;
		  // Clean up the buffer
		  if(newCmd)
		  {
			  for(uint8_t i=0; i < MAX_BUF_LENGTH; i++) buffer[i] = 0;
			  newCmd = false;
		  }
	  }*/
	  ///
	  /// Check if a new message has to be sent to USB and do it
	  ///
	  BaseType_t xStatus;
	  if((xStatus = xQueueReceive(xUsb_rx, &data_to_send,ticks)) == pdPASS)
	  {
		  uint8_t len = 0;
		  while (data_to_send[len] && len < MAX_BUF_LENGTH) len++;
		  CDC_Transmit_HS(data_to_send, len);
		  ///
		  ///	Print the PROMPT
		  ///
		  CDC_Transmit_HS((uint8_t*)"\r\nSTM32> ", 9);
	  }
#endif
//	  HAL_Delay(500);
  }
  /* USER CODE END 5 */
}

int _write(int file, char *ptr, int len)
{
  /* Implement your write code here, this is used by puts and printf for example */
  int i=0;
  for(i=0 ; i<len ; i++)
    ITM_SendChar((*ptr++));
  return len;
}

void MX_GPIO_Init(void)
{

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


PeltierApplication *appli = new PeltierApplication();
int main(void)
{
    hw_init();
//    MX_GPIO_Init();
#ifdef PRINTF_DBG
    printf("Init HW & GPIO\n");
#endif
#ifdef USE_USB
    MX_USB_DEVICE_Init();
    // Create Tx/Rx queues
    xUsb_tx = xQueueCreate(1, sizeof(cdc_buf_t));
    xUsb_rx = xQueueCreate(1, sizeof(cdc_buf_t));
#ifdef PRINTF_DBG
    printf("Init USB\n");
#endif
#endif
    touchgfx_init();
#ifdef PRINTF_DBG
    printf("Launched TouchGFF\nNow creating the tasks\n");
#endif

    //PeltierApplication *appli = new PeltierApplication();

    //appli->MX_USB_DEVICE_Init();
    xTaskCreate(xStartDefaultTask, "Default Task", 128, NULL, osPriorityBelowNormal, NULL);
 //   xTaskCreate(xStartDefaultTask, "Default Task", 2048, NULL, 3, NULL);
    xTaskCreate(ShellTask, "Shell Task", 128, NULL, osPriorityNormal, NULL);

    xTaskCreate(GUITask, "GUITask",
                configGUI_TASK_STK_SIZE,
                NULL,
                configGUI_TASK_PRIORITY,
                NULL);

    vTaskStartScheduler();

    for (;;);
}
