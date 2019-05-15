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

/**
 * Define the FreeRTOS task priorities and stack sizes
 */
#define configGUI_TASK_PRIORITY                 ( tskIDLE_PRIORITY + 3 )

#define configGUI_TASK_STK_SIZE                 ( 1024 )

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
}

static void xStartDefaultTask(void * argument)
{
  /* init code for USB_HOST */
//  MX_USB_HOST_Init();

  /* USER CODE BEGIN 5 */
	  /* init code for USB_DEVICE */
	  MX_USB_DEVICE_Init();
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

//PeltierApplication *appli = new PeltierApplication();
int main(void)
{
    hw_init();
    touchgfx_init();

    //appli->MX_USB_DEVICE_Init();
    xTaskCreate(xStartDefaultTask, "Default Task", 128, NULL, osPriorityBelowNormal, NULL);
 //   xTaskCreate(xStartDefaultTask, "Default Task", 2048, NULL, 3, NULL);
 //   xTaskCreate(ShellTask, "Shell Task", 128, NULL, osPriorityNormal, NULL);

    xTaskCreate(GUITask, "GUITask",
                configGUI_TASK_STK_SIZE,
                NULL,
                configGUI_TASK_PRIORITY,
                NULL);

    vTaskStartScheduler();

    for (;;);
}
