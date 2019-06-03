
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_dma2d.h"
#include "stm32f4xx_hal_ltdc.h"
#include "stm32f4xx_hal_pcd.h"

#include <stdbool.h>

extern DMA2D_HandleTypeDef hdma2d;

static volatile int overrunCnt;

extern bool os_inited;
void OS_SysTick(void);
PCD_HandleTypeDef hpcd_USB_OTG_HS;

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  SysTick interrupt handler
  * @param  None
  * @retval None
  */
void SysTick_Handler(void);
void SysTick_Handler(void)
{
  if (os_inited)
  {
    OS_SysTick();
  }
}

/**
  * @brief  DMA2D interrupt handler
  * @param  None
  * @retval None
  */
void DMA2D_IRQHandler(void);
void DMA2D_IRQHandler(void)
{
  HAL_DMA2D_IRQHandler(&hdma2d);
}

/**
  * @brief  LTDC_ER interrupt handler
  * @param  None
  * @retval None
  */
void LTDC_ER_IRQHandler(void);
void LTDC_ER_IRQHandler(void)
{
  unsigned int isr = LTDC->ISR;
  if (LTDC->ISR & 2)
  {
    LTDC->ICR = 2;
    overrunCnt++;
  }
}

/**
  * @brief This function handles USB On The Go HS global interrupt.
  */
void OTG_HS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_HS_IRQn 0 */

  /* USER CODE END OTG_HS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_HS);
  /* USER CODE BEGIN OTG_HS_IRQn 1 */

  /* USER CODE END OTG_HS_IRQn 1 */
}
