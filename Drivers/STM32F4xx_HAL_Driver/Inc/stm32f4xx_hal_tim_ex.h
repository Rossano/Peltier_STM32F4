/**
  ******************************************************************************
  * @file    stm32f4xx_hal_tim_ex.h
  * @author  MCD Application Team
  * @brief   Header file of TIM HAL Extended module.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STM32F4xx_HAL_TIM_EX_H
#define STM32F4xx_HAL_TIM_EX_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal_def.h"

/** @addtogroup STM32F4xx_HAL_Driver
  * @{
  */

/** @addtogroup TIMEx
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup TIMEx_Exported_Types TIM Extended Exported Types
  * @{
  */

/**
  * @brief  TIM Hall sensor Configuration Structure definition
  */

typedef struct
{
  uint32_t IC1Polarity;         /*!< Specifies the active edge of the input signal.
                                     This parameter can be a value of @ref TIM_Input_Capture_Polarity */

  uint32_t IC1Prescaler;        /*!< Specifies the Input Capture Prescaler.
                                     This parameter can be a value of @ref TIM_Input_Capture_Prescaler */

  uint32_t IC1Filter;           /*!< Specifies the input capture filter.
                                     This parameter can be a number between Min_Data = 0x0 and Max_Data = 0xF */

  uint32_t Commutation_Delay;   /*!< Specifies the pulse value to be loaded into the Capture Compare Register.
                                     This parameter can be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF */
} TIM_HallSensor_InitTypeDef;
/**
  * @}
  */
/* End of exported types -----------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
/** @defgroup TIMEx_Exported_Constants TIM Extended Exported Constants
  * @{
  */

/** @defgroup TIMEx_Remap TIM Extended Remapping
  * @{
  */
#if defined (TIM2)
#if defined(TIM8)
#define TIM_TIM2_TIM8_TRGO                     0x00000000U                              /*!< TIM2 ITR1 is connected to TIM8 TRGO */
#else
#define TIM_TIM2_ETH_PTP                       TIM_OR_ITR1_RMP_0                        /*!< TIM2 ITR1 is connected to PTP trigger output */
#endif /*  TIM8 */
#define TIM_TIM2_USBFS_SOF                     TIM_OR_ITR1_RMP_1                        /*!< TIM2 ITR1 is connected to OTG FS SOF */
#define TIM_TIM2_USBHS_SOF                     (TIM_OR_ITR1_RMP_1 | TIM_OR_ITR1_RMP_0)  /*!< TIM2 ITR1 is connected to OTG HS SOF */
#endif /* TIM2 */

#define TIM_TIM5_GPIO                          0x00000000U                              /*!< TIM5 TI4 is connected to GPIO */
#define TIM_TIM5_LSI                           TIM_OR_TI4_RMP_0                         /*!< TIM5 TI4 is connected to LSI */
#define TIM_TIM5_LSE                           TIM_OR_TI4_RMP_1                         /*!< TIM5 TI4 is connected to LSE */
#define TIM_TIM5_RTC                           (TIM_OR_TI4_RMP_1 | TIM_OR_TI4_RMP_0)    /*!< TIM5 TI4 is connected to the RTC wakeup interrupt */

#define TIM_TIM11_GPIO                         0x00000000U                              /*!< TIM11 TI1 is connected to GPIO */
#define TIM_TIM11_HSE                          TIM_OR_TI1_RMP_1                         /*!< TIM11 TI1 is connected to HSE_RTC clock */
#if defined(SPDIFRX)
#define TIM_TIM11_SPDIFRX                      TIM_OR_TI1_RMP_0                         /*!< TIM11 TI1 is connected to SPDIFRX_FRAME_SYNC */
#endif /* SPDIFRX*/

#if defined(LPTIM_OR_TIM1_ITR2_RMP) && defined(LPTIM_OR_TIM5_ITR1_RMP) && defined(LPTIM_OR_TIM5_ITR1_RMP)
#define LPTIM_REMAP_MASK                       0x10000000U

#define TIM_TIM9_TIM3_TRGO                     LPTIM_REMAP_MASK                             /*!< TIM9 ITR1 is connected to TIM3 TRGO */
#define TIM_TIM9_LPTIM                         (LPTIM_REMAP_MASK | LPTIM_OR_TIM9_ITR1_RMP)  /*!< TIM9 ITR1 is connected to LPTIM1 output */

#define TIM_TIM5_TIM3_TRGO                     LPTIM_REMAP_MASK                             /*!< TIM5 ITR1 is connected to TIM3 TRGO */
#define TIM_TIM5_LPTIM                         (LPTIM_REMAP_MASK | LPTIM_OR_TIM5_ITR1_RMP)  /*!< TIM5 ITR1 is connected to LPTIM1 output */

#define TIM_TIM1_TIM3_TRGO                     LPTIM_REMAP_MASK                             /*!< TIM1 ITR2 is connected to TIM3 TRGO */
#define TIM_TIM1_LPTIM                         (LPTIM_REMAP_MASK | LPTIM_OR_TIM1_ITR2_RMP)  /*!< TIM1 ITR2 is connected to LPTIM1 output */
#endif /* LPTIM_OR_TIM1_ITR2_RMP &&  LPTIM_OR_TIM5_ITR1_RMP && LPTIM_OR_TIM5_ITR1_RMP */
/**
  * @}
  */

/**
  * @}
  */
/* End of exported constants -------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
/** @defgroup TIMEx_Exported_Macros TIM Extended Exported Macros
  * @{
  */

/**
  * @}
  */
/* End of exported macro -----------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/** @defgroup TIMEx_Private_Macros TIM Extended Private Macros
  * @{
  */
#if defined(SPDIFRX)
#define IS_TIM_REMAP(INSTANCE, TIM_REMAP)                                       \
        ((((INSTANCE) == TIM2)  && (((TIM_REMAP) == TIM_TIM2_TIM8_TRGO)      || \
                                    ((TIM_REMAP) == TIM_TIM2_USBFS_SOF)      || \
                                    ((TIM_REMAP) == TIM_TIM2_USBHS_SOF)))    || \
         (((INSTANCE) == TIM5)  && (((TIM_REMAP) == TIM_TIM5_GPIO)           || \
                                    ((TIM_REMAP) == TIM_TIM5_LSI)            || \
                                    ((TIM_REMAP) == TIM_TIM5_LSE)            || \
                                    ((TIM_REMAP) == TIM_TIM5_RTC)))          || \
         (((INSTANCE) == TIM11) && (((TIM_REMAP) == TIM_TIM11_GPIO)          || \
                                    ((TIM_REMAP) == TIM_TIM11_SPDIFRX)       || \
                                    ((TIM_REMAP) == TIM_TIM11_HSE))))
#elif defined(TIM2)
#if defined(LPTIM_OR_TIM1_ITR2_RMP) && defined(LPTIM_OR_TIM5_ITR1_RMP) && defined(LPTIM_OR_TIM5_ITR1_RMP)
#define IS_TIM_REMAP(INSTANCE, TIM_REMAP)                                       \
        ((((INSTANCE) == TIM2)  && (((TIM_REMAP) == TIM_TIM2_TIM8_TRGO)      || \
                                    ((TIM_REMAP) == TIM_TIM2_USBFS_SOF)      || \
                                    ((TIM_REMAP) == TIM_TIM2_USBHS_SOF)))    || \
         (((INSTANCE) == TIM5)  && (((TIM_REMAP) == TIM_TIM5_GPIO)           || \
                                    ((TIM_REMAP) == TIM_TIM5_LSI)            || \
                                    ((TIM_REMAP) == TIM_TIM5_LSE)            || \
                                    ((TIM_REMAP) == TIM_TIM5_RTC)))          || \
         (((INSTANCE) == TIM11) && (((TIM_REMAP) == TIM_TIM11_GPIO)          || \
                                    ((TIM_REMAP) == TIM_TIM11_HSE)))         || \
         (((INSTANCE) == TIM1)  && (((TIM_REMAP) == TIM_TIM1_TIM3_TRGO)      || \
                                    ((TIM_REMAP) == TIM_TIM1_LPTIM)))        || \
         (((INSTANCE) == TIM5)  && (((TIM_REMAP) == TIM_TIM5_TIM3_TRGO)      || \
                                    ((TIM_REMAP) == TIM_TIM5_LPTIM)))        || \
         (((INSTANCE) == TIM9)  && (((TIM_REMAP) == TIM_TIM9_TIM3_TRGO)      || \
                                    ((TIM_REMAP) == TIM_TIM9_LPTIM))))
#elif defined(TIM8)
#define IS_TIM_REMAP(INSTANCE, TIM_REMAP)                                       \
        ((((INSTANCE) == TIM2)  && (((TIM_REMAP) == TIM_TIM2_TIM8_TRGO)      || \
                                    ((TIM_REMAP) == TIM_TIM2_USBFS_SOF)      || \
                                    ((TIM_REMAP) == TIM_TIM2_USBHS_SOF)))    || \
         (((INSTANCE) == TIM5)  && (((TIM_REMAP) == TIM_TIM5_GPIO)           || \
                                    ((TIM_REMAP) == TIM_TIM5_LSI)            || \
                                    ((TIM_REMAP) == TIM_TIM5_LSE)            || \
                                    ((TIM_REMAP) == TIM_TIM5_RTC)))          || \
         (((INSTANCE) == TIM11) && (((TIM_REMAP) == TIM_TIM11_GPIO)          || \
                                    ((TIM_REMAP) == TIM_TIM11_HSE))))
#else
#define IS_TIM_REMAP(INSTANCE, TIM_REMAP)                                       \
        ((((INSTANCE) == TIM2)  && (((TIM_REMAP) == TIM_TIM2_ETH_PTP)        || \
                                    ((TIM_REMAP) == TIM_TIM2_USBFS_SOF)      || \
                                    ((TIM_REMAP) == TIM_TIM2_USBHS_SOF)))    || \
         (((INSTANCE) == TIM5)  && (((TIM_REMAP) == TIM_TIM5_GPIO)           || \
                                    ((TIM_REMAP) == TIM_TIM5_LSI)            || \
                                    ((TIM_REMAP) == TIM_TIM5_LSE)            || \
                                    ((TIM_REMAP) == TIM_TIM5_RTC)))          || \
         (((INSTANCE) == TIM11) && (((TIM_REMAP) == TIM_TIM11_GPIO)          || \
                                    ((TIM_REMAP) == TIM_TIM11_HSE))))
#endif /* LPTIM_OR_TIM1_ITR2_RMP &&  LPTIM_OR_TIM5_ITR1_RMP && LPTIM_OR_TIM5_ITR1_RMP */
#else
#define IS_TIM_REMAP(INSTANCE, TIM_REMAP)                                       \
        ((((INSTANCE) == TIM5)  && (((TIM_REMAP) == TIM_TIM5_GPIO)           || \
                                    ((TIM_REMAP) == TIM_TIM5_LSI)            || \
                                    ((TIM_REMAP) == TIM_TIM5_LSE)            || \
                                    ((TIM_REMAP) == TIM_TIM5_RTC)))          || \
         (((INSTANCE) == TIM11) && (((TIM_REMAP) == TIM_TIM11_GPIO)          || \
                                    ((TIM_REMAP) == TIM_TIM11_HSE))))
#endif /* SPDIFRX */

/**
  * @}
  */
/* End of private macro ------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @addtogroup TIMEx_Exported_Functions TIM Extended Exported Functions
  * @{
  */

/** @addtogroup TIMEx_Exported_Functions_Group1 Extended Timer Hall Sensor functions
 *  @brief    Timer Hall Sensor functions
 * @{
 */
/*  Timer Hall Sensor functions  **********************************************/
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Init(TIM_HandleTypeDef *htim, TIM_HallSensor_InitTypeDef *sConfig);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_DeInit(TIM_HandleTypeDef *htim);

void HAL_TIMEx_HallSensor_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIMEx_HallSensor_MspDeInit(TIM_HandleTypeDef *htim);

/* Blocking mode: Polling */
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Start(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Stop(TIM_HandleTypeDef *htim);
/* Non-Blocking mode: Interrupt */
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Start_IT(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Stop_IT(TIM_HandleTypeDef *htim);
/* Non-Blocking mode: DMA */
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Start_DMA(TIM_HandleTypeDef *htim, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Stop_DMA(TIM_HandleTypeDef *htim);
/**
  * @}
  */

/** @addtogroup TIMEx_Exported_Functions_Group2 Extended Timer Complementary Output Compare functions
 *  @brief   Timer Complementary Output Compare functions
 * @{
 */
/*  Timer Complementary Output Compare functions  *****************************/
/* Blocking mode: Polling */
HAL_StatusTypeDef HAL_TIMEx_OCN_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_OCN_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);

/* Non-Blocking mode: Interrupt */
HAL_StatusTypeDef HAL_TIMEx_OCN_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_OCN_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);

/* Non-Blocking mode: DMA */
HAL_StatusTypeDef HAL_TIMEx_OCN_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIMEx_OCN_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);
/**
  * @}
  */

/** @addtogroup TIMEx_Exported_Functions_Group3 Extended Timer Complementary PWM functions
 *  @brief    Timer Complementary PWM functions
 * @{
 */
/*  Timer Complementary PWM functions  ****************************************/
/* Blocking mode: Polling */
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);

/* Non-Blocking mode: Interrupt */
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
/* Non-Blocking mode: DMA */
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);
/**
  * @}
  */

/** @addtogroup TIMEx_Exported_Functions_Group4 Extended Timer Complementary One Pulse functions
 *  @brief    Timer Complementary One Pulse functions
 * @{
 */
/*  Timer Complementary One Pulse functions  **********************************/
/* Blocking mode: Polling */
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Start(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Stop(TIM_HandleTypeDef *htim, uint32_t OutputChannel);

/* Non-Blocking mode: Interrupt */
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Start_IT(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Stop_IT(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
/**
  * @}
  */

/** @addtogroup TIMEx_Exported_Functions_Group5 Extended Peripheral Control functions
 *  @brief    Peripheral Control functions
 * @{
 */
/* Extended Control functions  ************************************************/
HAL_StatusTypeDef HAL_TIMEx_ConfigCommutEvent(TIM_HandleTypeDef *htim, uint32_t  InputTrigger, uint32_t  CommutationSource);
HAL_StatusTypeDef HAL_TIMEx_ConfigCommutEvent_IT(TIM_HandleTypeDef *htim, uint32_t  InputTrigger, uint32_t  CommutationSource);
HAL_StatusTypeDef HAL_TIMEx_ConfigCommutEvent_DMA(TIM_HandleTypeDef *htim, uint32_t  InputTrigger, uint32_t  CommutationSource);
HAL_StatusTypeDef HAL_TIMEx_MasterConfigSynchronization(TIM_HandleTypeDef *htim, TIM_MasterConfigTypeDef *sMasterConfig);
HAL_StatusTypeDef HAL_TIMEx_ConfigBreakDeadTime(TIM_HandleTypeDef *htim, TIM_BreakDeadTimeConfigTypeDef *sBreakDeadTimeConfig);
HAL_StatusTypeDef HAL_TIMEx_RemapConfig(TIM_HandleTypeDef *htim, uint32_t Remap);
/**
  * @}
  */

/** @addtogroup TIMEx_Exported_Functions_Group6 Extended Callbacks functions
  * @brief    Extended Callbacks functions
  * @{
  */
/* Extended Callback **********************************************************/
void HAL_TIMEx_CommutCallback(TIM_HandleTypeDef *htim);
void HAL_TIMEx_CommutHalfCpltCallback(TIM_HandleTypeDef *htim);
void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *htim);
/**
  * @}
  */

/** @addtogroup TIMEx_Exported_Functions_Group7 Extended Peripheral State functions
  * @brief    Extended Peripheral State functions
  * @{
  */
/* Extended Peripheral State functions  ***************************************/
HAL_TIM_StateTypeDef HAL_TIMEx_HallSensor_GetState(TIM_HandleTypeDef *htim);
/**
  * @}
  */

/**
  * @}
  */
/* End of exported functions -------------------------------------------------*/

/* Private functions----------------------------------------------------------*/
/** @defgroup TIMEx_Private_Functions TIM Extended Private Functions
  * @{
  */
void TIMEx_DMACommutationCplt(DMA_HandleTypeDef *hdma);
void TIMEx_DMACommutationHalfCplt(DMA_HandleTypeDef *hdma);
/**
  * @}
  */
/* End of private functions --------------------------------------------------*/

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif


#endif /* STM32F4xx_HAL_TIM_EX_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

#if 0
/**
  ******************************************************************************
  * @file    stm32f4xx_hal_tim_ex.h
  * @author  MCD Application Team
  * @brief   Header file of TIM HAL Extension module.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_HAL_TIM_EX_H
#define __STM32F4xx_HAL_TIM_EX_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal_def.h"

/** @addtogroup STM32F4xx_HAL_Driver
  * @{
  */

/** @addtogroup TIMEx
  * @{
  */ 

/* Exported types ------------------------------------------------------------*/ 
/** @defgroup TIMEx_Exported_Types TIM Exported Types
  * @{
  */
  
/** 
  * @brief  TIM Hall sensor Configuration Structure definition  
  */

typedef struct
{

  uint32_t IC1Polarity;            /*!< Specifies the active edge of the input signal.
                                        This parameter can be a value of @ref TIM_Input_Capture_Polarity */

  uint32_t IC1Prescaler;        /*!< Specifies the Input Capture Prescaler.
                                     This parameter can be a value of @ref TIM_Input_Capture_Prescaler */

  uint32_t IC1Filter;           /*!< Specifies the input capture filter.
                                     This parameter can be a number between Min_Data = 0x0 and Max_Data = 0xF */

  uint32_t Commutation_Delay;  /*!< Specifies the pulse value to be loaded into the Capture Compare Register. 
                                    This parameter can be a number between Min_Data = 0x0000U and Max_Data = 0xFFFFU */
} TIM_HallSensor_InitTypeDef;

/** 
  * @brief  TIM Master configuration Structure definition  
  */ 
typedef struct {
  uint32_t  MasterOutputTrigger;   /*!< Trigger output (TRGO) selection. 
                                      This parameter can be a value of @ref TIM_Master_Mode_Selection */

  uint32_t  MasterSlaveMode;       /*!< Master/slave mode selection. 
                                      This parameter can be a value of @ref TIM_Master_Slave_Mode */
}TIM_MasterConfigTypeDef;

/** 
  * @brief  TIM Break and Dead time configuration Structure definition  
  */ 
typedef struct
{
  uint32_t OffStateRunMode;            /*!< TIM off state in run mode.
                                         This parameter can be a value of @ref TIM_OSSR_Off_State_Selection_for_Run_mode_state */
  uint32_t OffStateIDLEMode;          /*!< TIM off state in IDLE mode.
                                         This parameter can be a value of @ref TIM_OSSI_Off_State_Selection_for_Idle_mode_state */
  uint32_t LockLevel;                     /*!< TIM Lock level.
                                         This parameter can be a value of @ref TIM_Lock_level */                             
  uint32_t DeadTime;                     /*!< TIM dead Time. 
                                         This parameter can be a number between Min_Data = 0x00 and Max_Data = 0xFF */
  uint32_t BreakState;                   /*!< TIM Break State. 
                                         This parameter can be a value of @ref TIM_Break_Input_enable_disable */
  uint32_t BreakPolarity;                 /*!< TIM Break input polarity. 
                                         This parameter can be a value of @ref TIM_Break_Polarity */
  uint32_t AutomaticOutput;               /*!< TIM Automatic Output Enable state. 
                                         This parameter can be a value of @ref TIM_AOE_Bit_Set_Reset */           
}TIM_BreakDeadTimeConfigTypeDef;
/**
  * @}
  */
  
/* Exported constants --------------------------------------------------------*/
/** @defgroup TIMEx_Exported_Constants  TIM Exported Constants
  * @{
  */
  
/** @defgroup TIMEx_Remap  TIM Remap
  * @{
  */
#define TIM_TIM2_TIM8_TRGO                     0x00000000U
#define TIM_TIM2_ETH_PTP                       0x00000400U
#define TIM_TIM2_USBFS_SOF                     0x00000800U
#define TIM_TIM2_USBHS_SOF                     0x00000C00U
#define TIM_TIM5_GPIO                          0x00000000U
#define TIM_TIM5_LSI                           0x00000040U
#define TIM_TIM5_LSE                           0x00000080U
#define TIM_TIM5_RTC                           0x000000C0U
#define TIM_TIM11_GPIO                         0x00000000U
#define TIM_TIM11_HSE                          0x00000002U

#if defined(STM32F413xx) || defined(STM32F423xx)
#define TIM_TIM9_TIM3_TRGO                     0x10000000U
#define TIM_TIM9_LPTIM                         0x10000010U
#define TIM_TIM5_TIM3_TRGO                     0x10000000U
#define TIM_TIM5_LPTIM                         0x10000008U
#define TIM_TIM1_TIM3_TRGO                     0x10000000U
#define TIM_TIM1_LPTIM                         0x10000004U
#endif /* STM32F413xx | STM32F423xx */

#if defined (STM32F446xx) 
#define TIM_TIM11_SPDIFRX                      0x00000001U
#endif /* STM32F446xx */
/**
  * @}
  */

#if defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx) || defined(STM32F413xx) || defined(STM32F423xx)
/** @defgroup TIMEx_SystemBreakInput  TIM System Break Input
  * @{
  */
#define TIM_SYSTEMBREAKINPUT_HARDFAULT     0x00000001U /* Core Lockup lock output(Hardfault) is connected to Break Input of TIM1 and TIM8 */
#define TIM_SYSTEMBREAKINPUT_PVD           0x00000004U /* PVD Interrupt is connected to Break Input of TIM1 and TIM8 */
#define TIM_SYSTEMBREAKINPUT_HARDFAULT_PVD 0x00000005U /* Core Lockup lock output(Hardfault) and PVD Interrupt are connected to Break Input of TIM1 and TIM8 */   
/**
  * @}
  */
#endif /* STM32F410Tx || STM32F410Cx || STM32F410Rx || STM32F413xx || STM32F423xx */

/**
  * @}
  */ 
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @addtogroup TIMEx_Exported_Functions
  * @{
  */

/** @addtogroup TIMEx_Exported_Functions_Group1
  * @{
  */
/*  Timer Hall Sensor functions  **********************************************/
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Init(TIM_HandleTypeDef* htim, TIM_HallSensor_InitTypeDef* sConfig);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_DeInit(TIM_HandleTypeDef* htim);

void HAL_TIMEx_HallSensor_MspInit(TIM_HandleTypeDef* htim);
void HAL_TIMEx_HallSensor_MspDeInit(TIM_HandleTypeDef* htim);

 /* Blocking mode: Polling */
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Start(TIM_HandleTypeDef* htim);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Stop(TIM_HandleTypeDef* htim);
/* Non-Blocking mode: Interrupt */
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Start_IT(TIM_HandleTypeDef* htim);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Stop_IT(TIM_HandleTypeDef* htim);
/* Non-Blocking mode: DMA */
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Start_DMA(TIM_HandleTypeDef* htim, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Stop_DMA(TIM_HandleTypeDef* htim);
/**
  * @}
  */

/** @addtogroup TIMEx_Exported_Functions_Group2
  * @{
  */
/*  Timer Complementary Output Compare functions  *****************************/
/* Blocking mode: Polling */
HAL_StatusTypeDef HAL_TIMEx_OCN_Start(TIM_HandleTypeDef* htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_OCN_Stop(TIM_HandleTypeDef* htim, uint32_t Channel);

/* Non-Blocking mode: Interrupt */
HAL_StatusTypeDef HAL_TIMEx_OCN_Start_IT(TIM_HandleTypeDef* htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_OCN_Stop_IT(TIM_HandleTypeDef* htim, uint32_t Channel);

/* Non-Blocking mode: DMA */
HAL_StatusTypeDef HAL_TIMEx_OCN_Start_DMA(TIM_HandleTypeDef* htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIMEx_OCN_Stop_DMA(TIM_HandleTypeDef* htim, uint32_t Channel);
/**
  * @}
  */

/** @addtogroup TIMEx_Exported_Functions_Group3
  * @{
  */
/*  Timer Complementary PWM functions  ****************************************/
/* Blocking mode: Polling */
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start(TIM_HandleTypeDef* htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop(TIM_HandleTypeDef* htim, uint32_t Channel);

/* Non-Blocking mode: Interrupt */
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start_IT(TIM_HandleTypeDef* htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop_IT(TIM_HandleTypeDef* htim, uint32_t Channel);
/* Non-Blocking mode: DMA */
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start_DMA(TIM_HandleTypeDef* htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop_DMA(TIM_HandleTypeDef* htim, uint32_t Channel);
/**
  * @}
  */

/** @addtogroup TIMEx_Exported_Functions_Group4
  * @{
  */
/*  Timer Complementary One Pulse functions  **********************************/
/* Blocking mode: Polling */
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Start(TIM_HandleTypeDef* htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Stop(TIM_HandleTypeDef* htim, uint32_t OutputChannel);

/* Non-Blocking mode: Interrupt */
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Start_IT(TIM_HandleTypeDef* htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Stop_IT(TIM_HandleTypeDef* htim, uint32_t OutputChannel);
/**
  * @}
  */

/** @addtogroup TIMEx_Exported_Functions_Group5
  * @{
  */
/* Extension Control functions  ************************************************/
HAL_StatusTypeDef HAL_TIMEx_ConfigCommutationEvent(TIM_HandleTypeDef* htim, uint32_t  InputTrigger, uint32_t  CommutationSource);
HAL_StatusTypeDef HAL_TIMEx_ConfigCommutationEvent_IT(TIM_HandleTypeDef* htim, uint32_t  InputTrigger, uint32_t  CommutationSource);
HAL_StatusTypeDef HAL_TIMEx_ConfigCommutationEvent_DMA(TIM_HandleTypeDef* htim, uint32_t  InputTrigger, uint32_t  CommutationSource);
HAL_StatusTypeDef HAL_TIMEx_MasterConfigSynchronization(TIM_HandleTypeDef* htim, TIM_MasterConfigTypeDef * sMasterConfig);
HAL_StatusTypeDef HAL_TIMEx_ConfigBreakDeadTime(TIM_HandleTypeDef* htim, TIM_BreakDeadTimeConfigTypeDef *sBreakDeadTimeConfig);
HAL_StatusTypeDef HAL_TIMEx_RemapConfig(TIM_HandleTypeDef* htim, uint32_t Remap);
/**
  * @}
  */

/** @addtogroup TIMEx_Exported_Functions_Group6
  * @{
  */ 
/* Extension Callback *********************************************************/
void HAL_TIMEx_CommutationCallback(TIM_HandleTypeDef* htim);
void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef* htim);
void TIMEx_DMACommutationCplt(DMA_HandleTypeDef *hdma);
/**
  * @}
  */

/** @addtogroup TIMEx_Exported_Functions_Group7
  * @{
  */
/* Extension Peripheral State functions  **************************************/
HAL_TIM_StateTypeDef HAL_TIMEx_HallSensor_GetState(TIM_HandleTypeDef* htim);
/**
  * @}
  */ 

/**
  * @}
  */ 

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/** @defgroup TIMEx_Private_Macros TIM Private Macros
  * @{
  */
#if defined (STM32F446xx) 
#define IS_TIM_REMAP(TIM_REMAP)   (((TIM_REMAP) == TIM_TIM2_TIM8_TRGO)||\
                                  ((TIM_REMAP) == TIM_TIM2_ETH_PTP)||\
                                  ((TIM_REMAP) == TIM_TIM2_USBFS_SOF)||\
                                  ((TIM_REMAP) == TIM_TIM2_USBHS_SOF)||\
                                  ((TIM_REMAP) == TIM_TIM5_GPIO)||\
                                  ((TIM_REMAP) == TIM_TIM5_LSI)||\
                                  ((TIM_REMAP) == TIM_TIM5_LSE)||\
                                  ((TIM_REMAP) == TIM_TIM5_RTC)||\
                                  ((TIM_REMAP) == TIM_TIM11_GPIO)||\
                                  ((TIM_REMAP) == TIM_TIM11_SPDIFRX)||\
                                  ((TIM_REMAP) == TIM_TIM11_HSE))
#elif defined(STM32F413xx) || defined(STM32F423xx)
#define IS_TIM_REMAP(TIM_REMAP)   (((TIM_REMAP) == TIM_TIM2_TIM8_TRGO)||\
                                  ((TIM_REMAP) == TIM_TIM2_ETH_PTP)||\
                                  ((TIM_REMAP) == TIM_TIM2_USBFS_SOF)||\
                                  ((TIM_REMAP) == TIM_TIM2_USBHS_SOF)||\
                                  ((TIM_REMAP) == TIM_TIM5_GPIO)||\
                                  ((TIM_REMAP) == TIM_TIM5_LSI)||\
                                  ((TIM_REMAP) == TIM_TIM5_LSE)||\
                                  ((TIM_REMAP) == TIM_TIM5_RTC)||\
                                  ((TIM_REMAP) == TIM_TIM11_GPIO)||\
                                  ((TIM_REMAP) == TIM_TIM11_HSE)||\
                                  ((TIM_REMAP) == TIM_TIM9_TIM3_TRGO)||\
                                  ((TIM_REMAP) == TIM_TIM9_LPTIM)||\
                                  ((TIM_REMAP) == TIM_TIM5_TIM3_TRGO)||\
                                  ((TIM_REMAP) == TIM_TIM5_LPTIM)||\
                                  ((TIM_REMAP) == TIM_TIM1_TIM3_TRGO)||\
                                  ((TIM_REMAP) == TIM_TIM1_LPTIM))
#else
#define IS_TIM_REMAP(TIM_REMAP)   (((TIM_REMAP) == TIM_TIM2_TIM8_TRGO)||\
                                  ((TIM_REMAP) == TIM_TIM2_ETH_PTP)||\
                                  ((TIM_REMAP) == TIM_TIM2_USBFS_SOF)||\
                                  ((TIM_REMAP) == TIM_TIM2_USBHS_SOF)||\
                                  ((TIM_REMAP) == TIM_TIM5_GPIO)||\
                                  ((TIM_REMAP) == TIM_TIM5_LSI)||\
                                  ((TIM_REMAP) == TIM_TIM5_LSE)||\
                                  ((TIM_REMAP) == TIM_TIM5_RTC)||\
                                  ((TIM_REMAP) == TIM_TIM11_GPIO)||\
                                  ((TIM_REMAP) == TIM_TIM11_HSE))
#endif /* STM32F446xx */

#if defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx) || defined(STM32F413xx) || defined(STM32F423xx)
#define IS_TIM_SYSTEMBREAKINPUT(BREAKINPUT)   (((BREAKINPUT) == TIM_SYSTEMBREAKINPUT_HARDFAULT)||\
                                               ((BREAKINPUT) == TIM_SYSTEMBREAKINPUT_PVD)||\
                                               ((BREAKINPUT) == TIM_SYSTEMBREAKINPUT_HARDFAULT_PVD))

#endif /* STM32F410Tx || STM32F410Cx || STM32F410Rx || STM32F413xx || STM32F423xx */ 

#define IS_TIM_DEADTIME(DEADTIME)      ((DEADTIME) <= 0xFFU) 
/**
  * @}
  */  
  
/* Private functions ---------------------------------------------------------*/
/** @defgroup TIMEx_Private_Functions TIM Private Functions
  * @{
  */
  
/**
  * @}
  */

/**
  * @}
  */ 

/**
  * @}
  */
    
#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_HAL_TIM_EX_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
#endif
