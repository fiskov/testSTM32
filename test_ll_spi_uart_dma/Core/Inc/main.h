/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "stm32f1xx_ll_adc.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_spi.h"
#include "stm32f1xx_ll_tim.h"
#include "stm32f1xx_ll_usart.h"
#include "stm32f1xx_ll_gpio.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define VTLINK_Pin LL_GPIO_PIN_13
#define VTLINK_GPIO_Port GPIOC
#define VTWORK_Pin LL_GPIO_PIN_14
#define VTWORK_GPIO_Port GPIOC
#define CS_LED_Pin LL_GPIO_PIN_4
#define CS_LED_GPIO_Port GPIOA
#define CLK_LED_Pin LL_GPIO_PIN_5
#define CLK_LED_GPIO_Port GPIOA
#define MOSI_LED_Pin LL_GPIO_PIN_7
#define MOSI_LED_GPIO_Port GPIOA
#define LINKA_OUT_Pin LL_GPIO_PIN_4
#define LINKA_OUT_GPIO_Port GPIOC
#define LINKB_OUT_Pin LL_GPIO_PIN_5
#define LINKB_OUT_GPIO_Port GPIOC
#define IRQ_Pin LL_GPIO_PIN_2
#define IRQ_GPIO_Port GPIOB
#define CS_Pin LL_GPIO_PIN_12
#define CS_GPIO_Port GPIOB
#define SCK_Pin LL_GPIO_PIN_13
#define SCK_GPIO_Port GPIOB
#define MISO_Pin LL_GPIO_PIN_14
#define MISO_GPIO_Port GPIOB
#define MOSI_Pin LL_GPIO_PIN_15
#define MOSI_GPIO_Port GPIOB
#define DBG6_Pin LL_GPIO_PIN_11
#define DBG6_GPIO_Port GPIOA
#define DBG5_Pin LL_GPIO_PIN_12
#define DBG5_GPIO_Port GPIOA
#define DBG4_Pin LL_GPIO_PIN_10
#define DBG4_GPIO_Port GPIOC
#define DBG3_Pin LL_GPIO_PIN_11
#define DBG3_GPIO_Port GPIOC
#define JDETECT_Pin LL_GPIO_PIN_12
#define JDETECT_GPIO_Port GPIOC
#define DBG2_Pin LL_GPIO_PIN_4
#define DBG2_GPIO_Port GPIOB
#define DBG1_Pin LL_GPIO_PIN_5
#define DBG1_GPIO_Port GPIOB
#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
