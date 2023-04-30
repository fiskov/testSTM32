/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include "display_gmg12864.h"
#include "disp_draw.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct dt_t {
  uint16_t tmr_bfr[8];
  uint32_t fps;
} dt_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static uint8_t disp_bfr[DISP_WIDTH * DISP_HEIGHT / 8];
static dt_t dt;

uint32_t timer_until(uint32_t *timer, uint32_t period_ms)
{
  uint32_t timer_1ms = HAL_GetTick();
  return (timer_1ms - *timer >= period_ms) ? (*timer = timer_1ms) : 0;
}

uint16_t timer_us_reset(void)
{
  uint16_t result = TIM1->CNT;
  TIM1->CNT = 0;
  return result;
}

static void disp_write_spi_cb(uint8_t * bfr, uint16_t length) 
{
  HAL_GPIO_WritePin(DISP_CS_GPIO_Port, DISP_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, bfr, length, 1000);
  HAL_GPIO_WritePin(DISP_CS_GPIO_Port, DISP_CS_Pin, GPIO_PIN_SET);
}

static void disp_set_pin_cb(display_gmg12864_pins_t * pins)
{
  GPIO_PinState states[] = {GPIO_PIN_RESET, GPIO_PIN_SET};
  HAL_GPIO_WritePin(DISP_RSE_GPIO_Port, DISP_RSE_Pin, states[pins->pin_reset] );
  HAL_GPIO_WritePin(DISP_RS_GPIO_Port, DISP_RS_Pin, states[pins->pin_data_cmd] );
}


#define clear_disp_bfr(X) memset(X,0,sizeof(disp_bfr))
static void update_disp_bfr(uint8_t * bfr, dt_t *dt)
{
  clear_disp_bfr(bfr);

  static int x=DISP_WIDTH/2, y=DISP_HEIGHT/2, dx=1, dy=1, r_circle=10;
  x+=dx; y+=dy;
  if (x>=DISP_WIDTH-r_circle-1 || x<=r_circle) dx=-dx;
  if (y>=DISP_HEIGHT-r_circle-1 || y<=r_circle) dy=-dy;
  for (int r=1; r<r_circle; r++)
    disp_draw_circle(disp_bfr, x, y, r);

  static char str[16] = {0}, cnt;
  sprintf(str,"fps=%d", dt->fps);
  disp_draw_str_simple(disp_bfr, str, strlen(str), 0, 0);
  
  if (cnt++ & 1) disp_draw_pixel(disp_bfr, DISP_WIDTH-1, DISP_HEIGHT-1);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  uint16_t pre = (HAL_RCC_GetHCLKFreq() / 1000000UL)-1;
  __HAL_TIM_SET_PRESCALER(&htim1, pre);
  HAL_TIM_Base_Start(&htim1);

  static display_gmg12864_t disp_drv = {
    .delay_ms_cb = HAL_Delay,
    .write_spi_cb = disp_write_spi_cb,
    .set_display_pin_cb = disp_set_pin_cb,
    .bfr = disp_bfr,
    .bfr_size = sizeof(disp_bfr)
  };

  display_gmg12864_init(&disp_drv);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  static uint32_t tmr_led = 0, fps_temp, fps;

  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);

  while (1)
  {
    // HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);

    if (timer_until(&tmr_led, 1000))
    {
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      dt.fps = fps_temp;
      fps_temp = 0;
    }

    fps_temp++;

    {
      update_disp_bfr(disp_bfr, &dt);

      HAL_GPIO_WritePin(DBG_GPIO_Port, DBG_Pin, 1);
      display_gmg12864_print();
      HAL_GPIO_WritePin(DBG_GPIO_Port, DBG_Pin, 0);
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
