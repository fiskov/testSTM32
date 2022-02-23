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
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>
#include <math.h>

#include "display_gmg12864.h"
#include "disp_draw.h"

#include "mag_HMC5883L.h"
#include "acc_ADXL345.h"
#include "clock_DS3231.h"
#include "gps_decode.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static uint8_t disp_bfr[128*64/8] = {0};

static void disp_spi_write_cb(uint8_t * bfr, uint16_t length) 
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

static void i2c_1_write_cb(uint8_t addr, uint8_t bfr[], uint16_t length)
{
  HAL_I2C_Master_Transmit(&hi2c1, addr << 1, bfr, length, 100);
}
static void i2c_1_read_cb(uint8_t addr, uint8_t bfr[], uint16_t length)
{
  HAL_I2C_Master_Receive(&hi2c1, addr << 1, bfr, length, 100);
}

static void acc_spi_write_cb(uint8_t * bfr, uint16_t length) 
{
  HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi2, bfr, length, 10);
  HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_SET);
}
static void acc_spi_write_read_cb(uint8_t * bfr_tx, uint8_t * bfr_rx, uint16_t length) 
{
  HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi2, bfr_tx, bfr_rx, length, 10);
  HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_SET);
}

typedef struct {
  int16_t x, y, z, t;
} point_t;

static acc_point_t acc;
static mag_point_t mag;
static clock_value_t clock_value;
static gpsValue_t gps_value;
static uint16_t current_value;

#define STR_LEN (128/6)
static void update_disp_bfr(uint8_t * bfr)
{ 
  char str[STR_LEN+6] = {0};
  
  disp_draw_str_simple(disp_bfr, "mag HMC5883L", 12, 0, 0);
  sprintf(str, "%6d %6d %6d ", mag.x, mag.y, mag.z);
  disp_draw_str_simple(disp_bfr, str, STR_LEN, 0, 1);

  sprintf(str, "acc ADXL345    t=%u", clock_value.temperature);
  disp_draw_str_simple(disp_bfr, str, strlen(str), 0, 2);  
  sprintf(str, "%6d %6d %6d ", acc.x, acc.y, acc.z);
  disp_draw_str_simple(disp_bfr, str, STR_LEN, 0, 3);

  sprintf(str, "Clock DS3231 %02d:%02d:%02d", clock_value.tm_hour , clock_value.tm_min, clock_value.tm_sec);
  disp_draw_str_simple(disp_bfr, str, STR_LEN, 0, 4);
  
  sprintf(str, "Current INA226 %5d", current_value);
  disp_draw_str_simple(disp_bfr, str, STR_LEN, 0, 5);  

  double integral;
  double fractional = modf(gps_value.lat, &integral)*10000;
  sprintf(str, "GPS NEO6 lat%4d.%04d",(int16_t)integral, (int16_t)(fractional));
  disp_draw_str_simple(disp_bfr, str, STR_LEN, 0, 6);

        fractional = modf(gps_value.lng, &integral)*10000;
  sprintf(str, " %3s    long%4d.%04d", (gps_value.ok ? "Ok " : "Err"), (int16_t)integral, (int16_t)(fractional));
  disp_draw_str_simple(disp_bfr, str, STR_LEN, 0, 7);  
}
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
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim1);
  
  //disp
  static display_gmg12864_t disp_drv = {
    .delay_ms_cb = HAL_Delay,
    .write_spi_cb = disp_spi_write_cb,
    .set_display_pin_cb = disp_set_pin_cb,
    .bfr = disp_bfr,
    .bfr_size = sizeof(disp_bfr)
  };

  display_gmg12864_init(&disp_drv);

  //mag
  static mag_drv_t mag_drv;
  mag_HMC5883L_default(&mag_drv);

  mag_drv.write_i2c_cb = i2c_1_write_cb;
  mag_drv.read_i2c_cb = i2c_1_read_cb;   
  mag_drv.delay_ms_cb = HAL_Delay;

  mag_HMC5883L_init(&mag_drv);
  //mag_HMC5883L_compenstation(&mag);

  //acc
  HAL_Delay(5);
  static acc_drv_t acc_drv = {
    .datarate = ACC_DATARATE_6_25,
    .spi_write_cb = acc_spi_write_cb,
    .spi_write_read_cb = acc_spi_write_read_cb,
    .delay_ms_cb = HAL_Delay
  };
  acc_ADXL345_init(&acc_drv);

  //clock
  static clock_drv_t clock_drv = {
    .addr = 0x68,
    .read_i2c_cb = i2c_1_read_cb,
    .write_i2c_cb = i2c_1_write_cb
  };
  clock_DS3231_init(&clock_drv);

  clock_value.tm_hour = 23;
  clock_value.tm_min = 34;
  clock_value.tm_sec = 45;

  clock_value.tm_wday = 6;
  clock_value.tm_mday = 19;
  clock_value.tm_mon = 02;
  clock_value.tm_year = 2022;

  //clock_DS3231_setTime(&clock_value);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_Delay(400);
    mag_HMC5883L_getXYZ(&mag);
    acc_ADXL345_getXYZ(&acc);    
    
    clock_DS3231_getTime(&clock_value);
    acc.x = clock_value.tm_year;
    acc.y = clock_value.tm_mon;
    acc.z = clock_value.tm_mday;

    update_disp_bfr(disp_bfr);
    display_gmg12864_print();

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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void tmr1_interrupt(void)
{
  static uint16_t tmr1000 = 1000;
  if (--tmr1000 == 0)
  {
    tmr1000 = 1000;   
        
  }
}
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

