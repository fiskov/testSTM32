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
#include "lvgl.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BIT_SET(a,b) ((a) |= (1U<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1U<<(b)))
#define MAX(X,Y) ((X)>(Y)?(X):(Y))
#define MIN(X,Y) ((X)<(Y)?(X):(Y))
#define ARRAY_SIZE(X) (sizeof(X)/sizeof(X[0]))

LV_IMG_DECLARE(logo_lvgl)
LV_IMG_DECLARE(ezgif)

#define LL_GPIO_WriteOutputPin(PORT,PIN,VALUE) do{if(VALUE) LL_GPIO_SetOutputPin(PORT,PIN); else LL_GPIO_ResetOutputPin(PORT,PIN);}while(0)
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
uint32_t timer_global_1ms = 0;
static uint8_t disp_bfr[DISP_WIDTH * DISP_HEIGHT / 8];

uint32_t timer_period_delta(uint32_t *timer, uint32_t period_ms)
{
  uint32_t timer_1ms = timer_global_1ms;
  uint32_t delta = (timer_1ms - *timer);
  if (delta >= period_ms)
  {
    *timer = timer_1ms;
    return delta;
  }
  return 0;
}
uint16_t timer_us_reset(void)
{
  uint16_t result = TIM1->CNT;
  TIM1->CNT = 0;
  return result;
}

static void disp_write_spi_cb(uint8_t * bfr, uint16_t length) 
{
  LL_GPIO_ResetOutputPin(DISP_CS_GPIO_Port, DISP_CS_Pin);
  while (length--)
  {
    while (LL_SPI_IsActiveFlag_BSY(SPI1));
    LL_SPI_TransmitData8(SPI1, *bfr++);
  }
  while (LL_SPI_IsActiveFlag_BSY(SPI1));  
  LL_GPIO_SetOutputPin(DISP_CS_GPIO_Port, DISP_CS_Pin);
}

static void disp_set_pin_cb(display_gmg12864_pins_t * pins)
{
  LL_GPIO_WriteOutputPin(DISP_RSE_GPIO_Port, DISP_RSE_Pin, pins->pin_reset ? 1 : 0 );
  LL_GPIO_WriteOutputPin(DISP_RS_GPIO_Port, DISP_RS_Pin, pins->pin_data_cmd ? 1 : 0 );
}

void disp_draw_pixel(uint8_t x, uint8_t y, uint8_t color)
{
  uint8_t *byte = &disp_bfr[(x % DISP_WIDTH) + (y % DISP_HEIGHT)/8*DISP_WIDTH];
  if (color)
    *byte |= 1 << (y&7);
  else
    *byte &= ~(1 << (y&7));
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Display flushing */
void my_disp_flush_cb( struct _lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
  for(uint8_t y = area->y1; y <= area->y2; y++)
      for(uint8_t x = area->x1; x <= area->x2; x++)
          disp_draw_pixel(x, y, *(uint8_t*)color_p++);

  /* Inform the graphics library that you are ready with the flushing*/
  lv_disp_flush_ready( disp_drv );
}

void lvgl_init(void)
{
  lv_init();

  /*A static or global variable to store the buffers*/
  static lv_disp_draw_buf_t disp_buf;

  /*Static or global buffer(s). The second buffer is optional*/
  static lv_color_t buf_1[DISP_WIDTH * DISP_HEIGHT / 4];

  lv_disp_draw_buf_init(&disp_buf, buf_1, NULL, ARRAY_SIZE(buf_1));

  static lv_disp_drv_t disp_drv;        /*A variable to hold the drivers. Must be static or global.*/
  lv_disp_drv_init(&disp_drv);          /*Basic initialization*/
  disp_drv.draw_buf = &disp_buf;        /*Set an initialized buffer*/
  disp_drv.flush_cb = my_disp_flush_cb; /*Set a flush callback to draw to the display*/
  disp_drv.hor_res = DISP_WIDTH;        /*Set the horizontal resolution in pixels*/
  disp_drv.ver_res = DISP_HEIGHT;       /*Set the vertical resolution in pixels*/

  lv_disp_t * disp;
  disp = lv_disp_drv_register(&disp_drv); /*Register the driver and save the created display objects*/
  (void)disp;
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
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));

  /** NOJTAG: JTAG-DP Disabled and SW-DP Enabled
  */
  LL_GPIO_AF_Remap_SWJ_NOJTAG();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  LL_SYSTICK_EnableIT();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  LL_SPI_Enable(SPI1);
  // timer 1us
  uint16_t pre = (SystemCoreClock / 1000000UL)-1;
  LL_TIM_SetPrescaler(TIM1, pre);
  LL_TIM_EnableCounter(TIM1);

  // init graphic-driver
  lvgl_init();

  // init display-driver
  static display_gmg12864_t disp_drv = {
    .delay_ms_cb = LL_mDelay,
    .write_spi_cb = disp_write_spi_cb,
    .set_display_pin_cb = disp_set_pin_cb,
    .bfr = disp_bfr,
    .bfr_size = sizeof(disp_bfr)
  };
  display_gmg12864_init(&disp_drv);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t tmr_led=0, tmr_lvgl=0, tmr_1s=0;
  short tmr_max=0, tmr_maxt=0;
  short pos_x=0, dx=3;
  static char s[64];

  lv_obj_t *label, *img1, *gif1;
  label = lv_label_create(lv_scr_act());
  lv_obj_align(label, LV_ALIGN_TOP_LEFT, 3, 3);

  img1 = lv_img_create(lv_scr_act());
  lv_img_set_src(img1, &logo_lvgl);
  lv_obj_align(img1, LV_ALIGN_CENTER, 0, 0);

  gif1 = lv_gif_create(lv_scr_act());
  lv_gif_set_src(gif1, &ezgif);
  lv_obj_align(gif1, LV_ALIGN_LEFT_MID, 2, 0);

  while (1)
  {
    short tmr = timer_us_reset();
    tmr_maxt = MAX(tmr_maxt, tmr);

    lv_tick_inc(timer_period_delta(&tmr_lvgl, 10));
    lv_task_handler();
    display_gmg12864_print(); // ~700us

    if (timer_period_delta(&tmr_led, 200))
    {
      LL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

      // update label-text
      sprintf(s, "max=%dus", tmr_max);

      lv_label_set_text(label, s);
      
      // move box
      pos_x += dx;
      if (pos_x < -5 || pos_x > 30) dx = -dx;
      
      lv_obj_set_pos(img1, pos_x, 0);
    }

    if (timer_period_delta(&tmr_1s, 1000))
    {
      tmr_max = tmr_maxt;
      tmr_maxt = 0;
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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(72000000);
  LL_SetSystemCoreClock(72000000);
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
