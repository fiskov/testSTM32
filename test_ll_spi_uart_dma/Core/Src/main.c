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
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdlib.h> 
#include <stdio.h>
#include <string.h>
#include "common_types.h"
#include "check_sum.h"
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

#define LED_BRIGHT_PERCENT_MAX 20
#define LED_BRIGHT_PERCENT_MIN 4
#define UART_CMD_SIZE 3

static data_t data;

static uint32_t period_1s = 0;
static uint8_t bfr_request[UART_CMD_SIZE] = {0xFF, 0, 0};

__IO uint16_t adcData[2] = {0};

    
typedef struct  flags_t {
  bool timer, timer1ms, uart1, uart3;
} flags_t;
flags_t flags = {0};

void status_led_set(uint16_t leds, uint8_t led_bright)
{
  // bright in percents
  uint16_t value = LL_TIM_GetAutoReload(TIM3) / 100.0 * (100.0 - led_bright);
  LL_TIM_OC_SetCompareCH2(TIM3, value);

  // put to STP16PT05
  LL_GPIO_ResetOutputPin(CS_LED_GPIO_Port, CS_LED_Pin);
  LL_SPI_TransmitData16(SPI1, leds);
  LL_GPIO_SetOutputPin(CS_LED_GPIO_Port, CS_LED_Pin);
}

uint16_t convert_adc_to_voltage(uint16_t adc_value)
{ 
  const float k_a = 7.3909; // resistor voltage divider
  const float k_b = 692.48;
  
  if (adc_value < 100) // minimal value for work
    return 0;
  else
    return (adc_value * k_a + k_b);
}

uint16_t exp_filter(uint16_t old_value, uint16_t new_value)
{
  const float a = 0.8;
  return old_value*(1-a)+new_value*a;
}

void USART_2_TX(uint8_t bfr[], uint16_t length)
{
  for (uint16_t pos = 0; pos < length; pos++)
  {
    while (!LL_USART_IsActiveFlag_TXE(USART2));
    LL_USART_TransmitData8(USART2, bfr[pos]);
  }
}

void USART1_TX (uint8_t bfr[], uint16_t length)
{
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);

  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_4, (uint32_t)bfr, LL_USART_DMA_GetRegAddr(USART1),
    LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_4));

  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_4, length);
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_4);
}
//--------------------------------------------------------
void USART1_RX (uint8_t* bfr, uint16_t length)
{
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_5);

  // ch5 = rx
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_5, LL_USART_DMA_GetRegAddr(USART1), (uint32_t)bfr, 
    LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_5));

  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_5, length);
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_5);
}

void USART3_TX (uint8_t bfr[], uint16_t length)
{
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);

  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_2, (uint32_t)bfr, LL_USART_DMA_GetRegAddr(USART3),
    LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2));

  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, length);
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
}
//--------------------------------------------------------
void USART3_RX (uint8_t* bfr, uint16_t length)
{
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);

  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_3, LL_USART_DMA_GetRegAddr(USART3), (uint32_t)bfr, 
    LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3));

  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, length);
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
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

  /** NONJTRST: Full SWJ (JTAG-DP + SW-DP) but without NJTRST
  */
  LL_GPIO_AF_Remap_SWJ_NONJTRST();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  //status led spi
  LL_SPI_Enable(SPI1);

  // PWM. Positive and Negative channels
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N);
  LL_TIM_EnableAllOutputs(TIM1);
  LL_TIM_EnableCounter(TIM1);
  
  // PWM LED brightness
  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH2);
  LL_TIM_EnableCounter(TIM3);
  
  // timer 1ms
  LL_TIM_EnableIT_UPDATE(TIM2);  
  LL_TIM_EnableCounter(TIM2);
  
  // ADC power voltages
  LL_DMA_ConfigTransfer(DMA1,
                      LL_DMA_CHANNEL_1,
                      LL_DMA_DIRECTION_PERIPH_TO_MEMORY |
                      LL_DMA_MODE_CIRCULAR              |
                      LL_DMA_PERIPH_NOINCREMENT         |
                      LL_DMA_MEMORY_INCREMENT           |
                      LL_DMA_PDATAALIGN_HALFWORD        |
                      LL_DMA_MDATAALIGN_HALFWORD        |
                      LL_DMA_PRIORITY_LOW               );
  LL_DMA_ConfigAddresses(DMA1,
                         LL_DMA_CHANNEL_1,
                         LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA),
                         (uint32_t)&adcData,
                         LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, 2);
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
  LL_ADC_Enable(ADC1);
  LL_mDelay(1);
  LL_ADC_StartCalibration(ADC1);
  LL_mDelay(1);
  LL_ADC_REG_StartConversionSWStart(ADC1);  



  //USART1
  LL_DMA_ClearFlag_GI4(DMA1);
  LL_DMA_ClearFlag_TC4(DMA1);
  LL_DMA_ClearFlag_TE4(DMA1); 

  LL_DMA_ClearFlag_GI5(DMA1);
  LL_DMA_ClearFlag_TC5(DMA1);
  LL_DMA_ClearFlag_TE5(DMA1);

  LL_USART_EnableDMAReq_RX(USART1);
  LL_USART_EnableDMAReq_TX(USART1);
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_4); // transfer complete
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_4); // transfer error
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_5);
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_5);


  //USART3
  LL_DMA_ClearFlag_GI2(DMA1);
  LL_DMA_ClearFlag_TC2(DMA1);
  LL_DMA_ClearFlag_TE2(DMA1); 

  LL_DMA_ClearFlag_GI3(DMA1);
  LL_DMA_ClearFlag_TC3(DMA1);
  LL_DMA_ClearFlag_TE3(DMA1);

  LL_USART_EnableDMAReq_RX(USART3);
  LL_USART_EnableDMAReq_TX(USART3);
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_2); // transfer complete
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_2); // transfer error
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_3);
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  static uint16_t cnt=0, led_value = 0;

  static int16_t  led_bright = LED_BRIGHT_PERCENT_MIN, led_bright_step=1;
  
  //prepare cmd for slave
  bfr_request[2] = crc8_dallas(bfr_request, 2);
  LL_RCC_ClocksTypeDef rcc;
  LL_RCC_GetSystemClocksFreq(&rcc);
  period_1s = rcc.SYSCLK_Frequency / ( LL_TIM_GetPrescaler(TIM2) + 1) / ( LL_TIM_GetAutoReload(TIM2) + 1);
  
  LL_GPIO_SetOutputPin(VTWORK_GPIO_Port, VTWORK_Pin);


  while (1)
  {    
    
    if (flags.timer1ms)
    {
      flags.timer1ms = false;
      
    }

    if (flags.timer) 
    {
      flags.timer = false;
      
      // status LED
      LL_GPIO_TogglePin(VTWORK_GPIO_Port, VTWORK_Pin); 
      LL_GPIO_TogglePin(VTLINK_GPIO_Port, VTLINK_Pin);
      
      status_led_set(led_value, led_bright);
      //status_led_set(data.d_in, led_bright);
      
      led_value ^= (1<<(cnt++ & 0xF)); // running-light
      led_bright += led_bright_step;   // brightness from low to high and back   
      if (led_bright < LED_BRIGHT_PERCENT_MIN) { led_bright = LED_BRIGHT_PERCENT_MIN; led_bright_step = 1; }
      if (led_bright > LED_BRIGHT_PERCENT_MAX) { led_bright = LED_BRIGHT_PERCENT_MAX; led_bright_step = -1; }
      
      // adc
      data.voltage1 = exp_filter(data.voltage1, convert_adc_to_voltage( adcData[0])); 
      data.voltage2 = exp_filter(data.voltage2, convert_adc_to_voltage( adcData[1]));      
      
      // uart-debug
      char str[64];
      
      sprintf(str, "%04X;%04X;%d.%d;%d.%d;%ld\r\n", cnt, data.d_in, 
        data.voltage1 / 1000, (data.voltage1 % 1000) / 100, 
        data.voltage2 / 1000, (data.voltage2 % 1000) / 100,
        data.uart_cnt);

      USART_2_TX((uint8_t*)str, strlen(str));

      static uint16_t cnt_1sec=0;
      if (++cnt_1sec >= 10) {
        data.uart_cnt = data.uart_cnt_t;
        data.uart_cnt_t = 0;
        cnt_1sec = 0;
      }
    }

    if (flags.uart1) 
    {
      flags.uart1 = false;
      
      if (crc8_dallas(data.uart_bfr1, 2) == data.uart_bfr1[2] )
      {
        data.d_in &= 0xFF00;
        data.d_in |= data.uart_bfr1[0];
      }
    }
    
    if (flags.uart3) 
    {
      flags.uart3 = false;
      
      if (crc8_dallas(data.uart_bfr3, 2) == data.uart_bfr3[2] )
      {
        data.d_in &= 0x00FF;
        data.d_in |= data.uart_bfr3[0] << 8;
      }
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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_1)
  {
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_3);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(36000000);
  LL_SetSystemCoreClock(36000000);
  LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSRC_PCLK2_DIV_8);
}

/* USER CODE BEGIN 4 */
void TIM2_Callback(void)
{
  if(LL_TIM_IsActiveFlag_UPDATE(TIM2))
  {
    LL_TIM_ClearFlag_UPDATE(TIM2);

    flags.timer1ms = true;
        
    USART1_TX(bfr_request, ARRAY_SIZE(bfr_request));
    USART3_TX(bfr_request, ARRAY_SIZE(bfr_request));
  
    static uint32_t timer10 = 0;
    if (++timer10 >= ( period_1s / 10) )
    {
      timer10 = 0;
      flags.timer = true;
    }

  }
}

void DMA1_USART1_RecieveComplete(void)
{
  data.uart_cnt_t++;
  flags.uart1 = true;
}
//-----------------------------------------
void DMA1_USART1_TransmitComplete(void)
{
  USART1_RX(data.uart_bfr1, UART_CMD_SIZE);
}

void DMA1_USART3_RecieveComplete(void)
{
  data.uart_cnt_t++;
  flags.uart3 = true;
}
//-----------------------------------------
void DMA1_USART3_TransmitComplete(void)
{
  USART3_RX(data.uart_bfr3, UART_CMD_SIZE);
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
