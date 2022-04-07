/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __COMMON_H
#define __COMMON_H


#include <stdint.h>

typedef struct flags_t {
  uint8_t timer1s;
} flags_t;


typedef struct data_t {
  uint32_t addr, addr0;
  flags_t flags;

  uint32_t reg_mode_spi, reg_mode_gpio;
  uint32_t reg_exti_scl_only, reg_exti_sda_only, reg_exti_none;
} data_t;

extern data_t data;

#define _EXTI_SCL_ONLY() do{ EXTI->IMR = EXTI_IMR_IM3; } while(0)
#define _EXTI_SDA_ONLY() do{ EXTI->IMR = EXTI_IMR_IM5; } while(0)
#define _EXTI_NONE() do{ EXTI->IMR = 0; } while(0)
#define TIME_0_1us  7
#define TIME_1_us 72

#endif
