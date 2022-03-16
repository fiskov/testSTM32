#ifndef COMMON_TYPES_H
#define COMMON_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#define ARRAY_SIZE(array) (sizeof(array) / sizeof(array)[0])
  
typedef struct data_t {
  uint16_t d_in;
  
  uint16_t voltage1, voltage2;
  
  uint8_t uart_bfr1[8], uart_bfr2[8], uart_bfr3[8];
  uint32_t uart_cnt, uart_cnt_t;
} data_t;


#endif
