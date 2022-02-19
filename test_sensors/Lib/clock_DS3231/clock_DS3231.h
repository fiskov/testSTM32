#ifndef CLOCK_DS3231_H
#define CLOCK_DS3231_H

#include <stdint.h>
#include <string.h>

typedef struct clock_drv_t {
  void (*write_i2c_cb)(uint8_t addr, uint8_t bfr[], uint16_t length);
  void (*read_i2c_cb)(uint8_t addr, uint8_t bfr[], uint16_t length);

   /** @brief I2C address (0x68 in datasheet) */
  uint8_t addr;  
} clock_drv_t;

typedef struct clock_value_t {
  uint8_t tm_hour, tm_min, tm_sec, tm_mday, tm_mon, tm_wday; 
  uint16_t tm_year;
  int8_t temperature;
} clock_value_t;

void clock_DS3231_init(clock_drv_t * clock_drv);

void clock_DS3231_getTime(clock_value_t * current_time);
void clock_DS3231_setTime(clock_value_t * new_time);

#endif