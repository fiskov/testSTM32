#ifndef POWER_INA226_H
#define POWER_INA226_H

#include <stdint.h>
#include <string.h>

typedef enum {
  POWER_AVG_1 = 0,
  POWER_AVG_4,
  POWER_AVG_16,
  POWER_AVG_64,
  POWER_AVG_128,
  POWER_AVG_256,
  POWER_AVG_512,
  POWER_AVG_1024
} power_INA226_avg_t;

typedef enum {
  POWER_TIME_0_1_ms = 0,
  POWER_TIME_0_2_ms,
  POWER_TIME_0_3_ms,
  POWER_TIME_0_6_ms,
  POWER_TIME_1_ms,
  POWER_TIME_2_ms,
  POWER_TIME_4_ms,
  POWER_TIME_8_ms
} power_INA226_conv_time_t;


typedef struct power_drv_t {
  void (*write_i2c_cb)(uint8_t addr, uint8_t bfr[], uint16_t length);
  void (*read_i2c_cb)(uint8_t addr, uint8_t bfr[], uint16_t length);
  void (*delay_ms_cb)(uint32_t milliseconds);

   /** @brief I2C address (0x40 = A1 to GND, A0 to GND) */
  uint8_t addr; 
  /** @brief Shunt resistor (0.1 Ohm in my case) */
  double r_shunt; 

  power_INA226_avg_t avg;
  power_INA226_conv_time_t conv_time;

  double current_LSB, current_Max;
} power_drv_t;

typedef struct power_value_t {
  double current_mA, power_mW, v_bus_V, v_shunt_mV;
} power_value_t;

void power_INA226_init(power_drv_t * power_drv);
void power_INA226_getValue(power_value_t * power_value);

/** @brief manufacturer id = 0x5449 */
uint16_t power_INA226_getId_manufactor(void);
/** @brief die id = 0x2260 */
uint16_t power_INA226_getId_die(void);


#endif