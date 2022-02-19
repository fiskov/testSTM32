#ifndef ACC_ADXL345_H
#define ACC_ADXL345_H

#include <stdint.h>

typedef enum {
  ACC_AVG_1 = 0,
  ACC_AVG_2,
  ACC_AVG_4,
  ACC_AVG_8
} acc_ADXL345_avg_t;

typedef enum {
  ACC_DATARATE_0_10 = 0,
  ACC_DATARATE_0_20,
  ACC_DATARATE_0_39,
  ACC_DATARATE_0_78,
  ACC_DATARATE_1_56,
  ACC_DATARATE_3_13,
  ACC_DATARATE_6_25,
  ACC_DATARATE_12_5,
  ACC_DATARATE_25,
  ACC_DATARATE_50,
  ACC_DATARATE_100,
  ACC_DATARATE_200,
  ACC_DATARATE_400,
  ACC_DATARATE_800,
  ACC_DATARATE_1600,
  ACC_DATARATE_3200
} acc_ADXL345_datarate_t;

typedef enum {
  ACC_GAIN_0 = 0,
  ACC_GAIN_1,
  ACC_GAIN_2,
  ACC_GAIN_3,
  ACC_GAIN_4,
  ACC_GAIN_5,
  ACC_GAIN_6,
  ACC_GAIN_7  
} acc_ADXL345_gain_t;

typedef struct {
  //void (*write_i2c_cb)(uint8_t addr, uint8_t bfr[], uint16_t length);
  //void (*read_i2c_cb)(uint8_t addr, uint8_t bfr[], uint16_t length);
  void (*spi_write_cb)(uint8_t bfr[], uint16_t length);
  void (*spi_write_read_cb)(uint8_t * bfr_tx, uint8_t * bfr_rx, uint16_t length);
  void (*delay_ms_cb)(uint32_t milliseconds);
  
  acc_ADXL345_avg_t avg;
  acc_ADXL345_datarate_t datarate;
  acc_ADXL345_gain_t gain;
} acc_drv_t;

typedef struct {
  int16_t x, y, z, t;
} acc_point_t;

/**
 * @brief Initialize acc_drv with default values
 * 
 * @param acc_drv 
 */
void acc_ADXL345_default(acc_drv_t * acc_drv);

/**
 * @brief Run device
 * 
 * @param acc_drv 
 */
void acc_ADXL345_init(acc_drv_t * acc_drv);

void acc_ADXL345_getXYZ(acc_point_t * point);

/**
 * @brief Get device id. Must be 0xE5=229
 * 
 * @return uint8_t 
 */
uint8_t acc_ADXL345_getID(void);

uint8_t acc_ADXL345_getStatus(void);

void acc_ADXL345_sleep(void);
void acc_ADXL345_wakeup(void);


#endif