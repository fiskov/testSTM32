#ifndef MAG_HMC5883L_H
#define MAG_HMC5883L_H

#include <stdint.h>

typedef enum {
  MAG_AVG_1 = 0,
  MAG_AVG_2,
  MAG_AVG_4,
  MAG_AVG_8
} mag_HMC5883L_avg_t;

typedef enum {
  MAG_DATARATE_0_75 = 0,
  MAG_DATARATE_1_5,
  MAG_DATARATE_3,
  MAG_DATARATE_7_5,
  MAG_DATARATE_15,
  MAG_DATARATE_30,
  MAG_DATARATE_75
} mag_HMC5883L_datarate_t;

typedef enum {
  MAG_GAIN_0 = 0,
  MAG_GAIN_1,
  MAG_GAIN_2,
  MAG_GAIN_3,
  MAG_GAIN_4,
  MAG_GAIN_5,
  MAG_GAIN_6,
  MAG_GAIN_7  
} mag_HMC5883L_gain_t;

typedef struct {
  void (*write_i2c_cb)(uint8_t addr, uint8_t bfr[], uint16_t length);
  void (*read_i2c_cb)(uint8_t addr, uint8_t bfr[], uint16_t length);
  void (*delay_ms_cb)(uint32_t milliseconds);
  uint8_t addr;
  double kx, ky, kz;
  mag_HMC5883L_avg_t avg;
  mag_HMC5883L_datarate_t datarate;
  mag_HMC5883L_gain_t gain;
} mag_drv_t;

typedef struct {
  int16_t x, y, z, t;
} mag_point_t;

/**
 * @brief Initialize mag_drv with default values
 * 
 * @param mag_drv 
 */
void mag_HMC5883L_default(mag_drv_t * mag_drv);

/**
 * @brief Run device
 * 
 * @param mag_drv 
 */
void mag_HMC5883L_init(mag_drv_t * mag_drv);

void mag_HMC5883L_getXYZ(mag_point_t * point);

/**
 * @brief Get device id. Must be 0x00483433
 * 
 * @return uint32_t 
 */
uint32_t mag_HMC5883L_getID(void);

uint8_t mag_HMC5883L_getStatus(void);

/**
 * @brief NOT WORKING! Process self-temperature-compensation. Duration ~0.6 s.
 * 
 * @param p_point 
 */
void mag_HMC5883L_compenstation(mag_point_t * p_point);

#endif