#include "mag_HMC5883L.h"

#define REG_CONF_A 0x00
#define REG_CONF_B 0x01
#define REG_MODE 0x02
#define REG_STATUS 0x09
#define REG_ID 0x0A
#define REG_DATA 0x03

#define MAG_X_0 453
#define MAG_Y_0 457
#define MAG_Z_0 424


static mag_drv_t * p_mag_drv;

static void mag_i2c_write(uint8_t bfr[], uint16_t length)
{
  if (p_mag_drv->write_i2c_cb)
    p_mag_drv->write_i2c_cb(p_mag_drv->addr, bfr, length);
}

static void mag_i2c_read(uint8_t bfr[], uint16_t length)
{
  if (p_mag_drv->read_i2c_cb)
    p_mag_drv->read_i2c_cb(p_mag_drv->addr, bfr, length);
}

static void mag_delay_ms(uint32_t milliseconds)
{
  if (p_mag_drv->delay_ms_cb)
    p_mag_drv->delay_ms_cb(milliseconds);
}

void mag_HMC5883L_default(mag_drv_t * p_mag_drv)
{
  const mag_drv_t mag_drv_default = {
    .addr = 0x1E,
    .datarate = MAG_DATARATE_15,
    .avg = MAG_AVG_8,
    .gain = MAG_GAIN_5,
    .kx = 1.0, .ky = 1.0, .kz = 1.0
  };
  *p_mag_drv = mag_drv_default;
}


void mag_HMC5883L_init(mag_drv_t * mag_drv)
{
  uint8_t bfr[2];

  p_mag_drv = mag_drv;

  // x-average, y Hz default, normal measurement
  uint8_t reg = (p_mag_drv->avg << 5) | (p_mag_drv->datarate << 2);

  bfr[0] = REG_CONF_A; bfr[1] = reg; mag_i2c_write(bfr, sizeof(bfr));

  //Gain
  reg = p_mag_drv->gain << 5;
  
  bfr[0] = REG_CONF_B; bfr[1] = reg; mag_i2c_write(bfr, sizeof(bfr));
  
  //Continuous-measurement mode
  bfr[0] = REG_MODE;   bfr[1] = 0;   mag_i2c_write(bfr, sizeof(bfr));  

  mag_delay_ms(1);
}

void mag_HMC5883L_getXYZ(mag_point_t * point)
{
  uint8_t bfr[6] = {0};
  bfr[0] = REG_DATA; //read 6 regs
  mag_i2c_write(bfr, 1);
  mag_i2c_read(bfr, 6);
  point->x = ((bfr[0] << 8) | bfr[1]) * p_mag_drv->kx;
  point->y = ((bfr[2] << 8) | bfr[3]) * p_mag_drv->ky;
  point->z = ((bfr[4] << 8) | bfr[5]) * p_mag_drv->kz;
}

uint32_t mag_HMC5883L_getID(void)
{
  uint8_t bfr[3] = {0};
  bfr[0] = REG_ID; 
  mag_i2c_write(bfr, 1);
  mag_i2c_read(bfr, 3);
  return (bfr[0] << 16) | (bfr[1] << 8) | bfr[2];
}

uint8_t mag_HMC5883L_getStatus(void)
{
  uint8_t bfr[1] = {0};
  bfr[0] = REG_STATUS; 
  mag_i2c_write(bfr, 1);
  mag_i2c_read(bfr, 1);
  return bfr[0];
}

void mag_HMC5883L_compenstation(mag_point_t * p_point)
{
  uint8_t bfr[2];  

  // 8-average, 15 Hz default, normal measurement  
  bfr[0]= REG_CONF_A; bfr[1] = 0x71; mag_i2c_write(bfr, 2);

  // gain = 5
  bfr[0]= REG_CONF_B; bfr[1] = 0xA0; mag_i2c_write(bfr, 2);

  // Continuous-measurement mode
  bfr[0]= REG_MODE;   bfr[1] = 0x00; mag_i2c_write(bfr, 2);

  mag_delay_ms(6);
  for (int i=0; i<15; i++) 
  {
    mag_delay_ms(1000 / 15); //15 Hz
    mag_HMC5883L_getXYZ(p_point);    
  }

  p_mag_drv->kx = (double)p_point->x / MAG_X_0;
  p_mag_drv->ky = (double)p_point->y / MAG_Y_0;
  p_mag_drv->kz = (double)p_point->z / MAG_Z_0;  

  mag_HMC5883L_init(p_mag_drv);
  mag_HMC5883L_getXYZ(p_point);
}
