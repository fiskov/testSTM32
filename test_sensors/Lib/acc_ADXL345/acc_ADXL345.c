#include "acc_ADXL345.h"

#define REG_ID  0x00
#define REG_BW_RATE   0x2C
#define REG_POWER_CTL 0x2D
#define REG_OFSX 0x1E
#define REG_OFSY 0x1F
#define REG_OFSZ 0x20


#define REG_DATA_FORMAT 0x31
#define REG_DATA  0x32

#define MODE_READ 0x80
#define MODE_MULTI 0x40

static acc_drv_t * p_acc_drv;
/*
static void acc_i2c_write(uint8_t bfr[], uint16_t length)
{
  if (p_acc_drv->write_i2c_cb)
    p_acc_drv->write_i2c_cb(p_acc_drv->addr, bfr, length);
}

static void acc_i2c_read(uint8_t bfr[], uint16_t length)
{
  if (p_acc_drv->read_i2c_cb)
    p_acc_drv->read_i2c_cb(p_acc_drv->addr, bfr, length);
}
*/
static void acc_spi_write_reg(uint8_t addr, uint8_t value)
{
  uint8_t bfr[] = {addr, value};
  
  if (p_acc_drv->spi_write_cb)
    p_acc_drv->spi_write_cb(bfr, sizeof(bfr));
}
static uint8_t acc_spi_read_reg(uint8_t addr)
{
  uint8_t bfr[2] = {addr | MODE_READ, 0};
  
  if (p_acc_drv->spi_write_read_cb)
    p_acc_drv->spi_write_read_cb(bfr, bfr, 2);

  return bfr[1];
}
static void acc_spi_write_read(uint8_t addr, uint8_t * bfr_rx, uint16_t length)
{
  if (p_acc_drv->spi_write_read_cb)
    p_acc_drv->spi_write_read_cb(bfr_rx, bfr_rx, length);
}
static void acc_delay_ms(uint32_t milliseconds)
{
  if (p_acc_drv->delay_ms_cb)
    p_acc_drv->delay_ms_cb(milliseconds);
}

void acc_ADXL345_init(acc_drv_t * acc_drv)
{
  uint8_t bfr[2];

  p_acc_drv = acc_drv;

  acc_spi_write_reg(REG_BW_RATE, p_acc_drv->datarate);
  acc_spi_write_reg(REG_DATA_FORMAT, 0);
  acc_spi_write_reg(REG_OFSX, 0);
  acc_spi_write_reg(REG_OFSY, 0);
  acc_spi_write_reg(REG_OFSZ, 0);

  acc_ADXL345_wakeup();
}

void acc_ADXL345_getXYZ(acc_point_t * point)
{
  uint8_t bfr[8] = {0};
  bfr[0] = REG_DATA | MODE_READ | MODE_MULTI;
  //for (int i=0; i<6; i++) bfr[i] = (REG_DATA + i) | MODE_READ;
  acc_spi_write_read(bfr, bfr, 7);

  point->x = (bfr[2] << 8) | bfr[1];
  point->y = (bfr[4] << 8) | bfr[3];
  point->z = (bfr[6] << 8) | bfr[5];
}

uint8_t acc_ADXL345_getID(void)
{  
  return acc_spi_read_reg(REG_ID);
}

void acc_ADXL345_sleep(void)
{  
  acc_spi_write_reg(REG_POWER_CTL, 0);
}

void acc_ADXL345_wakeup(void)
{
  acc_spi_write_reg(REG_POWER_CTL, 0x08);
}
