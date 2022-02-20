#include "power_INA226.h"

#define REG_CFG 0x0
#define REG_VSHUNT 0x1
#define REG_VBUS 0x2
#define REG_POWER 0x3
#define REG_CURRENT 0x4
#define REG_CALIBRATION 0x5
#define REG_MASK_ENABLE 0x6
#define REG_ALERT_LIMIT 0x7

#define REG_ID_MANUF 0xFE
#define REG_ID_DIE 0xFF

static power_drv_t * p_power_drv;

static void power_delay_ms(uint32_t milliseconds)
{
  if (p_power_drv->delay_ms_cb)
    p_power_drv->delay_ms_cb(milliseconds);
}

static void power_i2c_write(uint8_t bfr[], uint16_t length)
{
  if (p_power_drv->write_i2c_cb)
    p_power_drv->write_i2c_cb(p_power_drv->addr, bfr, length);
}

static void power_i2c_read(uint8_t bfr[], uint16_t length)
{
  if (p_power_drv->read_i2c_cb)
    p_power_drv->read_i2c_cb(p_power_drv->addr, bfr, length);
}

static int16_t power_read_reg(uint8_t reg)
{
  uint8_t bfr[3] = {reg, 0, 0};
  power_i2c_write(bfr, 1);
  power_i2c_read(bfr, 2);
  return (bfr[0]<<8) | bfr[1];
}
static void power_write_reg(uint8_t reg, uint16_t value)
{
  uint8_t bfr_tx[] = {reg, value >> 8, value};
  power_i2c_write(bfr_tx, 3);
}

void power_INA226_init(power_drv_t * power_drv)
{
  p_power_drv = power_drv;
  power_delay_ms(30);
  power_write_reg(REG_CFG, 0x8000 | ( power_drv->avg << 9 ) |
                        ( power_drv->conv_time << 6 ) |
                        ( power_drv->conv_time << 3 ) |
                        0x7  // Shunt and Bus, Continuous mode
  );
  power_delay_ms(30);
  static uint16_t bfr[11] = {0};
  for (int i=0; i<8; i++) bfr[i] = power_read_reg(i); 

  if (p_power_drv->r_shunt < 0.001) p_power_drv->r_shunt = 0.001;

  double v_shunt_max = 0.08192; //shunt V max = 81.92 mV
  double i_max = v_shunt_max / p_power_drv->r_shunt;
  p_power_drv->current_LSB = i_max / 32768; // 2**15
  uint16_t cal = 0.00512 / i_max / p_power_drv->r_shunt;

  power_write_reg( REG_CALIBRATION, cal);
}

void power_INA226_getValue(power_value_t * power_value)
{
  power_value->current_mA = power_read_reg(REG_CURRENT) * p_power_drv->current_LSB;
  power_value->power_mW = power_read_reg(REG_POWER) * 25 * p_power_drv->current_LSB;

  power_value->v_bus_V = power_read_reg(REG_VBUS) * 1.25;
  power_value->v_shunt_mV = power_read_reg(REG_VSHUNT) * 2.5;  

  /* IT ISN'T CORRECT! */
  power_value->current_mA = power_value->v_shunt_mV / p_power_drv->r_shunt / 1000;
}

uint16_t power_INA226_getId_manufactor(void)
{  
  return power_read_reg(REG_ID_MANUF);
}

uint16_t power_INA226_getId_die(void)
{
  return power_read_reg(REG_ID_DIE);
}
