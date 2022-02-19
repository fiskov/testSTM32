#include "clock_DS3231.h"

#define REG_SEC 0x0
#define REG_MIN 0x1
#define REG_HOUR 0x2

#define REG_DAYWEEK 0x3

#define REG_DAY 0x4
#define REG_MON 0x5
#define REG_YEAR 0x6
#define REG_CTRL 0xE
#define REG_TEMP 0x11


static clock_drv_t * p_clock_drv;

static inline uint8_t bcd2int(uint8_t value) {
  return (value >> 4) * 10 + (value & 0xF);
}
static inline uint8_t int2bcd(uint8_t value) {
  return ((value / 10) << 4) + (value % 10);
}

static void clock_i2c_write(uint8_t bfr[], uint16_t length)
{
  if (p_clock_drv->write_i2c_cb)
    p_clock_drv->write_i2c_cb(p_clock_drv->addr, bfr, length);
}

static void clock_i2c_read(uint8_t bfr[], uint16_t length)
{
  if (p_clock_drv->read_i2c_cb)
    p_clock_drv->read_i2c_cb(p_clock_drv->addr, bfr, length);
}

void clock_DS3231_init(clock_drv_t * clock_drv)
{
  p_clock_drv = clock_drv;

  uint8_t bfr[2] = {0x0E, 0x24};
  clock_i2c_write(bfr, sizeof(bfr));
}
  
void clock_DS3231_getTime(clock_value_t * current_time)
{  
  uint8_t bfr[7] = {0};
  
  clock_i2c_write(bfr, 1);
  clock_i2c_read(bfr, 7);  //read regs
  
  current_time->tm_sec = bcd2int(bfr[0]);
  current_time->tm_min = bcd2int(bfr[1]);
  current_time->tm_hour= bcd2int(bfr[2]);

  current_time->tm_mday = bcd2int(bfr[4]);
  current_time->tm_mon  = bcd2int(bfr[5] & 0x1F);
  current_time->tm_year = 2000 + bcd2int(bfr[6]);
}
uint8_t clock_DS3231_getTemperature(void)
{
  uint8_t bfr[2];
  bfr[0] = REG_TEMP; 
  clock_i2c_write(bfr, 1);
  clock_i2c_read(bfr, 1);  //read regs  
  return bfr[0];
}

void clock_DS3231_setTime(clock_value_t * new_time)
{
  uint8_t bfr0[] = {
    0xE,
    0x84  //stop clock
  };
  clock_i2c_write(bfr0, sizeof(bfr0));

  uint8_t bfr[] = {      
    0, // addr 0x00
   int2bcd(new_time->tm_sec), // sec 
   int2bcd(new_time->tm_min) , // min
   int2bcd(new_time->tm_hour) , // hour
   int2bcd(new_time->tm_wday) , // weekday  
   int2bcd(new_time->tm_mday) , // mon    
   int2bcd(new_time->tm_mon) | 0x80 , // mon + century
   int2bcd(new_time->tm_year - 2000) , // year
  };  
  clock_i2c_write(bfr, sizeof(bfr));

  bfr0[1] = 0x04;  //start clock
  clock_i2c_write(bfr0, sizeof(bfr0));
}
