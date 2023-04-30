#include "display_gmg12864.h"


#define DISP_CMD_SET_PAGE 0xB0
#define DISP_CMD_SET_COL_H 0x10
#define DISP_CMD_SET_COL_L 0x00


typedef enum mode_pin_t_ {
  MODE_PIN_CMD = 0,
  MODE_PIN_DATA
} mode_pin_t;

static display_gmg12864_t * p_disp;
static display_gmg12864_pins_t pins = {0};

static void disp_send_spi(uint8_t * bfr, uint16_t length)
{
  if (p_disp->write_spi_cb)
    p_disp->write_spi_cb(bfr, length);
}

static void disp_delay_ms(uint32_t milliseconds)
{
  if (p_disp->delay_ms_cb)
    p_disp->delay_ms_cb(milliseconds);
}

static void disp_pin_set(display_gmg12864_pins_t *p_pins)
{
  if (p_disp->set_display_pin_cb)
    p_disp->set_display_pin_cb(p_pins);
}

void display_gmg12864_init(display_gmg12864_t * display_settings)
{
  p_disp = display_settings;
  
  pins.pin_reset = 0;
  disp_pin_set(&pins);
  
  disp_delay_ms(1);
  
  pins.pin_reset = 1;
  pins.pin_data_cmd = MODE_PIN_CMD;
  disp_pin_set(&pins);
  
  uint8_t bfr1[] = {
    0xA2, //LCD panel Characteristics: LCD bias = 1/9
    0xA0, //ADC = normal: No flip on x-direction
    0xC8, //SHL = reverse: Flip on y-direction
    0x40, //Initial Display Line = 0
    0x2C  //Power Control: Follower off, Regulator off, Converter on
  };
  disp_send_spi(bfr1, sizeof(bfr1));
  disp_delay_ms(50);

  uint8_t bfr2[] = {
    0x2E  //Power Control: Follower off, Regulator on, Converter on
  };
  disp_send_spi(bfr2, sizeof(bfr2));
  disp_delay_ms(50);

  uint8_t bfr3[] = {
    0x2F  //Power Control: Follower off, Regulator on, Converter on
  };
  disp_send_spi(bfr3, sizeof(bfr3));
  disp_delay_ms(50);

  uint8_t bfr4[] = {
    0x26, //Set the built-in resistor ratio
    0x81, //Set Reference Voltage Mode
    0x1A, //Set Reference Voltage Resistor
    0xAF, //Turn on the LCD Display
    DISP_CMD_SET_PAGE, //Set page address = 0
    DISP_CMD_SET_COL_H, //Set column address (upper 4-bit = 0)
    DISP_CMD_SET_COL_L  //Set column address (lower 4-bit = 0)
  };
  disp_send_spi(bfr4, sizeof(bfr4));
  disp_delay_ms(1);  

  pins.pin_data_cmd = MODE_PIN_DATA;
  disp_pin_set(&pins);
}

void display_gmg12864_clear(void)
{

}

void display_gmg12864_print(void)
{
  uint8_t bfr_cmd[] = {
    DISP_CMD_SET_PAGE, 
    DISP_CMD_SET_COL_H, //set first column
    DISP_CMD_SET_COL_L  //
  };
  uint32_t page_size = p_disp->bfr_size / 8;
  
  for (uint8_t p=0; p<8; p++)
  {
    //set PAGE ( display has 8 pages )
    pins.pin_data_cmd = MODE_PIN_CMD;
    bfr_cmd[0] = DISP_CMD_SET_PAGE | p;
    disp_pin_set(&pins);
    disp_send_spi(bfr_cmd, sizeof(bfr_cmd));

    //send PAGE ( 1/8 of whole buffer )
    pins.pin_data_cmd = MODE_PIN_DATA;
    disp_pin_set(&pins);
    uint32_t bfr_pos = page_size * p;

    disp_send_spi(&(p_disp->bfr[bfr_pos]), page_size);
  }
}
