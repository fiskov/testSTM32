#ifndef DISPLAY_GMG12864_H
#define DISPLAY_GMG12864_H

#include <stdint.h>

#define DISP_WIDTH   128
#define DISP_HEIGHT   64

typedef struct _display_gmg12864_pins_t {
  uint8_t pin_reset : 1;
  uint8_t pin_data_cmd : 1;
} display_gmg12864_pins_t;

typedef struct _display_gmg12864_t {
  void (*write_spi_cb)(uint8_t * bfr, uint16_t length);
  void (*delay_ms_cb)(uint32_t milliseconds);
  void (*set_display_pin_cb)(display_gmg12864_pins_t * pins);
    
  uint8_t width;
  uint8_t * bfr;
  uint32_t bfr_size;
} display_gmg12864_t;

/// init display
void display_gmg12864_init(display_gmg12864_t * display_settings);

/// clear display 
void display_gmg12864_clear(void);

void display_gmg12864_print(void);

#endif
