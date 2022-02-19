#include "disp_draw.h"
#include "ascii_chars.h"

void disp_draw_str_simple(uint8_t disp_bfr[], char str[], uint8_t length, uint8_t x, uint8_t y_line)
{
  uint8_t * p_bfr = &disp_bfr[y_line * 128 + x];

  for (int i=0; i<length; i++)
    p_bfr = symbol_to_bfr(p_bfr, str[i], 0);
}
