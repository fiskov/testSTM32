#ifndef DISPLAY_DRAW_H
#define DISPLAY_DRAW_H

#include <stdint.h>

void disp_draw_str_simple(uint8_t disp_bfr[], char str[], uint8_t length, uint8_t x, uint8_t y_line);
void disp_draw_pixel(uint8_t disp_bfr[], uint8_t x, uint8_t y);
void disp_draw_line(uint8_t disp_bfr[], uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
void disp_draw_circle(uint8_t disp_bfr[], int8_t xm, int8_t ym, int8_t r);

#endif
