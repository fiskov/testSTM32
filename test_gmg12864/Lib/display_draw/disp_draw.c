#include "disp_draw.h"
#include "ascii_chars.h"
#include <stdlib.h>

void disp_draw_str_simple(uint8_t disp_bfr[], char str[], uint8_t length, uint8_t x, uint8_t y_line)
{
  uint8_t * p_bfr = &disp_bfr[y_line * 128 + x];

  for (int i=0; i<length; i++)
    p_bfr = symbol_to_bfr(p_bfr, str[i], 0);
}

void disp_draw_pixel(uint8_t disp_bfr[], uint8_t x, uint8_t y)
{
  disp_bfr[(x % 128) + (y % 64)/8*128] |= 1 << (y % 8);
}

void disp_draw_line(uint8_t disp_bfr[], uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
   int dx =  abs(x1-x0), sx = x0<x1 ? 1 : -1;
   int dy = -abs(y1-y0), sy = y0<y1 ? 1 : -1; 
   int err = dx+dy, e2; /* error value e_xy */
 
   for(;;){  /* loop */
      disp_draw_pixel(disp_bfr, x0, y0);
      if (x0==x1 && y0==y1) break;
      e2 = 2*err;
      if (e2 >= dy) { err += dy; x0 += sx; } /* e_xy+e_x > 0 */
      if (e2 <= dx) { err += dx; y0 += sy; } /* e_xy+e_y < 0 */
   }
}
void disp_draw_circle(uint8_t disp_bfr[], int8_t xm, int8_t ym, int8_t r)
{
   int x = -r, y = 0, err = 2-2*r; /* II. Quadrant */ 
   do {
      disp_draw_pixel(disp_bfr, xm-x, ym+y); /*   I. Quadrant */
      disp_draw_pixel(disp_bfr, xm-y, ym-x); /*  II. Quadrant */
      disp_draw_pixel(disp_bfr, xm+x, ym-y); /* III. Quadrant */
      disp_draw_pixel(disp_bfr, xm+y, ym+x); /*  IV. Quadrant */
      r = err;
      if (r <= y) err += ++y*2+1;           /* e_xy+e_y < 0 */
      if (r > x || err > y) err += ++x*2+1; /* e_xy+e_x > 0 or no 2nd y-step */
   } while (x < 0);
}