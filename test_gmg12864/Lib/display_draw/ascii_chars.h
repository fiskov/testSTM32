#ifndef ASCII_CHAR_H_
#define ASCII_CHAR_H_

#include <stdint.h>

#define FONT_7x5_WIDTH  6
#define FONT_7x5_HEIGHT 8

uint8_t * symbol_to_bfr(uint8_t bfr[], char ch, uint8_t invert_color);

#endif
