#ifdef __has_include
    #if __has_include("lvgl.h")
        #ifndef LV_LVGL_H_INCLUDE_SIMPLE
            #define LV_LVGL_H_INCLUDE_SIMPLE
        #endif
    #endif
#endif

#if defined(LV_LVGL_H_INCLUDE_SIMPLE)
    #include "lvgl.h"
#else
    #include "lvgl/lvgl.h"
#endif


#ifndef LV_ATTRIBUTE_MEM_ALIGN
#define LV_ATTRIBUTE_MEM_ALIGN
#endif

#ifndef LV_ATTRIBUTE_IMG_LOGO_WIDE
#define LV_ATTRIBUTE_IMG_LOGO_WIDE
#endif

const LV_ATTRIBUTE_MEM_ALIGN LV_ATTRIBUTE_LARGE_CONST LV_ATTRIBUTE_IMG_LOGO_WIDE uint8_t logo_wide_map[] = {
  0xea, 0xf4, 0xf4, 0xff, 	/*Color of index 0*/
  0x3f, 0x11, 0x04, 0xff, 	/*Color of index 1*/

  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0xff, 0xff, 0xc0, 0x03, 0xfe, 0x00, 0x27, 0xfc, 0x07, 0xff, 0xff, 0x80, 0xff, 0xc0, 0x00, 
  0xff, 0xff, 0xf0, 0x07, 0xff, 0x15, 0x0f, 0xfc, 0x07, 0xff, 0xff, 0xe0, 0xff, 0xe0, 0x00, 
  0xff, 0xff, 0xf8, 0x07, 0xff, 0x00, 0x0f, 0xfe, 0x07, 0xff, 0xff, 0xf1, 0xff, 0xe0, 0x00, 
  0xff, 0xff, 0xfc, 0x0f, 0xff, 0x95, 0x1f, 0xff, 0x07, 0xff, 0xff, 0xf9, 0xff, 0xf0, 0x00, 
  0xff, 0xff, 0xfe, 0x0f, 0xff, 0x80, 0x1f, 0xff, 0x07, 0xff, 0xff, 0xfb, 0xff, 0xf0, 0x00, 
  0x7f, 0xff, 0xfe, 0x1f, 0xff, 0xca, 0x3f, 0xff, 0x87, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x00, 
  0x00, 0x00, 0xff, 0x1f, 0xdf, 0xc0, 0x3f, 0xbf, 0x87, 0xf8, 0x01, 0xff, 0xff, 0xf8, 0x00, 
  0x0e, 0xff, 0xff, 0x3f, 0xdf, 0xea, 0x7f, 0xbf, 0xc7, 0xff, 0xff, 0xff, 0xf3, 0xfc, 0x00, 
  0x0f, 0xff, 0xff, 0x3f, 0x8f, 0xe6, 0x7f, 0x1f, 0xc7, 0xff, 0xff, 0xff, 0xf3, 0xfc, 0x00, 
  0x0f, 0xff, 0xff, 0x7f, 0x8f, 0xf6, 0xff, 0x1f, 0xe7, 0xff, 0xff, 0xff, 0xf1, 0xfe, 0x00, 
  0x0f, 0xff, 0xff, 0xff, 0x07, 0xf4, 0xff, 0x0f, 0xe7, 0xff, 0xff, 0xff, 0xe1, 0xfe, 0x00, 
  0x0f, 0xff, 0xff, 0xff, 0x07, 0xf9, 0xfe, 0x0f, 0xf7, 0xff, 0xff, 0xff, 0xe0, 0xff, 0x00, 
  0x0f, 0xff, 0xff, 0xfe, 0x03, 0xfd, 0xfe, 0x07, 0xf7, 0xff, 0xff, 0x3f, 0xc0, 0xff, 0x00, 
  0x00, 0x00, 0xff, 0xfe, 0x01, 0xff, 0xfc, 0x07, 0xff, 0xf8, 0x00, 0x3f, 0xc0, 0x7f, 0x00, 
  0xff, 0xff, 0xff, 0xfc, 0x01, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x00, 0x7f, 0xff, 0xff, 0x80, 
  0xff, 0xff, 0xff, 0xfc, 0x00, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x00, 0x7f, 0xff, 0xff, 0x80, 
  0xff, 0xff, 0xff, 0xf8, 0x00, 0xff, 0xfb, 0xff, 0xff, 0xf8, 0x00, 0xff, 0x3f, 0xff, 0xc0, 
  0xff, 0xff, 0xff, 0xf8, 0x00, 0x7f, 0xf3, 0xff, 0xff, 0xf8, 0x00, 0xff, 0x3f, 0xff, 0xc0, 
  0xff, 0xff, 0xef, 0xf0, 0x00, 0x7f, 0xf1, 0xff, 0xff, 0xf8, 0x00, 0xff, 0x1f, 0xff, 0xe0, 
  0xff, 0xff, 0x8f, 0xe0, 0x00, 0x3f, 0xe1, 0xff, 0xff, 0xf8, 0x01, 0xfe, 0x1f, 0xff, 0xe0, 
};

const lv_img_dsc_t logo_wide = {
  .header.cf = LV_IMG_CF_INDEXED_1BIT,
  .header.always_zero = 0,
  .header.reserved = 0,
  .header.w = 115,
  .header.h = 30,
  .data_size = 458,
  .data = logo_wide_map,
};