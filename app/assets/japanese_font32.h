#ifndef JAPANESE_FONT32_H
#define JAPANESE_FONT32_H

#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif

// 日本語フォントの宣言
extern const lv_font_t m_plus_1p_light_32;

inline const lv_font_t* get_japanese_font32() { return &m_plus_1p_light_32; }

#endif // JAPANESE_FONT32_H
