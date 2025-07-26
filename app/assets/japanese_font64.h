#ifndef JAPANESE_FONT64_H
#define JAPANESE_FONT64_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"

extern const lv_font_t m_plus_1p_light_64;

inline const lv_font_t* get_japanese_font64() {
    return &m_plus_1p_light_64;
}

#ifdef __cplusplus
}
#endif

#endif // JAPANESE_FONT64_H 