#ifndef JAPANESE_FONT128_H
#define JAPANESE_FONT128_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"

extern const lv_font_t m_plus_1p_light_128;

inline const lv_font_t* get_japanese_font128() {
    return &m_plus_1p_light_128;
}

#ifdef __cplusplus
}
#endif

#endif // JAPANESE_FONT64_H 