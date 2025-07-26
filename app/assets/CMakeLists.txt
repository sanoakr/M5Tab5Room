set(JP_FONT_SRC m_plus_1p_light_32.c)

add_library(japanese_font32 STATIC ${JP_FONT_SRC})
target_include_directories(japanese_font32 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
