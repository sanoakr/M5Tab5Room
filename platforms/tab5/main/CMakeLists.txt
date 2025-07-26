
file(GLOB_RECURSE APP_LAYER_SRCS
    ../../../app/*.c
    ../../../app/*.cc
    ../../../app/*.cpp
)

set(APP_LAYER_INCS
    ../../../app
)


# 日本語フォントCファイルを直接追加
set(JP_FONT_SRC ../../../app/assets/m_plus_1p_light_32.c)

file(GLOB_RECURSE MY_HAL_SRCS
    ./hal/*.c
    ./hal/*.cc
    ./hal/*.cpp
)

idf_component_register(SRCS "app_main.cpp" ${APP_LAYER_SRCS} ${MY_HAL_SRCS} ${JP_FONT_SRC}
                    INCLUDE_DIRS "." ${APP_LAYER_INCS} ../../../app/assets
                    EMBED_TXTFILES "../audio/canon_in_d.mp3" "../audio/startup_sfx.mp3" "../audio/shutdown_sfx.mp3")
