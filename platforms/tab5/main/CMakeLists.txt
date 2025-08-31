
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

# Build-time Wi-Fi credential injection from an external config file
# Expected format at platforms/tab5/wifi.conf:
#   ssid=YourSSID
#   password=YourPassword
set(WIFI_CONF_PATH "${CMAKE_CURRENT_LIST_DIR}/../wifi.conf")
set(WIFI_BUILD_SSID "")
set(WIFI_BUILD_PASS "")
if(EXISTS ${WIFI_CONF_PATH})
    # 読み取りやすいように行単位で取得し、key=value を解析
    file(STRINGS ${WIFI_CONF_PATH} WIFI_CONF_LINES REGEX "^[ \t]*(ssid|password)[ \t]*=")
    foreach(_line IN LISTS WIFI_CONF_LINES)
        string(REGEX MATCH "^[ \t]*([^=]+)[ \t]*=[ \t]*(.*)$" _match "${_line}")
        set(_key "${CMAKE_MATCH_1}")
        set(_val "${CMAKE_MATCH_2}")
        string(STRIP "${_key}" _key)
        string(STRIP "${_val}" _val)
        if(_key STREQUAL "ssid")
            set(WIFI_BUILD_SSID "${_val}")
        elseif(_key STREQUAL "password")
            set(WIFI_BUILD_PASS "${_val}")
        endif()
    endforeach()
endif()

configure_file(${CMAKE_CURRENT_LIST_DIR}/wifi_credentials.h.in
               ${CMAKE_CURRENT_BINARY_DIR}/wifi_credentials.h @ONLY)

target_include_directories(${COMPONENT_LIB} PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
