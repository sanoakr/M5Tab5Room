#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// WiFi status update function for room sign
void updateRoomSignStatus(const char* main_status, const char* sub_status, uint32_t color);

#ifdef __cplusplus
}
#endif