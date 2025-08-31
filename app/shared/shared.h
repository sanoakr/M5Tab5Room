/*
 * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#pragma once
#include <string>
#include <thread>
#include <mutex>
#include <string>
#include <vector>
#include <queue>
#include <functional>
#include <smooth_ui_toolkit.h>

/**
 * @brief 共享数据层，提供一个带互斥锁的全局共享数据单例
 *
 */
namespace shared_data {

/**
 * @brief 共享数据定义
 *
 */
struct SharedData_t {
    smooth_ui_toolkit::Signal<std::string> systemStateEvents;
    smooth_ui_toolkit::Signal<std::string> inputEvents;

    // WebサーバからUI層へのルームサイン更新要求キュー
    struct RoomStatus {
        std::string main_text;
        std::string sub_text;
        uint32_t color = 0xFFFFFF;
    };
    std::mutex roomStatusMutex;
    std::queue<RoomStatus> roomStatusQueue;
};

/**
 * @brief 获取共享数据实例
 *
 * @return SharedData_t&
 */
SharedData_t* Get();

/**
 * @brief 销毁共享数据实例
 *
 */
void Destroy();

}  // namespace shared_data

/**
 * @brief 获取共享数据实例
 *
 * @return SharedData_t&
 */
inline shared_data::SharedData_t* GetSharedData()
{
    return shared_data::Get();
}

inline smooth_ui_toolkit::Signal<std::string>& GetSystemStateEvents()
{
    return GetSharedData()->systemStateEvents;
}

inline smooth_ui_toolkit::Signal<std::string>& GetInputEvents()
{
    return GetSharedData()->inputEvents;
}

// ルームサイン更新要求をエンキュー
inline void EnqueueRoomStatus(const shared_data::SharedData_t::RoomStatus& status)
{
    auto* s = GetSharedData();
    std::lock_guard<std::mutex> lock(s->roomStatusMutex);
    s->roomStatusQueue.push(status);
}

// ルームサイン更新要求をデキュー（1件）。存在しない場合は false
inline bool TryDequeueRoomStatus(shared_data::SharedData_t::RoomStatus& out)
{
    auto* s = GetSharedData();
    std::lock_guard<std::mutex> lock(s->roomStatusMutex);
    if (s->roomStatusQueue.empty()) return false;
    out = s->roomStatusQueue.front();
    s->roomStatusQueue.pop();
    return true;
}
