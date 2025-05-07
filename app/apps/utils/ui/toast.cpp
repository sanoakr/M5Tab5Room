/*
 * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include "toast.h"
#include <hal/hal.h>
#include <mooncake.h>
#include <mooncake_log.h>
#include <memory>
#include <stdint.h>
#include <queue>

using namespace ui;
using namespace smooth_ui_toolkit;
using namespace smooth_ui_toolkit::lvgl_cpp;
using namespace mooncake;

struct ToastColor_t {
    uint32_t bg     = 0x000000;
    uint32_t border = 0x000000;
    uint32_t msg    = 0xFFFFFF;
};

static const std::string& _tag = "toast";

static ToastColor_t _toast_color_info    = {0xE6F1FE, 0xAFD1F9, 0x005BC4};
static ToastColor_t _toast_color_warning = {0xFEFCE8, 0xDBD38B, 0xC4841D};
static ToastColor_t _toast_color_error   = {0xFEE7EF, 0xFCB5CD, 0xC20E4D};
static ToastColor_t _toast_color_success = {0xE8FAF0, 0xA2E8C1, 0x12A150};
static ToastColor_t _toast_color_orange  = {0xFFDCD1, 0xFFBFAA, 0xF84C13};
static ToastColor_t _toast_color_gray    = {0xFFFFFF, 0xD9D9D9, 0x11181C};
static ToastColor_t _toast_color_rose    = {0xFFE9F4, 0xFFBFDF, 0xFF339A};
static ToastColor_t _toast_color_dark    = {0x383838, 0x222222, 0xF4F3F3};

static Toast::KeyFrame_t _toast_kf_closed = {-100, 320};
static Toast::KeyFrame_t _toast_kf_opened = {16, 640};
static int16_t _toast_x                   = -276;
static int16_t _toast_h                   = 80;

static ToastColor_t get_toast_color(toast_type::Type_t type)
{
    // switch (type) {
    //     case toast_type::info:
    //         return _toast_color_info;
    //     case toast_type::warning:
    //         return _toast_color_warning;
    //     case toast_type::error:
    //         return _toast_color_error;
    //     case toast_type::success:
    //         return _toast_color_success;
    //     case toast_type::orange:
    //         return _toast_color_orange;
    //     case toast_type::gray:
    //         return _toast_color_gray;
    //     case toast_type::rose:
    //         return _toast_color_rose;
    //     case toast_type::dark:
    //         return _toast_color_dark;
    //     default:
    //         return _toast_color_info;
    // }
    return _toast_color_dark;
}

void Toast::init(lv_obj_t* parent)
{
    auto toast_color = get_toast_color(config.type);

    _toast = std::make_unique<Container>(parent);
    _toast->setBorderWidth(1);
    _toast->setBorderColor(lv_color_hex(toast_color.border));
    _toast->setBgColor(lv_color_hex(toast_color.bg));
    _toast->setRadius(24);
    _toast->setAlign(LV_ALIGN_TOP_MID);
    _toast->removeFlag(LV_OBJ_FLAG_SCROLLABLE);
    _toast->onClick().connect([&]() { close(); });

    _msg_label = std::make_unique<Label>(_toast->get());
    _msg_label->setTextFont(&lv_font_montserrat_24);
    _msg_label->setText(config.msg);
    _msg_label->setTextColor(lv_color_hex(toast_color.msg));
    _msg_label->align(LV_ALIGN_CENTER, 0, 0);
    if (_msg_label->getWidth() > 580) {
        _msg_label->setWidth(580);
        _msg_label->setLongMode(LV_LABEL_LONG_SCROLL_CIRCULAR);
    }

    _anim_y.springOptions().visualDuration = 0.4;
    _anim_y.springOptions().bounce         = 0.3;
    _anim_w.springOptions().visualDuration = 0.4;
    _anim_w.springOptions().bounce         = 0.3;

    close(true);
}

void Toast::update()
{
    // Apply animation
    if (!_anim_y.done()) {
        _toast->setPos(_toast_x, _anim_y);
    }
    if (!_anim_w.done()) {
        _toast->setSize(_anim_w, _toast_h);
    }

    // Update state
    if (_anim_y.done() && _anim_w.done()) {
        if (_state == Opening) {
            _state      = Opened;
            _time_count = GetHAL()->millis();
        } else if (_state == Closing) {
            _state = Closed;
        }
    }

    if (_state == Opened) {
        if (GetHAL()->millis() - _time_count > config.durationMs) {
            close();
        }
    }
}

void Toast::close(bool teleport)
{
    _state = Closing;
    update_anim(_toast_kf_closed, teleport);
}

void Toast::open(bool teleport)
{
    _state = Opening;
    update_anim(_toast_kf_opened, teleport);
}

void Toast::stack(bool teleport)
{
    if (_state == Closing || _state == Closed) {
        return;
    }

    _stack_depth++;

    auto kf_stack = _toast_kf_opened;
    kf_stack.y += _stack_depth * (12 - _stack_depth * 2);
    kf_stack.w -= _stack_depth * 20 * 2;

    update_anim(kf_stack, teleport);
}

void Toast::update_anim(const KeyFrame_t& target, bool teleport)
{
    if (teleport) {
        _anim_y.teleport(target.y);
        _anim_w.teleport(target.w);

        _toast->setPos(_toast_x, _anim_y);
        _toast->setSize(_anim_w, _toast_h);
    } else {
        _anim_y = target.y;
        _anim_w = target.w;
    }
}

/* -------------------------------------------------------------------------- */
/*                                Toast Manager                               */
/* -------------------------------------------------------------------------- */
static std::queue<Toast::Config_t> _toast_request_queue;
static int _toast_manager_id = -1;

void ToastManager::onCreate()
{
    // 3 toasts max
    _toast_list.resize(3);
}

void ToastManager::onRunning()
{
    LvglLockGuard lock;

    // Handle toast request
    if (!_toast_request_queue.empty()) {
        auto toast_request = _toast_request_queue.front();
        _toast_request_queue.pop();

        // Stack current toast
        for (auto& toast : _toast_list) {
            if (toast) {
                toast->stack();
            }
        }

        // Create new toast
        _toast_list[_current_toast_index]         = std::make_unique<Toast>();
        _toast_list[_current_toast_index]->config = toast_request;
        _toast_list[_current_toast_index]->init(lv_screen_active());
        _toast_list[_current_toast_index]->open();

        _current_toast_index++;
        if (_current_toast_index >= _toast_list.size()) {
            _current_toast_index = 0;
        }
    }

    // Update toast
    for (auto& toast : _toast_list) {
        if (toast) {
            toast->update();
            if (toast->getState() == Toast::Closed) {
                toast.reset();
            }
        }
    }
}

void ui::pop_a_toast(std::string msg, toast_type::Type_t type, uint32_t durationMs)
{
    if (_toast_manager_id < 0) {
        _toast_manager_id = GetMooncake().extensionManager()->createAbility(std::make_unique<ToastManager>());
        if (_toast_manager_id < 0) {
            mclog::tagError(_tag, "create toast manager failed");
            return;
        }
        mclog::tagInfo(_tag, "create toast manager success");
    }

    if (_toast_request_queue.size() >= 5) {
        mclog::tagError(_tag, "toast request queue is full: {}", _toast_request_queue.size());
        return;
    }
    _toast_request_queue.push({type, durationMs, msg});
}
