/*
 * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include "view.h"
#include <lvgl.h>
#include <hal/hal.h>
#include <mooncake_log.h>
#include <assets/assets.h>
#include "japanese_font32.h"
#include "japanese_font64.h"
#include <smooth_ui_toolkit.h>
#include <smooth_lvgl.h>
#include <apps/utils/audio/audio.h>

using namespace launcher_view;
using namespace smooth_ui_toolkit;
using namespace smooth_ui_toolkit::lvgl_cpp;

static const std::string _tag = "launcher-view";

void LauncherView::init()
{
    mclog::tagInfo(_tag, "init");

    ui::signal_window_opened().clear();
    ui::signal_window_opened().connect([&](bool opened) { _is_stacked = opened; });

    LvglLockGuard lock;

    // Base screen
    lv_obj_remove_flag(lv_screen_active(), LV_OBJ_FLAG_SCROLLABLE);

    // Background image
    _img_bg = std::make_unique<Image>(lv_screen_active());
    _img_bg->setAlign(LV_ALIGN_CENTER);
    _img_bg->setSrc(&launcher_bg);

    // Install panels
    _panels.push_back(std::make_unique<PanelRtc>());
    _panels.push_back(std::make_unique<PanelLcdBacklight>());
    _panels.push_back(std::make_unique<PanelSpeakerVolume>());
    _panels.push_back(std::make_unique<PanelPowerMonitor>());
    _panels.push_back(std::make_unique<PanelImu>());
    _panels.push_back(std::make_unique<PanelSwitches>());
    _panels.push_back(std::make_unique<PanelPower>());
    _panels.push_back(std::make_unique<PanelCamera>());
    _panels.push_back(std::make_unique<PanelDualMic>());
    _panels.push_back(std::make_unique<PanelHeadphone>());
    _panels.push_back(std::make_unique<PanelSdCard>());
    _panels.push_back(std::make_unique<PanelI2cScan>());
    _panels.push_back(std::make_unique<PanelGpioTest>());
    _panels.push_back(std::make_unique<PanelMusic>());
    _panels.push_back(std::make_unique<PanelComMonitor>());

    for (auto& panel : _panels) {
        panel->init();
    }

    // 日本語テキストボックスの作成
    if (!_jp_textbox) {
        // 画面サイズ: 1280x720 (横向き)
        int left_margin = 20;
        int right_margin = 20;
        int top_margin = 20;
        int bottom_margin = 720 / 4; // テキストボックスの下マージン
        int button_width = 146; // OFF/SLEEPボタンの幅
        int textbox_width = 1280 - left_margin - right_margin - button_width - 20; // ボタンとの間隔20px
        int textbox_height = 720 - top_margin - bottom_margin;
        int textbox_x = -(1280/2) + left_margin + textbox_width/2;
        int textbox_y = -(720/2) + top_margin + textbox_height/2;

        _jp_textbox = lv_textarea_create(lv_screen_active());
        lv_obj_set_size(_jp_textbox, textbox_width, textbox_height);
        lv_obj_align(_jp_textbox, LV_ALIGN_CENTER, textbox_x, textbox_y);
        lv_textarea_set_text(_jp_textbox, "あいうえお");
        lv_obj_set_style_text_font(_jp_textbox, get_japanese_font64(), 0);
        lv_obj_set_style_text_align(_jp_textbox, LV_TEXT_ALIGN_LEFT, 0);
        lv_obj_set_style_text_line_space(_jp_textbox, 64, 0);
        lv_obj_set_style_pad_all(_jp_textbox, 20, 0);
        lv_obj_set_style_bg_color(_jp_textbox, lv_color_hex(0x2A2A2A), 0);
        lv_obj_set_style_bg_opa(_jp_textbox, LV_OPA_80, 0);
        lv_obj_set_style_border_width(_jp_textbox, 2, 0);
        lv_obj_set_style_border_color(_jp_textbox, lv_color_hex(0x666666), 0);
        lv_obj_set_style_radius(_jp_textbox, 10, 0);
    }

    // ボタンをテキストボックス外・画面最下部に配置
    // テキストボックスの横幅に合わせて均等配置
    const char* button_texts[] = {"あ", "い", "う", "え", "お"};
    int left_margin = 0;
    int right_margin = 20;
    int button_width = 146;
    int textbox_width = 1280 - 20 - right_margin - button_width - 20;
    int button_bottom_margin = 20; // ボタン下のマージン
    int button_spacing = 10;
    int available_width = textbox_width - (button_spacing * 4); // 5ボタン4間隔
    int jp_button_width = available_width / 5;
    int jp_button_height = 140;
    int width_carry = available_width - (jp_button_width * 5); // 端数分

    lv_obj_t* btn_container = lv_obj_create(lv_screen_active());
    lv_obj_set_size(btn_container, textbox_width, jp_button_height);
    // テキストボックスと同じ左端・画面最下部＋下マージンに配置
    // テキストボックスの左端座標を計算
    int btn_container_x = left_margin;
    lv_obj_align(btn_container, LV_ALIGN_BOTTOM_LEFT, btn_container_x, -button_bottom_margin);
    lv_obj_set_style_bg_opa(btn_container, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(btn_container, 0, 0);
    lv_obj_set_scrollbar_mode(btn_container, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_flex_flow(btn_container, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(btn_container, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    for (int i = 0; i < 5; i++) {
        _jp_buttons[i] = lv_btn_create(btn_container);
        int btn_width = jp_button_width;
        if (i == 4) btn_width += width_carry; // 端数は最後のボタンに加算
        lv_obj_set_size(_jp_buttons[i], btn_width, jp_button_height);
        lv_obj_set_style_margin_left(_jp_buttons[i], (i == 0) ? 0 : button_spacing, 0);
        lv_obj_set_style_margin_right(_jp_buttons[i], 0, 0);

        // ボタンのラベルを作成
        lv_obj_t* label = lv_label_create(_jp_buttons[i]);
        lv_label_set_text(label, button_texts[i]);
        lv_obj_set_style_text_font(label, get_japanese_font32(), 0);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
        // 上下中央揃えを確実にする
        lv_obj_set_style_pad_ver(label, (jp_button_height - 32) / 2, 0); // 32はフォント高さ目安
        lv_obj_center(label);

        // ボタンのスタイル設定
        lv_obj_set_style_bg_color(_jp_buttons[i], lv_color_hex(0x4A90E2), 0);
        lv_obj_set_style_bg_opa(_jp_buttons[i], LV_OPA_100, 0);
        lv_obj_set_style_radius(_jp_buttons[i], 8, 0);
        lv_obj_set_style_border_width(_jp_buttons[i], 2, 0);
        lv_obj_set_style_border_color(_jp_buttons[i], lv_color_hex(0x2E5C8A), 0);

        // ボタンのクリックイベントを設定
        lv_obj_add_event_cb(_jp_buttons[i], [](lv_event_t* e) {
            lv_obj_t* btn = static_cast<lv_obj_t*>(lv_event_get_target(e));
            lv_obj_t* label = lv_obj_get_child(btn, 0);
            const char* text = lv_label_get_text(label);

            // テキストボックスの内容を変更
            LauncherView* view = static_cast<LauncherView*>(lv_event_get_user_data(e));
            if (view && view->_jp_textbox) {
                lv_textarea_set_text(view->_jp_textbox, text);
            }
        }, LV_EVENT_CLICKED, this);
    }
}

void LauncherView::update()
{
    LvglLockGuard lock;

    for (auto& panel : _panels) {
        panel->update(_is_stacked);
    }
}
