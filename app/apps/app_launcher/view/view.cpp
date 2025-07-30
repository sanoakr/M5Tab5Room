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
#include "japanese_font128.h"
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
        int tb_left_margin = 20;
        int tb_right_margin = 20;
        int tb_top_margin = 20;
        int tb_bottom_margin = 720 / 4; // テキストボックスの下マージン
        int tb_button_width = 146; // OFF/SLEEPボタンの幅
        int tb_width = 1280 - tb_left_margin - tb_right_margin - tb_button_width - 20; // ボタンとの間隔20px
        int tb_height = 720 - tb_top_margin - tb_bottom_margin;
        int tb_x = -(1280/2) + tb_left_margin + tb_width/2;
        int tb_y = -(720/2) + tb_top_margin + tb_height/2;

        _jp_textbox = lv_obj_create(lv_screen_active());
        lv_obj_set_size(_jp_textbox, tb_width, tb_height);
        lv_obj_align(_jp_textbox, LV_ALIGN_CENTER, tb_x, tb_y);
        lv_obj_set_style_bg_color(_jp_textbox, lv_color_hex(0x2A2A2A), 0);
        lv_obj_set_style_bg_opa(_jp_textbox, LV_OPA_90, 0);
        lv_obj_set_style_border_width(_jp_textbox, 2, 0);
        lv_obj_set_style_border_color(_jp_textbox, lv_color_hex(0x666666), 0);
        lv_obj_set_style_radius(_jp_textbox, 10, 0);
        lv_obj_set_style_pad_all(_jp_textbox, 20, 0);
        lv_obj_set_style_pad_row(_jp_textbox, 16, 0);

        // top
        _label_top = lv_label_create(_jp_textbox);
        lv_label_set_text(_label_top, "さのは（おそらく）");
        lv_obj_set_style_text_font(_label_top, get_japanese_font64(), 0);
        lv_obj_set_style_text_align(_label_top, LV_TEXT_ALIGN_LEFT, 0);
        lv_obj_align(_label_top, LV_ALIGN_TOP_LEFT, 0, 0);

        // main
        _label_main = lv_label_create(_jp_textbox);
        lv_label_set_text(_label_main, "在室中です");
        lv_obj_set_style_text_font(_label_main, get_japanese_font128(), 0);
        lv_obj_set_style_text_align(_label_main, LV_TEXT_ALIGN_LEFT, 0);
        lv_obj_align_to(_label_main, _label_top, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 70);

        // sub1
        _label_sub1 = lv_label_create(_jp_textbox);
        lv_label_set_text(_label_sub1, "あいうえおあいうえおあいうえお");
        lv_obj_set_style_text_font(_label_sub1, get_japanese_font64(), 0);
        lv_obj_set_style_text_align(_label_sub1, LV_TEXT_ALIGN_LEFT, 0);
        lv_obj_align_to(_label_sub1, _label_main, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 70);
    // sub2
    //     _label_sub2 = lv_label_create(_jp_textbox);
    //     lv_label_set_text(_label_sub2, "あいうえおあいうえおあいうえお");
    //     lv_obj_set_style_text_font(_label_sub2, get_japanese_font64(), 0);
    //     lv_obj_set_style_text_align(_label_sub2, LV_TEXT_ALIGN_LEFT, 0);
    //     lv_obj_align_to(_label_sub2, _label_sub1, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 20);
    }

    // ボタンをテキストボックス外・画面最下部に配置
    // テキストボックスの横幅に合わせて均等配置
    const char* button_texts[] = {"在室", "不在", "学内", "ミーティング", "オンライン"};
    int btn_left_margin = 0;
    int btn_right_margin = 20;
    int btn_button_width = 146;
    int btn_container_width = 1280 - btn_left_margin - btn_right_margin - btn_button_width - 20;
    int btn_bottom_margin = 20; // ボタン下のマージン
    int btn_spacing = 10;
    int num_buttons = 5;
    int num_spaces = num_buttons - 1;
    int btn_height = 140;
    int btn_available_width = btn_container_width - (btn_spacing * num_spaces);
    int btn_width = btn_available_width / num_buttons - 15;
    //int btn_width_carry = btn_available_width - (btn_width * num_buttons); // 端数分

    lv_obj_t* btn_container = lv_obj_create(lv_screen_active());
    lv_obj_set_size(btn_container, btn_container_width, btn_height);
    // テキストボックスと同じ左端・画面最下部＋下マージンに配置
    // テキストボックスの左端座標を計算
    int btn_container_x = btn_left_margin;
    lv_obj_align(btn_container, LV_ALIGN_BOTTOM_LEFT, btn_container_x, -btn_bottom_margin);
    lv_obj_set_style_bg_opa(btn_container, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(btn_container, 0, 0);
    lv_obj_set_scrollbar_mode(btn_container, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_flex_flow(btn_container, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(btn_container, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    for (int i = 0; i < num_buttons; i++) {
        _jp_buttons[i] = lv_btn_create(btn_container);
        int width = btn_width;
        //if (i == num_buttons - 1) width += btn_width_carry; // 端数は最後のボタンに加算
        lv_obj_set_size(_jp_buttons[i], width, btn_height);
        lv_obj_set_style_margin_left(_jp_buttons[i], (i == 0) ? 0 : btn_spacing, 0);
        lv_obj_set_style_margin_right(_jp_buttons[i], 0, 0);

        // ボタンのラベルを作成
        lv_obj_t* label = lv_label_create(_jp_buttons[i]);
        lv_label_set_text(label, button_texts[i]);
        lv_obj_set_style_text_font(label, get_japanese_font32(), 0);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
        // 32pxフォントに対応した中央配置
        lv_obj_set_size(label, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
        lv_obj_center(label);

        // ボタンのスタイル設定
        lv_obj_set_style_bg_color(_jp_buttons[i], lv_color_hex(0x4A90E2), 0);
        lv_obj_set_style_bg_opa(_jp_buttons[i], LV_OPA_90, 0);
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
            if (view && view->_label_main) {
                lv_label_set_text(view->_label_main, text);
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
