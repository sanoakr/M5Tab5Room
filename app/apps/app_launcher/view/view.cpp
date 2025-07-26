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
        _jp_textbox = lv_textarea_create(lv_screen_active());
        
        // サイズ設定: 左側はOFF/SLEEPボタンに重ならないように、右と上部は画面のいっぱいまで、下部は画面の下1/4を開ける
        // 画面サイズ: 1280x720 (横向き)
        // OFF/SLEEPボタン位置: x=562 (右側)
        // マージン: 20px
        int left_margin = 20;
        int right_margin = 20;
        int top_margin = 20;
        int bottom_margin = 720 / 4; // 画面の下1/4
        int button_width = 146; // OFF/SLEEPボタンの幅
        
        int textbox_width = 1280 - left_margin - right_margin - button_width - 20; // ボタンとの間隔20px
        int textbox_height = 720 - top_margin - bottom_margin;
        
        // LVGLでは画面の中心が(0,0)なので、正しい位置を計算
        // 画面の左上を基準に配置
        int textbox_x = -(1280/2) + left_margin + textbox_width/2; // 画面中心からのX座標
        int textbox_y = -(720/2) + top_margin + textbox_height/2; // 画面中心からのY座標
        
        lv_obj_set_size(_jp_textbox, textbox_width, textbox_height);
        lv_obj_align(_jp_textbox, LV_ALIGN_CENTER, textbox_x, textbox_y);
        
        lv_textarea_set_text(_jp_textbox, "あいうえお");
        lv_obj_set_style_text_font(_jp_textbox, get_japanese_font64(), 0); // 64pxフォントを使用
        lv_obj_set_style_text_align(_jp_textbox, LV_TEXT_ALIGN_LEFT, 0);
        lv_obj_set_style_text_line_space(_jp_textbox, 64, 0); // 行間を64pxに
        lv_obj_set_style_pad_all(_jp_textbox, 20, 0); // 内側のパディング
        lv_obj_set_style_bg_color(_jp_textbox, lv_color_hex(0x2A2A2A), 0); // 背景色
        lv_obj_set_style_bg_opa(_jp_textbox, LV_OPA_80, 0); // 背景の透明度
        lv_obj_set_style_border_width(_jp_textbox, 2, 0); // ボーダー
        lv_obj_set_style_border_color(_jp_textbox, lv_color_hex(0x666666), 0); // ボーダー色
        lv_obj_set_style_radius(_jp_textbox, 10, 0); // 角丸
        
        // テキストボックス内に5つのボタンを作成（あいうえお）
        const char* button_texts[] = {"あ", "い", "う", "え", "お"};
        int button_margin = 10; // ボタンとテキストボックス端のマージン
        int button_spacing = 10; // ボタン間の間隔
        int available_width = textbox_width - (button_margin * 2) - (button_spacing * 4); // 利用可能な幅
        int jp_button_width = available_width / 5; // 5つのボタンで均等に分割
        int jp_button_height = 80; // ボタンの高さ
        
        for (int i = 0; i < 5; i++) {
            _jp_buttons[i] = lv_btn_create(_jp_textbox); // テキストボックス内に作成
            lv_obj_set_size(_jp_buttons[i], jp_button_width, jp_button_height);
            
            // テキストボックス内の位置を計算（左上を基準）
            int button_x = button_margin + i * (jp_button_width + button_spacing) + jp_button_width/2 - textbox_width/2;
            int button_y = textbox_height/2 - jp_button_height/2 - 20; // テキストボックス内の上部に配置
            
            lv_obj_align(_jp_buttons[i], LV_ALIGN_CENTER, button_x, button_y);
            
            // ボタンのラベルを作成
            lv_obj_t* label = lv_label_create(_jp_buttons[i]);
            lv_label_set_text(label, button_texts[i]);
            lv_obj_set_style_text_font(label, get_japanese_font64(), 0); // 64pxフォントを使用
            lv_obj_center(label); // センタリング
            
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
}

void LauncherView::update()
{
    LvglLockGuard lock;

    for (auto& panel : _panels) {
        panel->update(_is_stacked);
    }
}
