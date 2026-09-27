#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "config.h"

struct SingleJoyState {
    uint8_t raw_x;            // I2C生のX値
    uint8_t raw_y;            // I2C生のY値
    uint8_t center_x;         // キャリブレーション中心X
    uint8_t center_y;         // キャリブレーション中心Y
    bool button_pressed;      // ボタン押下状態
    float norm_x;             // フィルタ後X (-1.0〜1.0)
    float norm_y;             // フィルタ後Y (-1.0〜1.0)
    bool connected;           // 接続フラグ
    uint8_t i2c_addr;         // I2Cアドレス
};

struct DualJoystickData {
    SingleJoyState move_joy;  // 移動用ジョイスティック (0x63 または 0x52)
    SingleJoyState turn_joy;  // 旋回用ジョイスティック (0x64)
    float vx;                 // 最終合成 Vx
    float vy;                 // 最終合成 Vy
    float omega;              // 最終合成 Omega
    bool any_connected;       // 少なくとも1台接続されているか
};

// ジョイスティック初期化
bool initJoystick(TwoWire &wireInstance = Wire);

// ジョイスティック読み取り & 指令値への変換
bool updateJoystick(TwoWire &wireInstance = Wire);

// ジョイスティック中央値キャリブレーション（ゼロ点補正）
void calibrateJoystickCenter(TwoWire &wireInstance = Wire);

// Joystick2 (STM32) の I2Cアドレスを変更する
// 例: 0x63 から 0x64 へ変更
bool changeJoystick2Address(uint8_t old_addr, uint8_t new_addr, TwoWire &wireInstance = Wire);

// Joystick2 のフルカラーRGB LEDを設定する (24bit: 0xRRGGBB)
void setJoystick2RGB(uint8_t addr, uint32_t rgb_color, TwoWire &wireInstance = Wire);

// ジョイスティック状態の取得
const DualJoystickData& getJoystickData();
