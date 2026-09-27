#pragma once

#include <Arduino.h>
#include <M5Unified.h>
#include "motor_driver.h"
#include "joystick_controller.h"
#include "ble_controller.h"

enum ControlSource {
    SOURCE_NONE,
    SOURCE_JOYSTICK,
    SOURCE_BLE,
    SOURCE_FAILSAFE,
    SOURCE_ESTOP
};

// UIの初期化
void initDisplayUI();

// 画面デバッグ描画（縦向き 135x240）
void updateDisplayUI(
    ControlSource source,
    const RoverMotorState &motorState,
    const DualJoystickData &dualJoy,
    const BleStatus &bleStatus,
    bool estopActive
);

// アドレス変更時の一時ポップアップ画面表示
void showAddressChangeMessage(const char *title, const char *status, uint16_t color);
