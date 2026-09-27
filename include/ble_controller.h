#pragma once

#include <Arduino.h>
#include "config.h"

struct BleStatus {
    bool connected;
    float vx;
    float vy;
    float omega;
    uint32_t last_packet_ms;
    uint32_t rx_packet_count;
    char last_raw_msg[32];
    bool failsafe_triggered;
};

// BLE UARTの初期化とアドバタイズ開始
void initBleController();

// BLEからの制御入力の更新・フェールセーフチェック
// 戻り値: 有効なBLE制御コマンドがアクティブな場合は true
bool updateBleController(float &out_vx, float &out_vy, float &out_omega);

// BLEクライアントへメッセージ送信（TX Notify）
void sendBleMessage(const char *msg);

// BLE状態の取得
const BleStatus& getBleStatus();
