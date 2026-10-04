#pragma once

#include <Arduino.h>

// ==========================================
// ハードウェアピン設定
// ==========================================
// M5StickC Plus SE の Grove ポート
#define I2C_SDA_PIN               32
#define I2C_SCL_PIN               33
#define I2C_FREQ_HZ               400000

// ==========================================
// I2C スレーブデバイスアドレス
// ==========================================
#define PCA9685_I2C_ADDR          0x40
#define PCA9685_PWM_FREQ          1000  // 1000 Hz

// ツインスティック用アドレス定義
#define JOYSTICK_MOVE_ADDR        0x63  // 移動用スティック (Vx, Vy) デフォルト
#define JOYSTICK_TURN_ADDR        0x64  // 旋回用スティック (Omega) 変更後
#define JOYSTICK_LEGACY_ADDR      0x52  // 初代 Unit Joystick 用フォールバック

// ==========================================
// PCA9685 出力チャンネルアサイン (L298N x 2台)
// ==========================================
// FL: 左前 (Front-Left)
#define CH_FL_IN1                 0
#define CH_FL_IN2                 1

// FR: 右前 (Front-Right)
#define CH_FR_IN3                 2
#define CH_FR_IN4                 3

// RL: 左後 (Rear-Left)
#define CH_RL_IN1                 4
#define CH_RL_IN2                 5

// RR: 右後 (Rear-Right)
#define CH_RR_IN3                 6
#define CH_RR_IN4                 7

// ==========================================
// モーター制御パラメータ
// ==========================================
// 12bit PWM 最大値 (0〜4095)
#define PWM_MAX_RESOLUTION        4095
// PWMデューティ制限 (MAX_PWM = 2252)
#define PWM_DUTY_MAX_LIMIT        2252

// ==========================================
// ジョイスティック制御パラメータ
// ==========================================
#define SWAP_JOYSTICK_XY          false // false: X軸(左右), Y軸(前後)
#define INVERT_JOYSTICK_X         false // スティック右倒しで右移動 / 右旋回 (正)
#define INVERT_JOYSTICK_Y         true  // スティック前倒しで前進 (正)

// デッドゾーン (不感帯): ±0.15
#define JOYSTICK_DEADZONE         0.15f

// 指数移動平均フィルタ係数 (0.0 < alpha <= 1.0)
#define JOYSTICK_FILTER_ALPHA     0.35f

// ==========================================
// フェールセーフ設定
// ==========================================
#define BLE_PACKET_TIMEOUT_MS     500   // 500ms通信途絶で自動停止

// ==========================================
// BLE 設定 (Nordic UART Service: NUS)
// ==========================================
#define BLE_DEVICE_NAME           "M5-Mecanum-Rover"
#define BLE_SERVICE_UUID          "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_CHAR_RX_UUID          "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_CHAR_TX_UUID          "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
