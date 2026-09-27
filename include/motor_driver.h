#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "config.h"

// 車輪ごとの状態構造体
struct WheelStatus {
    float target_v;      // 正規化目標速度 (-1.0 〜 1.0)
    uint16_t pwm_val;    // 適用されたPWM値 (0 〜 3071)
    int8_t direction;    // 1: 正転, -1: 逆転, 0: 停止
};

// 全輪のステータス構造体
struct RoverMotorState {
    WheelStatus fl;      // 左前
    WheelStatus fr;      // 右前
    WheelStatus rl;      // 左後
    WheelStatus rr;      // 右後
    float current_vx;    // 指令Vx
    float current_vy;    // 指令Vy
    float current_omega; // 指令Omega
    bool initialized;    // PCA9685初期化成功フラグ
};

// 初期化
bool initMotorDriver(TwoWire &wireInstance = Wire);

// メカナム運動学に基づく4輪速度指令
void setMotorVelocities(float vx, float vy, float omega);

// 全モーター即時停止
void stopMotors();

// 現在のモーター状態の取得（デバッグ描画用）
const RoverMotorState& getMotorState();
