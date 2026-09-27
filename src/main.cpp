#include <Arduino.h>
#include <M5Unified.h>
#include <Wire.h>

#include "config.h"
#include "motor_driver.h"
#include "joystick_controller.h"
#include "ble_controller.h"
#include "display_ui.h"

// 非常停止 (E-STOP) フラグ
static bool g_estopActive = false;
static uint32_t g_lastDisplayUpdateMs = 0;
static uint32_t g_btnBPressedMs = 0;
static bool g_btnBLongPressedHandled = false;

void setup() {
    auto cfg = M5.config();
    M5.begin(cfg);

    Serial.begin(115200);
    delay(100);
    Serial.println("\n=================================");
    Serial.println("  M5StickC Plus SE Mecanum Rover ");
    Serial.println("     Dual Joystick Supported     ");
    Serial.println("=================================");

    // 1. UIの初期化
    initDisplayUI();

    // 2. I2Cバス初期化
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, (uint32_t)I2C_FREQ_HZ);
    delay(50);

    // 3. モータードライバー初期化
    if (initMotorDriver(Wire)) {
        Serial.println("[OK] PCA9685 Initialized (0x40)");
    } else {
        Serial.println("[ERR] PCA9685 Not Found!");
    }

    // 4. ジョイスティック初期化
    if (initJoystick(Wire)) {
        const auto &dj = getJoystickData();
        Serial.printf("[OK] Joysticks Detected: Move(0x63):%s, Turn(0x64):%s\n",
            dj.move_joy.connected ? "YES" : "NO",
            dj.turn_joy.connected ? "YES" : "NO");
    } else {
        Serial.println("[INFO] No Joystick detected at startup (Will scan in loop)");
    }

    // 5. BLE UART初期化
    initBleController();
    Serial.println("[OK] BLE UART Initialized (M5-Mecanum-Rover)");

    Serial.println("Ready.");
}

void loop() {
    M5.update();

    // -------------------------------------------------------------
    // ボタン操作
    // -------------------------------------------------------------
    // ボタンA (正面): 非常停止 (E-STOP) トグル
    if (M5.BtnA.wasPressed()) {
        g_estopActive = !g_estopActive;
        if (g_estopActive) {
            stopMotors();
            Serial.println("[SAFETY] Emergency Stop ACTIVATED!");
        } else {
            Serial.println("[SAFETY] Emergency Stop RELEASED.");
        }
    }

    // ボタンB (側面):
    // ・短押し: ジョイスティックゼロ点キャリブレーション
    // ・2秒長押し: 接続中の Joystick2 を 0x63 -> 0x64 にアドレス変更！
    if (M5.BtnB.isPressed()) {
        if (g_btnBPressedMs == 0) {
            g_btnBPressedMs = millis();
            g_btnBLongPressedHandled = false;
        } else if (!g_btnBLongPressedHandled && (millis() - g_btnBPressedMs >= 2000)) {
            // 2秒長押し検知: アドレス変更を実行
            g_btnBLongPressedHandled = true;
            stopMotors();
            showAddressChangeMessage("CHANGING ADDR", "Writing 0xFF -> 0x64...", TFT_YELLOW);
            delay(500);

            // 0x63 を 0x64 に書き換え
            if (changeJoystick2Address(JOYSTICK_MOVE_ADDR, JOYSTICK_TURN_ADDR, Wire)) {
                showAddressChangeMessage("SUCCESS!", "Addr changed to 0x64", TFT_GREEN);
                Serial.println("[JOY] Address changed to 0x64 successfully!");
            } else {
                showAddressChangeMessage("FAILED!", "Check 0x63 wiring", TFT_RED);
                Serial.println("[JOY] Failed to change address!");
            }
            delay(2000);
            initJoystick(Wire); // 新アドレスで再スキャン
        }
    } else {
        if (g_btnBPressedMs > 0) {
            if (!g_btnBLongPressedHandled) {
                // 短押し: キャリブレーション
                Serial.println("[JOY] Calibrating center position...");
                calibrateJoystickCenter(Wire);
            }
            g_btnBPressedMs = 0;
            g_btnBLongPressedHandled = false;
        }
    }

    // -------------------------------------------------------------
    // コントロール入力の判定とモーター指令
    // -------------------------------------------------------------
    ControlSource activeSource = SOURCE_NONE;
    const BleStatus &ble = getBleStatus();

    if (g_estopActive) {
        stopMotors();
        activeSource = SOURCE_ESTOP;
    } else if (ble.connected) {
        float bleVx = 0.0f, bleVy = 0.0f, bleOmega = 0.0f;
        updateBleController(bleVx, bleVy, bleOmega);

        if (ble.failsafe_triggered) {
            stopMotors();
            activeSource = SOURCE_FAILSAFE;
        } else {
            setMotorVelocities(bleVx, bleVy, bleOmega);
            activeSource = SOURCE_BLE;
        }
    } else {
        // ジョイスティック入力処理 (0x63, 0x64)
        if (updateJoystick(Wire)) {
            const DualJoystickData &dj = getJoystickData();
            setMotorVelocities(dj.vx, dj.vy, dj.omega);
            activeSource = SOURCE_JOYSTICK;
        } else {
            stopMotors();
            activeSource = SOURCE_NONE;
        }
    }

    // -------------------------------------------------------------
    // 画面のリアルタイム更新 (約30fps)
    // -------------------------------------------------------------
    uint32_t now = millis();
    if (now - g_lastDisplayUpdateMs >= 33) {
        g_lastDisplayUpdateMs = now;
        updateDisplayUI(
            activeSource,
            getMotorState(),
            getJoystickData(),
            getBleStatus(),
            g_estopActive
        );
    }

    delay(2);
}