#include "joystick_controller.h"
#include <math.h>

static DualJoystickData g_dualJoy = {};

static float g_filter_move_x = 0.0f;
static float g_filter_move_y = 0.0f;
static float g_filter_turn_x = 0.0f;
static float g_filter_turn_y = 0.0f;

// 滑らかなデッドゾーン処理
static float applySmoothDeadzone(float val, float deadzone) {
    if (fabsf(val) <= deadzone) {
        return 0.0f;
    }
    if (val > 0.0f) {
        return (val - deadzone) / (1.0f - deadzone);
    } else {
        return (val + deadzone) / (1.0f - deadzone);
    }
}

// I2C生データの読み取り
static bool readRawI2C(TwoWire &wire, uint8_t addr, uint8_t &rx, uint8_t &ry, bool &btn) {
    if (addr == JOYSTICK_LEGACY_ADDR) { // 0x52 (初代 Unit Joystick)
        while (wire.available()) wire.read();
        if (wire.requestFrom((uint8_t)JOYSTICK_LEGACY_ADDR, (uint8_t)3) < 3) return false;
        rx = wire.read();
        ry = wire.read();
        uint8_t b = wire.read();
        btn = (b == 0);
        return true;
    } else { // 0x63, 0x64 等 (Joystick2: STM32)
        // 8bit ADC 値 (レジスタ 0x10)
        wire.beginTransmission(addr);
        wire.write(0x10);
        if (wire.endTransmission(false) != 0) return false;
        if (wire.requestFrom(addr, (uint8_t)2) < 2) return false;
        rx = wire.read();
        ry = wire.read();

        // ボタン状態 (レジスタ 0x20)
        wire.beginTransmission(addr);
        wire.write(0x20);
        if (wire.endTransmission(false) != 0) return false;
        if (wire.requestFrom(addr, (uint8_t)1) < 1) return false;
        uint8_t b = wire.read();
        btn = (b == 0);
        return true;
    }
}

void setJoystick2RGB(uint8_t addr, uint32_t rgb_color, TwoWire &wireInstance) {
    wireInstance.beginTransmission(addr);
    wireInstance.write(0x30); // JOYSTICK2_RGB_REG
    wireInstance.write((uint8_t)(rgb_color & 0xFF));
    wireInstance.write((uint8_t)((rgb_color >> 8) & 0xFF));
    wireInstance.write((uint8_t)((rgb_color >> 16) & 0xFF));
    wireInstance.write((uint8_t)0);
    wireInstance.endTransmission();
}

bool changeJoystick2Address(uint8_t old_addr, uint8_t new_addr, TwoWire &wireInstance) {
    Serial.printf("[JOY] Attempting to change address from 0x%02X to 0x%02X...\n", old_addr, new_addr);

    // 存在確認
    wireInstance.beginTransmission(old_addr);
    if (wireInstance.endTransmission() != 0) {
        Serial.printf("[JOY] Device not found at 0x%02X!\n", old_addr);
        return false;
    }

    // レジスタ 0xFF に新アドレスを書き込み
    wireInstance.beginTransmission(old_addr);
    wireInstance.write(0xFF);
    wireInstance.write(new_addr);
    if (wireInstance.endTransmission() != 0) {
        Serial.println("[JOY] Failed to send address change command!");
        return false;
    }

    delay(100);

    // 新アドレスでの疎通確認
    wireInstance.beginTransmission(new_addr);
    if (wireInstance.endTransmission() == 0) {
        Serial.printf("[JOY] SUCCESS! New address is 0x%02X\n", new_addr);
        setJoystick2RGB(new_addr, 0x00FF00, wireInstance); // 成功の緑点灯
        return true;
    }

    Serial.println("[JOY] Device did not respond at new address.");
    return false;
}

static void calibrateSingleJoy(SingleJoyState &joy, TwoWire &wireInstance) {
    if (!joy.connected) return;

    uint32_t sum_x = 0;
    uint32_t sum_y = 0;
    const int SAMPLES = 16;
    int valid_count = 0;

    for (int i = 0; i < SAMPLES; i++) {
        uint8_t rx, ry;
        bool btn;
        if (readRawI2C(wireInstance, joy.i2c_addr, rx, ry, btn)) {
            if (SWAP_JOYSTICK_XY) {
                uint8_t temp = rx; rx = ry; ry = temp;
            }
            sum_x += rx;
            sum_y += ry;
            valid_count++;
        }
        delay(5);
    }

    if (valid_count > 0) {
        joy.center_x = (uint8_t)(sum_x / valid_count);
        joy.center_y = (uint8_t)(sum_y / valid_count);
        Serial.printf("[JOY 0x%02X] Center: X=%u, Y=%u\n", joy.i2c_addr, joy.center_x, joy.center_y);
    }
}

void calibrateJoystickCenter(TwoWire &wireInstance) {
    calibrateSingleJoy(g_dualJoy.move_joy, wireInstance);
    calibrateSingleJoy(g_dualJoy.turn_joy, wireInstance);
    g_filter_move_x = 0.0f;
    g_filter_move_y = 0.0f;
    g_filter_turn_x = 0.0f;
    g_filter_turn_y = 0.0f;
}

static bool checkDevice(TwoWire &wire, uint8_t addr) {
    wire.beginTransmission(addr);
    return (wire.endTransmission() == 0);
}

bool initJoystick(TwoWire &wireInstance) {
    g_dualJoy.move_joy.connected = false;
    g_dualJoy.turn_joy.connected = false;

    // 1. 移動用スティック (0x63 または 0x52)
    if (checkDevice(wireInstance, JOYSTICK_MOVE_ADDR)) {
        g_dualJoy.move_joy.i2c_addr = JOYSTICK_MOVE_ADDR;
        g_dualJoy.move_joy.connected = true;
        setJoystick2RGB(JOYSTICK_MOVE_ADDR, 0x0000FF, wireInstance); // 移動用: 青色LED
    } else if (checkDevice(wireInstance, JOYSTICK_LEGACY_ADDR)) {
        g_dualJoy.move_joy.i2c_addr = JOYSTICK_LEGACY_ADDR;
        g_dualJoy.move_joy.connected = true;
    }

    // 2. 旋回用スティック (0x64)
    if (checkDevice(wireInstance, JOYSTICK_TURN_ADDR)) {
        g_dualJoy.turn_joy.i2c_addr = JOYSTICK_TURN_ADDR;
        g_dualJoy.turn_joy.connected = true;
        setJoystick2RGB(JOYSTICK_TURN_ADDR, 0x00FF00, wireInstance); // 旋回用: 緑色LED
    }

    g_dualJoy.any_connected = g_dualJoy.move_joy.connected || g_dualJoy.turn_joy.connected;

    if (g_dualJoy.move_joy.connected) calibrateSingleJoy(g_dualJoy.move_joy, wireInstance);
    if (g_dualJoy.turn_joy.connected) calibrateSingleJoy(g_dualJoy.turn_joy, wireInstance);

    return g_dualJoy.any_connected;
}

static void processSingleJoy(SingleJoyState &joy, float &filt_x, float &filt_y, TwoWire &wireInstance) {
    if (!joy.connected) return;

    uint8_t rx, ry;
    bool btn;
    if (!readRawI2C(wireInstance, joy.i2c_addr, rx, ry, btn)) {
        joy.connected = false;
        joy.norm_x = 0.0f;
        joy.norm_y = 0.0f;
        return;
    }

    joy.raw_x = rx;
    joy.raw_y = ry;
    joy.button_pressed = btn;

    uint8_t ax = rx;
    uint8_t ay = ry;
    if (SWAP_JOYSTICK_XY) {
        ax = ry;
        ay = rx;
    }

    float nx = 0.0f;
    float ny = 0.0f;

    if (ax >= joy.center_x) {
        float span = 255.0f - (float)joy.center_x;
        nx = (span > 0) ? ((float)ax - (float)joy.center_x) / span : 0.0f;
    } else {
        float span = (float)joy.center_x;
        nx = (span > 0) ? ((float)ax - (float)joy.center_x) / span : 0.0f;
    }

    if (ay >= joy.center_y) {
        float span = 255.0f - (float)joy.center_y;
        ny = (span > 0) ? ((float)ay - (float)joy.center_y) / span : 0.0f;
    } else {
        float span = (float)joy.center_y;
        ny = (span > 0) ? ((float)ay - (float)joy.center_y) / span : 0.0f;
    }

    if (INVERT_JOYSTICK_X) nx = -nx;
    if (INVERT_JOYSTICK_Y) ny = -ny;

    nx = constrain(nx, -1.0f, 1.0f);
    ny = constrain(ny, -1.0f, 1.0f);

    filt_x = filt_x * (1.0f - JOYSTICK_FILTER_ALPHA) + nx * JOYSTICK_FILTER_ALPHA;
    filt_y = filt_y * (1.0f - JOYSTICK_FILTER_ALPHA) + ny * JOYSTICK_FILTER_ALPHA;

    joy.norm_x = applySmoothDeadzone(filt_x, JOYSTICK_DEADZONE);
    joy.norm_y = applySmoothDeadzone(filt_y, JOYSTICK_DEADZONE);
}

bool updateJoystick(TwoWire &wireInstance) {
    // どちらも未接続の場合は再検出
    if (!g_dualJoy.any_connected) {
        if (!initJoystick(wireInstance)) {
            g_dualJoy.vx = 0.0f;
            g_dualJoy.vy = 0.0f;
            g_dualJoy.omega = 0.0f;
            return false;
        }
    }

    if (g_dualJoy.move_joy.connected) {
        processSingleJoy(g_dualJoy.move_joy, g_filter_move_x, g_filter_move_y, wireInstance);
    }
    if (g_dualJoy.turn_joy.connected) {
        processSingleJoy(g_dualJoy.turn_joy, g_filter_turn_x, g_filter_turn_y, wireInstance);
    }

    g_dualJoy.any_connected = g_dualJoy.move_joy.connected || g_dualJoy.turn_joy.connected;
    if (!g_dualJoy.any_connected) {
        g_dualJoy.vx = 0.0f;
        g_dualJoy.vy = 0.0f;
        g_dualJoy.omega = 0.0f;
        return false;
    }

    // 指令値の合成
    if (g_dualJoy.move_joy.connected && g_dualJoy.turn_joy.connected) {
        // 【ツインスティック完全モード】
        // move_joy (0x63): 前後左右移動
        g_dualJoy.vx    = g_dualJoy.move_joy.norm_x;
        g_dualJoy.vy    = -g_dualJoy.move_joy.norm_y; // ① Y軸反転
        // turn_joy (0x64): 旋回 (スティック左右)
        g_dualJoy.omega = g_dualJoy.turn_joy.norm_x;
    } else {
        // 【ジョイスティック1基接続モード (0x63, 0x64, 0x52共通)】
        // Y軸: Vy（反転処理適用）
        // X軸: ボタン非押下時は左右平行移動(Vx)、ボタン押下時はその場旋回(Omega)
        SingleJoyState &j = g_dualJoy.turn_joy.connected ? g_dualJoy.turn_joy : g_dualJoy.move_joy;
        if (j.button_pressed) {
            g_dualJoy.vx    = 0.0f;
            g_dualJoy.vy    = -j.norm_y; // ① Y軸反転
            g_dualJoy.omega = j.norm_x;  // ボタン押下時: その場旋回
        } else {
            g_dualJoy.vx    = j.norm_x;  // ボタン非押下時: 左右平行移動
            g_dualJoy.vy    = -j.norm_y; // ① Y軸反転
            g_dualJoy.omega = 0.0f;
        }
    }

    return true;
}

const DualJoystickData& getJoystickData() {
    return g_dualJoy;
}
