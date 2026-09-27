#include "display_ui.h"

static M5Canvas g_canvas(&M5.Display);

void initDisplayUI() {
    M5.Display.setRotation(0); // 縦向き (ボタンAが下: 135x240)
    M5.Display.setBrightness(128);
    g_canvas.createSprite(135, 240);
    g_canvas.setTextSize(1);
}

void showAddressChangeMessage(const char *title, const char *status, uint16_t color) {
    g_canvas.fillSprite(TFT_BLACK);
    g_canvas.fillRect(0, 0, 135, 30, color);
    g_canvas.setTextColor(TFT_WHITE, color);
    g_canvas.drawCenterString(title, 67, 8, &fonts::Font0);

    g_canvas.drawRoundRect(4, 40, 127, 180, 4, color);
    g_canvas.setTextColor(TFT_WHITE, TFT_BLACK);
    g_canvas.drawCenterString("I2C ADDR CHANGER", 67, 60, &fonts::Font0);

    g_canvas.setTextColor(color, TFT_BLACK);
    g_canvas.drawCenterString(status, 67, 110, &fonts::Font0);

    g_canvas.pushSprite(0, 0);
}

void updateDisplayUI(
    ControlSource source,
    const RoverMotorState &motorState,
    const DualJoystickData &dualJoy,
    const BleStatus &bleStatus,
    bool estopActive
) {
    g_canvas.fillSprite(TFT_BLACK);

    // ==========================================
    // 1. ヘッダーバー (Y: 0〜21, 幅: 135)
    // ==========================================
    uint16_t headerBg = 0x18E3;
    if (estopActive) {
        headerBg = TFT_RED;
    } else if (source == SOURCE_FAILSAFE) {
        headerBg = 0xD800;
    } else if (source == SOURCE_BLE) {
        headerBg = 0x0015;
    } else if (source == SOURCE_JOYSTICK) {
        headerBg = 0x03E0;
    }

    g_canvas.fillRect(0, 0, 135, 22, headerBg);

    // タイトルとバッテリー
    g_canvas.setTextColor(TFT_WHITE, headerBg);
    g_canvas.drawString("ROVER", 4, 2, &fonts::Font0);

    int batLevel = M5.Power.getBatteryLevel();
    int batMv = M5.Power.getBatteryVoltage();
    char batStr[16];
    snprintf(batStr, sizeof(batStr), "%d%% %0.1fV", batLevel, (float)batMv / 1000.0f);
    g_canvas.drawRightString(batStr, 132, 2, &fonts::Font0);

    // 動作モードバッジ
    const char *modeTag = "[ IDLE ]";
    if (estopActive) modeTag = "[ E-STOP ]";
    else if (source == SOURCE_FAILSAFE) modeTag = "[ FAILSAFE ]";
    else if (source == SOURCE_BLE) modeTag = "[ BLE-NUS ]";
    else if (dualJoy.move_joy.connected && dualJoy.turn_joy.connected) modeTag = "[ DUAL-JOY ]";
    else if (dualJoy.turn_joy.connected) modeTag = "[ TURN-JOY ]";
    else if (dualJoy.move_joy.connected) modeTag = "[ MOVE-JOY ]";
    g_canvas.drawCenterString(modeTag, 67, 12, &fonts::Font0);

    // ==========================================
    // 2. 入力ベクトルカード (Y: 25〜103, X: 2〜133)
    // ==========================================
    g_canvas.drawRoundRect(2, 25, 131, 79, 3, 0x39E7);
    g_canvas.setTextColor(TFT_CYAN, TFT_BLACK);
    g_canvas.drawString("INPUT VECTOR", 6, 28, &fonts::Font0);

    char valStr[32];
    g_canvas.setTextColor(TFT_WHITE, TFT_BLACK);
    snprintf(valStr, sizeof(valStr), "Vx: %+0.2f", motorState.current_vx);
    g_canvas.drawString(valStr, 6, 40, &fonts::Font0);

    snprintf(valStr, sizeof(valStr), "Vy: %+0.2f", motorState.current_vy);
    g_canvas.drawString(valStr, 6, 51, &fonts::Font0);

    snprintf(valStr, sizeof(valStr), "W : %+0.2f", motorState.current_omega);
    g_canvas.drawString(valStr, 6, 62, &fonts::Font0);

    // デバッグ情報（接続中のスティック生値）
    g_canvas.setTextColor(0x9CD3, TFT_BLACK);
    if (dualJoy.turn_joy.connected) {
        snprintf(valStr, sizeof(valStr), "T(64):%3u,%3u", dualJoy.turn_joy.raw_x, dualJoy.turn_joy.raw_y);
        g_canvas.drawString(valStr, 6, 75, &fonts::Font0);
    } else if (dualJoy.move_joy.connected) {
        snprintf(valStr, sizeof(valStr), "M(63):%3u,%3u", dualJoy.move_joy.raw_x, dualJoy.move_joy.raw_y);
        g_canvas.drawString(valStr, 6, 75, &fonts::Font0);
    } else {
        g_canvas.drawString("NO JOYSTICK", 6, 75, &fonts::Font0);
    }

    if (dualJoy.move_joy.connected && dualJoy.turn_joy.connected) {
        snprintf(valStr, sizeof(valStr), "M(63):%3u,%3u", dualJoy.move_joy.raw_x, dualJoy.move_joy.raw_y);
        g_canvas.drawString(valStr, 6, 87, &fonts::Font0);
    } else {
        snprintf(valStr, sizeof(valStr), "HOLD B:0x63->64");
        g_canvas.drawString(valStr, 6, 87, &fonts::Font0);
    }

    // 入力ベクトルのミニ2Dクロスヘア描画 (中心 x: 104, cy: 68, 半径: 18)
    int cx = 104;
    int cy = 68;
    int r = 18;
    g_canvas.drawCircle(cx, cy, r, 0x5AEB);
    g_canvas.drawFastHLine(cx - r, cy, r * 2, 0x39E7);
    g_canvas.drawFastVLine(cx, cy - r, r * 2, 0x39E7);

    int px = cx + (int)(motorState.current_vx * (float)(r - 2));
    int py = cy - (int)(motorState.current_vy * (float)(r - 2));
    g_canvas.fillCircle(px, py, 3, TFT_YELLOW);

    // 旋回インジケータ
    if (fabsf(motorState.current_omega) > 0.05f) {
        g_canvas.drawCircle(cx, cy, r + 2, motorState.current_omega > 0 ? TFT_MAGENTA : TFT_GREEN);
    }

    // ==========================================
    // 3. 4輪モーターPWM出力カード (Y: 106〜196, X: 2〜133)
    // ==========================================
    g_canvas.drawRoundRect(2, 106, 131, 91, 3, 0x39E7);
    g_canvas.setTextColor(TFT_GOLD, TFT_BLACK);
    g_canvas.drawString("MOTORS (MAX 3071)", 6, 109, &fonts::Font0);

    auto drawWheelBox = [&](int x, int y, const char *name, const WheelStatus &ws) {
        uint16_t boxColor = 0x2104;
        uint16_t txtColor = TFT_WHITE;
        const char *dirStr = "--";

        if (ws.direction > 0) {
            dirStr = "FWD";
            boxColor = 0x0320;
            txtColor = TFT_GREEN;
        } else if (ws.direction < 0) {
            dirStr = "REV";
            boxColor = 0x4000;
            txtColor = TFT_RED;
        }

        g_canvas.fillRoundRect(x, y, 59, 32, 2, boxColor);
        g_canvas.drawRoundRect(x, y, 59, 32, 2, 0x52AA);
        
        g_canvas.setTextColor(TFT_WHITE, boxColor);
        g_canvas.drawString(name, x + 3, y + 3, &fonts::Font0);

        g_canvas.setTextColor(txtColor, boxColor);
        g_canvas.drawRightString(dirStr, x + 56, y + 3, &fonts::Font0);

        char pwmStr[16];
        snprintf(pwmStr, sizeof(pwmStr), "%4u", ws.pwm_val);
        g_canvas.drawString(pwmStr, x + 8, y + 17, &fonts::Font0);
    };

    drawWheelBox(6, 122, "FL", motorState.fl);
    drawWheelBox(69, 122, "FR", motorState.fr);
    drawWheelBox(6, 158, "RL", motorState.rl);
    drawWheelBox(69, 158, "RR", motorState.rr);

    // ==========================================
    // 4. 下部システムステータスカード (Y: 199〜238, X: 2〜133)
    // ==========================================
    g_canvas.drawRoundRect(2, 199, 131, 39, 3, 0x2965);

    // 1行目: デバイス状態
    if (motorState.initialized) {
        g_canvas.setTextColor(TFT_GREEN, TFT_BLACK);
        g_canvas.drawString("PCA:OK", 6, 203, &fonts::Font0);
    } else {
        g_canvas.setTextColor(TFT_RED, TFT_BLACK);
        g_canvas.drawString("PCA:NG", 6, 203, &fonts::Font0);
    }

    char jStr[32];
    snprintf(jStr, sizeof(jStr), "M:%s T:%s",
        dualJoy.move_joy.connected ? "OK" : "--",
        dualJoy.turn_joy.connected ? "OK" : "--");
    g_canvas.setTextColor(TFT_WHITE, TFT_BLACK);
    g_canvas.drawRightString(jStr, 130, 203, &fonts::Font0);

    // 2行目: BLE 状態
    if (bleStatus.connected) {
        if (bleStatus.failsafe_triggered) {
            g_canvas.setTextColor(TFT_RED, TFT_BLACK);
            g_canvas.drawString("BLE:TIMEOUT(FAILSAFE)", 6, 215, &fonts::Font0);
        } else {
            g_canvas.setTextColor(TFT_CYAN, TFT_BLACK);
            char bStr[28];
            uint32_t dt = millis() - bleStatus.last_packet_ms;
            snprintf(bStr, sizeof(bStr), "BLE:OK (%ums)", dt);
            g_canvas.drawString(bStr, 6, 215, &fonts::Font0);
        }
    } else {
        g_canvas.setTextColor(0x7BEF, TFT_BLACK);
        g_canvas.drawString("BLE:DISCONNECTED", 6, 215, &fonts::Font0);
    }

    // 3行目: 操作ガイド
    g_canvas.setTextColor(0x632C, TFT_BLACK);
    g_canvas.drawString("B短押:CAL | B長押:63->64", 6, 227, &fonts::Font0);

    g_canvas.pushSprite(0, 0);
}
