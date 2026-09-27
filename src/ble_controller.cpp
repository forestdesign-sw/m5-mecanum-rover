#include "ble_controller.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <string.h>
#include <ctype.h>

static BLEServer *g_pServer = nullptr;
static BLECharacteristic *g_pTxCharacteristic = nullptr;
static BleStatus g_bleStatus = {
    .connected = false,
    .vx = 0.0f,
    .vy = 0.0f,
    .omega = 0.0f,
    .last_packet_ms = 0,
    .rx_packet_count = 0,
    .last_raw_msg = {0},
    .failsafe_triggered = false
};

// サーバー接続コールバック
class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) override {
        g_bleStatus.connected = true;
        g_bleStatus.last_packet_ms = millis();
        g_bleStatus.failsafe_triggered = false;
    }

    void onDisconnect(BLEServer* pServer) override {
        g_bleStatus.connected = false;
        g_bleStatus.failsafe_triggered = true;
        g_bleStatus.vx = 0.0f;
        g_bleStatus.vy = 0.0f;
        g_bleStatus.omega = 0.0f;
        // 切断後に再度アドバタイズを開始して再接続を可能にする
        pServer->startAdvertising();
    }
};

// コマンド文字列のトリム（前後の空白・改行削除）
static void trimWhitespace(char *str) {
    if (!str) return;
    char *p = str;
    while (*p && (*p == ' ' || *p == '\r' || *p == '\n' || *p == '\t')) p++;
    if (p != str) memmove(str, p, strlen(p) + 1);

    int len = strlen(str);
    while (len > 0 && (str[len - 1] == ' ' || str[len - 1] == '\r' || str[len - 1] == '\n' || str[len - 1] == '\t')) {
        str[len - 1] = '\0';
        len--;
    }
}

// データ受信コールバック
class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) override {
        std::string rxValue = pCharacteristic->getValue();
        if (rxValue.length() == 0) return;

        // バイナリプロトコル判定: 0xAA 0x55 <int8_t vx> <int8_t vy> <int8_t omega>
        if (rxValue.length() >= 5 && (uint8_t)rxValue[0] == 0xAA && (uint8_t)rxValue[1] == 0x55) {
            int8_t raw_vx = (int8_t)rxValue[2];
            int8_t raw_vy = (int8_t)rxValue[3];
            int8_t raw_w  = (int8_t)rxValue[4];

            g_bleStatus.vx = constrain((float)raw_vx / 100.0f, -1.0f, 1.0f);
            g_bleStatus.vy = constrain((float)raw_vy / 100.0f, -1.0f, 1.0f);
            g_bleStatus.omega = constrain((float)raw_w / 100.0f, -1.0f, 1.0f);

            snprintf(g_bleStatus.last_raw_msg, sizeof(g_bleStatus.last_raw_msg), "BIN:%d,%d,%d", raw_vx, raw_vy, raw_w);
            g_bleStatus.last_packet_ms = millis();
            g_bleStatus.rx_packet_count++;
            g_bleStatus.failsafe_triggered = false;
            return;
        }

        // テキストプロトコル処理
        char buffer[64];
        size_t len = (rxValue.length() < sizeof(buffer) - 1) ? rxValue.length() : sizeof(buffer) - 1;
        memcpy(buffer, rxValue.data(), len);
        buffer[len] = '\0';

        // ログ用コピー
        strncpy(g_bleStatus.last_raw_msg, buffer, sizeof(g_bleStatus.last_raw_msg) - 1);
        g_bleStatus.last_raw_msg[sizeof(g_bleStatus.last_raw_msg) - 1] = '\0';
        trimWhitespace(g_bleStatus.last_raw_msg);

        trimWhitespace(buffer);

        // 1. カンマ区切りフォーマット: Vx,Vy,Omega (例: "0.5,-0.2,0.0")
        float parsed_vx = 0.0f, parsed_vy = 0.0f, parsed_omega = 0.0f;
        int count = sscanf(buffer, "%f,%f,%f", &parsed_vx, &parsed_vy, &parsed_omega);
        if (count == 3) {
            g_bleStatus.vx = constrain(parsed_vx, -1.0f, 1.0f);
            g_bleStatus.vy = constrain(parsed_vy, -1.0f, 1.0f);
            g_bleStatus.omega = constrain(parsed_omega, -1.0f, 1.0f);
            g_bleStatus.last_packet_ms = millis();
            g_bleStatus.rx_packet_count++;
            g_bleStatus.failsafe_triggered = false;
            return;
        }

        // 2. ショートカットコマンド判定（スマホのターミナルアプリ等のボタン向け）
        if (strcasecmp(buffer, "F") == 0) {
            g_bleStatus.vx = 0.0f; g_bleStatus.vy = 0.7f; g_bleStatus.omega = 0.0f;
        } else if (strcasecmp(buffer, "B") == 0) {
            g_bleStatus.vx = 0.0f; g_bleStatus.vy = -0.7f; g_bleStatus.omega = 0.0f;
        } else if (strcasecmp(buffer, "L") == 0) {
            g_bleStatus.vx = -0.7f; g_bleStatus.vy = 0.0f; g_bleStatus.omega = 0.0f;
        } else if (strcasecmp(buffer, "R") == 0) {
            g_bleStatus.vx = 0.7f; g_bleStatus.vy = 0.0f; g_bleStatus.omega = 0.0f;
        } else if (strcasecmp(buffer, "CW") == 0 || strcasecmp(buffer, "TR") == 0) {
            g_bleStatus.vx = 0.0f; g_bleStatus.vy = 0.0f; g_bleStatus.omega = 0.7f;
        } else if (strcasecmp(buffer, "CCW") == 0 || strcasecmp(buffer, "TL") == 0) {
            g_bleStatus.vx = 0.0f; g_bleStatus.vy = 0.0f; g_bleStatus.omega = -0.7f;
        } else if (strcasecmp(buffer, "S") == 0 || strcasecmp(buffer, "STOP") == 0) {
            g_bleStatus.vx = 0.0f; g_bleStatus.vy = 0.0f; g_bleStatus.omega = 0.0f;
        } else {
            // 不明なコマンドは無視
            return;
        }

        g_bleStatus.last_packet_ms = millis();
        g_bleStatus.rx_packet_count++;
        g_bleStatus.failsafe_triggered = false;
    }
};

void initBleController() {
    BLEDevice::init(BLE_DEVICE_NAME);
    g_pServer = BLEDevice::createServer();
    g_pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService = g_pServer->createService(BLE_SERVICE_UUID);

    // TX Characteristic (Notify)
    g_pTxCharacteristic = pService->createCharacteristic(
        BLE_CHAR_TX_UUID,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    g_pTxCharacteristic->addDescriptor(new BLE2902());

    // RX Characteristic (Write / Write Without Response)
    BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
        BLE_CHAR_RX_UUID,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
    );
    pRxCharacteristic->setCallbacks(new MyCharacteristicCallbacks());

    pService->start();

    // アドバタイズ開始
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(BLE_SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06); // iPhone等との接続安定化設定
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
}

bool updateBleController(float &out_vx, float &out_vy, float &out_omega) {
    if (!g_bleStatus.connected) {
        return false;
    }

    // フェールセーフ判定: パケット受信間隔が 500ms を超えたら即座に停止
    uint32_t elapsed = millis() - g_bleStatus.last_packet_ms;
    if (elapsed > BLE_PACKET_TIMEOUT_MS) {
        g_bleStatus.failsafe_triggered = true;
        g_bleStatus.vx = 0.0f;
        g_bleStatus.vy = 0.0f;
        g_bleStatus.omega = 0.0f;
        out_vx = 0.0f;
        out_vy = 0.0f;
        out_omega = 0.0f;
        return true; // BLEアクティブだがフェールセーフ停止
    }

    g_bleStatus.failsafe_triggered = false;
    out_vx = g_bleStatus.vx;
    out_vy = g_bleStatus.vy;
    out_omega = g_bleStatus.omega;
    return true;
}

void sendBleMessage(const char *msg) {
    if (g_bleStatus.connected && g_pTxCharacteristic != nullptr && msg != nullptr) {
        g_pTxCharacteristic->setValue((uint8_t*)msg, strlen(msg));
        g_pTxCharacteristic->notify();
    }
}

const BleStatus& getBleStatus() {
    return g_bleStatus;
}
