#include "WebUIManager.h"
#include "WebUI_HTML.h"
#include "WebUI_Favicon.h"
#include "Settings.h"
#include "AlertManager.h"
#include "QZSS_Parser.h"
#include "EWS_Decoder.h"
#include "DisplayManager.h"
#include <WiFi.h>

WebUIManager webUIManager;

// main/ESP32.ino 内のグローバル変数への参照
extern int svCount;
extern char timeStr[10];
extern char lastGga[];

WebUIManager::WebUIManager() : server(80), apMode(false), localIP("0.0.0.0"), activeSSID("") {}

void WebUIManager::init() {
    Serial.println("\n--- WiFi初期化 ---");
    
    if (settings.wifiSSID.length() > 0) {
        Serial.printf("WiFi 接続試行中... SSID: %s\n", settings.wifiSSID.c_str());
        WiFi.mode(WIFI_AP_STA); // APも有効化できるように設定
        WiFi.begin(settings.wifiSSID.c_str(), settings.wifiPassword.c_str());
        
        int attempts = 0;
        // 最大10秒待機 (100ms * 100)
        while (WiFi.status() != WL_CONNECTED && attempts < 100) {
            delay(100);
            attempts++;
            if (attempts % 10 == 0) {
                Serial.print(".");
            }
        }
        Serial.println();
        
        if (WiFi.status() == WL_CONNECTED) {
            apMode = false;
            localIP = WiFi.localIP().toString();
            activeSSID = settings.wifiSSID;
            Serial.printf("WiFi 接続成功! IP: %s\n", localIP.c_str());
            
            // 無駄な電波出力を防ぐためSTAモードのみにする
            WiFi.mode(WIFI_STA);
        } else {
            Serial.println("WiFi 接続タイムアウト。APモードに切り替えます。");
            apMode = true;
        }
    } else {
        Serial.println("WiFi SSIDが設定されていません。APモードで起動します。");
        apMode = true;
    }
    
    if (apMode) {
        WiFi.mode(WIFI_AP);
        String apName = "ESP32-Alert-Decoder";
        WiFi.softAP(apName.c_str(), "");
        localIP = WiFi.softAPIP().toString();
        activeSSID = apName;
        Serial.printf("APモード起動。SSID: %s, IP: %s\n", apName.c_str(), localIP.c_str());
    }
    
    setupRoutes();
    server.begin();
    Serial.println("HTTP Webサーバー起動完了 (Port: 80)");
}

void WebUIManager::handleClient() {
    server.handleClient();
}

String WebUIManager::getIPAddress() const {
    return localIP;
}

String WebUIManager::getSSID() const {
    return activeSSID;
}

bool WebUIManager::isAPMode() const {
    return apMode;
}

void WebUIManager::setupRoutes() {
    server.on("/", HTTP_GET, std::bind(&WebUIManager::handleRoot, this));
    server.on("/favicon.ico", HTTP_GET, std::bind(&WebUIManager::handleFavicon, this));
    server.on("/api/status", HTTP_GET, std::bind(&WebUIManager::handleGetStatus, this));
    server.on("/api/settings", HTTP_POST, std::bind(&WebUIManager::handlePostSettings, this));
    server.on("/api/test", HTTP_POST, std::bind(&WebUIManager::handlePostTest, this));
    server.on("/api/clear", HTTP_POST, std::bind(&WebUIManager::handlePostClear, this));
}

void WebUIManager::handleRoot() {
    server.send_P(200, "text/html", WEBUI_HTML);
}

// JSON特殊文字エスケープ用のヘルパー関数
static String escapeJsonString(const String& input) {
    String output = "";
    output.reserve(input.length() + 8); // 少し余裕を持たせて確保
    for (size_t i = 0; i < input.length(); i++) {
        char c = input[i];
        if (c == '"') {
            output += "\\\"";
        } else if (c == '\\') {
            output += "\\\\";
        } else if (c == '/') {
            output += "\\/";
        } else if (c == '\b') {
            output += "\\b";
        } else if (c == '\f') {
            output += "\\f";
        } else if (c == '\n') {
            output += "\\n";
        } else if (c == '\r') {
            output += "\\r";
        } else if (c == '\t') {
            output += "\\t";
        } else if (c >= 0 && c < 32) {
            char hex[8];
            snprintf(hex, sizeof(hex), "\\u%04x", c);
            output += hex;
        } else {
            output += c;
        }
    }
    return output;
}

void WebUIManager::handleGetStatus() {
    // 警報リストのコピー
    Alert activeAlerts[AlertManager::MAX_ALERTS];
    int count = alertManager.copyAlerts(activeAlerts, AlertManager::MAX_ALERTS);
    
    // 警報履歴のコピー
    HistoryAlert histAlerts[AlertManager::MAX_HISTORY];
    int histCount = alertManager.copyHistory(histAlerts, AlertManager::MAX_HISTORY);
    
    String json;
    json.reserve(4096); // Pre-allocate to prevent heap fragmentation
    
    char headerBuf[512];
    snprintf(headerBuf, sizeof(headerBuf),
             "{\"freq\":%.2f,\"rssi\":%d,\"volume\":%d,\"region\":%d,\"svCount\":%d,\"time\":\"%s\",\"ewsState\":%d,\"wifiMode\":\"%s\",\"ip\":\"%s\",\"mute\":%s,\"screenOff\":%s,\"alerts\":[",
             ewsDecoder.getFrequency() / 100.0,
             ewsDecoder.getRssi(),
             settings.volume,
             settings.myRegionCode,
             svCount,
             timeStr,
             (int)ewsDecoder.getState(),
             apMode ? "AP" : "STA",
             localIP.c_str(),
             ewsDecoder.getMute() ? "true" : "false",
             displayManager.getScreenOff() ? "true" : "false");
    json += headerBuf;
    
    uint32_t now = millis();
    for (int i = 0; i < count; i++) {
        uint32_t remainingSec = 0;
        if (activeAlerts[i].expiry > now) {
            remainingSec = (activeAlerts[i].expiry - now) / 1000;
        }
        
        String escapedText = escapeJsonString(activeAlerts[i].text);
        
        char maskStr[32];
        uint32_t prefMaskHigh = (uint32_t)(activeAlerts[i].prefMask >> 32);
        uint32_t prefMaskLow = (uint32_t)(activeAlerts[i].prefMask & 0xFFFFFFFFULL);
        snprintf(maskStr, sizeof(maskStr), "\"%08x%08x\"", prefMaskHigh, prefMaskLow);
        
        char alertBuf[512];
        snprintf(alertBuf, sizeof(alertBuf), 
                 "{\"text\":\"%s\",\"remaining\":%u,\"isTest\":%s,\"lat\":%.4f,\"lon\":%.4f,\"cat\":%d,\"code\":%d,\"prefMask\":%s,\"outOfRegion\":%s}",
                 escapedText.c_str(),
                 remainingSec,
                 activeAlerts[i].isTest ? "true" : "false",
                 activeAlerts[i].latitude,
                 activeAlerts[i].longitude,
                 activeAlerts[i].disasterCat,
                 activeAlerts[i].code,
                 maskStr,
                 activeAlerts[i].isOutOfRegion ? "true" : "false");
                 
        json += alertBuf;
        if (i < count - 1) {
            json += ",";
        }
    }
    json += "],";
    
    // 警報履歴のシリアライズ
    json += "\"history\":[";
    for (int i = 0; i < histCount; i++) {
        String escapedText = escapeJsonString(histAlerts[i].text);
        char histBuf[256];
        snprintf(histBuf, sizeof(histBuf),
                 "{\"text\":\"%s\",\"isTest\":%s,\"outOfRegion\":%s,\"time\":\"%s\"}",
                 escapedText.c_str(),
                 histAlerts[i].isTest ? "true" : "false",
                 histAlerts[i].isOutOfRegion ? "true" : "false",
                 histAlerts[i].receivedTimeStr);
        json += histBuf;
        if (i < histCount - 1) {
            json += ",";
        }
    }
    json += "],";
    
    // GPS & みちびき デバッグ情報
    String escapedGga = escapeJsonString(lastGga);
    String lastL1sHexStr = qzssParser.getLastL1sHex(); // Safe thread read
    uint32_t sinceLastL1s = (qzssParser.lastL1sTime > 0) ? (millis() - qzssParser.lastL1sTime) / 1000 : 99999;
    
    char tailBuf[512];
    snprintf(tailBuf, sizeof(tailBuf), 
             "\"gpsGga\":\"%s\",\"qzssSfrbxCount\":%u,\"qzssMt43Count\":%u,\"qzssMt44Count\":%u,\"lastL1sHex\":\"%s\",\"sinceLastL1s\":%u",
             escapedGga.c_str(),
             (unsigned int)qzssParser.sfrbxCount,
             (unsigned int)qzssParser.mt43Count,
             (unsigned int)qzssParser.mt44Count,
             lastL1sHexStr.c_str(),
             (unsigned int)sinceLastL1s);
             
    json += tailBuf;
    json += "}";
    
    // シリアルモニタへのデバッグ出力（JSON内容確認用）
    Serial.println("\n--- [WebUI Status JSON Send] ---");
    Serial.println(json);
    Serial.println("--------------------------------\n");
    
    server.send(200, "application/json", json);
}

void WebUIManager::handlePostSettings() {
    bool updated = false;
    
    if (server.hasArg("freq")) {
        float f = server.arg("freq").toFloat();
        if (f >= 76.0 && f <= 108.0) {
            int freqInt = (int)(f * 100.0);
            settings.setFreq(freqInt);
            ewsDecoder.setFrequency(freqInt);
            updated = true;
            Serial.printf("[WebUI] 周波数設定変更: %.2f MHz\n", f);
        }
    }
    
    if (server.hasArg("volume")) {
        int vol = server.arg("volume").toInt();
        if (vol >= 0 && vol <= 15) {
            settings.setVolume(vol);
            ewsDecoder.setVolume(vol);
            updated = true;
            Serial.printf("[WebUI] 音量設定変更: %d\n", vol);
        }
    }
    
    if (server.hasArg("region")) {
        int reg = server.arg("region").toInt();
        if (reg >= 0 && reg <= 99) {
            settings.setRegion(reg);
            updated = true;
            Serial.printf("[WebUI] 地域コード設定変更: %d\n", reg);
        }
    }
    
    if (server.hasArg("mute")) {
        bool m = (server.arg("mute") == "1");
        ewsDecoder.setMute(m);
        updated = true;
        Serial.printf("[WebUI] 消音設定変更: %s\n", m ? "ON (消音)" : "OFF (音声出力)");
    }
    
    if (server.hasArg("screenOff")) {
        bool off = (server.arg("screenOff") == "1");
        displayManager.setScreenOff(off);
        updated = true;
        Serial.printf("[WebUI] 画面消灯設定変更: %s\n", off ? "ON (通常時消灯)" : "OFF (常時点灯)");
    }
    
    if (server.hasArg("ssid")) {
        String ssid = server.arg("ssid");
        String pass = server.arg("pass");
        
        settings.setWiFi(ssid, pass);
        Serial.printf("[WebUI] WiFi設定保存: SSID: %s (再起動します)\n", ssid.c_str());
        
        server.send(200, "text/plain", "OK");
        delay(500);
        ESP.restart();
        return;
    }
    
    if (updated) {
        server.send(200, "text/plain", "OK");
    } else {
        server.send(400, "text/plain", "Bad Request");
    }
}

void WebUIManager::handlePostTest() {
    if (server.hasArg("type")) {
        String type = server.arg("type");
        
        if (type == "eq") {
            Serial.println("[WebUI] 模擬緊急地震速報テスト発報");
            alertManager.addAlert("【模擬】緊急地震速報：震源地は関東地方、最大震度６強と推定。強い揺れに警戒してください。", 120000, false, 35.6, 140.0, 2, 350);
            server.send(200, "text/plain", "OK");
        } 
        else if (type == "tsunami") {
            Serial.println("[WebUI] 模擬大津波警報テスト発報");
            alertManager.addAlert("【模擬】大津波警報：太平洋沿岸部。直ちに高台などの安全な場所に避難してください。", 120000, false, 38.2, 141.5, 5, 220);
            server.send(200, "text/plain", "OK");
        } 
        else if (type == "jalert") {
            Serial.println("[WebUI] 模擬Jアラートテスト発報");
            alertManager.addAlert("【模擬】ミサイル発射：北朝鮮方面からミサイルが発射された模様。頑丈な建物や地下に避難してください。", 120000, false, 0.0, 0.0, 44, 1, 0xC000000000000000ULL);
            server.send(200, "text/plain", "OK");
        } 
        else if (type == "hex" && server.hasArg("hex")) {
            String hex = server.arg("hex");
            hex.trim();
            if (hex.length() == 64) {
                uint8_t l1s_msg[32];
                for (int i = 0; i < 32; i++) {
                    char h[3] = { hex[i*2], hex[i*2+1], '\0' };
                    l1s_msg[i] = (uint8_t)strtol(h, NULL, 16);
                }
                
                uint8_t mt = (l1s_msg[1] >> 2) & 0x3F;
                Serial.printf("[WebUI] 生パケットデコードテスト (MT%d): %s\n", mt, hex.c_str());
                
                if (mt == 43) {
                    qzssParser.decodeMT43(l1s_msg);
                } else if (mt == 44) {
                    qzssParser.decodeMT44(l1s_msg);
                }
                server.send(200, "text/plain", "OK");
            } else {
                server.send(400, "text/plain", "Invalid hex length");
            }
        } 
        else {
            server.send(400, "text/plain", "Unknown test type");
        }
    } else {
        server.send(400, "text/plain", "Missing parameter: type");
    }
}

void WebUIManager::handlePostClear() {
    Serial.println("[WebUI] アラート手動クリア指示");
    qzssParser.resetAlert();
    ewsDecoder.resetState();
    alertManager.clear();
    server.send(200, "text/plain", "OK");
}

void WebUIManager::handleFavicon() {
    server.sendHeader("Cache-Control", "public, max-age=2592000"); // 30 days cache
    server.setContentLength(FAVICON_ICO_LEN);
    server.send(200, "image/x-icon", "");
    // PROGMEM→RAMにコピーして送信（send_Pはバイナリのnullバイトで途切れる場合がある）
    uint8_t buf[256];
    size_t sent = 0;
    while (sent < FAVICON_ICO_LEN) {
        size_t chunk = min((size_t)256, FAVICON_ICO_LEN - sent);
        memcpy_P(buf, FAVICON_ICO + sent, chunk);
        server.sendContent((const char*)buf, chunk);
        sent += chunk;
    }
}

