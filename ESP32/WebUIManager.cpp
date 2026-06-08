#include "WebUIManager.h"
#include "WebUI_HTML.h"
#include "WebUI_Favicon.h"
#include "Settings.h"
#include "AlertManager.h"
#include "QZSS_Parser.h"
#include "EWS_Decoder.h"
#include <WiFi.h>

WebUIManager webUIManager;

// main/ESP32.ino 内のグローバル変数への参照
extern int svCount;
extern char timeStr[10];

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
    server.send(200, "text/html", WEBUI_HTML);
}

void WebUIManager::handleGetStatus() {
    // 警報リストのコピー
    Alert activeAlerts[4];
    int count = alertManager.copyAlerts(activeAlerts, 4);
    
    String json = "{";
    json += "\"freq\":" + String(ewsDecoder.getFrequency() / 100.0, 2) + ",";
    json += "\"rssi\":" + String(ewsDecoder.getRssi()) + ",";
    json += "\"volume\":" + String(settings.volume) + ",";
    json += "\"region\":" + String(settings.myRegionCode) + ",";
    json += "\"svCount\":" + String(svCount) + ",";
    json += "\"time\":\"" + String(timeStr) + "\",";
    json += "\"ewsState\":" + String((int)ewsDecoder.getState()) + ",";
    json += "\"wifiMode\":\"" + (apMode ? String("AP") : String("STA")) + "\",";
    json += "\"ip\":\"" + localIP + "\",";
    json += "\"alerts\":[";
    
    uint32_t now = millis();
    for (int i = 0; i < count; i++) {
        uint32_t remainingSec = 0;
        if (activeAlerts[i].expiry > now) {
            remainingSec = (activeAlerts[i].expiry - now) / 1000;
        }
        
        json += "{";
        json += "\"text\":\"" + String(activeAlerts[i].text) + "\",";
        json += "\"remaining\":" + String(remainingSec) + ",";
        json += "\"isTest\":" + String(activeAlerts[i].isTest ? "true" : "false");
        json += "}";
        
        if (i < count - 1) {
            json += ",";
        }
    }
    json += "]";
    json += "}";
    
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
            alertManager.addAlert("【模擬】緊急地震速報：震源地は関東地方、最大震度６強と推定。強い揺れに警戒してください。", 120000, false);
            server.send(200, "text/plain", "OK");
        } 
        else if (type == "tsunami") {
            Serial.println("[WebUI] 模擬大津波警報テスト発報");
            alertManager.addAlert("【模擬】大津波警報：太平洋沿岸部。直ちに高台などの安全な場所に避難してください。", 120000, false);
            server.send(200, "text/plain", "OK");
        } 
        else if (type == "jalert") {
            Serial.println("[WebUI] 模擬Jアラートテスト発報");
            alertManager.addAlert("【模擬】ミサイル発射：北朝鮮方面からミサイルが発射された模様。頑丈な建物や地下に避難してください。", 120000, false);
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
    server.send_P(200, "image/x-icon", (const char*)FAVICON_ICO, FAVICON_ICO_LEN);
}

