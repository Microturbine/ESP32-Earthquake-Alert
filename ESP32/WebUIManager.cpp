#include "WebUIManager.h"
#include "WebUI_HTML.h"
#include "WebUI_Favicon.h"
#include "Settings.h"
#include "AlertManager.h"
#include "QZSS_Parser.h"
#include "EWS_Decoder.h"
#include "DisplayManager.h"
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <Update.h>

WebUIManager webUIManager;

// main/ESP32.ino 内のグローバル変数への参照
extern int svCount;
extern char timeStr[10];
extern char lastGga[];

WebUIManager::WebUIManager() : server(80), apMode(false), localIP("0.0.0.0"), activeSSID("") {}

void WebUIManager::init() {
    jsonResponseBuf.reserve(4096);
    Serial.println("\n--- WiFi初期化 ---");
    
    // 自動保存を無効化し、以前の接続状態を完全にクリーンアップしてハードウェアをリセット
    WiFi.persistent(false);
    WiFi.disconnect(true, false);
    WiFi.mode(WIFI_OFF);
    delay(100);
    
    if (settings.wifiSSID.length() > 0) {
        Serial.printf("WiFi 接続試行中... SSID: %s\n", settings.wifiSSID.c_str());
        // 最初からSTAモードで開始
        WiFi.mode(WIFI_STA);
        WiFi.begin(settings.wifiSSID.c_str(), settings.wifiPassword.c_str());
        
        int attempts = 0;
        // 最大15秒待機 (100ms * 150)
        while (WiFi.status() != WL_CONNECTED && attempts < 150) {
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
            // すでにWIFI_STAモードで動作しているため、不要なWiFi.mode()設定は行いません
        } else {
            Serial.println("WiFi 接続タイムアウト。APモードに切り替えます。");
            apMode = true;
        }
    } else {
        Serial.println("WiFi SSIDが設定されていません。APモードで起動します。");
        apMode = true;
    }
    
    if (apMode) {
        WiFi.disconnect(true); // STA接続設定をリセット
        WiFi.mode(WIFI_AP);
        String apName = "ESP32-Alert-Decoder";
        WiFi.softAP(apName.c_str(), "");
        localIP = WiFi.softAPIP().toString();
        activeSSID = apName;
        Serial.printf("APモード起動。SSID: %s, IP: %s\n", apName.c_str(), localIP.c_str());
    }
    
    if (MDNS.begin("esp32-alert-decoder")) {
        Serial.println("mDNS responder started: http://esp32-alert-decoder.local/");
    }
    setupOTA();
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
    
    // Web OTA アップデート用ルート
    server.on("/update", HTTP_GET, std::bind(&WebUIManager::handleGetUpdate, this));
    server.on("/update", HTTP_POST, 
              std::bind(&WebUIManager::handlePostUpdate, this), 
              [this]() {
                  HTTPUpload& upload = server.upload();
                  if (upload.status == UPLOAD_FILE_START) {
                      Serial.printf("Update File Start: %s\n", upload.filename.c_str());
                      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { // 最大空きサイズで開始
                          Update.printError(Serial);
                      }
                  } else if (upload.status == UPLOAD_FILE_WRITE) {
                      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
                          Update.printError(Serial);
                      }
                  } else if (upload.status == UPLOAD_FILE_END) {
                      if (Update.end(true)) { // 進捗に合わせてサイズを確定
                          Serial.printf("Update Success: %u bytes\n", upload.totalSize);
                      } else {
                          Update.printError(Serial);
                      }
                  }
              });
}

void WebUIManager::handleRoot() {
    server.send_P(200, "text/html", WEBUI_HTML);
}

static String escapeJsonString(const String& input) {
    int len = input.length();
    char buf[512];
    int j = 0;
    for (int i = 0; i < len && j < 510; i++) {
        char c = input[i];
        if (c == '"')       { buf[j++] = '\\'; buf[j++] = '"'; }
        else if (c == '\\') { buf[j++] = '\\'; buf[j++] = '\\'; }
        else if (c == '/')  { buf[j++] = '\\'; buf[j++] = '/'; }
        else if (c == '\b') { buf[j++] = '\\'; buf[j++] = 'b'; }
        else if (c == '\f') { buf[j++] = '\\'; buf[j++] = 'f'; }
        else if (c == '\n') { buf[j++] = '\\'; buf[j++] = 'n'; }
        else if (c == '\r') { buf[j++] = '\\'; buf[j++] = 'r'; }
        else if (c == '\t') { buf[j++] = '\\'; buf[j++] = 't'; }
        else if (c >= 0 && c < 32) {
            if (j + 6 < 512) {
                j += snprintf(&buf[j], 7, "\\u%04x", c);
            }
        } else {
            buf[j++] = c;
        }
    }
    buf[j] = '\0';
    return String(buf);
}

void WebUIManager::handleGetStatus() {
    // 静的バッファにスレッドセーフにコピーして動的ヒープ確保を排除
    int count = alertManager.copyAlerts(statusAlerts, AlertManager::MAX_ALERTS);
    int histCount = alertManager.copyHistory(statusHistAlerts, AlertManager::MAX_HISTORY);
    
    // バッファ容量を維持したままクリアし再利用
    jsonResponseBuf = "";
    
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
    jsonResponseBuf += headerBuf;
    
    uint32_t now = millis();
    for (int i = 0; i < count; i++) {
        uint32_t remainingSec = 0;
        if (statusAlerts[i].expiry > now) {
            remainingSec = (statusAlerts[i].expiry - now) / 1000;
        }
        
        String escapedText = escapeJsonString(statusAlerts[i].text);
        
        char maskStr[32];
        uint32_t prefMaskHigh = (uint32_t)(statusAlerts[i].prefMask >> 32);
        uint32_t prefMaskLow = (uint32_t)(statusAlerts[i].prefMask & 0xFFFFFFFFULL);
        snprintf(maskStr, sizeof(maskStr), "\"%08x%08x\"", prefMaskHigh, prefMaskLow);
        
        char alertBuf[600];
        snprintf(alertBuf, sizeof(alertBuf), 
                 "{\"text\":\"%s\",\"remaining\":%u,\"isTest\":%s,\"lat\":%.4f,\"lon\":%.4f,\"cat\":%d,\"code\":%d,\"prefMask\":%s,\"outOfRegion\":%s,\"elMajor\":%.3f,\"elMinor\":%.3f,\"elAzimuth\":%.2f}",
                 escapedText.c_str(),
                 remainingSec,
                 statusAlerts[i].isTest ? "true" : "false",
                 statusAlerts[i].latitude,
                 statusAlerts[i].longitude,
                 statusAlerts[i].disasterCat,
                 statusAlerts[i].code,
                 maskStr,
                 statusAlerts[i].isOutOfRegion ? "true" : "false",
                 statusAlerts[i].ellipseMajor,
                 statusAlerts[i].ellipseMinor,
                 statusAlerts[i].ellipseAzimuth);
                 
        jsonResponseBuf += alertBuf;
        if (i < count - 1) {
            jsonResponseBuf += ",";
        }
    }
    jsonResponseBuf += "],";
    
    // 警報履歴のシリアライズ
    jsonResponseBuf += "\"history\":[";
    for (int i = 0; i < histCount; i++) {
        String escapedText = escapeJsonString(statusHistAlerts[i].text);
        char histBuf[256];
        snprintf(histBuf, sizeof(histBuf),
                 "{\"text\":\"%s\",\"isTest\":%s,\"outOfRegion\":%s,\"time\":\"%s\"}",
                 escapedText.c_str(),
                 statusHistAlerts[i].isTest ? "true" : "false",
                 statusHistAlerts[i].isOutOfRegion ? "true" : "false",
                 statusHistAlerts[i].receivedTimeStr);
        jsonResponseBuf += histBuf;
        if (i < histCount - 1) {
            jsonResponseBuf += ",";
        }
    }
    jsonResponseBuf += "],";
    
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
             
    jsonResponseBuf += tailBuf;
    jsonResponseBuf += "}";
    
    server.send(200, "application/json", jsonResponseBuf);
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
        else if (type == "lalert") {
            Serial.println("[WebUI] 模擬Lアラート避難指示テスト発報");
            // 避難指示：都庁付近（長半径 10km, 短半径 5km, 方位角 45度, 東京 prefMask=0x0008000000000000ULL）
            alertManager.addAlert("【模擬】避難指示：大雨特別警報に伴う土砂災害の危険が高まりました。直ちに避難所等の安全な場所に避難してください。", 120000, false, 35.689, 139.692, 44, 76, 0x0008000000000000ULL, false, 10.0, 5.0, 45.0);
            server.send(200, "text/plain", "OK");
        } 
        else if (type == "marine") {
            Serial.println("[WebUI] 模擬海上警報テスト発報");
            alertManager.addAlert("【模擬】海上警報：三陸沖西部で海上暴風警報が発表されました。警戒してください。", 120000, false, 0.0, 0.0, 14, 2020);
            alertManager.addAlert("【模擬】海上警報：その他の地方海上予報区で海上強風警報が発表されました。注意してください。", 120000, false, 0.0, 0.0, 14, 10000);
            alertManager.addAlert("【模擬】海上警報：北海道東方海上で海上強風警報が発表されました。注意してください。", 120000, false, 0.0, 0.0, 14, 1110);
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

void WebUIManager::handleGetUpdate() {
    const char* updateHtml = R"rawhtml(
<!DOCTYPE html>
<html lang="ja">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ファームウェア更新 | 災害警報デコーダー</title>
    <style>
        :root {
            --bg-primary: #0a0712;
            --bg-secondary: #130f22;
            --card-bg: rgba(25, 20, 42, 0.55);
            --card-border: rgba(255, 255, 255, 0.08);
            --text-primary: #f4f3f6;
            --text-secondary: #9c97aa;
            --accent: #8b5cf6;
            --accent-hover: #a78bfa;
            --accent-glow: rgba(139, 92, 246, 0.3);
        }
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body {
            font-family: sans-serif;
            background-color: var(--bg-primary);
            background-image: radial-gradient(at 50% 50%, rgba(139, 92, 246, 0.15) 0px, transparent 60%);
            color: var(--text-primary);
            min-height: 100vh;
            display: flex;
            align-items: center;
            justify-content: center;
            padding: 1.5rem;
        }
        .card {
            background: var(--card-bg);
            border: 1px solid var(--card-border);
            border-radius: 16px;
            padding: 2rem;
            backdrop-filter: blur(16px);
            -webkit-backdrop-filter: blur(16px);
            width: 100%;
            max-width: 420px;
            box-shadow: 0 8px 32px var(--accent-glow);
            text-align: center;
        }
        h2 {
            font-size: 1.4rem;
            margin-bottom: 0.5rem;
            background: linear-gradient(135deg, #fff 0%, var(--text-secondary) 100%);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
        }
        p {
            font-size: 0.85rem;
            color: var(--text-secondary);
            margin-bottom: 1.5rem;
        }
        .file-drop-area {
            border: 2px dashed rgba(255, 255, 255, 0.15);
            border-radius: 12px;
            padding: 2rem 1rem;
            background: rgba(255, 255, 255, 0.02);
            cursor: pointer;
            transition: all 0.3s ease;
            margin-bottom: 1.5rem;
            position: relative;
        }
        .file-drop-area:hover {
            border-color: var(--accent);
            background: rgba(139, 92, 246, 0.05);
        }
        .file-input {
            position: absolute;
            left: 0; top: 0; width: 100%; height: 100%;
            opacity: 0;
            cursor: pointer;
        }
        .file-msg {
            font-size: 0.9rem;
            color: var(--text-secondary);
        }
        .btn {
            background: var(--accent);
            color: #fff;
            border: none;
            border-radius: 8px;
            padding: 0.75rem 1.5rem;
            font-size: 0.9rem;
            font-weight: 600;
            cursor: pointer;
            width: 100%;
            transition: all 0.2s ease;
            box-shadow: 0 4px 12px var(--accent-glow);
        }
        .btn:hover {
            background: var(--accent-hover);
            box-shadow: 0 0 16px var(--accent-glow);
        }
        .btn:disabled {
            background: #3f3b4f;
            color: #7d788c;
            cursor: not-allowed;
            box-shadow: none;
        }
        .back-link {
            display: inline-block;
            margin-top: 1.5rem;
            color: var(--text-secondary);
            text-decoration: none;
            font-size: 0.8rem;
            transition: color 0.2s;
        }
        .back-link:hover { color: #fff; }
        #progress-bar-container {
            width: 100%;
            height: 6px;
            background: rgba(255, 255, 255, 0.08);
            border-radius: 9999px;
            overflow: hidden;
            margin-top: 1.2rem;
            display: none;
        }
        #progress-bar {
            height: 100%;
            width: 0%;
            background: var(--accent);
            border-radius: 9999px;
            transition: width 0.1s ease;
        }
        #status-msg {
            margin-top: 0.8rem;
            font-size: 0.85rem;
            display: none;
        }
    </style>
</head>
<body>
    <div class="card">
        <h2>ファームウェア更新 (OTA)</h2>
        <p>コンパイルされたバイナリファイル (.bin) を選択してください。</p>
        
        <form id="upload-form" method="POST" action="/update" enctype="multipart/form-data">
            <div class="file-drop-area" id="drop-area">
                <span class="file-msg" id="file-name-display">ファイルを選択するか、ここにドラッグ＆ドロップ</span>
                <input type="file" name="update" id="file-input" class="file-input" required accept=".bin" onchange="fileSelected()">
            </div>
            <button type="submit" class="btn" id="submit-btn" disabled>アップデート開始</button>
        </form>
        
        <div id="progress-bar-container">
            <div id="progress-bar"></div>
        </div>
        <div id="status-msg"></div>
        
        <a href="/" class="back-link">← ダッシュボードへ戻る</a>
    </div>

    <script>
        const fileInput = document.getElementById('file-input');
        const submitBtn = document.getElementById('submit-btn');
        const fileNameDisplay = document.getElementById('file-name-display');
        const uploadForm = document.getElementById('upload-form');
        const progressBarContainer = document.getElementById('progress-bar-container');
        const progressBar = document.getElementById('progress-bar');
        const statusMsg = document.getElementById('status-msg');

        function fileSelected() {
            if (fileInput.files.length > 0) {
                const file = fileInput.files[0];
                fileNameDisplay.innerText = file.name + ' (' + (file.size / 1024 / 1024).toFixed(2) + ' MB)';
                submitBtn.disabled = false;
            } else {
                fileNameDisplay.innerText = 'ファイルを選択するか、ここにドラッグ＆ドロップ';
                submitBtn.disabled = true;
            }
        }

        uploadForm.onsubmit = function(e) {
            e.preventDefault();
            const file = fileInput.files[0];
            if (!file) return;

            submitBtn.disabled = true;
            progressBarContainer.style.display = 'block';
            statusMsg.style.display = 'block';
            statusMsg.style.color = 'var(--text-secondary)';
            statusMsg.innerText = 'アップロード中...';

            const xhr = new XMLHttpRequest();
            xhr.open('POST', '/update', true);

            xhr.upload.onprogress = function(e) {
                if (e.lengthComputable) {
                    const percent = (e.loaded / e.total) * 100;
                    progressBar.style.width = percent + '%';
                    statusMsg.innerText = 'アップロード中... ' + percent.toFixed(0) + '%';
                }
            };

            xhr.onload = function() {
                if (xhr.status === 200) {
                    statusMsg.style.color = '#10b981';
                    statusMsg.innerText = '更新が成功しました！デバイスが自動再起動します。5秒後にダッシュボードに戻ります。';
                    setTimeout(() => {
                        window.location.href = '/';
                    }, 5000);
                } else {
                    statusMsg.style.color = '#ef4444';
                    statusMsg.innerText = '更新に失敗しました: ' + xhr.responseText;
                    submitBtn.disabled = false;
                }
            };

            xhr.onerror = function() {
                statusMsg.style.color = '#ef4444';
                statusMsg.innerText = '接続エラーが発生しました。';
                submitBtn.disabled = false;
            };

            const formData = new FormData(uploadForm);
            xhr.send(formData);
        };
    </script>
</body>
</html>
)rawhtml";
    server.send(200, "text/html", updateHtml);
}

void WebUIManager::handlePostUpdate() {
    server.sendHeader("Connection", "close");
    if (Update.hasError()) {
        server.send(500, "text/plain", "Update Failed: " + String(Update.errorString()));
    } else {
        server.send(200, "text/html", "<html><body><h2>Update Success! Rebooting...</h2></body></html>");
        delay(1000);
        ESP.restart();
    }
}

void WebUIManager::setupOTA() {
    String host = "esp32-alert-decoder";
    ArduinoOTA.setHostname(host.c_str());

    ArduinoOTA.onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH) {
            type = "sketch";
        } else {
            type = "filesystem";
        }
        Serial.println("Start updating " + type);
    });
    ArduinoOTA.onEnd([]() {
        Serial.println("\nEnd");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

    ArduinoOTA.begin();
    Serial.println("ArduinoOTA started");
}

