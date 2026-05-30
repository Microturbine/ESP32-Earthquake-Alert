/*
 * ESP32 災害警報デコーダ (リファクタリング版)
 * ハードウェア: ESP32 + RDA5807M + NEO-M8N + ST7789
 */

#include <Arduino.h>
#include "Settings.h"
#include "DisplayManager.h"
#include "EWS_Decoder.h"
#include "QZSS_Parser.h"

// ハードウェアピン
#define AUDIO_IN_PIN 4
#define I2C_SDA 19
#define I2C_SCL 18
#define GPS_RX_PIN 22
#define GPS_TX_PIN 23
#define ALERT_LED 2
#define BOOT_BUTTON_PIN 0

HardwareSerial gpsSerial(1);
TaskHandle_t taskCore0Handle;

int svCount = 0;
char timeStr[10] = "--:--:--";
char nmeaBuffer[82];
int nmeaIndex = 80;

// 差分描画用変数
int lastFreq = -1, lastRssi = -1, lastSvCount = -1, lastVol = -1;
char lastTimeStr[10] = "";
EwsState lastState = SEARCH_SYNC;
int lastQzssState = 0;
EwsState lastEwsState = SEARCH_SYNC;
int lastEwsAlertState = 0;

// LED点滅制御用変数
int ledBlinkCount = 0;
uint32_t nextLedToggleTime = 0;
bool ledState = false;

void processCommand(char* cmd) {
    if (strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0) {
        Serial.println("\n--- コマンド一覧 ---");
        Serial.println("help または ?     : このヘルプを表示します");
        Serial.println("status            : 現在のステータス(周波数、音量、RSSI、設定地域コード)を表示");
        Serial.println("v[数字]           : 音量を設定します (例: v5 で音量5に設定 [0-15])");
        Serial.println("area [コード]     : アラート対象とする地域コードを設定します (例: area 13)");
        Serial.println("                    ※都道府県コード等 (例: 13=東京, 27=大阪, 0=全域受信)");
        Serial.println("[数字]            : FMラジオの周波数を設定します (例: 85.2 で 85.2MHz)");
        Serial.println("test [43/44] [16進] : 指定した16進データでみちびきデコードをテストします");
        Serial.println("                    (例: test 44 533B0604DE195524CDA30305...)");
        Serial.println("--------------------\n");
    } else if (strncmp(cmd, "area ", 5) == 0) {
        uint32_t code = strtoul(&cmd[5], NULL, 10);
        settings.setRegion(code);
        Serial.printf("地域コード設定: %d\n", code);
    } else if (strncmp(cmd, "test ", 5) == 0) {
        char* p = &cmd[5];
        int mt = atoi(p);
        while(*p && *p != ' ') p++;
        if (*p == ' ') p++;
        
        uint8_t l1s_msg[32];
        memset(l1s_msg, 0, sizeof(l1s_msg));
        int len = strlen(p);
        for (int i = 0; i < 32 && i * 2 < len; i++) {
            char hex[3] = { p[i*2], p[i*2+1], '\0' };
            l1s_msg[i] = (uint8_t)strtol(hex, NULL, 16);
        }
        
        Serial.printf("デバッグデコードテスト (MT%d): ", mt);
        for(int i=0; i<32; i++) Serial.printf("%02X", l1s_msg[i]);
        Serial.println();
        
        if (mt == 43) {
            qzssParser.decodeMT43(l1s_msg);
        } else if (mt == 44) {
            qzssParser.decodeMT44(l1s_msg);
        }
        Serial.printf("結果: %s\n", qzssParser.getAlertText());
    } else if (cmd[0] == 'v' || cmd[0] == 'V') {
        int vol = atoi(&cmd[1]);
        if (vol >= 0 && vol <= 15) {
            settings.setVolume(vol);
            ewsDecoder.setVolume(vol);
            Serial.printf("音量設定: %d\n", vol);
        }
    } else if (strcmp(cmd, "status") == 0) {
        int freq = ewsDecoder.getFrequency();
        int rssi = ewsDecoder.getRssi();
        Serial.printf("現在ステータス: %d.%d MHz, 音量:%d, RSSI:%d, 地域コード:%d\n", 
                      freq / 100, freq % 100, settings.volume, rssi, settings.myRegionCode);
    } else {
        float f = atof(cmd);
        if (f >= 76.0 && f <= 108.0) {
            int freqInt = (int)(f * 100);
            settings.setFreq(freqInt);
            ewsDecoder.setFrequency(freqInt);
            Serial.printf("周波数設定: %d.%d MHz\n", freqInt / 100, freqInt % 100);
        }
    }
}

void taskCore0(void *pvParameters) {
    char cmdBuffer[128];
    int cmdIndex = 0;
    uint32_t lastUpdate = 0;

    for (;;) {
        uint32_t now = millis();

        // Bootボタン監視（押下でLOW）
        if (digitalRead(BOOT_BUTTON_PIN) == LOW) {
            qzssParser.resetAlert();
            ewsDecoder.resetState();
            ledBlinkCount = 0;
            digitalWrite(ALERT_LED, LOW);
        }

        // 1. LCD更新
        if (now - lastUpdate > 1000) {
            lastUpdate = now;
            int freq = ewsDecoder.getFrequency();
            int rssi = ewsDecoder.getRssi();
            int vol = settings.volume;
            
            qzssParser.updateTimeouts(now);
            ewsDecoder.updateTimeouts(now);
            
            int qState = qzssParser.getQzssState();
            EwsState eState = ewsDecoder.getState();
            int eAlertState = ewsDecoder.getEwsAlertState();

            // 新しいアラートの検知 (QZSSがONになった瞬間、またはEWSの受信開始時、またはEWSの警報確定時)
            bool newAlert = (qState > 0 && lastQzssState == 0) || 
                            (eState == DECODE_FRAME && lastEwsState == SEARCH_SYNC) ||
                            (eAlertState > 0 && lastEwsAlertState == 0);
            if (newAlert) {
                ledBlinkCount = 10; // 5回点滅
                nextLedToggleTime = millis();
            }

            bool changed = (freq != lastFreq) || (abs(rssi - lastRssi) > 2) || 
                           (svCount != lastSvCount) || (vol != lastVol) ||
                           (strcmp(timeStr, lastTimeStr) != 0) || (eState != lastState) || 
                           (qState != lastQzssState) || (eAlertState != lastEwsAlertState);

            if (changed) {
                lastFreq = freq; lastRssi = rssi; lastSvCount = svCount;
                lastVol = vol; lastState = eState; lastQzssState = qState; 
                lastEwsState = eState; lastEwsAlertState = eAlertState;
                strcpy(lastTimeStr, timeStr);
                
                displayManager.update(freq, rssi, vol, svCount, timeStr, eState, qState, qzssParser.getAlertText(), eAlertState, ewsDecoder.getEwsAlertText());
            }
        }

        // LED点滅の非同期処理
        if (ledBlinkCount > 0 && now >= nextLedToggleTime) {
            nextLedToggleTime = now + 250; // 250msおきに反転
            ledState = !ledState;
            digitalWrite(ALERT_LED, ledState ? HIGH : LOW);
            ledBlinkCount--;
            if (ledBlinkCount == 0) {
                digitalWrite(ALERT_LED, LOW); // 消灯
            }
        }

        // 2. シリアルコマンド
        while (Serial.available() > 0) {
            char c = Serial.read();
            if (c == '\n' || c == '\r') {
                if (cmdIndex > 0) {
                    cmdBuffer[cmdIndex] = '\0';
                    processCommand(cmdBuffer);
                    cmdIndex = 0;
                }
            } else if (cmdIndex < 127) {
                cmdBuffer[cmdIndex++] = c;
            }
        }

        // 3. GPS NMEA & UBX
        while (gpsSerial.available()) {
            uint8_t c = gpsSerial.read();

            if (c == '$') nmeaIndex = 0;
            if (nmeaIndex < 80) nmeaBuffer[nmeaIndex++] = c;
            if (c == '\n' && nmeaIndex > 6) {
                nmeaBuffer[nmeaIndex] = '\0';
                if (nmeaBuffer[0] == '$' && nmeaBuffer[3] == 'G' && nmeaBuffer[4] == 'G' && nmeaBuffer[5] == 'A') {
                    int commaCount = 0;
                    char* timeStart = NULL;
                    char* satStart = NULL;
                    for (int i = 0; i < nmeaIndex; i++) {
                        if (nmeaBuffer[i] == ',') {
                            commaCount++;
                            if (commaCount == 1) timeStart = &nmeaBuffer[i+1];
                            if (commaCount == 7) satStart = &nmeaBuffer[i+1];
                        }
                    }
                    if (timeStart && satStart) {
                        svCount = atoi(satStart);
                        if (timeStart[0] >= '0' && timeStart[0] <= '9' && timeStart[1] >= '0') {
                            int h = (timeStart[0]-'0')*10 + (timeStart[1]-'0');
                            int m = (timeStart[2]-'0')*10 + (timeStart[3]-'0');
                            int s = (timeStart[4]-'0')*10 + (timeStart[5]-'0');
                            h = (h + 9) % 24; // JST (日本標準時)
                            sprintf(timeStr, "%02d:%02d:%02d", h, m, s);
                        }
                    }
                }
                nmeaIndex = 80;
            }

            qzssParser.parseUbx(c);
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void setup() {
    Serial.begin(115200);
    settings.load();
    
    gpsSerial.begin(38400, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    const uint8_t enableSFRBX[] = {
      0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x02, 0x13, 0x01, 0x20, 0x6C
    };
    gpsSerial.write(enableSFRBX, sizeof(enableSFRBX));
    
    pinMode(ALERT_LED, OUTPUT);
    digitalWrite(ALERT_LED, LOW);
    pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);
    
    displayManager.init();
    ewsDecoder.init(I2C_SDA, I2C_SCL, AUDIO_IN_PIN);
    ewsDecoder.setFrequency(settings.defaultFreq);
    ewsDecoder.setVolume(settings.volume);
    
    xTaskCreatePinnedToCore(
      taskCore0, "TaskCore0", 8192, NULL, 1, &taskCore0Handle, 0
    );
}

void loop() {
    ewsDecoder.processAudio();
}
