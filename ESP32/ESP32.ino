/*
 * ESP32 EWS Decoder via FM Radio
 * Hardware: ESP32 + RDA5807M
 *
 * デコード機能:
 * 1. 64bps FSK復調 (1024Hz/640Hz)
 * 2. 同期フラグ (0xB135) の検出
 * 3. 分類コード (開始/テスト等) の判定
 */

#include <RDA5807.h>
#include <Wire.h>
#define LGFX_USE_V1
#include <LovyanGFX.hpp>

// --- LovyanGFX (ST7789) 設定 ---
class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ST7789  _panel_instance;
  lgfx::Bus_SPI       _bus_instance;
  lgfx::Light_PWM     _light_instance;

public:
  LGFX(void) {
    {
      auto cfg = _bus_instance.config();
      cfg.spi_host = VSPI_HOST;
      cfg.spi_mode = 3;
      cfg.freq_write = 20000000; // 20MHz
      cfg.pin_sclk = 14; 
      cfg.pin_mosi = 13;
      cfg.pin_miso = -1;
      cfg.pin_dc   = 15;
      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }
    {
      auto cfg = _panel_instance.config();
      cfg.pin_cs           = 5;
      cfg.pin_rst          = 27;
      cfg.panel_width      = 76;
      cfg.panel_height     = 284;
      cfg.offset_x         = 82;
      cfg.offset_y         = 18;
      cfg.offset_rotation  = 0;
      _panel_instance.config(cfg);
    }
    {
      auto cfg = _light_instance.config();
      cfg.pin_bl = 21;
      cfg.invert = false;
      cfg.freq   = 44100;
      cfg.pwm_channel = 7;
      _light_instance.config(cfg);
      _panel_instance.setLight(&_light_instance);
    }
    setPanel(&_panel_instance);
  }
};
LGFX tft;
LGFX_Sprite canvas(&tft); // フリッカー防止用の仮想画面

// 設定
#define FM_STATION 8520 // 受信周波数
#define AUDIO_IN_PIN 4
#define SAMPLING_RATE 8000      // 8000Hz / 125 = 64Hz (1ビットの時間に相当)
#define N 125                   
#define SIGNAL_THRESHOLD 130000 // 判定しきい値（ノイズが多い場合は大きく）
#define I2C_SDA 19              // SDAピン
#define I2C_SCL 18              // SCLピン

// GPS (NEO-M8N) 設定
#define GPS_RX_PIN 22
#define GPS_TX_PIN 23
HardwareSerial gpsSerial(1);

// 本体LED (アラート用)
#define ALERT_LED 2

// アナログEWS仕様に基づく定数
const uint16_t SYNC_TYPE_I = 0x0E6D;  // 0000 1110 0110 1101
const uint16_t SYNC_TYPE_II = 0xF192; // 1111 0001 1001 0010

// 地域符号 (12bit reversed)
const uint16_t AREA_ALL =
    0xB2C; // 地域共通 (0011 0100 1101 -> rev: 1011 0010 1100)
const uint16_t AREA_KANTO =
    0x4BA; // 関東広域 (0101 1101 0010 -> rev: 0100 1011 1010)
const uint16_t AREA_TOKYO =
    0x355; // 東京都   (1010 1010 1100 -> rev: 0011 0101 0101)
const uint16_t AREA_OSAKA =
    0x4DB; // 大阪府   (1100 1011 0010 -> rev: 0100 1101 1011)

enum EwsState { SEARCH_SYNC, DECODE_FRAME };
EwsState currentState = SEARCH_SYNC;
bool isEndSignal = false;
uint16_t currentSyncType = 0;

uint8_t frameBits[100]; // 96ビット分のバッファ
int bitIndex = 0;
uint32_t bitBuffer = 0;
int displayBitCount = 0;
unsigned long nextBitStartTime = 0;
int currentVolume = 1; // 音量初期値
RDA5807 rx;

// FreeRTOS関連
SemaphoreHandle_t i2cMutex;
TaskHandle_t taskCore0Handle;

// UBX Parser State
enum UBXState { SYNC1, SYNC2, CLASS, ID, LEN1, LEN2, PAYLOAD, CK_A, CK_B };
UBXState ubxState = SYNC1;
uint8_t msgClass, msgId;
uint16_t msgLen, payloadIndex;
uint8_t ubxPayload[256];
uint8_t expectedCkA, expectedCkB;

// 地域符号と名称の対応
struct RegionMap {
  uint16_t code;
  const char *name;
};

// リストに基づく反転後HEX
const RegionMap PREFECTURES[] = {
    {0xB2C, "地域共通"}, {0x16B, "北海道"},   {0x467, "青森県"},
    {0x5D4, "岩手県"},   {0x758, "宮城県"},   {0xAC6, "秋田県"},
    {0xE4C, "山形県"},   {0x1AE, "福島県"},   {0xC69, "茨城県"},
    {0xE38, "栃木県"},   {0x98B, "群馬県"},   {0xC96, "埼玉県"},
    {0xE31, "千葉県"},   {0x355, "東京都"},   {0x56D, "神奈川県"},
    {0x99E, "新潟県"},   {0xA71, "富山県"},   {0xD46, "石川県"},
    {0x92D, "福井県"},   {0xD4A, "山梨県"},   {0x992, "長野県"},
    {0xA65, "岐阜県"},   {0xA5A, "静岡県"},   {0x966, "愛知県"},
    {0xB70, "三重県"},   {0xCE4, "滋賀県"},   {0xB32, "京都府"},
    {0x4DB, "大阪府"},   {0xCE8, "兵庫県"},   {0xA93, "奈良県"},
    {0xE5C, "和歌山県"}, {0xD23, "鳥取県"},   {0xC66, "島根県"},
    {0xAD5, "岡山県"},   {0xB31, "広島県"},   {0xB98, "山口県"},
    {0xE62, "徳島県"},   {0x9B4, "香川県"},   {0xCE1, "愛媛県"},
    {0xB83, "高知県"},   {0xC56, "福岡県"},   {0x959, "佐賀県"},
    {0xA2B, "長崎県"},   {0x8A7, "熊本県"},   {0xC8D, "大分県"},
    {0xD1C, "宮崎県"},   {0xD45, "鹿児島県"}, {0xDC8, "沖縄県"}};

const char *getRegionName(uint16_t code) {
  for (auto &p : PREFECTURES) {
    if (p.code == code)
      return p.name;
  }
  return "未知の地域";
}

// 日時符号テーブル (5bit)
const uint8_t TABLE_DAY[] = {
    0b10000, 0b01000, 0b11000, 0b00100, 0b10100, 0b01100, 0b11100,
    0b00010, // 1-8
    0b10010, 0b01010, 0b11010, 0b00110, 0b10110, 0b01110, 0b11110,
    0b00001, // 9-16
    0b10001, 0b01001, 0b11001, 0b00101, 0b10101, 0b01101, 0b11101,
    0b00011,                                                      // 17-24
    0b10011, 0b01011, 0b11011, 0b00111, 0b10111, 0b01111, 0b11111 // 25-31
};

const uint8_t TABLE_MONTH[] = {
    0b10001, 0b01001, 0b11001, 0b00101, 0b10101, 0b01101, // 1-6
    0b11101, 0b00011, 0b10011, 0b01011, 0b11011, 0b00111  // 7-12
};

const uint8_t TABLE_HOUR[] = {
    0b00011, 0b10011, 0b01011, 0b11011,
    0b00111, 0b10111, 0b01111, 0b11111, // 0-7
    0b00001, 0b10001, 0b01001, 0b11001,
    0b00101, 0b10101, 0b01101, 0b11101, // 8-15
    0b00010, 0b10010, 0b01010, 0b11010,
    0b00110, 0b10110, 0b01110, 0b11110 // 16-23
};

// 年符号 (昭和60年=5を基準)
const uint8_t TABLE_YEAR[] = {0b00001, 0b10001, 0b01001, 0b11001, 0b00101,
                              0b10101, 0b01101, 0b11101, // 0-7 (5が基準)
                              0b00011, 0b10011, 0b01011, 0b11011, 0b00111,
                              0b10111, 0b01111, 0b11111};

int lookup(uint8_t val, const uint8_t *table, int size, int offset = 0) {
  for (int i = 0; i < size; i++) {
    if (table[i] == val)
      return i + offset;
  }
  return -1;
}

// Goertzel周波数解析
float goertzel(int *samples, float targetFreq, int numSamples) {
  float k = 0.5 + ((float)numSamples * targetFreq) / (float)SAMPLING_RATE;
  float omega = (2.0 * PI * k) / (float)numSamples;
  float coeff = 2.0 * cos(omega);
  float q0 = 0, q1 = 0, q2 = 0;
  for (int i = 0; i < numSamples; i++) {
    q0 = coeff * q1 - q2 + (float)samples[i];
    q2 = q1;
    q1 = q0;
  }
  return (q1 * q1) + (q2 * q2) - (q1 * q2 * coeff);
}

// 指定したビット幅でビット順序を反転
uint32_t reverseBits(uint32_t val, int width) {
  uint32_t res = 0;
  for (int i = 0; i < width; i++) {
    res <<= 1;
    res |= (val & 1);
    val >>= 1;
  }
  return res;
}

void setup() {
  Serial.begin(115200);

  // I2Cバス排他制御用のMutexを作成
  i2cMutex = xSemaphoreCreateMutex();

  // GPS用シリアルの初期化
  gpsSerial.begin(38400, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  // UBX-CFG-MSG: RXM-SFRBX の出力を有効化
  const uint8_t enableSFRBX[] = {
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x02, 0x13, 0x01, 0x20, 0x6C
  };
  gpsSerial.write(enableSFRBX, sizeof(enableSFRBX));
  Serial.println("Sent UBX command to enable RXM-SFRBX.");

  delay(2000); // 電源安定待機

  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.println("\n--- RDA5807 FM Radio Initializing ---");

  bool detected = false;
  for (int i = 0; i < 5; i++) {
    Wire.beginTransmission(0x10);
    if (Wire.endTransmission() == 0) {
      detected = true;
      Serial.println("RDA5807 detected!");
      break;
    }
    Serial.printf("Searching for RDA5807 (Attempt %d/5)...\n", i + 1);
    delay(500);
  }

  if (!detected) {
    Serial.println("Error: RDA5807 not found. Please check wiring or press RESET.");
  }

  // Mutexを取得してラジオを初期化
  if (xSemaphoreTake(i2cMutex, portMAX_DELAY)) {
    rx.setup();
    rx.setBand(1);
    rx.setFrequency(FM_STATION);
    rx.setVolume(currentVolume);
    rx.setMute(false);
    rx.setMono(true);
    xSemaphoreGive(i2cMutex);
  }

  analogReadResolution(12);
  Serial.printf("Tuned to: %d.%d MHz\n", FM_STATION / 100, FM_STATION % 100);
  Serial.println("EWS Decoder Ready (Core 1).");

  // 液晶初期化
  tft.init();
  tft.setRotation(1); // 横向き(ランドスケープ)に設定
  tft.fillScreen(TFT_BLACK);
  tft.println("Initializing...");
  tft.setBrightness(128);

  // スプライト（仮想画面）の作成
  canvas.createSprite(284, 76);
  canvas.setTextColor(TFT_WHITE);
  canvas.setTextSize(1);

  // LEDの初期化
  pinMode(ALERT_LED, OUTPUT);
  digitalWrite(ALERT_LED, LOW);

  // Core 0 でタスク起動
  xTaskCreatePinnedToCore(
    taskCore0,         /* 実行する関数 */
    "TaskCore0",       /* タスク名 */
    8192,              /* スタックサイズ */
    NULL,              /* パラメータ */
    1,                 /* 優先度 */
    &taskCore0Handle,  /* タスクハンドル */
    0                  /* ピン留めするコア (0: Pro Core) */
  );
  Serial.println("GPS/Command Processor Ready (Core 0).");
}

// Core 0: GPS/UBX受信 と ユーザーコマンド処理
void processCommand(char* cmd) {
  if (cmd[0] == 'v' || cmd[0] == 'V') {
    int vol = atoi(&cmd[1]);
    if (vol >= 0 && vol <= 15) {
      currentVolume = vol;
      if (xSemaphoreTake(i2cMutex, portMAX_DELAY)) {
        rx.setVolume(currentVolume);
        xSemaphoreGive(i2cMutex);
      }
      Serial.printf("Volume set to: %d\n", currentVolume);
    }
  } else if (strcmp(cmd, "status") == 0) {
    int freq = 0, rssi = 0;
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY)) {
      freq = rx.getFrequency();
      rssi = rx.getRssi();
      xSemaphoreGive(i2cMutex);
    }
    Serial.printf("Current Status: %d.%d MHz, Vol:%d, RSSI:%d\n", freq / 100, freq % 100, currentVolume, rssi);
  } else {
    float f = atof(cmd);
    if (f >= 76.0 && f <= 108.0) {
      int freqInt = (int)(f * 100);
      if (xSemaphoreTake(i2cMutex, portMAX_DELAY)) {
        rx.setFrequency(freqInt);
        xSemaphoreGive(i2cMutex);
      }
      Serial.printf("Frequency set to: %d.%d MHz\n", freqInt / 100, freqInt % 100);
    }
  }
}

uint32_t getUbxBits(const uint8_t* data, int offset, int length) {
  uint32_t result = 0;
  for (int i = 0; i < length; i++) {
    int bitPos = offset + i;
    int byteIdx = bitPos / 8;
    int bitIdx = 7 - (bitPos % 8);
    uint8_t bit = (data[byteIdx] >> bitIdx) & 1;
    result = (result << 1) | bit;
  }
  return result;
}

void taskCore0(void *pvParameters) {
  char cmdBuffer[32];
  int cmdIndex = 0;
  uint32_t lastUpdate = 0;
  
  // 前回の値を保持する変数 (差分更新用)
  int lastFreq = -1, lastRssi = -1, lastSvCount = -1, lastVol = -1;
  char lastTimeStr[10] = "";
  EwsState lastState = SEARCH_SYNC;

  int svCount = 0;
  char timeStr[10] = "--:--:--";

  char nmeaBuffer[82];
  int nmeaIndex = 80;

  for (;;) {
    uint32_t now = millis();

    // 1. 液晶描画処理 (1000ms周期 + 差分更新)
    if (now - lastUpdate > 1000) {
      lastUpdate = now;
      int freq = 0, rssi = 0;
      
      if (xSemaphoreTake(i2cMutex, (TickType_t)5)) {
        freq = rx.getFrequency();
        rssi = rx.getRssi();
        xSemaphoreGive(i2cMutex);
      }

      // 変化があったかチェック (RSSIは変動しやすいため、差が2以上の場合のみ更新)
      bool changed = (freq != lastFreq) || (abs(rssi - lastRssi) > 2) || 
                     (svCount != lastSvCount) || (currentVolume != lastVol) ||
                     (strcmp(timeStr, lastTimeStr) != 0) || (currentState != lastState);

      if (changed) {
        lastFreq = freq; lastRssi = rssi; lastSvCount = svCount;
        lastVol = currentVolume; lastState = currentState;
        strcpy(lastTimeStr, timeStr);

        canvas.fillScreen(TFT_BLACK);

        // 1行目: [タイトル] と [周波数]
        canvas.setFont(&fonts::lgfxJapanGothic_16); // 日本語ゴシック 16px
        canvas.setTextSize(1);
        canvas.setTextColor(TFT_WHITE);
        canvas.setCursor(5, 2);
        canvas.print("QZSS受信機"); 
        
        canvas.setFont(&fonts::Font0); // 数字のために標準フォントに戻す
        canvas.setTextSize(2);
        canvas.setTextColor(TFT_ORANGE);
        canvas.setCursor(155, 2);
        canvas.printf("%d.%dMHz", freq / 100, (freq % 100) / 10);

        canvas.drawFastHLine(0, 20, 284, TFT_DARKGREY);

        // 2行目: RSSIバー & 音量
        canvas.setCursor(5, 25);
        canvas.setTextColor(TFT_LIGHTGREY);
        canvas.setTextSize(1);
        canvas.print("SIG");
        int rssiBar = map(rssi, 0, 60, 0, 180);
        canvas.drawRect(35, 25, 184, 10, TFT_WHITE);
        canvas.fillRect(37, 27, constrain(rssiBar, 0, 180), 6, (rssi > 30) ? TFT_GREEN : TFT_YELLOW);
        canvas.setCursor(230, 25);
        canvas.printf("V:%d", currentVolume);

        // 3行目: GPS & QZSS 情報
        canvas.setCursor(5, 42);
        canvas.setTextColor(TFT_CYAN);
        canvas.setTextSize(1);
        canvas.printf("SATS:%d  TIME:%s", svCount, timeStr);
        
        // 4行目 (最下段): システムステータス
        canvas.fillRect(0, 58, 284, 18, (currentState == DECODE_FRAME) ? TFT_RED : 0x2104);
        canvas.setCursor(5, 61); // 枠の中央になるようY座標を少し調整
        canvas.setTextColor(TFT_WHITE);
        canvas.setFont(&fonts::lgfxJapanGothic_12); // 日本語ゴシック 12px
        canvas.setTextSize(1);
        canvas.print(currentState == DECODE_FRAME ? "警報信号を受信・解析中..." : "システム監視中 - 待機状態");
        canvas.setFont(&fonts::Font0); // 標準フォントに戻す

        canvas.pushSprite(0, 0);
      }
    }

    // 2. シリアルコマンド処理
    while (Serial.available() > 0) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') {
        if (cmdIndex > 0) {
          cmdBuffer[cmdIndex] = '\0';
          processCommand(cmdBuffer);
          cmdIndex = 0;
        }
      } else if (cmdIndex < 31) {
        cmdBuffer[cmdIndex++] = c;
      }
    }

    // 2. GPS / UBX 解析処理
    while (gpsSerial.available()) {
      uint8_t c = gpsSerial.read();

      // --- NMEA パース処理 ($GNGGA または $GPGGA) ---
      if (c == '$') {
        nmeaIndex = 0;
      }
      if (nmeaIndex < 80) {
        nmeaBuffer[nmeaIndex++] = c;
      }
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
              h = (h + 9) % 24; // JST (+9)
              sprintf(timeStr, "%02d:%02d:%02d", h, m, s);
            }
          }
        }
        nmeaIndex = 80; // 次の '$' まで記録しない
      }

      // UBXステートマシン
      switch (ubxState) {
        case SYNC1: if (c == 0xB5) ubxState = SYNC2; break;
        case SYNC2: if (c == 0x62) ubxState = CLASS; else ubxState = SYNC1; break;
        case CLASS: msgClass = c; expectedCkA = c; expectedCkB = c; ubxState = ID; break;
        case ID: msgId = c; expectedCkA += c; expectedCkB += expectedCkA; ubxState = LEN1; break;
        case LEN1: msgLen = c; expectedCkA += c; expectedCkB += expectedCkA; ubxState = LEN2; break;
        case LEN2:
          msgLen |= (c << 8); expectedCkA += c; expectedCkB += expectedCkA;
          if (msgLen > 256) ubxState = SYNC1;
          else { payloadIndex = 0; ubxState = msgLen > 0 ? PAYLOAD : CK_A; }
          break;
        case PAYLOAD:
          ubxPayload[payloadIndex++] = c; expectedCkA += c; expectedCkB += expectedCkA;
          if (payloadIndex == msgLen) ubxState = CK_A;
          break;
        case CK_A: if (c == expectedCkA) ubxState = CK_B; else ubxState = SYNC1; break;
        case CK_B:
          if (c == expectedCkB) {
            if (msgClass == 0x02 && msgId == 0x13) {
              uint8_t gnssId = ubxPayload[0];
              uint8_t svId = ubxPayload[1];
              // デバッグログが多すぎるため、通常パケットの受信通知はコメントアウト
              // Serial.printf("\n[UBX] RXM-SFRBX Received! GNSS: %d, SV: %d\n", gnssId, svId);
              if (gnssId == 5) {
                uint8_t numWords = ubxPayload[4];
                if (numWords == 8) {
                  uint8_t l1s_msg[32];
                  // 8ワード(256bit)をビッグエンディアン配列に変換
                  for (int i = 0; i < 8; i++) {
                    uint32_t w = ubxPayload[8 + i * 4] | (ubxPayload[9 + i * 4] << 8) |
                                 (ubxPayload[10 + i * 4] << 16) | (ubxPayload[11 + i * 4] << 24);
                    l1s_msg[i * 4 + 0] = (w >> 24) & 0xFF;
                    l1s_msg[i * 4 + 1] = (w >> 16) & 0xFF;
                    l1s_msg[i * 4 + 2] = (w >> 8) & 0xFF;
                    l1s_msg[i * 4 + 3] = w & 0xFF;
                  }

                  uint32_t preamble = getUbxBits(l1s_msg, 0, 8);
                  uint32_t mt = getUbxBits(l1s_msg, 8, 6);

                  if (preamble == 0x53) {
                    if (mt == 43) {
                      Serial.println("\n=============================================");
                      Serial.println(">>> 🚨 QZSS 災危通報 (MT43) 受信! 🚨 <<<");
                      uint32_t reportClass = getUbxBits(l1s_msg, 14, 3);
                      uint32_t disasterCat = getUbxBits(l1s_msg, 17, 4);
                      uint32_t month = getUbxBits(l1s_msg, 21, 4);
                      uint32_t day   = getUbxBits(l1s_msg, 25, 5);
                      uint32_t hour  = getUbxBits(l1s_msg, 30, 5);
                      uint32_t min   = getUbxBits(l1s_msg, 35, 6);
                      
                      const char* rcStr = "不明";
                      if (reportClass == 1) rcStr = "緊急 (Maximum)";
                      else if (reportClass == 2) rcStr = "優先 (Prior)";
                      else if (reportClass == 3) rcStr = "通常 (Normal)";
                      else if (reportClass == 7) rcStr = "訓練/テスト (Test)";

                      const char* dcStr = "不明";
                      if (disasterCat == 1) dcStr = "地震動 (Earthquake/EEW)";
                      else if (disasterCat == 2) dcStr = "津波 (Tsunami)";
                      else if (disasterCat == 8) dcStr = "弾道ミサイル (Missile)";
                      else if (disasterCat == 14) dcStr = "その他の情報 (Periodic Heartbeat)";

                      Serial.printf("区分: %d - %s\n", reportClass, rcStr);
                      Serial.printf("災害種別: %d - %s\n", disasterCat, dcStr);
                      Serial.printf("発表日時: %d月%d日 %02d:%02d\n", month, day, hour, min);
                      
                      // 41ビット目以降の可変長ペイロードをHexダンプ (約26バイト)
                      Serial.print("ペイロード(Hex): ");
                      for(int b = 41; b < 250; b += 8) {
                        int len = (250 - b >= 8) ? 8 : (250 - b);
                        Serial.printf("%02X ", getUbxBits(l1s_msg, b, len));
                      }
                      Serial.println();

                      switch (disasterCat) {
                        case 1: { // 地震動 (Earthquake)
                           uint32_t infoType = getUbxBits(l1s_msg, 41, 1);
                           if (infoType == 1) {
                              Serial.println(">> 情報タイプ: 緊急地震速報 (EEW)");
                              // TODO: 震度、震央、マグニチュードなどのデコード
                           } else {
                              Serial.println(">> 情報タイプ: 震度速報");
                           }
                           break;
                        }
                        case 2: // 津波
                           Serial.println(">> 情報タイプ: 津波警報・注意報");
                           // TODO: 津波予報区、予想高さのデコード
                           break;
                        case 3: // 火山
                        case 4: // 降灰
                           Serial.println(">> 情報タイプ: 噴火警報・降灰予報");
                           break;
                        case 5: // 気象
                        case 6: // 洪水
                        case 7: // 土砂災害
                           Serial.println(">> 情報タイプ: 気象特別警報・洪水・土砂災害");
                           break;
                        case 8:  // 弾道ミサイル
                        case 9:  // 航空攻撃
                        case 10: // ゲリラ
                        case 11: // 大規模テロ
                           Serial.println(">> 情報タイプ: Jアラート (国民保護情報)");
                           // TODO: 対象地域コードのデコード
                           break;
                        case 14: // その他の情報 / 死活監視
                           Serial.println(">> (平常時の定期通信パケットです)");
                           break;
                        default:
                           Serial.println(">> 未知・その他の情報タイプ");
                           break;
                      }
                      Serial.println("=============================================\n");
                    } else {
                      // MT43以外は定期的に送信されるので、通常は表示しない（MT50などはSBAS等）
                      // Serial.printf("[QZSS L1S] MT: %d\n", mt);
                    }
                  }
                }
              }
            }
          }
          ubxState = SYNC1;
          break;
      }
      
      // NMEAパススルー表示（ログが流れるのを防ぐためコメントアウト）
      /*
      if (ubxState == SYNC1 && (c == '$' || c == '\n' || c == '\r' || (c >= 32 && c <= 126))) {
        Serial.write(c);
      }
      */
    }

    // WDTリセットとCPU負荷軽減のためのスリープ (10ms)
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// Core 1: EWS (FSK) 音声解析 (メインループ)

void loop() {
  if (nextBitStartTime == 0)
    nextBitStartTime = micros();

  int samples[N];
  unsigned long startTime = nextBitStartTime;

  // 1ビット分の時間をサンプリング (約15.6ms)
  for (int i = 0; i < N; i++) {
    samples[i] = analogRead(AUDIO_IN_PIN) - 2048;
    while (micros() - startTime < (i + 1) * (1000000 / SAMPLING_RATE))
      ;
  }

  // 次のビットの開始予定時刻を更新（累積誤差を防ぐ）
  nextBitStartTime += 15625; // 1,000,000us / 64bps = 15625us

  // 周波数強度の計算 (640Hz vs 1024Hz)
  float p1024 = goertzel(samples, 1024.0, N);
  float p640 = goertzel(samples, 640.0, N);

  // ビット判定 (1 or 0)
  bool bit = (p1024 > p640);

  // ビットストリームの処理
  bitBuffer = (bitBuffer << 1) | (bit ? 1 : 0);

  switch (currentState) {
  case SEARCH_SYNC: {
    // 前置符号(4bit:1100) + 固定符号(16bit) の検出
    uint32_t pattern = bitBuffer & 0xFFFFF; // 20bit分
    uint16_t syncPart = pattern & 0xFFFF;
    uint8_t prePart = (pattern >> 16) & 0x0F;

    if ((syncPart == SYNC_TYPE_I || syncPart == SYNC_TYPE_II) &&
        (prePart == 0x0C || prePart == 0x03)) {
      isEndSignal = (prePart == 0x03);
      currentSyncType = syncPart;

      // ラジオの状態を取得 (Mutex保護)
      int rssi = 0;
      if (xSemaphoreTake(i2cMutex, (TickType_t)10)) {
        rssi = rx.getRssi();
        xSemaphoreGive(i2cMutex);
      }
      
      Serial.printf("\n[SYNC FOUND: %s %s | RSSI: %d]\n",
                    isEndSignal ? "END" : "START",
                    (syncPart == SYNC_TYPE_II) ? "Type II" : "Type I", rssi);

      currentState = DECODE_FRAME;
      bitIndex = 20;
      for (int j = 0; j < 20; j++)
        frameBits[19 - j] = (bitBuffer >> j) & 1;
    }
    break;
  }

  case DECODE_FRAME:
    frameBits[bitIndex++] = bit ? 1 : 0;

    // 100ビット溜まったら詳細解析
    if (bitIndex >= 100) {
      Serial.println("\n--- [EWS Block Decoded] ---");
      if (!isEndSignal) {
        if (currentSyncType == SYNC_TYPE_II)
          Serial.println("【 津波警報：開始 】");
        else
          Serial.println("【 第一種：開始（地震・要請） 】");
      } else {
        Serial.println("【 終了信号（警報解除） 】");
      }

      auto getBlock = [&](int start) {
        uint16_t val = 0;
        for (int i = 0; i < 16; i++)
          val = (val << 1) | frameBits[start + i];
        return val;
      };

      uint16_t areaCode = getBlock(20);
      uint16_t dateCode = getBlock(52);
      uint16_t timeCode = getBlock(84);

      // 各項目のデコード処理
      uint16_t areaData = reverseBits((areaCode >> 2) & 0x0FFF, 12);
      Serial.printf("地域: %03X (%s)\n", areaData, getRegionName(areaData));

      int day = lookup((dateCode >> 8) & 0x1F, TABLE_DAY, 31, 1);
      int month = lookup(((dateCode >> 3) & 0x0F) << 1 | 1, TABLE_MONTH, 12, 1);
      bool dateFlag = (dateCode >> 7) & 1;
      Serial.printf("日付: %d月 %d日 (%s)\n", month, day,
                    dateFlag ? "前日/翌日" : "当日");

      int hour = lookup((timeCode >> 8) & 0x1F, TABLE_HOUR, 24, 0);
      int year_digit = reverseBits((timeCode >> 3) & 0x0F, 4);
      if (year_digit == 10)
        year_digit = 0;
      bool timeFlag = (timeCode >> 7) & 1;

      int year_full = 2020 + year_digit;
      if (year_full > 2029)
        year_full -= 10;
      if (year_full < 2020 && year_digit > 5)
        year_full -= 10;

      Serial.printf("時刻: %d時台 (%s) / 年: %d年\n", hour,
                    timeFlag ? "前後1時間" : "現時", year_full);
      Serial.println("---------------------------");

      if (!isEndSignal || bitIndex >= 192) {
        currentState = SEARCH_SYNC;
        bitIndex = 0;
        bitBuffer = 0;
      }
    }
    break;
  }

  // デバッグ：信号強度がある程度ある時だけビットをドットで表示
  if (p1024 > SIGNAL_THRESHOLD || p640 > SIGNAL_THRESHOLD) {
    // 0連続を防ぐため、通常時は出さず同期発見時のみ表示
  }
}
