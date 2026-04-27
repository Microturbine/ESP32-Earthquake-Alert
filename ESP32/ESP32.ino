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

// --- 設定 ---
#define FM_STATION 8250 // 受信周波数 (NHK-FMなど)
#define AUDIO_IN_PIN 4
#define SAMPLING_RATE 8000
#define N 125                   // 8000Hz / 125 = 64Hz (1ビットの時間に相当)
#define SIGNAL_THRESHOLD 130000 // 判定しきい値（ノイズが多い場合は大きくする）

// アナログEWS仕様に基づく定数
const uint16_t SYNC_TYPE_I = 0x0E6D;  // 0000 1110 0110 1101
const uint16_t SYNC_TYPE_II = 0xF192; // 1111 0001 1001 0010

// --- チャイム検知用設定 ---
const float CHIME_CHORD1[] = {1046.5, 1318.5, 1568.0}; // 和音1: C6, E6, G6
const float CHIME_CHORD2[] = {1108.7, 1396.9, 1760.0}; // 和音2: C#6, F6, A6
#define CHIME_THRESHOLD 150000.0                       // チャイム判定しきい値
#define CHIME_MIN_COUNT 15 // 約0.23秒相当 (15.6ms * 15)
#define CHIME_MAX_GAP 10   // 和音間の最大許容隙間 (約0.15秒)

enum ChimeState { CHIME_IDLE, CHIME_DETECTING_C1, CHIME_DETECTING_C2 };
ChimeState currentChimeState = CHIME_IDLE;
int chimeCount = 0;
int gapCount = 0;

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
RDA5807 rx;

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

// 指定したビット幅でビット順序を反転させるヘルパー関数
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
  Wire.begin();
  rx.setup();
  rx.setVolume(10);
  rx.setFrequency(FM_STATION);
  analogReadResolution(12);
  Serial.println("EWS Decoder Ready. Monitoring...");
}

void loop() {
  if (nextBitStartTime == 0)
    nextBitStartTime = micros();

  int samples[N];
  unsigned long startTime = nextBitStartTime;

  // 1ビット分の時間をサンプリング (約15.6ms)
  // バイアス回路により、無音時に analogRead が約2048 (1.65V) になることを想定
  for (int i = 0; i < N; i++) {
    samples[i] = analogRead(AUDIO_IN_PIN) - 2048;
    while (micros() - startTime < (i + 1) * (1000000 / SAMPLING_RATE))
      ;
  }

  // 次のビットの開始予定時刻を更新（累積誤差を防ぐ）
  nextBitStartTime += 15625; // 1,000,000us / 64bps = 15625us

  // 周波数強度の計算
  float p1024 = goertzel(samples, 1024.0, N);
  float p640 = goertzel(samples, 640.0, N);

  // チャイム周波数の計算
  bool c1_active = true, c2_active = true;
  for (int i = 0; i < 3; i++) {
    float p1 = goertzel(samples, CHIME_CHORD1[i], N);
    if (p1 < CHIME_THRESHOLD)
      c1_active = false;
    float p2 = goertzel(samples, CHIME_CHORD2[i], N);
    if (p2 < CHIME_THRESHOLD)
      c2_active = false;
  }

  // チャイム検知ロジック (State Machine)
  switch (currentChimeState) {
  case CHIME_IDLE:
    if (c1_active) {
      chimeCount++;
      if (chimeCount >= CHIME_MIN_COUNT) {
        currentChimeState = CHIME_DETECTING_C1;
        chimeCount = 0;
        gapCount = 0;
        Serial.println("\n[CHIME: Chord 1 Detected]");
      }
    } else {
      chimeCount = 0;
    }
    break;

  case CHIME_DETECTING_C1:
    if (c2_active) {
      chimeCount++;
      if (chimeCount >= CHIME_MIN_COUNT) {
        Serial.println("\n************************************");
        Serial.println("【 緊急地震速報（チャイム検知）！！ 】");
        Serial.println("************************************");
        currentChimeState = CHIME_IDLE;
        chimeCount = 0;
      }
    } else if (c1_active) {
      // 継続中
    } else {
      gapCount++;
      if (gapCount > CHIME_MAX_GAP) {
        currentChimeState = CHIME_IDLE;
        chimeCount = 0;
      }
    }
    break;
  }

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
      Serial.printf("\n[SYNC FOUND: %s %s]\n", isEndSignal ? "END" : "START",
                    (syncPart == SYNC_TYPE_II) ? "Type II" : "Type I");
      currentState = DECODE_FRAME;
      bitIndex = 20;
      for (int j = 0; j < 20; j++)
        frameBits[19 - j] = (bitBuffer >> j) & 1;
    }
    break;
  }

  case DECODE_FRAME:
    frameBits[bitIndex++] = bit ? 1 : 0;

    // 100ビット（前置4 + 固定・データ96）溜まったら詳細解析
    // ※終了信号の場合、この後192ビットまで無信号が続くが、データは100ビット目までで抽出可能
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

      // 地域符号の解析
      uint16_t areaData = reverseBits((areaCode >> 2) & 0x0FFF, 12);
      Serial.printf("地域: %03X (%s)\n", areaData, getRegionName(areaData));

      // 月日の分解 (テーブル照合)
      uint8_t day_bits = (dateCode >> 8) & 0x1F;
      uint8_t month_bits =
          ((dateCode >> 2) & 0x1E) | ((dateCode >> 7) & 1); // 予備含め5bit抽出
      // 実際には月は下位4bit+予備のようなので、提供テーブルに合わせて調整
      uint8_t month_val_bits = (dateCode >> 3) & 0x0F;
      bool dateFlag = (dateCode >> 7) & 1;

      int day = lookup(day_bits, TABLE_DAY, 31, 1);
      int month = lookup((month_val_bits << 1) | 1, TABLE_MONTH, 12,
                         1); // 予備bit=1を付加して照合

      Serial.printf("日付: %d月 %d日 (%s)\n", month, day,
                    dateFlag ? "前日/翌日" : "当日");

      // 年時の分解 (テーブル照合)
      uint8_t hour_bits = (timeCode >> 8) & 0x1F;
      uint8_t year_val_bits = (timeCode >> 3) & 0x0F;
      bool timeFlag = (timeCode >> 7) & 1;

      int hour = lookup(hour_bits, TABLE_HOUR, 24, 0);

      // 年符号の解析 (下1桁方式: 5=5年, 10=0年)
      int year_digit = reverseBits(year_val_bits, 4);
      if (year_digit == 10)
        year_digit = 0;

      // 現在の年代 (2020年代) を基準に西暦を合成
      int year_full = 2020 + year_digit;
      // もし2013年のような過去の信号を扱う場合のための自動調整
      // (2025年にいると想定)
      if (year_full > 2029)
        year_full -= 10;
      if (year_full < 2020 && year_digit > 5)
        year_full -= 10; // 2013年等の対応

      Serial.printf("時刻: %d時台 (%s) / 年: %d年\n", hour,
                    timeFlag ? "前後1時間" : "現時", year_full);
      Serial.println("---------------------------");

      // 終了信号の場合は 192ビットまで待機してから SEARCH_SYNC に戻る
      if (!isEndSignal || bitIndex >= 192) {
        currentState = SEARCH_SYNC;
        bitIndex = 0;
        bitBuffer = 0;
      }
    }
    break;
  }

  // ドット表示（生存確認用）
  if (p1024 > SIGNAL_THRESHOLD || p640 > SIGNAL_THRESHOLD) {
    Serial.print(bit ? "1" : "0");
    if (++displayBitCount >= 64) {
      Serial.println();
      displayBitCount = 0;
    }
  }
}
