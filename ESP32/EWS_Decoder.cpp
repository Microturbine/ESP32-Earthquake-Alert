#include "EWS_Decoder.h"

EwsDecoder ewsDecoder;

// アナログEWS仕様に基づく定数
const uint16_t SYNC_TYPE_I = 0x0E6D;
const uint16_t SYNC_TYPE_II = 0xF192;

#define SAMPLING_RATE 8000
#define N 125                   
#define SIGNAL_THRESHOLD 130000

// 地域符号と名称の対応
struct RegionMap {
  uint16_t code;
  const char *name;
};

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

const char* EwsDecoder::getRegionName(uint16_t code) {
  for (auto &p : PREFECTURES) {
    if (p.code == code)
      return p.name;
  }
  return "未知の地域";
}

const uint8_t TABLE_DAY[] = {
    0b10000, 0b01000, 0b11000, 0b00100, 0b10100, 0b01100, 0b11100,
    0b00010, // 1-8
    0b10010, 0b01010, 0b11010, 0b00110, 0b10110, 0b01110, 0b11110,
    0b00001, // 9-16
    0b10001, 0b01001, 0b11001, 0b00101, 0b10101, 0b01101, 0b11101,
    0b00011, // 17-24
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

int EwsDecoder::lookup(uint8_t val, const uint8_t *table, int size, int offset) {
  for (int i = 0; i < size; i++) {
    if (table[i] == val)
      return i + offset;
  }
  return -1;
}

float EwsDecoder::goertzel(int *samples, float targetFreq, int numSamples) {
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

uint32_t EwsDecoder::reverseBits(uint32_t val, int width) {
  uint32_t res = 0;
  for (int i = 0; i < width; i++) {
    res <<= 1;
    res |= (val & 1);
    val >>= 1;
  }
  return res;
}

void EwsDecoder::init(int i2c_sda, int i2c_scl, int audio_pin) {
    audioInPin = audio_pin;
    currentState = SEARCH_SYNC;
    nextBitStartTime = 0;
    bitBuffer = 0;
    bitIndex = 0;
    isEndSignal = false;
    currentSyncType = 0;

    i2cMutex = xSemaphoreCreateMutex();
    Wire.begin(i2c_sda, i2c_scl);
    
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY)) {
        rx.setup();
        rx.setBand(1);
        rx.setMute(false);
        rx.setMono(true);
        xSemaphoreGive(i2cMutex);
    }
    analogReadResolution(12);
}

void EwsDecoder::setFrequency(int freq) {
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY)) {
        rx.setFrequency(freq);
        xSemaphoreGive(i2cMutex);
    }
}

void EwsDecoder::setVolume(int vol) {
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY)) {
        rx.setVolume(vol);
        xSemaphoreGive(i2cMutex);
    }
}

int EwsDecoder::getFrequency() {
    int f = 0;
    if (xSemaphoreTake(i2cMutex, (TickType_t)5)) {
        f = rx.getFrequency();
        xSemaphoreGive(i2cMutex);
    }
    return f;
}

int EwsDecoder::getRssi() {
    int r = 0;
    if (xSemaphoreTake(i2cMutex, (TickType_t)5)) {
        r = rx.getRssi();
        xSemaphoreGive(i2cMutex);
    }
    return r;
}

EwsState EwsDecoder::getState() {
    return currentState;
}

void EwsDecoder::processAudio() {
    if (nextBitStartTime == 0)
        nextBitStartTime = micros();

    int samples[N];
    unsigned long startTime = nextBitStartTime;

    for (int i = 0; i < N; i++) {
        samples[i] = analogRead(audioInPin) - 2048;
        while (micros() - startTime < (i + 1) * (1000000 / SAMPLING_RATE));
    }

    nextBitStartTime += 15625;

    float p1024 = goertzel(samples, 1024.0, N);
    float p640 = goertzel(samples, 640.0, N);

    bool bit = (p1024 > p640);
    bitBuffer = (bitBuffer << 1) | (bit ? 1 : 0);

    switch (currentState) {
    case SEARCH_SYNC: {
        uint32_t pattern = bitBuffer & 0xFFFFF;
        uint16_t syncPart = pattern & 0xFFFF;
        uint8_t prePart = (pattern >> 16) & 0x0F;

        if ((syncPart == SYNC_TYPE_I || syncPart == SYNC_TYPE_II) &&
            (prePart == 0x0C || prePart == 0x03)) {
            isEndSignal = (prePart == 0x03);
            currentSyncType = syncPart;
            currentState = DECODE_FRAME;
            bitIndex = 20;
            for (int j = 0; j < 20; j++)
                frameBits[19 - j] = (bitBuffer >> j) & 1;
        }
        break;
    }

    case DECODE_FRAME:
        frameBits[bitIndex++] = bit ? 1 : 0;

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

            uint16_t areaData = reverseBits((areaCode >> 2) & 0x0FFF, 12);
            Serial.printf("地域: %03X (%s)\n", areaData, getRegionName(areaData));

            int day = lookup((dateCode >> 8) & 0x1F, TABLE_DAY, 31, 1);
            int month = lookup(((dateCode >> 3) & 0x0F) << 1 | 1, TABLE_MONTH, 12, 1);
            bool dateFlag = (dateCode >> 7) & 1;
            Serial.printf("日付: %d月 %d日 (%s)\n", month, day, dateFlag ? "前日/翌日" : "当日");

            int hour = lookup((timeCode >> 8) & 0x1F, TABLE_HOUR, 24, 0);
            int year_digit = reverseBits((timeCode >> 3) & 0x0F, 4);
            if (year_digit == 10) year_digit = 0;
            bool timeFlag = (timeCode >> 7) & 1;

            int year_full = 2020 + year_digit;
            if (year_full > 2029) year_full -= 10;
            if (year_full < 2020 && year_digit > 5) year_full -= 10;

            Serial.printf("時刻: %d時台 (%s) / 年: %d年\n", hour, timeFlag ? "前後1時間" : "現時", year_full);
            Serial.println("---------------------------");

            if (!isEndSignal || bitIndex >= 192) {
                currentState = SEARCH_SYNC;
                bitIndex = 0;
                bitBuffer = 0;
            }
        }
        break;
    }
}
