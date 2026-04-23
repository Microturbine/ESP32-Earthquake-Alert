/*
 * ESP32 EWS Decoder via FM Radio
 * Hardware: ESP32 + RDA5807M
 * 
 * デコード機能:
 * 1. 64bps FSK復調 (1024Hz/640Hz)
 * 2. 同期フラグ (0xB135) の検出
 * 3. 分類コード (開始/テスト等) の判定
 */

#include <Wire.h>
#include <RDA5807.h>

// --- 設定 ---
#define FM_STATION       8250    // 受信周波数 (NHK-FMなど)
#define AUDIO_IN_PIN     34      
#define SAMPLING_RATE    8000    
#define N                125     // 8000Hz / 125 = 64Hz (1ビットの時間に相当)

// EWS定数
const uint16_t EWS_SYNC_WORD = 0xB135; // 同期フラグ

enum EwsState {
    SEARCH_PREAMBLE,
    SEARCH_SYNC,
    DECODE_DATA
};

EwsState currentState = SEARCH_PREAMBLE;
uint32_t bitBuffer = 0;
int bitCount = 0;
RDA5807 rx;

// Goertzel周波数解析
float goertzel(int* samples, float targetFreq, int numSamples) {
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
    int samples[N];
    unsigned long startTime = micros();

    // 1ビット分の時間をサンプリング (約15.6ms)
    for (int i = 0; i < N; i++) {
        samples[i] = analogRead(AUDIO_IN_PIN) - 2048;
        while (micros() - startTime < (i + 1) * (1000000 / SAMPLING_RATE));
    }

    // 周波数強度の計算
    float p1024 = goertzel(samples, 1024.0, N);
    float p640  = goertzel(samples, 640.0, N);

    // ビット判定 (1 or 0)
    bool bit = (p1024 > p640);
    
    // ビットストリームの処理
    bitBuffer = (bitBuffer << 1) | (bit ? 1 : 0);

    switch (currentState) {
        case SEARCH_PREAMBLE:
            // プリアンブル(1010...)の連続を確認（簡易的に同期信号を待つ）
            if ((bitBuffer & 0xFFFF) == EWS_SYNC_WORD) {
                Serial.println("\n[SYNC FOUND]");
                currentState = DECODE_DATA;
                bitCount = 0;
            }
            break;

        case DECODE_DATA:
            bitCount++;
            // 24ビット程度（分類コード4bit + 地域コード等）溜まったら解析
            if (bitCount >= 24) {
                uint8_t category = (bitBuffer >> 20) & 0x0F;
                Serial.print("EWS Message Received! Category: ");
                switch(category) {
                    case 0x01: Serial.println("START (Type 1)"); break;
                    case 0x02: Serial.println("TEST (Type 2)"); break;
                    case 0x03: Serial.println("END (Type 3)"); break;
                    default:   Serial.println("UNKNOWN"); break;
                }
                currentState = SEARCH_PREAMBLE;
                bitBuffer = 0;
            }
            break;
    }

    // ドット表示（生存確認用）
    if (p1024 > 50000 || p640 > 50000) {
        Serial.print(bit ? "1" : "0");
    }
}
