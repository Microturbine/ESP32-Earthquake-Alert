/*
 * ESP32 EEW Chime Detector
 * Hardware: ESP32 + RDA5807M
 * 機能: NHKの緊急地震速報チャイム音(和音)を音声解析で検知する
 */

#include <RDA5807.h>
#include <Wire.h>

// --- 設定 ---
#define FM_STATION 8250 // 受信周波数
#define AUDIO_IN_PIN 4
#define SAMPLING_RATE 8000
#define N_ANALYSIS 500 // 8000Hz / 500 = 16Hz
#define LED_PIN 2      // 内蔵LED

// --- チャイム検知用設定 ---
const float CHIME_CHORD1[] = {1046.5, 1318.5, 1568.0}; // 和音1: C6, E6, G6
const float CHIME_CHORD2[] = {1108.7, 1396.9, 1760.0}; // 和音2: C#6, F6, A6
#define CHIME_THRESHOLD 150000.0                       // 判定しきい値
#define CHIME_MIN_COUNT 3 // 約0.2秒継続 (62.5ms * 3)
#define CHIME_MAX_GAP 10  // 隙間の許容範囲 (約0.6秒)

enum ChimeState { CHIME_IDLE, CHIME_DETECTING_C1, CHIME_DETECTING_C2 };
ChimeState currentChimeState = CHIME_IDLE;
int chimeCount = 0;
int gapCount = 0;
int chimePairCount = 0;           // セット数
unsigned long lastPairTime = 0;   // 最後にセットを検知した時刻
unsigned long alertStartTime = 0; // LED点灯開始時刻
bool isLedActive = false;         // LEDが現在点灯中か
#define LED_ON_MS 5000            // 点灯継続時間

RDA5807 rx;

// Goertzel
float goertzel(int *samples, float targetFreq, int numSamples,
               bool useWindow = true) {
  float k = 0.5 + ((float)numSamples * targetFreq) / (float)SAMPLING_RATE;
  float omega = (2.0 * PI * k) / (float)numSamples;
  float coeff = 2.0 * cos(omega);
  float q0 = 0, q1 = 0, q2 = 0;

  for (int i = 0; i < numSamples; i++) {
    float s = (float)samples[i];
    if (useWindow) {
      // ハン窓の適用
      s *= 0.5f * (1.0f - cos(2.0f * PI * i / (numSamples - 1)));
    }
    q0 = coeff * q1 - q2 + s;
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
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // 初期状態は消灯
  Serial.println("EEW Chime Detector Ready.");
}

void loop() {
  int samples[N_ANALYSIS];
  unsigned long startTime = micros();

  // サンプリング (62.5ms)
  for (int i = 0; i < N_ANALYSIS; i++) {
    samples[i] = analogRead(AUDIO_IN_PIN) - 2048;
    while (micros() - startTime < (i + 1) * (1000000 / SAMPLING_RATE))
      ;
  }

  // 和音パワー計算
  float p1_total = 0, p2_total = 0;
  for (int i = 0; i < 3; i++) {
    p1_total += goertzel(samples, CHIME_CHORD1[i], N_ANALYSIS, true);
    p2_total += goertzel(samples, CHIME_CHORD2[i], N_ANALYSIS, true);
  }

  // 判定ロジック
  bool c1_active =
      (p1_total > CHIME_THRESHOLD * 3) && (p1_total > p2_total * 1.5);
  bool c2_active =
      (p2_total > CHIME_THRESHOLD * 3) && (p2_total > p1_total * 1.5);

  // デバッグ表示
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 300) {
    lastDebug = millis();
    if (p1_total > 50000 || p2_total > 50000) {
      Serial.printf("[DEBUG] C1:%s C2:%s | p1:%.0f, p2:%.0f | State:%d\n",
                    c1_active ? "ON " : "OFF", c2_active ? "ON " : "OFF",
                    p1_total, p2_total, currentChimeState);
    }
  }

  // 状態遷移
  switch (currentChimeState) {
  case CHIME_IDLE:
    if (c1_active) {
      chimeCount++;
      if (chimeCount >= CHIME_MIN_COUNT) {
        currentChimeState = CHIME_DETECTING_C1;
        chimeCount = 0;
        gapCount = 0;
        Serial.println("\n[CHIME: Phase 1 Detected (Chord 1)]");
      }
    } else {
      chimeCount = 0;
    }
    break;

  case CHIME_DETECTING_C1:
    if (c2_active) {
      chimeCount++;
      if (chimeCount >= CHIME_MIN_COUNT) {
        chimePairCount++;
        lastPairTime = millis();
        Serial.printf("\n[CHIME: Pair %d Detected]\n", chimePairCount);

        if (chimePairCount >= 2) {
          Serial.println("\n************************************");
          Serial.println("【 緊急地震速報（チャイム検知）！！ 】");
          Serial.println("************************************");
          digitalWrite(LED_PIN, HIGH); // LED点灯
          alertStartTime = millis();   // 時刻を記録
          isLedActive = true;
          chimePairCount = 0; // 発報したのでリセット
        }
        currentChimeState = CHIME_IDLE;
        chimeCount = 0;
      }
    } else if (c1_active) {
      gapCount = 0; // 継続中
    } else {
      gapCount++;
      if (gapCount > CHIME_MAX_GAP) {
        currentChimeState = CHIME_IDLE;
        chimeCount = 0;
        Serial.println("\n[CHIME: Reset (Timeout)]");
      }
    }
    break;
  }

  // 1回目検知後、2.5秒以上経っても次が来なければリセット
  if (chimePairCount > 0 && (millis() - lastPairTime > 2500)) {
    chimePairCount = 0;
    Serial.println("\n[CHIME: Pair count reset (Interval too long)]");
  }

  // LEDの自動消灯チェック
  if (isLedActive && (millis() - alertStartTime > LED_ON_MS)) {
    digitalWrite(LED_PIN, LOW);
    isLedActive = false;
    Serial.println("[SYSTEM: Alert LED turned off]");
  }
}
