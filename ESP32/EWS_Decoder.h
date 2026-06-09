#ifndef EWS_DECODER_H
#define EWS_DECODER_H

#include <Arduino.h>
#include <RDA5807.h>
#include <Wire.h>

enum EwsState { SEARCH_SYNC, DECODE_FRAME };

class EwsDecoder {
public:
    void init(int i2c_sda, int i2c_scl, int audio_pin);
    void processAudio();
    void setFrequency(int freq);
    void setVolume(int vol);
    int getFrequency();
    int getRssi();
    EwsState getState();
    void resetState();
    
    // アラート保持・表示用
    int getEwsAlertState(); // 0:なし, 1:警報
    String getEwsAlertText();
    void resetAlert();
    void updateTimeouts(uint32_t now);
    
    // 消音制御
    void setMute(bool mute);
    bool getMute();
    
private:
    SemaphoreHandle_t i2cMutex;
    SemaphoreHandle_t stateMutex;
    RDA5807 rx;
    int audioInPin;
    EwsState currentState;
    unsigned long nextBitStartTime;
    uint32_t bitBuffer;
    int bitIndex;
    uint8_t frameBits[100];
    bool isEndSignal;
    uint16_t currentSyncType;
    unsigned long lastEwsActivityTime;
    
    // アラート保持・表示用
    int ewsAlertState;
    char ewsAlertText[64];
    unsigned long ewsAlertTimeout;
    
    int cachedFreq;
    int cachedRssi;
    bool manualUnmute;
    bool isMuted;
    
    float goertzel(int *samples, float targetFreq, int numSamples);
    uint32_t reverseBits(uint32_t val, int width);
    const char* getRegionName(uint16_t code);
    int lookup(uint8_t val, const uint8_t *table, int size, int offset);
    bool decodeBCH3216(uint32_t &codeword);
    uint16_t getPrefectureCodeFromEwsArea(uint16_t areaData);
};

extern EwsDecoder ewsDecoder;

#endif
