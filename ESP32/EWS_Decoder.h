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
    
    // Made public so main task can use it if needed, or we just encapsulate
    SemaphoreHandle_t i2cMutex;

private:
    RDA5807 rx;
    int audioInPin;
    EwsState currentState;
    unsigned long nextBitStartTime;
    uint32_t bitBuffer;
    int bitIndex;
    uint8_t frameBits[100];
    bool isEndSignal;
    uint16_t currentSyncType;
    
    float goertzel(int *samples, float targetFreq, int numSamples);
    uint32_t reverseBits(uint32_t val, int width);
    const char* getRegionName(uint16_t code);
    int lookup(uint8_t val, const uint8_t *table, int size, int offset);
};

extern EwsDecoder ewsDecoder;

#endif
