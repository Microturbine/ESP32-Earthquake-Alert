#ifndef QZSS_PARSER_H
#define QZSS_PARSER_H

#include <Arduino.h>

class QzssParser {
public:
    QzssParser();
    void parseUbx(uint8_t c);
    
    // Status accessors
    int getQzssState(); // 0: None, 1: Test, 2: Real Alert
    const char* getAlertText();
    void updateTimeouts(uint32_t now);

private:
    enum UBXState { SYNC1, SYNC2, CLASS, ID, LEN1, LEN2, PAYLOAD, CK_A, CK_B };
    UBXState ubxState;
    uint8_t msgClass, msgId;
    uint16_t msgLen, payloadIndex;
    uint8_t ubxPayload[256];
    uint8_t expectedCkA, expectedCkB;

    int qzssState;
    uint32_t qzssTimeout;
    char alertText[128];

    void processRxmSfrbx();
    void decodeMT43(const uint8_t* l1s_msg);
    void decodeMT44(const uint8_t* l1s_msg);
    uint32_t getUbxBits(const uint8_t* data, int offset, int length);
};

extern QzssParser qzssParser;

#endif
