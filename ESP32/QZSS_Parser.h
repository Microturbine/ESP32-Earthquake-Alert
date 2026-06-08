#ifndef QZSS_PARSER_H
#define QZSS_PARSER_H

#include <Arduino.h>

class QzssParser {
public:
    QzssParser();
    void parseUbx(uint8_t c);
    
    // ステータス取得用
    int getQzssState(); // 0:なし, 1:試験, 2:災害警報
    const char* getAlertText();
    void updateTimeouts(uint32_t now);
    void decodeMT43(const uint8_t* l1s_msg);
    void decodeMT44(const uint8_t* l1s_msg);
    void resetAlert();
    
    // GPS & みちびき デバッグ用統計情報
    uint32_t sfrbxCount;
    uint32_t mt43Count;
    uint32_t mt44Count;
    uint32_t lastL1sTime;
    char lastL1sHex[65];

private:
    enum UBXState { SYNC1, SYNC2, CLASS, ID, LEN1, LEN2, PAYLOAD, CK_A, CK_B };
    UBXState ubxState;
    uint8_t msgClass, msgId;
    uint16_t msgLen, payloadIndex;
    uint8_t ubxPayload[256];
    uint8_t expectedCkA, expectedCkB;

    int qzssState;
    uint32_t qzssTimeout;
    char alertText[256];

    void processRxmSfrbx();
    uint32_t getUbxBits(const uint8_t* data, int offset, int length);
    uint64_t getUbxBits64(const uint8_t* data, int offset, int length);
};

extern QzssParser qzssParser;

#endif
