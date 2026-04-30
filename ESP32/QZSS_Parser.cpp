#include "QZSS_Parser.h"
#include "Settings.h"
#include <stdio.h>
#include <string.h>

QzssParser qzssParser;

QzssParser::QzssParser() {
    ubxState = SYNC1;
    qzssState = 0;
    qzssTimeout = 0;
    alertText[0] = '\0';
}

int QzssParser::getQzssState() { return qzssState; }
const char* QzssParser::getAlertText() { return alertText; }

void QzssParser::updateTimeouts(uint32_t now) {
    if (qzssState > 0 && now > qzssTimeout) {
        qzssState = 0;
        alertText[0] = '\0';
    }
}

uint32_t QzssParser::getUbxBits(const uint8_t* data, int offset, int length) {
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

void QzssParser::parseUbx(uint8_t c) {
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
            if (msgClass == 0x02 && msgId == 0x13) { // RXM-SFRBX
                processRxmSfrbx();
            }
        }
        ubxState = SYNC1;
        break;
    }
}

void QzssParser::processRxmSfrbx() {
    uint8_t gnssId = ubxPayload[0];
    if (gnssId == 5) { // QZSS
        uint8_t numWords = ubxPayload[4];
        if (numWords == 8) {
            uint8_t l1s_msg[32];
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
                    decodeMT43(l1s_msg);
                } else if (mt == 44) {
                    decodeMT44(l1s_msg);
                }
            }
        }
    }
}

void QzssParser::decodeMT43(const uint8_t* l1s_msg) {
    uint32_t reportClass = getUbxBits(l1s_msg, 14, 3);
    uint32_t disasterCat = getUbxBits(l1s_msg, 17, 4);
    
    if (reportClass == 7) {
        qzssState = 1;
        qzssTimeout = millis() + 15000;
        snprintf(alertText, sizeof(alertText), "QZSS 訓練/試験(DC:%d)", disasterCat);
    } else if (reportClass >= 1 && reportClass <= 3 && disasterCat != 14) {
        qzssState = 2;
        qzssTimeout = millis() + 30000;
        
        if (disasterCat == 1) { // EEW
            uint32_t it = getUbxBits(l1s_msg, 41, 2);
            if (it == 0) {
                uint32_t mag = getUbxBits(l1s_msg, 76, 7); // Magnitude (Ma)
                uint32_t intLower = getUbxBits(l1s_msg, 93, 4); // Intensity Lower (Ll)
                float m = mag / 10.0;
                snprintf(alertText, sizeof(alertText), "緊急地震速報(M%.1f/震度階%d)", m, intLower);
                Serial.printf("\n[MT43 EEW] Magnitude: %.1f, Intensity: %d\n", m, intLower);
            } else if (it == 2) {
                snprintf(alertText, sizeof(alertText), "緊急地震速報 取消");
            }
        } else if (disasterCat == 5) { // Tsunami
            uint32_t it = getUbxBits(l1s_msg, 41, 2);
            if (it == 0) {
                uint32_t warningCode = getUbxBits(l1s_msg, 60, 4);
                snprintf(alertText, sizeof(alertText), "津波警報 (コード:%d)", warningCode);
            } else if (it == 2) {
                snprintf(alertText, sizeof(alertText), "津波警報 解除");
            }
        } else {
            snprintf(alertText, sizeof(alertText), "QZSS 災害情報 (種別:%d)", disasterCat);
        }
    }
}

void QzssParser::decodeMT44(const uint8_t* l1s_msg) {
    // MT44 format: PAB(8), MT(6), SD(10), CAMF(122), EX(74), RSV(6), CRC(24)
    // CAMF starts at bit 24.
    uint32_t msgType = getUbxBits(l1s_msg, 24, 2); // A1
    uint32_t country = getUbxBits(l1s_msg, 26, 9); // A2
    uint32_t provider = getUbxBits(l1s_msg, 35, 5); // A3
    uint32_t hazardCat = getUbxBits(l1s_msg, 40, 7); // A4
    uint32_t severity = getUbxBits(l1s_msg, 47, 2); // A5
    uint32_t guidance = getUbxBits(l1s_msg, 70, 10); // A11

    if (country == 0b001101111) { // Japan
        if (msgType == 0) {
            qzssState = 1;
            qzssTimeout = millis() + 15000;
            snprintf(alertText, sizeof(alertText), "DCX 訓練/試験メッセージ");
        } else if (msgType == 1 || msgType == 2) { // Alert or Update
            qzssState = 3; // J-Alert / L-Alert
            qzssTimeout = millis() + 30000;
            
            if (provider == 2 || provider == 3) {
                snprintf(alertText, sizeof(alertText), "Jアラート受信! (Cat:%d)", hazardCat);
            } else if (provider == 1 || provider == 4) {
                snprintf(alertText, sizeof(alertText), "Lアラート/避難情報 (Cat:%d)", hazardCat);
            } else {
                snprintf(alertText, sizeof(alertText), "DCX 警報 (Prov:%d)", provider);
            }
            Serial.printf("\n[MT44] Provider: %d, Category: %d, Severity: %d, Guidance: %d\n", provider, hazardCat, severity, guidance);
        } else if (msgType == 3) { // All Clear
            qzssState = 1;
            qzssTimeout = millis() + 15000;
            snprintf(alertText, sizeof(alertText), "DCX 警報解除");
        }
    }
}
