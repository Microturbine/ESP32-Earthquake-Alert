#include "QZSS_Parser.h"
#include "Settings.h"
#include "QZSS_Tables.h"
#include "AlertManager.h"
#include <stdio.h>
#include <string.h>

static bool isPrefectureAffected(const char* regName, const char* prefName) {
    if (!regName || !prefName) return false;
    char prefClean[32];
    strncpy(prefClean, prefName, sizeof(prefClean) - 1);
    prefClean[sizeof(prefClean) - 1] = '\0';
    int len = strlen(prefClean);
    if (len > 3) {
        if (strcmp(&prefClean[len - 3], "県") == 0 ||
            strcmp(&prefClean[len - 3], "府") == 0 ||
            strcmp(&prefClean[len - 3], "都") == 0) {
            prefClean[len - 3] = '\0';
        }
    }
    return (strstr(regName, prefClean) != nullptr);
}

QzssParser qzssParser;

QzssParser::QzssParser() {
    ubxState = SYNC1;
    qzssState = 0;
    qzssTimeout = 0;
    alertText[0] = '\0';
    sfrbxCount = 0;
    mt43Count = 0;
    mt44Count = 0;
    lastL1sTime = 0;
    strcpy(lastL1sHex, "None");
    qzssMutex = xSemaphoreCreateMutex();
}

int QzssParser::getQzssState() {
    int s = 0;
    if (xSemaphoreTake(qzssMutex, portMAX_DELAY) == pdTRUE) {
        s = qzssState;
        xSemaphoreGive(qzssMutex);
    }
    return s;
}

String QzssParser::getAlertText() {
    String text = "";
    if (xSemaphoreTake(qzssMutex, portMAX_DELAY) == pdTRUE) {
        text = alertText;
        xSemaphoreGive(qzssMutex);
    }
    return text;
}

String QzssParser::getLastL1sHex() {
    String hex = "";
    if (xSemaphoreTake(qzssMutex, portMAX_DELAY) == pdTRUE) {
        hex = lastL1sHex;
        xSemaphoreGive(qzssMutex);
    }
    return hex;
}

void QzssParser::updateTimeouts(uint32_t now) {
    if (xSemaphoreTake(qzssMutex, portMAX_DELAY) == pdTRUE) {
        if (qzssState > 0 && qzssTimeout > 0 && (int32_t)(now - qzssTimeout) > 0) {
            qzssState = 0;
            alertText[0] = '\0';
        }
        xSemaphoreGive(qzssMutex);
    }
}

void QzssParser::resetAlert() {
    if (xSemaphoreTake(qzssMutex, portMAX_DELAY) == pdTRUE) {
        qzssState = 0;
        alertText[0] = '\0';
        xSemaphoreGive(qzssMutex);
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

uint64_t QzssParser::getUbxBits64(const uint8_t* data, int offset, int length) {
    uint64_t result = 0;
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
    case CLASS:
        msgClass = c;
        expectedCkA = c;                // CK_A = 0 + msgClass
        expectedCkB = expectedCkA;      // CK_B = 0 + CK_A
        ubxState = ID;
        break;
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
            if (msgClass == 0x02 && msgId == 0x13) { // RXM-SFRBXメッセージ
                processRxmSfrbx();
            }
        }
        ubxState = SYNC1;
        break;
    }
}

void QzssParser::processRxmSfrbx() {
    uint8_t gnssId = ubxPayload[0];
    if (gnssId == 5) { // 準天頂衛星(QZSS)
        sfrbxCount++;
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

            if (preamble == 0x53 || preamble == 0x9A || preamble == 0xC6) {
                lastL1sTime = millis();
                // Store hex (スレッドセーフに保護して書き込み)
                if (xSemaphoreTake(qzssMutex, portMAX_DELAY) == pdTRUE) {
                    char* p = lastL1sHex;
                    for (int i = 0; i < 32; i++) {
                        sprintf(p, "%02X", l1s_msg[i]);
                        p += 2;
                    }
                    *p = '\0';
                    xSemaphoreGive(qzssMutex);
                }

                if (mt == 43) {
                    mt43Count++;
                    decodeMT43(l1s_msg);
                } else if (mt == 44) {
                    mt44Count++;
                    decodeMT44(l1s_msg);
                }
            }
        }
    }
}

static const char* getIntensityName(uint32_t code) {
    switch (code) {
        case 1: return "0";
        case 2: return "1";
        case 3: return "2";
        case 4: return "3";
        case 5: return "4";
        case 6: return "5弱";
        case 7: return "5強";
        case 8: return "6弱";
        case 9: return "6強";
        case 10: return "7";
        case 11: return "~程度以上";
        case 14: return "なし";
        default: return "不明";
    }
}

void QzssParser::decodeMT43(const uint8_t* l1s_msg) {
    uint32_t reportClass = getUbxBits(l1s_msg, 14, 3);
    uint32_t disasterCat = getUbxBits(l1s_msg, 17, 4);
    
    bool isOutOfRegion = false;
    const char* myPrefName = nullptr;
    if (settings.myRegionCode != 0) {
        myPrefName = getPrefectureJisName(settings.myRegionCode);
    }
    
    if (reportClass == 7) {
        if (alertManager.hasRealAlert()) {
            return;
        }
        char localText[128];
        snprintf(localText, sizeof(localText), "QZSS 訓練/試験(DC:%d)", disasterCat);
        
        if (xSemaphoreTake(qzssMutex, portMAX_DELAY) == pdTRUE) {
            qzssState = 1;
            qzssTimeout = millis() + 15000;
            strncpy(alertText, localText, sizeof(alertText) - 1);
            alertText[sizeof(alertText) - 1] = '\0';
            xSemaphoreGive(qzssMutex);
        }
        alertManager.addAlert(localText, 15000, true, 0.0, 0.0, 0, 0, 0, isOutOfRegion);
        return;
    }
    
    if (reportClass >= 1 && reportClass <= 3) {
        uint32_t it = getUbxBits(l1s_msg, 41, 2);
        if (it == 2) {
            char localText[128];
            snprintf(localText, sizeof(localText), "災害警報(DC:%d) 取消/解除", disasterCat);
            
            if (xSemaphoreTake(qzssMutex, portMAX_DELAY) == pdTRUE) {
                qzssTimeout = millis() + 60000;
                strncpy(alertText, localText, sizeof(alertText) - 1);
                alertText[sizeof(alertText) - 1] = '\0';
                xSemaphoreGive(qzssMutex);
            }
            
            if (disasterCat == 1 || disasterCat == 2 || disasterCat == 3) {
                alertManager.removeAlertsStartWith("緊急地震");
                alertManager.removeAlertsStartWith("震源");
                alertManager.removeAlertsStartWith("震度速報");
            } else if (disasterCat == 5) {
                alertManager.removeAlertsStartWith("津波警報");
                alertManager.removeAlertsStartWith("大津波警報");
                alertManager.removeAlertsStartWith("注意報/警報");
            } else if (disasterCat == 6) {
                alertManager.removeAlertsStartWith("北西太平洋津波");
            } else if (disasterCat == 8) {
                alertManager.removeAlertsStartWith("火山警報");
            } else if (disasterCat == 10) {
                alertManager.removeAlertsStartWith("気象");
            } else if (disasterCat == 11) {
                alertManager.removeAlertsStartWith("洪水");
            } else if (disasterCat == 14) {
                alertManager.removeAlertsStartWith("海上");
            }
            alertManager.addAlert(localText, 60000, false, 0.0, 0.0, 0, 0, 0, isOutOfRegion);
            return;
        }

        double latitude = 0.0;
        double longitude = 0.0;
        int code = 0;
        char localText[128] = "";

        switch (disasterCat) {
        case 1: { // 緊急地震速報 (EEW)
            uint32_t mag = getUbxBits(l1s_msg, 105, 7); // Ma
            uint32_t epi = getUbxBits(l1s_msg, 112, 10); // Ep
            uint32_t intLower = getUbxBits(l1s_msg, 122, 4); // Ll
            
            const char* epiName = getEpicenterName(epi);
            if (!epiName) epiName = "不明";
            
            snprintf(localText, sizeof(localText), "緊急地震(%s/M%.1f/震度%s)", epiName, mag/10.0, getIntensityName(intLower));
            code = epi;
            break;
        }
        case 2: { // 震源
            uint32_t depth = getUbxBits(l1s_msg, 96, 9);
            uint32_t mag = getUbxBits(l1s_msg, 105, 7);
            uint32_t epi = getUbxBits(l1s_msg, 112, 10);
            
            uint32_t latNs = getUbxBits(l1s_msg, 122, 1);
            uint32_t latD = getUbxBits(l1s_msg, 123, 7);
            uint32_t latM = getUbxBits(l1s_msg, 130, 6);
            uint32_t latS = getUbxBits(l1s_msg, 136, 6);
            uint32_t lonEw = getUbxBits(l1s_msg, 142, 1);
            uint32_t lonD = getUbxBits(l1s_msg, 143, 8);
            uint32_t lonM = getUbxBits(l1s_msg, 151, 6);
            uint32_t lonS = getUbxBits(l1s_msg, 157, 6);

            const char* epiName = getEpicenterName(epi);
            if (!epiName) epiName = "不明";

            char magStr[16];
            if (mag == 127) {
                snprintf(magStr, sizeof(magStr), "M不明");
            } else if (mag == 101) {
                snprintf(magStr, sizeof(magStr), "M10.0以上");
            } else if (mag == 126) {
                snprintf(magStr, sizeof(magStr), "M8.0以上");
            } else {
                snprintf(magStr, sizeof(magStr), "M%.1f", mag / 10.0);
            }

            char depthStr[24];
            if (depth == 511) {
                snprintf(depthStr, sizeof(depthStr), "深さ不明");
            } else if (depth == 501) {
                snprintf(depthStr, sizeof(depthStr), "深さ500km以上");
            } else {
                snprintf(depthStr, sizeof(depthStr), "深さ%dkm", depth);
            }

            snprintf(localText, sizeof(localText), "震源(%s/%s/%s/%s%d度%d分%d秒/%s%d度%d分%d秒)", 
                     epiName, magStr, depthStr, 
                     latNs ? "S" : "N", latD, latM, latS, 
                     lonEw ? "W" : "E", lonD, lonM, lonS);
            
            latitude = latD + latM / 60.0 + latS / 3600.0;
            if (latNs) latitude = -latitude;
            longitude = lonD + lonM / 60.0 + lonS / 3600.0;
            if (lonEw) longitude = -longitude;
            code = epi;
            break;
        }
        case 3: { // 震度
            uint32_t maxInt = 0;
            uint32_t maxPl = 0;
            bool affectsMyRegion = false;
            for (int i = 0; i < 16; i++) {
                uint32_t es = getUbxBits(l1s_msg, 70 + i*9, 3);
                uint32_t pl = getUbxBits(l1s_msg, 73 + i*9, 6);
                if (es > maxInt && pl > 0) {
                    maxInt = es;
                    maxPl = pl;
                }
                if (pl > 0 && settings.myRegionCode != 0 && pl == settings.myRegionCode) {
                    affectsMyRegion = true;
                }
            }
            if (maxInt > 0) {
                const char* plName = getPrefectureJisName(maxPl);
                if (!plName) plName = "各地";
                snprintf(localText, sizeof(localText), "震度速報(最大震度%s / %s等)", getIntensityName(maxInt), plName);
                code = maxPl;
                if (settings.myRegionCode != 0 && !affectsMyRegion) {
                    isOutOfRegion = true;
                }
            } else {
                snprintf(localText, sizeof(localText), "各地の震度情報");
            }
            break;
        }
        case 4: { // 南海トラフ
            uint32_t is = getUbxBits(l1s_msg, 54, 4);
            const char* infoName = "関連情報";
            if (is == 1) infoName = "調査中A";
            else if (is == 2) infoName = "調査中B";
            else if (is == 3) infoName = "調査中C";
            else if (is == 4) infoName = "巨大地震警戒";
            else if (is == 5) infoName = "巨大地震注意";
            else if (is == 6) infoName = "調査終了";
            
            snprintf(localText, sizeof(localText), "南海トラフ情報(%s)", infoName);
            code = is;
            break;
        }
        case 5: { // 津波警報
            uint32_t codeVal = getUbxBits(l1s_msg, 80, 4); // Dw
            uint32_t reg = getUbxBits(l1s_msg, 100, 10); // Pl_1
            const char* regName = getTsunamiRegionName(reg);
            if (!regName) regName = "一部沿岸";

            if (settings.myRegionCode != 0 && myPrefName) {
                if (!isPrefectureAffected(regName, myPrefName)) {
                    isOutOfRegion = true;
                }
            }

            if (codeVal == 2) {
                if (xSemaphoreTake(qzssMutex, portMAX_DELAY) == pdTRUE) {
                    qzssTimeout = millis() + 60000;
                    snprintf(alertText, sizeof(alertText), "津波警報 解除(%s等)", regName);
                    xSemaphoreGive(qzssMutex);
                }
                alertManager.removeAlertsStartWith("津波警報");
                alertManager.removeAlertsStartWith("大津波警報");
                alertManager.removeAlertsStartWith("注意報/警報");
                char cancelText[128];
                snprintf(cancelText, sizeof(cancelText), "津波警報 解除(%s等)", regName);
                alertManager.addAlert(cancelText, 60000, false, 0.0, 0.0, 0, 0, 0, isOutOfRegion);
                return;
            }

            const char* warnName = "注意報/警報";
            if (codeVal == 3) warnName = "津波警報";
            else if (codeVal == 4) warnName = "大津波警報";
            else if (codeVal == 5) warnName = "大津波警報発表";

            snprintf(localText, sizeof(localText), "%s(%s等)", warnName, regName);
            code = reg;
            break;
        }
        case 6: { // 北西太平洋津波
            uint32_t tp = getUbxBits(l1s_msg, 54, 3);
            uint32_t reg = getUbxBits(l1s_msg, 77, 7); // Pl_1
            const char* regName = getCoastalRegionName(reg);
            if (!regName) regName = "沿岸";
            
            if (settings.myRegionCode != 0 && myPrefName) {
                if (!isPrefectureAffected(regName, myPrefName)) {
                    isOutOfRegion = true;
                }
            }

            snprintf(localText, sizeof(localText), "北西太平洋津波(%s/Tp:%d)", regName, tp);
            code = reg;
            break;
        }
        case 8: { // 火山
            uint32_t dw = getUbxBits(l1s_msg, 70, 7);
            uint32_t vo = getUbxBits(l1s_msg, 77, 12);
            const char* voName = getVolcanoName(vo);
            if (!voName) voName = "付近の火山";
            
            const char* levelStr = "警報";
            if (dw == 11) levelStr = "レベル1";
            else if (dw == 12) levelStr = "レベル2";
            else if (dw == 13) levelStr = "レベル3";
            else if (dw == 14) levelStr = "レベル4";
            else if (dw == 15) levelStr = "レベル5";
            else if (dw == 52) levelStr = "噴火";
            else if (dw == 62) levelStr = "噴火確認";

            snprintf(localText, sizeof(localText), "火山警報(%s/%s)", voName, levelStr);
            code = vo;
            break;
        }
        case 9: { // 降灰
            uint32_t vo = getUbxBits(l1s_msg, 72, 12);
            const char* voName = getVolcanoName(vo);
            if (!voName) voName = "火山";
            snprintf(localText, sizeof(localText), "降灰予報(%s)", voName);
            code = vo;
            break;
        }
        case 10: { // 気象
            bool hasAlertAdded = false;
            for (int pairIdx = 0; pairIdx < 5; pairIdx++) {
                int wwOffset = 57 + pairIdx * 24;
                int plOffset = 62 + pairIdx * 24;
                
                uint32_t ww = getUbxBits(l1s_msg, wwOffset, 5);
                uint32_t pl = getUbxBits(l1s_msg, plOffset, 19);
                
                if (ww == 0 && pl == 0) {
                    continue; // 未使用スロット
                }
                
                const char* wwName = getWeatherSubCatName(ww);
                if (!wwName) wwName = "気象特別警報";
                const char* plName = getPrefecturalForecastName(pl);
                if (!plName) plName = "一部地域";
                
                bool isOutOfRegionPair = false;
                if (settings.myRegionCode != 0 && myPrefName) {
                    if (!isPrefectureAffected(plName, myPrefName)) {
                        isOutOfRegionPair = true;
                    }
                }

                char localText[128];
                snprintf(localText, sizeof(localText), "%s(%s)", wwName, plName);
                
                alertManager.addAlert(localText, 60000, false, latitude, longitude, disasterCat, pl, 0, isOutOfRegionPair);
                
                if (ww != 0) {
                    if (xSemaphoreTake(qzssMutex, portMAX_DELAY) == pdTRUE) {
                        qzssState = 2;
                        qzssTimeout = millis() + 60000;
                        strncpy(alertText, localText, sizeof(alertText) - 1);
                        alertText[sizeof(alertText) - 1] = '\0';
                        xSemaphoreGive(qzssMutex);
                    }
                    hasAlertAdded = true;
                }
            }
            if (hasAlertAdded) {
                return;
            }
            break;
        }
        case 11: { // 洪水
            uint32_t lv = getUbxBits(l1s_msg, 93, 4); // Lv_1
            uint64_t riverCode = getUbxBits64(l1s_msg, 53, 40); // Pl_1
            
            const char* riverName = getRiverName(riverCode);
            if (!riverName) riverName = "指定河川";

            if (settings.myRegionCode != 0 && myPrefName) {
                if (!isPrefectureAffected(riverName, myPrefName)) {
                    isOutOfRegion = true;
                }
            }

            if (lv == 1) {
                if (xSemaphoreTake(qzssMutex, portMAX_DELAY) == pdTRUE) {
                    qzssTimeout = millis() + 60000;
                    snprintf(alertText, sizeof(alertText), "洪水予報 解除:%s", riverName);
                    xSemaphoreGive(qzssMutex);
                }
                alertManager.removeAlertsStartWith("洪水予報");
                char cancelText[128];
                snprintf(cancelText, sizeof(cancelText), "洪水予報 解除:%s", riverName);
                alertManager.addAlert(cancelText, 60000, false, 0.0, 0.0, 0, 0, 0, isOutOfRegion);
                return;
            }

            const char* warnLevelStr = "警戒情報";
            if (lv == 2) warnLevelStr = "氾濫警戒";
            else if (lv == 3) warnLevelStr = "氾濫危険";
            else if (lv == 4) warnLevelStr = "氾濫発生";

            snprintf(localText, sizeof(localText), "洪水予報:%s(%s)", riverName, warnLevelStr);
            code = lv;
            break;
        }
        case 12: { // 台風
            uint32_t tn = getUbxBits(l1s_msg, 88, 7); // Tn (台風番号)
            uint32_t pr = getUbxBits(l1s_msg, 144, 11); // Pr (気圧)
            snprintf(localText, sizeof(localText), "台風第%d号情報(中心気圧:%dhPa)", tn, pr);
            code = tn;
            break;
        }
        case 14: { // 海上
            bool hasAlertAdded = false;
            for (int pairIdx = 0; pairIdx < 8; pairIdx++) {
                int dwOffset = 54 + pairIdx * 19;
                int plOffset = 59 + pairIdx * 19;
                
                uint32_t dw = getUbxBits(l1s_msg, dwOffset, 5); // Dw_x
                uint32_t pl = getUbxBits(l1s_msg, plOffset, 14); // Pl_x
                
                if (dw == 0 && pl == 0) {
                    continue; // 未使用スロット
                }
                
                const char* dwName = getMarineWarningName(dw);
                char dwNameBuf[32];
                if (!dwName) {
                    snprintf(dwNameBuf, sizeof(dwNameBuf), "海上警報(コード:%d)", dw);
                    dwName = dwNameBuf;
                }
                const char* plName = getMarineForecastName(pl);
                char plNameBuf[64];
                if (!plName) {
                    snprintf(plNameBuf, sizeof(plNameBuf), "海域(コード:%d)", pl);
                    plName = plNameBuf;
                }

                bool isOutOfRegionPair = false;
                if (settings.myRegionCode != 0 && myPrefName) {
                    if (!isPrefectureAffected(plName, myPrefName)) {
                        isOutOfRegionPair = true;
                    }
                }

                char localText[128];
                if (dw == 0) {
                    snprintf(localText, sizeof(localText), "海上警報解除(%s等)", plName);
                    alertManager.removeAlertsStartWith("海上");
                } else {
                    snprintf(localText, sizeof(localText), "%s(%s)", dwName, plName);
                }

                alertManager.addAlert(localText, 60000, false, latitude, longitude, disasterCat, pl, 0, isOutOfRegionPair);
                
                if (dw != 0) {
                    if (xSemaphoreTake(qzssMutex, portMAX_DELAY) == pdTRUE) {
                        qzssState = 2;
                        qzssTimeout = millis() + 60000;
                        strncpy(alertText, localText, sizeof(alertText) - 1);
                        alertText[sizeof(alertText) - 1] = '\0';
                        xSemaphoreGive(qzssMutex);
                    }
                    hasAlertAdded = true;
                }
            }
            if (hasAlertAdded) {
                return;
            }
            break;
        }
        default:
            snprintf(localText, sizeof(localText), "QZSS 災害情報 (種別:%d)", disasterCat);
            break;
        }

        if (xSemaphoreTake(qzssMutex, portMAX_DELAY) == pdTRUE) {
            qzssState = 2;
            qzssTimeout = millis() + 60000;
            strncpy(alertText, localText, sizeof(alertText) - 1);
            alertText[sizeof(alertText) - 1] = '\0';
            xSemaphoreGive(qzssMutex);
        }
        alertManager.addAlert(localText, 60000, false, latitude, longitude, disasterCat, code, 0, isOutOfRegion);
    }
}

void QzssParser::decodeMT44(const uint8_t* l1s_msg) {
    uint32_t msgType = getUbxBits(l1s_msg, 24, 2); // メッセージ種別(A1)
    uint32_t country = getUbxBits(l1s_msg, 26, 9); // 国コード(A2)
    uint32_t provider = getUbxBits(l1s_msg, 35, 5); // プロバイダ(A3)
    uint32_t hazardCat = getUbxBits(l1s_msg, 40, 7); // 災害カテゴリ(A4)
    uint32_t guidance = getUbxBits(l1s_msg, 70, 10); // 避難勧告(A11)
    uint32_t a12 = getUbxBits(l1s_msg, 80, 16); // 楕円中心緯度(A12)
    uint32_t a13 = getUbxBits(l1s_msg, 96, 17); // 楕円中心経度(A13)
    uint32_t a14 = getUbxBits(l1s_msg, 113, 5); // 長軸半径(A14)
    uint32_t a15 = getUbxBits(l1s_msg, 118, 5); // 短軸半径(A15)
    uint32_t a16 = getUbxBits(l1s_msg, 123, 6); // 方位角(A16)

    bool isOutOfRegion = false;
    if (country == 111) { // 日本国コード
        if (msgType == 0) {
            if (alertManager.hasRealAlert()) {
                return;
            }
            char localText[256] = "DCX 訓練/試験メッセージ";
            if (xSemaphoreTake(qzssMutex, portMAX_DELAY) == pdTRUE) {
                qzssState = 1;
                qzssTimeout = millis() + 15000;
                strncpy(alertText, localText, sizeof(alertText) - 1);
                alertText[sizeof(alertText) - 1] = '\0';
                xSemaphoreGive(qzssMutex);
            }
            alertManager.addAlert(localText, 15000, true, 0.0, 0.0, 0, 0, 0, isOutOfRegion);
        } else if (msgType == 1 || msgType == 2) {
            uint64_t prefMask = 0;

            // 1. 災害種別の日本語化
            const char* hazardName = getDcxHazardName(hazardCat);
            if (!hazardName) hazardName = "災害情報";

            // 2. 避難ガイダンステキストの構築 (QZSS_Tables.h の高精度定義を使用)
            String guidanceText = getDcxGuidanceText(guidance);
            char guidanceStr[128];
            if (guidanceText.length() > 0) {
                strncpy(guidanceStr, guidanceText.c_str(), sizeof(guidanceStr) - 1);
                guidanceStr[sizeof(guidanceStr) - 1] = '\0';
            } else {
                strcpy(guidanceStr, "指示なし");
            }

            // 楕円パラメータの変換 (A12-A16 が 0 でない場合)
            double latitude = 0.0;
            double longitude = 0.0;
            double ellipseMajor = 0.0;
            double ellipseMinor = 0.0;
            double ellipseAzimuth = 0.0;

            if (a12 != 0 || a13 != 0 || a14 != 0) {
                latitude = -90.0 + (180.0 / 65535.0) * a12;
                longitude = -180.0 + (360.0 / 131071.0) * a13;
                ellipseMajor = getDcxEllipseRadius(a14);
                ellipseMinor = getDcxEllipseRadius(a15);
                ellipseAzimuth = -90.0 + (180.0 / 64.0) * a16;
            }

            // 3. 対象地域の構築
            char areaStr[64] = "全国";
            if (provider == 2 || provider == 3) { // J-Alert
                uint32_t ex8 = getUbxBits(l1s_msg, 147, 1);
                uint64_t ex9 = getUbxBits64(l1s_msg, 148, 64);
                if (ex8 == 0) { // 都道府県コード
                    prefMask = ex9;
                    int count = 0;
                    areaStr[0] = '\0';
                    for (int i = 1; i <= 47; i++) {
                        uint64_t bitMask = 1ULL << (64 - i);
                        if (ex9 & bitMask) {
                            const char* prefName = getPrefectureJisName(i);
                            if (prefName) {
                                if (count > 0) strncat(areaStr, ",", sizeof(areaStr) - strlen(areaStr) - 1);
                                strncat(areaStr, prefName, sizeof(areaStr) - strlen(areaStr) - 1);
                                count++;
                                if (count >= 3) {
                                    strncat(areaStr, "等", sizeof(areaStr) - strlen(areaStr) - 1);
                                    break;
                                }
                            }
                        }
                    }
                    if (count == 0) strcpy(areaStr, "全国");
                } else { // 市区町村コード
                    int count = 0;
                    areaStr[0] = '\0';
                    for (int i = 0; i < 4; i++) {
                        uint32_t cityCode = getUbxBits(l1s_msg, 148 + i*16, 16);
                        if (cityCode > 0) {
                            uint32_t prefId = cityCode / 1000;
                            if (prefId >= 1 && prefId <= 47) {
                                prefMask |= (1ULL << (64 - prefId));
                            }
                            const char* prefName = getPrefectureJisName(prefId);
                            if (prefName) {
                                if (count > 0) strncat(areaStr, ",", sizeof(areaStr) - strlen(areaStr) - 1);
                                strncat(areaStr, prefName, sizeof(areaStr) - strlen(areaStr) - 1);
                                count++;
                                if (count >= 2) {
                                    strncat(areaStr, "等市町村", sizeof(areaStr) - strlen(areaStr) - 1);
                                    break;
                                }
                            }
                        }
                    }
                    if (count == 0) strcpy(areaStr, "一部市町村");
                }
            } else { // L-Alert / Local Government
                uint32_t ex1 = getUbxBits(l1s_msg, 147, 16);
                if (ex1 > 0) {
                    uint32_t prefId = ex1 / 1000;
                    if (prefId >= 1 && prefId <= 47) {
                        prefMask |= (1ULL << (64 - prefId));
                    }
                    const char* prefName = getPrefectureJisName(prefId);
                    if (prefName) {
                        snprintf(areaStr, sizeof(areaStr), "%s内", prefName);
                    } else {
                        strcpy(areaStr, "一部地域");
                    }
                }
            }

            // Region filtering for J-Alert/L-Alert based on prefMask
            if (settings.myRegionCode >= 1 && settings.myRegionCode <= 47 && prefMask != 0) {
                uint64_t bitMask = 1ULL << (64 - settings.myRegionCode);
                if ((prefMask & bitMask) == 0) {
                    isOutOfRegion = true;
                    Serial.printf("[QZSS] 地域フィルタ: 自地域コード %d が prefMask %08x%08x に含まれないため、警告LEDを無効にします\n",
                                  settings.myRegionCode, (uint32_t)(prefMask >> 32), (uint32_t)(prefMask & 0xFFFFFFFF));
                }
            }

            const char* typeStr = (provider == 2 || provider == 3) ? "Jアラート" : "Lアラート";
            char localText[256];
            snprintf(localText, sizeof(localText), "%s:%s(%s) %s", typeStr, hazardName, areaStr, guidanceStr);
            Serial.printf("\n[MT44] %s: %s, 対象:%s, 指示:%s\n", typeStr, hazardName, areaStr, guidanceStr);
            
            if (xSemaphoreTake(qzssMutex, portMAX_DELAY) == pdTRUE) {
                qzssState = 3;
                qzssTimeout = millis() + 60000;
                strncpy(alertText, localText, sizeof(alertText) - 1);
                alertText[sizeof(alertText) - 1] = '\0';
                xSemaphoreGive(qzssMutex);
            }
            alertManager.addAlert(localText, 60000, false, latitude, longitude, 44, hazardCat, prefMask, isOutOfRegion, ellipseMajor, ellipseMinor, ellipseAzimuth);
        } else if (msgType == 3) {
            char localText[256] = "Jアラート/Lアラート 警報解除";
            if (xSemaphoreTake(qzssMutex, portMAX_DELAY) == pdTRUE) {
                qzssState = 1;
                qzssTimeout = millis() + 60000;
                strncpy(alertText, localText, sizeof(alertText) - 1);
                alertText[sizeof(alertText) - 1] = '\0';
                xSemaphoreGive(qzssMutex);
            }
            alertManager.removeAlertsStartWith("Jアラート");
            alertManager.removeAlertsStartWith("Lアラート");
            alertManager.addAlert(localText, 60000, false, 0.0, 0.0, 0, 0, 0, isOutOfRegion);
        }
    }
}
