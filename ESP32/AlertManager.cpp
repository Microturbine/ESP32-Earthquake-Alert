#include "AlertManager.h"
#include <string.h>

AlertManager alertManager;

AlertManager::AlertManager() {
    alertCount = 0;
    newAlertFlag = false;
    changedFlag = false;
    mutex = xSemaphoreCreateMutex();
}

bool AlertManager::addAlert(const char* text, uint32_t timeoutMs, bool isTest,
                            double latitude, double longitude, int disasterCat, int code, uint64_t prefMask) {
    bool added = false;
    if (xSemaphoreTake(mutex, portMAX_DELAY)) {
        uint32_t now = millis();
        uint32_t expiry = now + timeoutMs;
        
        if (isTest) {
            // 本番警報がすでにある場合はテスト信号を無視する
            if (hasRealAlertInternal()) {
                xSemaphoreGive(mutex);
                return false;
            }
        } else {
            // 本番警報を受信した場合は、既存のテスト信号をすべてクリアする
            int writeIdx = 0;
            for (int i = 0; i < alertCount; i++) {
                if (!alerts[i].isTest) {
                    alerts[writeIdx++] = alerts[i];
                } else {
                    changedFlag = true;
                }
            }
            alertCount = writeIdx;
        }
        
        // 重複排除 (Deduplication)
        bool found = false;
        for (int i = 0; i < alertCount; i++) {
            if (strcmp(alerts[i].text, text) == 0) {
                if (expiry > alerts[i].expiry) {
                    alerts[i].expiry = expiry;
                    changedFlag = true;
                }
                alerts[i].isTest = isTest;
                alerts[i].latitude = latitude;
                alerts[i].longitude = longitude;
                alerts[i].disasterCat = disasterCat;
                alerts[i].code = code;
                alerts[i].prefMask = prefMask;
                found = true;
                break;
            }
        }
        
        if (!found) {
            // リストがいっぱいの場合は、最も早く有効期限が切れるものを削除して空きを作る
            if (alertCount >= MAX_ALERTS) {
                int earliestIdx = 0;
                uint32_t earliestExpiry = alerts[0].expiry;
                for (int i = 1; i < alertCount; i++) {
                    if (alerts[i].expiry < earliestExpiry) {
                        earliestExpiry = alerts[i].expiry;
                        earliestIdx = i;
                    }
                }
                for (int i = earliestIdx; i < alertCount - 1; i++) {
                    alerts[i] = alerts[i + 1];
                }
                alertCount--;
                changedFlag = true;
            }
            
            strncpy(alerts[alertCount].text, text, sizeof(alerts[alertCount].text) - 1);
            alerts[alertCount].text[sizeof(alerts[alertCount].text) - 1] = '\0';
            alerts[alertCount].expiry = expiry;
            alerts[alertCount].isTest = isTest;
            alerts[alertCount].latitude = latitude;
            alerts[alertCount].longitude = longitude;
            alerts[alertCount].disasterCat = disasterCat;
            alerts[alertCount].code = code;
            alerts[alertCount].prefMask = prefMask;
            alertCount++;
            added = true;
            newAlertFlag = true;
            changedFlag = true;
        }
        
        xSemaphoreGive(mutex);
    }
    return added;
}

void AlertManager::removeAlertsStartWith(const char* prefix) {
    if (xSemaphoreTake(mutex, portMAX_DELAY)) {
        int writeIdx = 0;
        int prefixLen = strlen(prefix);
        for (int i = 0; i < alertCount; i++) {
            if (strncmp(alerts[i].text, prefix, prefixLen) != 0) {
                alerts[writeIdx++] = alerts[i];
            } else {
                changedFlag = true;
            }
        }
        alertCount = writeIdx;
        xSemaphoreGive(mutex);
    }
}

void AlertManager::update(uint32_t now) {
    if (xSemaphoreTake(mutex, portMAX_DELAY)) {
        int writeIdx = 0;
        for (int i = 0; i < alertCount; i++) {
            if (alerts[i].expiry == 0 || now < alerts[i].expiry) {
                alerts[writeIdx++] = alerts[i];
            } else {
                changedFlag = true;
            }
        }
        alertCount = writeIdx;
        xSemaphoreGive(mutex);
    }
}

void AlertManager::clear() {
    if (xSemaphoreTake(mutex, portMAX_DELAY)) {
        if (alertCount > 0) {
            alertCount = 0;
            changedFlag = true;
        }
        xSemaphoreGive(mutex);
    }
}

bool AlertManager::hasActiveAlert() {
    bool active = false;
    if (xSemaphoreTake(mutex, portMAX_DELAY)) {
        active = (alertCount > 0);
        xSemaphoreGive(mutex);
    }
    return active;
}

bool AlertManager::hasRealAlert() {
    bool real = false;
    if (xSemaphoreTake(mutex, portMAX_DELAY)) {
        real = hasRealAlertInternal();
        xSemaphoreGive(mutex);
    }
    return real;
}

bool AlertManager::hasRealAlertInternal() const {
    for (int i = 0; i < alertCount; i++) {
        if (!alerts[i].isTest) {
            return true;
        }
    }
    return false;
}

bool AlertManager::hasTestAlert() {
    bool test = false;
    if (xSemaphoreTake(mutex, portMAX_DELAY)) {
        for (int i = 0; i < alertCount; i++) {
            if (alerts[i].isTest) {
                test = true;
                break;
            }
        }
        xSemaphoreGive(mutex);
    }
    return test;
}

bool AlertManager::checkAndClearNewAlert() {
    bool res = false;
    if (xSemaphoreTake(mutex, portMAX_DELAY)) {
        res = newAlertFlag;
        newAlertFlag = false;
        xSemaphoreGive(mutex);
    }
    return res;
}

bool AlertManager::checkAndClearChangedFlag() {
    bool res = false;
    if (xSemaphoreTake(mutex, portMAX_DELAY)) {
        res = changedFlag;
        changedFlag = false;
        xSemaphoreGive(mutex);
    }
    return res;
}

int AlertManager::copyAlerts(Alert* dest, int maxCount) {
    int count = 0;
    if (xSemaphoreTake(mutex, portMAX_DELAY)) {
        count = (alertCount < maxCount) ? alertCount : maxCount;
        for (int i = 0; i < count; i++) {
            dest[i] = alerts[i];
        }
        xSemaphoreGive(mutex);
    }
    return count;
}
