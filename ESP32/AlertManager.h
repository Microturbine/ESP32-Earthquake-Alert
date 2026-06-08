#ifndef ALERT_MANAGER_H
#define ALERT_MANAGER_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

struct Alert {
    char text[256];
    uint32_t expiry;
    bool isTest;
    double latitude;   // 0.0 means invalid/none
    double longitude;  // 0.0 means invalid/none
    int disasterCat;   // Disaster category code
    int code;          // Region/Epicenter/Volcano code
    uint64_t prefMask; // Affected prefectures bitmask (for MT44 etc)
};

class AlertManager {
public:
    AlertManager();
    
    // 警報を追加する。新たに追加された場合はtrueを返す
    bool addAlert(const char* text, uint32_t timeoutMs, bool isTest,
                  double latitude = 0.0, double longitude = 0.0,
                  int disasterCat = 0, int code = 0, uint64_t prefMask = 0);
    
    // 指定した文字列から始まる警報を削除する
    void removeAlertsStartWith(const char* prefix);
    
    // 期限切れの警報を削除する
    void update(uint32_t now);
    
    // すべての警報をクリアする
    void clear();
    
    // アクティブな警報があるか
    bool hasActiveAlert();
    
    // 本番の災害情報があるか
    bool hasRealAlert();
    
    // テスト信号があるか
    bool hasTestAlert();
    
    // 新規警報フラグの取得とクリア
    bool checkAndClearNewAlert();
    
    // 警報情報に変更があったかのフラグ取得とクリア
    bool checkAndClearChangedFlag();
    
    // 警報リストをスレッドセーフにコピーする
    int copyAlerts(Alert* dest, int maxCount);

private:
    static const int MAX_ALERTS = 4;
    Alert alerts[MAX_ALERTS];
    int alertCount;
    
    bool newAlertFlag;
    bool changedFlag;
    
    SemaphoreHandle_t mutex;

    bool hasRealAlertInternal() const;
};

extern AlertManager alertManager;

#endif // ALERT_MANAGER_H
