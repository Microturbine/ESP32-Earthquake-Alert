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
    bool isOutOfRegion;
    double ellipseMajor;   // in km (0.0 means none)
    double ellipseMinor;   // in km
    double ellipseAzimuth; // in degrees (-90 to 90)
};

struct HistoryAlert {
    char text[256];
    bool isTest;
    bool isOutOfRegion;
    uint32_t receivedMillis;
    char receivedTimeStr[10]; // e.g. "12:34:56" or "No GPS"
};

class AlertManager {
public:
    AlertManager();
    
    // 警報を追加する。新たに追加された場合はtrueを返す
    bool addAlert(const char* text, uint32_t timeoutMs, bool isTest,
                  double latitude = 0.0, double longitude = 0.0,
                  int disasterCat = 0, int code = 0, uint64_t prefMask = 0,
                  bool isOutOfRegion = false,
                  double ellipseMajor = 0.0, double ellipseMinor = 0.0, double ellipseAzimuth = 0.0);
    
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
    static const int MAX_ALERTS = 16;
    
    // 警報履歴リストをスレッドセーフにコピーする
    int copyHistory(HistoryAlert* dest, int maxCount);
    static const int MAX_HISTORY = 10;

private:
    Alert alerts[MAX_ALERTS];
    int alertCount;
    
    HistoryAlert history[MAX_HISTORY];
    int historyCount;
    
    bool newAlertFlag;
    bool changedFlag;
    
    SemaphoreHandle_t mutex;

    bool hasRealAlertInternal() const;
};

extern AlertManager alertManager;

#endif // ALERT_MANAGER_H
