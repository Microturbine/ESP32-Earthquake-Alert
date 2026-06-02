#ifndef ALERT_MANAGER_H
#define ALERT_MANAGER_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

struct Alert {
    char text[128];
    uint32_t expiry;
    bool isTest;
};

class AlertManager {
public:
    AlertManager();
    
    // 警報を追加する。新たに追加された場合はtrueを返す
    bool addAlert(const char* text, uint32_t timeoutMs, bool isTest);
    
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
