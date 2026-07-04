#ifndef WEB_UI_MANAGER_H
#define WEB_UI_MANAGER_H

#include <Arduino.h>
#include <WebServer.h>
#include "AlertManager.h"

class WebUIManager {
public:
    WebUIManager();
    void init();
    void handleClient();
    String getIPAddress() const;
    String getSSID() const;
    bool isAPMode() const;

private:
    WebServer server;
    bool apMode;
    String localIP;
    String activeSSID;

    // Polling buffers to prevent heap fragmentation
    Alert statusAlerts[AlertManager::MAX_ALERTS];
    HistoryAlert statusHistAlerts[AlertManager::MAX_HISTORY];
    String jsonResponseBuf;

    void setupRoutes();
    void handleRoot();
    void handleFavicon();
    void handleGetStatus();
    void handlePostSettings();
    void handlePostTest();
    void handlePostClear();
    void handleGetUpdate();
    void handlePostUpdate();
    void setupOTA();
};

extern WebUIManager webUIManager;

#endif // WEB_UI_MANAGER_H
