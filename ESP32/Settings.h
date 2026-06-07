#ifndef SETTINGS_H
#define SETTINGS_H

#include <Arduino.h>
#include <Preferences.h>

class Settings {
public:
    int defaultFreq;
    int volume;
    uint32_t myRegionCode; // フィルタ用気象庁地域コード
    String wifiSSID;
    String wifiPassword;

    Settings();
    void load();
    void save();
    void setFreq(int freq);
    void setVolume(int vol);
    void setRegion(uint32_t region);
    void setWiFi(const String& ssid, const String& pass);
    
private:
    Preferences prefs;
};

extern Settings settings;

#endif
