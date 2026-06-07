#include "Settings.h"

Settings settings;

Settings::Settings() {
    defaultFreq = 8520;
    volume = 1;
    myRegionCode = 0; // 0はフィルタなし（全地域対象）
    wifiSSID = "";
    wifiPassword = "";
}

void Settings::load() {
    prefs.begin("ews_config", true); // 読み取り専用
    defaultFreq = prefs.getInt("freq", 8520);
    volume = prefs.getInt("vol", 1);
    myRegionCode = prefs.getUInt("region", 0);
    wifiSSID = prefs.getString("wifi_ssid", "");
    wifiPassword = prefs.getString("wifi_pass", "");
    prefs.end();
}

void Settings::save() {
    prefs.begin("ews_config", false); // 読み書き可能
    prefs.putInt("freq", defaultFreq);
    prefs.putInt("vol", volume);
    prefs.putUInt("region", myRegionCode);
    prefs.putString("wifi_ssid", wifiSSID);
    prefs.putString("wifi_pass", wifiPassword);
    prefs.end();
}

void Settings::setFreq(int freq) {
    defaultFreq = freq;
    save();
}

void Settings::setVolume(int vol) {
    volume = vol;
    save();
}

void Settings::setRegion(uint32_t region) {
    myRegionCode = region;
    save();
}

void Settings::setWiFi(const String& ssid, const String& pass) {
    wifiSSID = ssid;
    wifiPassword = pass;
    save();
}
