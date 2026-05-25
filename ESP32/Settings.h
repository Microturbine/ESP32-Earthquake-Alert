#ifndef SETTINGS_H
#define SETTINGS_H

#include <Preferences.h>

class Settings {
public:
    int defaultFreq;
    int volume;
    uint32_t myRegionCode; // フィルタ用気象庁地域コード

    Settings();
    void load();
    void save();
    void setFreq(int freq);
    void setVolume(int vol);
    void setRegion(uint32_t region);
    
private:
    Preferences prefs;
};

extern Settings settings;

#endif
