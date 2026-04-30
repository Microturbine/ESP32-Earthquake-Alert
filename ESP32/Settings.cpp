#include "Settings.h"

Settings settings;

Settings::Settings() {
    defaultFreq = 8520;
    volume = 1;
    myRegionCode = 0; // 0 means no filter / all regions
}

void Settings::load() {
    prefs.begin("ews_config", true); // read-only
    defaultFreq = prefs.getInt("freq", 8520);
    volume = prefs.getInt("vol", 1);
    myRegionCode = prefs.getUInt("region", 0);
    prefs.end();
}

void Settings::save() {
    prefs.begin("ews_config", false); // read-write
    prefs.putInt("freq", defaultFreq);
    prefs.putInt("vol", volume);
    prefs.putUInt("region", myRegionCode);
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
