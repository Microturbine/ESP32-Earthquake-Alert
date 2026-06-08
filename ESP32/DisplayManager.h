#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#define LGFX_USE_V1
#include <LovyanGFX.hpp>

#include "EWS_Decoder.h"

class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ST7789  _panel_instance;
  lgfx::Bus_SPI       _bus_instance;
  lgfx::Light_PWM     _light_instance;
public:
  LGFX(void);
};

class DisplayManager {
public:
    void init();
    void update(int freq, int rssi, int vol, int svCount, const char* timeStr, EwsState ewsState);
    
private:
    LGFX tft;
    LGFX_Sprite* canvas;
};

extern DisplayManager displayManager;

#endif
