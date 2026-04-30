#include "DisplayManager.h"

LGFX::LGFX(void) {
    {
      auto cfg = _bus_instance.config();
      cfg.spi_host = VSPI_HOST;
      cfg.spi_mode = 3;
      cfg.freq_write = 40000000; // 40MHz for faster transfer (shorter noise burst)
      cfg.pin_sclk = 14; 
      cfg.pin_mosi = 13;
      cfg.pin_miso = -1;
      cfg.pin_dc   = 15;
      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }
    {
      auto cfg = _panel_instance.config();
      cfg.pin_cs           = 5;
      cfg.pin_rst          = 27;
      cfg.panel_width      = 76;
      cfg.panel_height     = 284;
      cfg.offset_x         = 82;
      cfg.offset_y         = 18;
      cfg.offset_rotation  = 0;
      _panel_instance.config(cfg);
    }
    {
      auto cfg = _light_instance.config();
      cfg.pin_bl = 21;
      cfg.invert = false;
      cfg.freq   = 1200; // Standard backlight PWM freq
      cfg.pwm_channel = 7;
      _light_instance.config(cfg);
      _panel_instance.setLight(&_light_instance);
    }
    setPanel(&_panel_instance);
}

DisplayManager displayManager;

void DisplayManager::init() {
    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    tft.println("Initializing...");
    tft.setBrightness(128);
}

void DisplayManager::update(int freq, int rssi, int vol, int svCount, const char* timeStr, int ewsState, int qzssState, const char* qzssText) {
    tft.startWrite();

    if (initialDraw) {
        tft.fillScreen(TFT_BLACK);
        
        tft.setFont(&fonts::lgfxJapanGothic_16);
        tft.setTextSize(1);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.setCursor(5, 2);
        tft.print("QZSS/EWS 受信機"); 

        tft.drawFastHLine(0, 20, 284, TFT_DARKGREY);

        tft.setFont(&fonts::Font0);
        tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
        tft.setTextSize(1);
        tft.setCursor(5, 25);
        tft.print("SIG");
        tft.drawRect(35, 25, 184, 10, TFT_WHITE);
        
        initialDraw = false;
    }

    if (freq != lastFreq) {
        tft.setFont(&fonts::Font0);
        tft.setTextSize(2);
        tft.setTextColor(TFT_ORANGE, TFT_BLACK);
        tft.setCursor(155, 2);
        tft.printf("%d.%dMHz ", freq / 100, (freq % 100) / 10);
        lastFreq = freq;
    }

    if (rssi != lastRssi) {
        int rssiBar = map(rssi, 0, 60, 0, 180);
        tft.fillRect(36, 26, 182, 8, TFT_BLACK);
        tft.fillRect(36, 26, constrain(rssiBar, 0, 182), 8, (rssi > 30) ? TFT_GREEN : TFT_YELLOW);
        lastRssi = rssi;
    }

    if (vol != lastVol) {
        tft.setFont(&fonts::Font0);
        tft.setTextSize(1);
        tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
        tft.setCursor(230, 25);
        tft.printf("V:%-2d", vol);
        lastVol = vol;
    }

    if (svCount != lastSvCount || strcmp(timeStr, lastTimeStr) != 0) {
        tft.setFont(&fonts::Font0);
        tft.setTextColor(TFT_CYAN, TFT_BLACK);
        tft.setTextSize(1);
        tft.setCursor(5, 42);
        tft.printf("SATS:%-2d  TIME:%-8s", svCount, timeStr);
        lastSvCount = svCount;
        strcpy(lastTimeStr, timeStr);
    }

    bool statusChanged = false;
    if (ewsState != lastEwsState || qzssState != lastQzssState) {
        statusChanged = true;
    }
    if (qzssText != nullptr) {
        if (strcmp(qzssText, lastQzssText) != 0) {
            statusChanged = true;
            strcpy(lastQzssText, qzssText);
        }
    } else {
        if (lastQzssText[0] != '\0') {
            statusChanged = true;
            lastQzssText[0] = '\0';
        }
    }

    if (statusChanged) {
        tft.setFont(&fonts::lgfxJapanGothic_12);
        tft.setTextSize(1);
        tft.setTextColor(TFT_WHITE);
        
        if (qzssState == 2 || qzssState == 3) {
            tft.fillRect(0, 58, 284, 18, TFT_RED);
            tft.setCursor(5, 61);
            if (qzssText) tft.print(qzssText);
            else tft.print("QZSS 災害情報を受信しました！");
        } else if (qzssState == 1) {
            tft.fillRect(0, 58, 284, 18, TFT_MAGENTA);
            tft.setCursor(5, 61);
            if (qzssText) tft.print(qzssText);
            else tft.print("QZSS 訓練/試験メッセージを受信中");
        } else {
            tft.fillRect(0, 58, 284, 18, (ewsState == 1) ? TFT_RED : 0x2104);
            tft.setCursor(5, 61);
            tft.print(ewsState == 1 ? "FM 警報信号を受信・解析中..." : "システム監視中 - 待機状態");
        }
        lastEwsState = ewsState;
        lastQzssState = qzssState;
    }

    tft.endWrite();
}
