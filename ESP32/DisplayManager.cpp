#include "DisplayManager.h"

LGFX::LGFX(void) {
    {
      auto cfg = _bus_instance.config();
      cfg.spi_host = VSPI_HOST;
      cfg.spi_mode = 3;
      cfg.freq_write = 20000000;
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
      cfg.freq   = 44100;
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
    
    canvas = new LGFX_Sprite(&tft);
    canvas->createSprite(284, 76);
    canvas->setTextColor(TFT_WHITE);
    canvas->setTextSize(1);
}

void DisplayManager::update(int freq, int rssi, int vol, int svCount, const char* timeStr, int ewsState, int qzssState, const char* qzssText) {
    canvas->fillScreen(TFT_BLACK);

    // Row 1
    canvas->setFont(&fonts::lgfxJapanGothic_16);
    canvas->setTextSize(1);
    canvas->setTextColor(TFT_WHITE);
    canvas->setCursor(5, 2);
    canvas->print("QZSS/EWS 受信機"); 
    
    canvas->setFont(&fonts::Font0);
    canvas->setTextSize(2);
    canvas->setTextColor(TFT_ORANGE);
    canvas->setCursor(155, 2);
    canvas->printf("%d.%dMHz", freq / 100, (freq % 100) / 10);

    canvas->drawFastHLine(0, 20, 284, TFT_DARKGREY);

    // Row 2: RSSI & Volume
    canvas->setCursor(5, 25);
    canvas->setTextColor(TFT_LIGHTGREY);
    canvas->setTextSize(1);
    canvas->print("SIG");
    int rssiBar = map(rssi, 0, 60, 0, 180);
    canvas->drawRect(35, 25, 184, 10, TFT_WHITE);
    canvas->fillRect(37, 27, constrain(rssiBar, 0, 180), 6, (rssi > 30) ? TFT_GREEN : TFT_YELLOW);
    canvas->setCursor(230, 25);
    canvas->printf("V:%d", vol);

    // Row 3: GPS
    canvas->setCursor(5, 42);
    canvas->setTextColor(TFT_CYAN);
    canvas->setTextSize(1);
    canvas->printf("SATS:%d  TIME:%s", svCount, timeStr);
    
    // Row 4: Status
    if (qzssState == 2 || qzssState == 3) {
      canvas->fillRect(0, 58, 284, 18, TFT_RED);
      canvas->setCursor(5, 61);
      canvas->setTextColor(TFT_WHITE);
      canvas->setFont(&fonts::lgfxJapanGothic_12);
      canvas->setTextSize(1);
      if (qzssText) canvas->print(qzssText);
      else canvas->print("QZSS 災害情報を受信しました！");
    } else if (qzssState == 1) {
      canvas->fillRect(0, 58, 284, 18, TFT_MAGENTA);
      canvas->setCursor(5, 61);
      canvas->setTextColor(TFT_WHITE);
      canvas->setFont(&fonts::lgfxJapanGothic_12);
      canvas->setTextSize(1);
      if (qzssText) canvas->print(qzssText);
      else canvas->print("QZSS 訓練/試験メッセージを受信中");
    } else {
      canvas->fillRect(0, 58, 284, 18, (ewsState == 1) ? TFT_RED : 0x2104);
      canvas->setCursor(5, 61);
      canvas->setTextColor(TFT_WHITE);
      canvas->setFont(&fonts::lgfxJapanGothic_12);
      canvas->setTextSize(1);
      canvas->print(ewsState == 1 ? "FM 警報信号を受信・解析中..." : "システム監視中 - 待機状態");
    }
    canvas->setFont(&fonts::Font0);
    
    canvas->pushSprite(0, 0);
}
