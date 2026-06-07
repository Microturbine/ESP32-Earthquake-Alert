#include "DisplayManager.h"
#include "AlertManager.h"
#include "WebUIManager.h"

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
    tft.println("初期化中...");
    tft.setBrightness(128);
    
    canvas = new LGFX_Sprite(&tft);
    canvas->createSprite(284, 76);
    canvas->setTextColor(TFT_WHITE);
    canvas->setTextSize(1);
}

void DisplayManager::update(int freq, int rssi, int vol, int svCount, const char* timeStr, int ewsState) {
    canvas->fillScreen(TFT_BLACK);

    // 1行目
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

    bool alertActive = alertManager.hasRealAlert();
    bool testActive = alertManager.hasTestAlert();

    if (alertActive || testActive) {
      uint16_t bgColor = testActive ? TFT_MAGENTA : TFT_RED;
      canvas->fillRect(0, 20, 284, 56, bgColor);
      canvas->setTextColor(TFT_WHITE);
      canvas->setFont(&fonts::lgfxJapanGothic_12);
      canvas->setTextSize(1);
      
      Alert activeAlerts[4];
      int count = alertManager.copyAlerts(activeAlerts, 4);
      
      if (count == 1) {
          canvas->setCursor(5, 24);
          canvas->setTextWrap(true);
          canvas->print(activeAlerts[0].text);
      } else if (count > 1) {
          canvas->setTextWrap(false);
          int y = 22;
          for (int i = 0; i < count && y < 76 - 12; i++) {
              canvas->setCursor(5, y);
              canvas->print(activeAlerts[i].text);
              y += 13;
          }
      }
    } else {
      // 2行目: RSSIと音量
      canvas->setCursor(5, 25);
      canvas->setTextColor(TFT_LIGHTGREY);
      canvas->setTextSize(1);
      canvas->print("SIG");
      int rssiBar = map(rssi, 0, 60, 0, 180);
      canvas->drawRect(35, 25, 184, 10, TFT_WHITE);
      canvas->fillRect(37, 27, constrain(rssiBar, 0, 180), 6, (rssi > 30) ? TFT_GREEN : TFT_YELLOW);
      canvas->setCursor(230, 25);
      canvas->printf("V:%d", vol);

      // 3行目: GPS & IP
      canvas->setCursor(5, 42);
      canvas->setTextColor(TFT_CYAN);
      canvas->setTextSize(1);
      canvas->printf("SATS:%d  TIME:%s", svCount, timeStr);
      
      canvas->setCursor(160, 42);
      canvas->setTextColor(TFT_GREEN);
      canvas->printf("IP:%s", webUIManager.getIPAddress().c_str());
      
      // 4行目: ステータス
      canvas->fillRect(0, 58, 284, 18, (ewsState == 1) ? TFT_RED : 0x2104);
      canvas->setCursor(5, 61);
      canvas->setTextColor(TFT_WHITE);
      canvas->setFont(&fonts::lgfxJapanGothic_12);
      canvas->setTextSize(1);
      canvas->setTextWrap(false);
      canvas->print(ewsState == 1 ? "FM 警報信号を受信・解析中..." : "システム監視中 - 待機状態");
    }
    canvas->setFont(&fonts::Font0);
    
    canvas->pushSprite(0, 0);
}
