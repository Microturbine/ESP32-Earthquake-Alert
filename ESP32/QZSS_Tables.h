#ifndef QZSS_TABLES_H
#define QZSS_TABLES_H

#include <Arduino.h>

struct QzssCodeMap {
    uint32_t code;
    const char* name;
};

struct QzssRiverCodeMap {
    uint64_t code;
    const char* name;
};

// カプセル化されたルックアップ関数群
const char* getEpicenterName(uint32_t code);
const char* getPrefectureJisName(uint32_t code);
const char* getTsunamiRegionName(uint32_t code);
const char* getCoastalRegionName(uint32_t code);
const char* getVolcanoName(uint32_t code);
const char* getWeatherSubCatName(uint32_t code);
const char* getPrefecturalForecastName(uint32_t code);
const char* getRiverName(uint64_t code);
const char* getMarineWarningName(uint32_t code);
const char* getMarineForecastName(uint32_t code);
const char* getDcxHazardName(uint32_t code);
const char* getDcxGuidanceTextName(uint32_t code);
String getDcxGuidanceText(uint32_t a11Code);
double getDcxEllipseRadius(uint32_t code);

#endif // QZSS_TABLES_H
