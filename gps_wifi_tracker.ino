/*
 * GPS + WiFi tracker для LILYGO T-LoRa Pager (ESP32-S3, LilyGoLib)
 * -----------------------------------------------------------------
 * - Статус на экране: ПОДГОТОВКА -> ПОИСК СПУТНИКОВ -> ЗАПИСЬ
 * - Время (UTC) берётся с самого GPS
 * - Пока нет фикса (валидные координаты + >= MIN_SATS_FOR_FIX спутников) - НИЧЕГО не пишем
 * - На экране: чёрный фон, красный трек, координаты, число спутников
 * - SD-карта: новая папка на каждый день и на каждый час:
 *     /sd/gps/2026-07-22/14/track.csv
 *     /sd/gps/2026-07-22/14/wifi.csv
 * - Параллельно (без блокировки GPS/экрана) раз в WIFI_SCAN_INTERVAL_MS
 *   сканируются WiFi-сети, все видимые сети (SSID, BSSID, канал, RSSI)
 *   дописываются в wifi.csv
 *
 * Библиотека: LilyGoLib (официальная, https://github.com/Xinyuan-LilyGO/LilyGoLib)
 * Плата (Arduino IDE / arduino-cli fqbn): esp32:esp32:tlora_pager
 */

#include <LilyGoLib.h>
#include <LV_Helper.h>
#include <WiFi.h>
#include <SD.h>

// ---------------------------------------------------------------------------
// Настройки
// ---------------------------------------------------------------------------
#define WIFI_SCAN_INTERVAL_MS 30000UL   // период сканирования WiFi
#define DISPLAY_UPDATE_MS     500UL     // период обновления текста статуса
#define MAX_TRACK_POINTS      2000      // точек трека в памяти для отрисовки
#define MIN_SATS_FOR_FIX      3         // минимум спутников для валидного фикса

// SD монтируется LilyGoLib в точку /sd
#define SD_ROOT "/sd/gps"

// Область экрана 480x222, верхняя полоса - статус, ниже - трек
#define SCR_W      480
#define SCR_H      222
#define HEADER_H   40
#define TRACK_H    (SCR_H - HEADER_H)

// ---------------------------------------------------------------------------
// Состояние
// ---------------------------------------------------------------------------
enum TrackerState { ST_INIT, ST_SEARCHING, ST_RECORDING };
TrackerState state = ST_INIT;

struct Pt { float lat, lon; };
static Pt track[MAX_TRACK_POINTS];
static int trackCount = 0;
static bool haveBBox = false;
static float minLat, maxLat, minLon, maxLon;

char currentDir[64] = "";   // текущая папка /sd/gps/YYYY-MM-DD/HH
bool sdReady = false;

unsigned long lastDisplayUpdate = 0;
unsigned long lastWifiScanStart = 0;
bool wifiScanRunning = false;

// LVGL объекты
lv_obj_t *statusLabel;
lv_obj_t *coordLabel;
lv_obj_t *canvas;
static void *canvasBuf = nullptr;

// ---------------------------------------------------------------------------
// Время: unix timestamp из даты/времени GPS (UTC), без libc time()
// ---------------------------------------------------------------------------
static int64_t daysFromCivil(int y, int m, int d) {
    y -= (m <= 2);
    int64_t era = (y >= 0 ? y : y - 399) / 400;
    unsigned yoe = (unsigned)(y - era * 400);
    unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097LL + (int64_t)doe - 719468;
}

static uint32_t toUnixTime(int y, int mo, int d, int h, int mi, int s) {
    int64_t days = daysFromCivil(y, mo, d);
    return (uint32_t)(days * 86400LL + h * 3600 + mi * 60 + s);
}

static void isoTime(char *out, size_t outSize) {
    snprintf(out, outSize, "%04d-%02d-%02dT%02d:%02d:%02dZ",
             instance.gps.date.year(), instance.gps.date.month(), instance.gps.date.day(),
             instance.gps.time.hour(), instance.gps.time.minute(), instance.gps.time.second());
}

// ---------------------------------------------------------------------------
// SD: подготовка папки текущего часа
// ---------------------------------------------------------------------------
static bool ensureHourFolder() {
    if (!sdReady) return false;
    if (!instance.gps.date.isValid() || !instance.gps.time.isValid()) return false;

    char dayDir[40];
    char hourDir[64];
    snprintf(dayDir, sizeof(dayDir), "%s/%04d-%02d-%02d", SD_ROOT,
             instance.gps.date.year(), instance.gps.date.month(), instance.gps.date.day());
    snprintf(hourDir, sizeof(hourDir), "%s/%02d", dayDir, instance.gps.time.hour());

    if (strcmp(hourDir, currentDir) == 0) return true;

    if (!SD.exists(SD_ROOT)) SD.mkdir(SD_ROOT);
    if (!SD.exists(dayDir))  SD.mkdir(dayDir);
    if (!SD.exists(hourDir)) SD.mkdir(hourDir);

    strncpy(currentDir, hourDir, sizeof(currentDir) - 1);

    char trackPath[96], wifiPath[96];
    snprintf(trackPath, sizeof(trackPath), "%s/track.csv", currentDir);
    snprintf(wifiPath,  sizeof(wifiPath),  "%s/wifi.csv",  currentDir);

    if (!SD.exists(trackPath)) {
        File f = SD.open(trackPath, FILE_WRITE);
        if (f) { f.println("unix_ts,iso_time,lat,lon,alt_m,speed_kmh,sats,hdop"); f.close(); }
    }
    if (!SD.exists(wifiPath)) {
        File f = SD.open(wifiPath, FILE_WRITE);
        if (f) { f.println("unix_ts,iso_time,ssid,bssid,channel,rssi_dbm"); f.close(); }
    }
    return true;
}

static void appendTrackPoint() {
    if (!ensureHourFolder()) return;

    char path[96];
    snprintf(path, sizeof(path), "%s/track.csv", currentDir);
    File f = SD.open(path, FILE_APPEND);
    if (!f) return;

    char iso[32];
    isoTime(iso, sizeof(iso));
    uint32_t ts = toUnixTime(instance.gps.date.year(), instance.gps.date.month(), instance.gps.date.day(),
                             instance.gps.time.hour(), instance.gps.time.minute(), instance.gps.time.second());

    f.printf("%lu,%s,%.6f,%.6f,%.1f,%.1f,%d,%.2f\n",
             (unsigned long)ts, iso,
             instance.gps.location.lat(), instance.gps.location.lng(),
             instance.gps.altitude.isValid() ? instance.gps.altitude.meters() : 0.0,
             instance.gps.speed.isValid() ? instance.gps.speed.kmph() : 0.0,
             instance.gps.satellites.value(),
             instance.gps.hdop.isValid() ? instance.gps.hdop.hdop() : 0.0);
    f.close();
}

static void appendWifiResults(int n) {
    if (!sdReady || strlen(currentDir) == 0) return;

    char path[96];
    snprintf(path, sizeof(path), "%s/wifi.csv", currentDir);
    File f = SD.open(path, FILE_APPEND);
    if (!f) return;

    char iso[32] = "unknown";
    uint32_t ts = 0;
    bool haveTime = instance.gps.date.isValid() && instance.gps.time.isValid();
    if (haveTime) {
        isoTime(iso, sizeof(iso));
        ts = toUnixTime(instance.gps.date.year(), instance.gps.date.month(), instance.gps.date.day(),
                        instance.gps.time.hour(), instance.gps.time.minute(), instance.gps.time.second());
    }

    for (int i = 0; i < n; i++) {
        String ssid = WiFi.SSID(i);
        ssid.replace(",", " ");
        f.printf("%lu,%s,%s,%s,%d,%d\n",
                 (unsigned long)ts, iso, ssid.c_str(),
                 WiFi.BSSIDstr(i).c_str(), WiFi.channel(i), WiFi.RSSI(i));
    }
    f.close();
}

// ---------------------------------------------------------------------------
// Экран (LVGL)
// ---------------------------------------------------------------------------
static void updateStatusText() {
    const char *statusText;
    switch (state) {
        case ST_INIT:      statusText = "ПОДГОТОВКА...";      break;
        case ST_SEARCHING: statusText = "ПОИСК СПУТНИКОВ...";  break;
        case ST_RECORDING: statusText = "ЗАПИСЬ";              break;
        default:           statusText = "";                    break;
    }

    if (instance.gps.location.isValid()) {
        lv_label_set_text_fmt(coordLabel, "Lat:%.6f  Lon:%.6f   Sat:%d",
                               instance.gps.location.lat(), instance.gps.location.lng(),
                               instance.gps.satellites.isValid() ? instance.gps.satellites.value() : 0);
    } else {
        lv_label_set_text_fmt(coordLabel, "Lat:--.------  Lon:--.------   Sat:%d",
                               instance.gps.satellites.isValid() ? instance.gps.satellites.value() : 0);
    }
    lv_label_set_text(statusLabel, statusText);
}

static void mapLatLonToCanvas(float lat, float lon, int32_t &x, int32_t &y) {
    float spanLat = (maxLat - minLat);
    float spanLon = (maxLon - minLon);
    if (spanLat < 0.00005f) spanLat = 0.00005f;
    if (spanLon < 0.00005f) spanLon = 0.00005f;

    float px = (lon - minLon) / spanLon;
    float py = (lat - maxLat) / (-spanLat);

    x = (int32_t)(px * (SCR_W - 8)) + 4;
    y = (int32_t)(py * (TRACK_H - 8)) + 4;
}

// Рисует один красный отрезок на канвасе (LVGL v9: через layer + lv_draw_line)
static void drawTrackSegment(int32_t x0, int32_t y0, int32_t x1, int32_t y1) {
    lv_layer_t layer;
    lv_canvas_init_layer(canvas, &layer);

    lv_draw_line_dsc_t dsc;
    lv_draw_line_dsc_init(&dsc);
    dsc.color = lv_color_make(255, 0, 0);
    dsc.width = 2;
    dsc.round_start = 1;
    dsc.round_end = 1;
    dsc.p1.x = x0; dsc.p1.y = y0;
    dsc.p2.x = x1; dsc.p2.y = y1;
    lv_draw_line(&layer, &dsc);

    lv_canvas_finish_layer(canvas, &layer);
}

static void redrawFullTrack() {
    lv_canvas_fill_bg(canvas, lv_color_black(), LV_OPA_COVER);
    if (trackCount < 2) return;
    int32_t x0, y0, x1, y1;
    mapLatLonToCanvas(track[0].lat, track[0].lon, x0, y0);
    for (int i = 1; i < trackCount; i++) {
        mapLatLonToCanvas(track[i].lat, track[i].lon, x1, y1);
        drawTrackSegment(x0, y0, x1, y1);
        x0 = x1; y0 = y1;
    }
}

static void addTrackPointAndDraw(float lat, float lon) {
    bool needFullRedraw = false;

    if (!haveBBox) {
        minLat = maxLat = lat;
        minLon = maxLon = lon;
        haveBBox = true;
        needFullRedraw = true; // первая точка - просто очищаем канвас
    } else {
        if (lat < minLat) { minLat = lat; needFullRedraw = true; }
        if (lat > maxLat) { maxLat = lat; needFullRedraw = true; }
        if (lon < minLon) { minLon = lon; needFullRedraw = true; }
        if (lon > maxLon) { maxLon = lon; needFullRedraw = true; }
    }

    int32_t prevX = 0, prevY = 0;
    bool hadPrev = (trackCount > 0);
    if (hadPrev) mapLatLonToCanvas(track[trackCount - 1].lat, track[trackCount - 1].lon, prevX, prevY);

    if (trackCount < MAX_TRACK_POINTS) {
        track[trackCount].lat = lat;
        track[trackCount].lon = lon;
        trackCount++;
    } else {
        memmove(&track[0], &track[1], sizeof(Pt) * (MAX_TRACK_POINTS - 1));
        track[MAX_TRACK_POINTS - 1].lat = lat;
        track[MAX_TRACK_POINTS - 1].lon = lon;
    }

    if (needFullRedraw) {
        redrawFullTrack();
    } else if (hadPrev) {
        int32_t x1, y1;
        mapLatLonToCanvas(lat, lon, x1, y1);
        drawTrackSegment(prevX, prevY, x1, y1);
    }
}

// ---------------------------------------------------------------------------
// setup / loop
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);

    // Инициализация платы: экран, питание, клавиатура, GPS (Serial1 38400 baud), и т.д.
    instance.begin();

    beginLvglHelper(instance);

    instance.setBrightness(DEVICE_MAX_BRIGHTNESS_LEVEL);

    // Чёрный фон на весь экран
    lv_obj_set_style_bg_color(lv_screen_active(), lv_color_black(), 0);
    lv_obj_set_style_bg_opa(lv_screen_active(), LV_OPA_COVER, 0);

    // Статус (верхняя строка)
    statusLabel = lv_label_create(lv_screen_active());
    lv_obj_set_style_text_color(statusLabel, lv_color_white(), 0);
    lv_obj_set_style_text_font(statusLabel, &lv_font_montserrat_20, 0);
    lv_obj_align(statusLabel, LV_ALIGN_TOP_LEFT, 4, 2);
    lv_label_set_text(statusLabel, "ПОДГОТОВКА...");

    // Координаты + спутники (вторая строка)
    coordLabel = lv_label_create(lv_screen_active());
    lv_obj_set_style_text_color(coordLabel, lv_color_white(), 0);
    lv_obj_align(coordLabel, LV_ALIGN_TOP_LEFT, 4, 24);
    lv_label_set_text(coordLabel, "Lat:--.------  Lon:--.------   Sat:0");

    // Канвас для красного трека (нижняя часть экрана)
    size_t bufSize = (size_t)SCR_W * TRACK_H * 2; // RGB565 = 2 байта/пиксель
    canvasBuf = ps_malloc(bufSize);
    canvas = lv_canvas_create(lv_screen_active());
    lv_canvas_set_buffer(canvas, canvasBuf, SCR_W, TRACK_H, LV_COLOR_FORMAT_RGB565);
    lv_obj_align(canvas, LV_ALIGN_TOP_LEFT, 0, HEADER_H);
    lv_canvas_fill_bg(canvas, lv_color_black(), LV_OPA_COVER);

    lv_timer_handler();

    // SD карта (LilyGoLib монтирует её в /sd)
    int retry = 5;
    do {
        sdReady = instance.installSD();
        if (!sdReady) delay(500);
    } while (!sdReady && --retry > 0);

    if (!sdReady) {
        Serial.println("SD init failed!");
    }

    // WiFi только на сканирование, без подключения к сети
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    state = ST_SEARCHING;
    updateStatusText();
}

static void handleGPS() {
    instance.gps.loop();   // читает Serial1 и скармливает данные внутреннему TinyGPSPlus

    bool fixOk = instance.gps.location.isValid() &&
                 instance.gps.satellites.isValid() &&
                 instance.gps.satellites.value() >= MIN_SATS_FOR_FIX;

    if (fixOk && state != ST_RECORDING) {
        state = ST_RECORDING;
    } else if (!fixOk && state == ST_RECORDING) {
        state = ST_SEARCHING;
    }

    if (fixOk && instance.gps.location.isUpdated()) {
        appendTrackPoint();
        addTrackPointAndDraw(instance.gps.location.lat(), instance.gps.location.lng());
    }
}

static void handleWifiScan() {
    unsigned long now = millis();

    if (!wifiScanRunning && (lastWifiScanStart == 0 || now - lastWifiScanStart >= WIFI_SCAN_INTERVAL_MS)) {
        WiFi.scanNetworks(true /* async */, true /* show_hidden */);
        wifiScanRunning = true;
        lastWifiScanStart = now;
    }

    if (wifiScanRunning) {
        int n = WiFi.scanComplete();
        if (n >= 0) {
            appendWifiResults(n);
            WiFi.scanDelete();
            wifiScanRunning = false;
        } else if (n == WIFI_SCAN_FAILED) {
            wifiScanRunning = false;
        }
    }
}

void loop() {
    handleGPS();
    handleWifiScan();

    unsigned long now = millis();
    if (now - lastDisplayUpdate >= DISPLAY_UPDATE_MS) {
        updateStatusText();
        lastDisplayUpdate = now;
    }

    lv_timer_handler();
    delay(2);
}
