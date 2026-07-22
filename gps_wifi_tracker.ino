/*
 * GPS + WiFi tracker для LILYGO T-LoRa Pager (ESP32-S3)
 * -----------------------------------------------------
 * - Ждёт спутники GPS (MIA-M10Q), статус на экране: ПОДГОТОВКА -> ПОИСК -> ЗАПИСЬ
 * - При наличии фикса пишет CSV на SD: время (unix + iso), координаты, кол-во спутников
 * - Пока фикса нет - НИЧЕГО не пишет
 * - На экране: чёрный фон, красный трек, координаты, число спутников, статус
 * - Параллельно сканирует WiFi (асинхронно, не блокируя GPS/экран) и пишет
 *   все видимые сети (SSID, BSSID, канал, RSSI) в отдельный CSV
 * - Структура на SD:  /gps/YYYY-MM-DD/HH/track.csv
 *                      /gps/YYYY-MM-DD/HH/wifi.csv
 *
 * Плата (Arduino IDE):  LilyGo-T-LoRa-Pager  (требуется arduino-esp32 >= 3.3.0-alpha1)
 * Библиотеки: LilyGoLib, LilyGoLib-ThirdParty (см. README.md), TinyGPSPlus
 */

#include <Arduino.h>
#include <LilyGoLib.h>
#include <TinyGPSPlus.h>
#include <SPI.h>
#include <SD.h>
#include <WiFi.h>

// ---------------------------------------------------------------------------
// Пины платы (официальная распиновка T-LoRaPager)
// ---------------------------------------------------------------------------
#define BOARD_DISP_CS       38
#define BOARD_SPI_SCK       35
#define BOARD_SPI_MOSI      34
#define BOARD_SPI_MISO      33
#define BOARD_SDCARD_CS     21
#define RADIO_CS_PIN        36
#define BOARD_NFC_CS        39
#define BOARD_GPS_RX_PIN    12   // ESP32 RX <- GPS TX
#define BOARD_GPS_TX_PIN     4   // ESP32 TX -> GPS RX

// ---------------------------------------------------------------------------
// Настройки
// ---------------------------------------------------------------------------
#define GPS_BAUD              9600
#define WIFI_SCAN_INTERVAL_MS 30000UL   // период сканирования WiFi
#define DISPLAY_UPDATE_MS     500UL     // период обновления текста на экране
#define MAX_TRACK_POINTS      4000      // точек трека в памяти для отрисовки
#define MIN_SATS_FOR_FIX      3         // минимум спутников, чтобы считать фикс валидным

// Область экрана: 480x222
#define SCR_W   480
#define SCR_H   222
#define HEADER_H 44                     // верхняя полоса с текстом
#define TRACK_TOP  (HEADER_H + 2)
#define TRACK_H  (SCR_H - TRACK_TOP)

// ---------------------------------------------------------------------------
// Глобальные объекты
// ---------------------------------------------------------------------------
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

enum TrackerState { ST_INIT, ST_SEARCHING, ST_RECORDING };
TrackerState state = ST_INIT;

struct Pt { float lat, lon; };
static Pt track[MAX_TRACK_POINTS];
static int trackCount = 0;
static bool haveBBox = false;
static float minLat, maxLat, minLon, maxLon;

char currentDir[48] = "";     // текущая папка /gps/YYYY-MM-DD/HH
bool sdReady = false;

unsigned long lastDisplayUpdate = 0;
unsigned long lastWifiScanStart = 0;
bool wifiScanRunning = false;

// ---------------------------------------------------------------------------
// Время: перевод даты/времени GPS (UTC) в unix timestamp без libc
// (алгоритм Howard Hinnant, days_from_civil)
// ---------------------------------------------------------------------------
static int64_t daysFromCivil(int y, int m, int d) {
    y -= (m <= 2);
    int64_t era = (y >= 0 ? y : y - 399) / 400;
    unsigned yoe = (unsigned)(y - era * 400);              // [0, 399]
    unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1; // [0, 365]
    unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;   // [0, 146096]
    return era * 146097LL + (int64_t)doe - 719468;
}

static uint32_t toUnixTime(int y, int mo, int d, int h, int mi, int s) {
    int64_t days = daysFromCivil(y, mo, d);
    return (uint32_t)(days * 86400LL + h * 3600 + mi * 60 + s);
}

static void isoTime(char *out, size_t outSize) {
    snprintf(out, outSize, "%04d-%02d-%02dT%02d:%02d:%02dZ",
              gps.date.year(), gps.date.month(), gps.date.day(),
              gps.time.hour(), gps.time.minute(), gps.time.second());
}

// ---------------------------------------------------------------------------
// SD: подготовка папки текущего часа
// ---------------------------------------------------------------------------
static bool ensureHourFolder() {
    if (!gps.date.isValid() || !gps.time.isValid()) return false;

    char dayDir[24];
    char hourDir[48];
    snprintf(dayDir, sizeof(dayDir), "/gps/%04d-%02d-%02d",
              gps.date.year(), gps.date.month(), gps.date.day());
    snprintf(hourDir, sizeof(hourDir), "%s/%02d", dayDir, gps.time.hour());

    if (strcmp(hourDir, currentDir) == 0) return true; // уже в этой папке

    if (!SD.exists("/gps"))   SD.mkdir("/gps");
    if (!SD.exists(dayDir))   SD.mkdir(dayDir);
    if (!SD.exists(hourDir))  SD.mkdir(hourDir);

    strncpy(currentDir, hourDir, sizeof(currentDir));

    // создаём файлы с заголовками, если их ещё нет
    char trackPath[80], wifiPath[80];
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
    if (!sdReady) return;
    if (!ensureHourFolder()) return;

    char path[80];
    snprintf(path, sizeof(path), "%s/track.csv", currentDir);
    File f = SD.open(path, FILE_APPEND);
    if (!f) return;

    char iso[32];
    isoTime(iso, sizeof(iso));
    uint32_t ts = toUnixTime(gps.date.year(), gps.date.month(), gps.date.day(),
                              gps.time.hour(), gps.time.minute(), gps.time.second());

    f.printf("%lu,%s,%.6f,%.6f,%.1f,%.1f,%d,%.2f\n",
              (unsigned long)ts, iso,
              gps.location.lat(), gps.location.lng(),
              gps.altitude.isValid() ? gps.altitude.meters() : 0.0,
              gps.speed.isValid() ? gps.speed.kmph() : 0.0,
              gps.satellites.value(),
              gps.hdop.isValid() ? gps.hdop.hdop() : 0.0);
    f.close();
}

static void appendWifiResults(int n) {
    if (!sdReady) return;
    // используем время GPS если оно валидно, иначе millis-метку без времени
    bool haveTime = gps.date.isValid() && gps.time.isValid();
    if (!haveTime && strlen(currentDir) == 0) return; // некуда писать, пока не было ни одной папки

    char path[80];
    if (strlen(currentDir) == 0) return;
    snprintf(path, sizeof(path), "%s/wifi.csv", currentDir);
    File f = SD.open(path, FILE_APPEND);
    if (!f) return;

    char iso[32] = "unknown";
    uint32_t ts = 0;
    if (haveTime) {
        isoTime(iso, sizeof(iso));
        ts = toUnixTime(gps.date.year(), gps.date.month(), gps.date.day(),
                         gps.time.hour(), gps.time.minute(), gps.time.second());
    }

    for (int i = 0; i < n; i++) {
        String ssid = WiFi.SSID(i);
        ssid.replace(",", " ");   // защита от запятых в CSV
        f.printf("%lu,%s,%s,%s,%d,%d\n",
                  (unsigned long)ts, iso, ssid.c_str(),
                  WiFi.BSSIDstr(i).c_str(), WiFi.channel(i), WiFi.RSSI(i));
    }
    f.close();
}

// ---------------------------------------------------------------------------
// Экран
// ---------------------------------------------------------------------------
static void drawStatusHeader() {
    board.display->fillRect(0, 0, SCR_W, HEADER_H, TFT_BLACK);
    board.display->setTextColor(TFT_WHITE, TFT_BLACK);
    board.display->setTextSize(2);

    const char *statusText;
    switch (state) {
        case ST_INIT:       statusText = "ПОДГОТОВКА...";        break;
        case ST_SEARCHING:  statusText = "ПОИСК СПУТНИКОВ...";   break;
        case ST_RECORDING:  statusText = "ЗАПИСЬ";               break;
        default:            statusText = "";                     break;
    }
    board.display->setCursor(4, 2);
    board.display->print(statusText);

    board.display->setTextSize(1);
    board.display->setCursor(4, 24);
    if (gps.location.isValid()) {
        board.display->printf("Lat:%.6f Lon:%.6f", gps.location.lat(), gps.location.lng());
    } else {
        board.display->print("Lat: --.------ Lon: --.------");
    }
    board.display->setCursor(360, 24);
    board.display->printf("Sat:%2d", gps.satellites.isValid() ? gps.satellites.value() : 0);
}

static void mapLatLonToScreen(float lat, float lon, int &x, int &y) {
    float spanLat = (maxLat - minLat);
    float spanLon = (maxLon - minLon);
    if (spanLat < 0.00005f) spanLat = 0.00005f;
    if (spanLon < 0.00005f) spanLon = 0.00005f;

    // немного отступов по краям
    float px = (lon - minLon) / spanLon;
    float py = (lat - maxLat) / (-spanLat); // инверсия: широта растёт вверх

    x = (int)(px * (SCR_W - 8)) + 4;
    y = TRACK_TOP + (int)(py * (TRACK_H - 8)) + 4;
}

static void redrawFullTrack() {
    board.display->fillRect(0, TRACK_TOP, SCR_W, TRACK_H, TFT_BLACK);
    if (trackCount < 2) return;
    int x0, y0, x1, y1;
    mapLatLonToScreen(track[0].lat, track[0].lon, x0, y0);
    for (int i = 1; i < trackCount; i++) {
        mapLatLonToScreen(track[i].lat, track[i].lon, x1, y1);
        board.display->drawLine(x0, y0, x1, y1, TFT_RED);
        x0 = x1; y0 = y1;
    }
}

static void addTrackPointAndDraw(float lat, float lon) {
    bool needFullRedraw = false;

    if (!haveBBox) {
        minLat = maxLat = lat;
        minLon = maxLon = lon;
        haveBBox = true;
    } else {
        if (lat < minLat) { minLat = lat; needFullRedraw = true; }
        if (lat > maxLat) { maxLat = lat; needFullRedraw = true; }
        if (lon < minLon) { minLon = lon; needFullRedraw = true; }
        if (lon > maxLon) { maxLon = lon; needFullRedraw = true; }
    }

    int prevX = 0, prevY = 0;
    bool hadPrev = (trackCount > 0);
    if (hadPrev) mapLatLonToScreen(track[trackCount - 1].lat, track[trackCount - 1].lon, prevX, prevY);

    if (trackCount < MAX_TRACK_POINTS) {
        track[trackCount].lat = lat;
        track[trackCount].lon = lon;
        trackCount++;
    } else {
        // сдвигаем буфер, теряя самую старую точку (упрощённо)
        memmove(&track[0], &track[1], sizeof(Pt) * (MAX_TRACK_POINTS - 1));
        track[MAX_TRACK_POINTS - 1].lat = lat;
        track[MAX_TRACK_POINTS - 1].lon = lon;
    }

    if (needFullRedraw) {
        redrawFullTrack();
    } else if (hadPrev) {
        int x1, y1;
        mapLatLonToScreen(lat, lon, x1, y1);
        board.display->drawLine(prevX, prevY, x1, y1, TFT_RED);
    }
}

// ---------------------------------------------------------------------------
// setup / loop
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);

    board.begin();  // инициализация экрана, питания, XL9555 и т.д. (LilyGoLib)

    board.display->fillScreen(TFT_BLACK);
    board.display->setTextColor(TFT_WHITE, TFT_BLACK);
    drawStatusHeader();

    // GPS UART
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, BOARD_GPS_RX_PIN, BOARD_GPS_TX_PIN);

    // Гарантируем, что остальные устройства на общей SPI-шине отпущены
    pinMode(RADIO_CS_PIN, OUTPUT);   digitalWrite(RADIO_CS_PIN, HIGH);
    pinMode(BOARD_NFC_CS, OUTPUT);   digitalWrite(BOARD_NFC_CS, HIGH);

    SPI.begin(BOARD_SPI_SCK, BOARD_SPI_MISO, BOARD_SPI_MOSI);
    sdReady = SD.begin(BOARD_SDCARD_CS);
    if (!sdReady) {
        Serial.println("SD init failed!");
    }

    // WiFi только на сканирование, без подключения
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    state = ST_SEARCHING;
    drawStatusHeader();
}

static void handleGPS() {
    while (gpsSerial.available()) {
        gps.encode(gpsSerial.read());
    }

    bool fixOk = gps.location.isValid() &&
                 gps.satellites.isValid() &&
                 gps.satellites.value() >= MIN_SATS_FOR_FIX;

    if (fixOk && state != ST_RECORDING) {
        state = ST_RECORDING;
    } else if (!fixOk && state == ST_RECORDING) {
        state = ST_SEARCHING;
    }

    if (fixOk && gps.location.isUpdated()) {
        appendTrackPoint();
        addTrackPointAndDraw(gps.location.lat(), gps.location.lng());
    }
}

static void handleWifiScan() {
    unsigned long now = millis();

    if (!wifiScanRunning && (now - lastWifiScanStart >= WIFI_SCAN_INTERVAL_MS || lastWifiScanStart == 0)) {
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
        // n == WIFI_SCAN_RUNNING (-1) -> просто ждём, не блокируя loop()
    }
}

void loop() {
    handleGPS();
    handleWifiScan();

    unsigned long now = millis();
    if (now - lastDisplayUpdate >= DISPLAY_UPDATE_MS) {
        drawStatusHeader();
        lastDisplayUpdate = now;
    }
}
