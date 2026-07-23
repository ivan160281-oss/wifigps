/*
 * GPS + WiFi трекер для LILYGO T-LoRa Pager (ESP32-S3, LilyGoLib)
 * -----------------------------------------------------------------
 * Логика по ТЗ:
 *  - Стартовый экран (запись НЕ идёт): статус GPS, статус WiFi, время (UTC+3),
 *    кол-во видимых WiFi сетей, "ENTER - начать запись"
 *  - ENTER (физическая клавиатура T-LoRa Pager) - старт записи
 *  - ENTER во время записи - стоп записи, файл закрывается (flush), возврат
 *    на стартовый экран
 *  - "E" - полный выход: запись останавливается, файл закрывается, устройство
 *    показывает экран "остановлено" и дальше ничего не делает
 *  - Во время записи: таблица (спутники, координаты, кол-во WiFi сетей,
 *    скорость - среднее по последним 5 точкам, размер текущего файла) +
 *    красный трек на чёрном фоне
 *  - SD: папка /sd/WIFIGPS/, файл log_ГГГГММДД_ЧЧММСС.txt (новый каждые 30 мин)
 *  - Строка в файле раз в 30 сек: HH:MM:SS_lat_lon_wifi1,wifi2,wifi3
 *    (время - UTC+3, если сети не видны - пусто после последнего '_')
 *  - WiFi-сканирование асинхронное, не блокирует GPS/экран
 *
 * Библиотека: LilyGoLib (https://github.com/Xinyuan-LilyGO/LilyGoLib)
 * Плата (fqbn): esp32:esp32:tlora_pager
 */

#include <LilyGoLib.h>
#include <LV_Helper.h>
#include <WiFi.h>
#include <SD.h>
#include <math.h>

// ---------------------------------------------------------------------------
// Настройки
// ---------------------------------------------------------------------------
#define WIFI_SCAN_INTERVAL_MS   15000UL   // период сканирования WiFi (не блокирует основной цикл)
#define WRITE_INTERVAL_MS       30000UL   // запись строки в файл - раз в 30 сек
#define FILE_ROTATE_MS          (30UL * 60UL * 1000UL) // новый файл каждые 30 минут записи
#define DISPLAY_UPDATE_MS       500UL     // период обновления текста на экране
#define MAX_TRACK_POINTS        2000      // точек трека в памяти для отрисовки
#define MIN_SATS_FOR_FIX        3         // минимум спутников для валидного фикса
#define SPEED_AVG_POINTS        5         // усреднение скорости по последним N точкам
#define UTC_OFFSET_SECONDS      (3 * 3600) // UTC+3

#define SD_ROOT "/sd/WIFIGPS"

// Область экрана 480x222
#define SCR_W      480
#define SCR_H      222
#define HEADER_H   96                     // верхний блок текста (статус/таблица)
#define TRACK_H    (SCR_H - HEADER_H)

// ---------------------------------------------------------------------------
// Состояние приложения
// ---------------------------------------------------------------------------
enum AppState { APP_IDLE, APP_RECORDING, APP_STOPPED };
AppState appState = APP_IDLE;

struct Pt { float lat, lon; uint32_t ts; };
static Pt track[MAX_TRACK_POINTS];
static int trackCount = 0;
static bool haveBBox = false;
static float minLat, maxLat, minLon, maxLon;

// Для расчёта скорости - последние точки с временем (unix ts, сек)
static Pt speedBuf[SPEED_AVG_POINTS];
static int speedBufCount = 0;
static int speedBufHead = 0;

File currentLogFile;
unsigned long recordingStartMs = 0;
unsigned long lastFileRotateMs = 0;
unsigned long lastWriteMs = 0;
unsigned long sessionLinesWritten = 0;
unsigned long sessionWifiScansTotal = 0;   // суммарно сетей насканировано за сессию
int lastWifiCount = 0;                     // кол-во сетей в последнем завершённом скане
String lastWifiSSIDs = "";                 // "ssid1,ssid2,ssid3" последнего скана

unsigned long lastDisplayUpdate = 0;
unsigned long lastWifiScanStart = 0;
bool wifiScanRunning = false;
bool sdReady = false;

// LVGL объекты
lv_obj_t *idleScreen;
lv_obj_t *idleStatusLabel;
lv_obj_t *idleWifiLabel;
lv_obj_t *idleTimeLabel;
lv_obj_t *idleHintLabel;

lv_obj_t *recScreen;
lv_obj_t *recTableLabel;
lv_obj_t *canvas;
static void *canvasBuf = nullptr;

lv_obj_t *stoppedScreen;

// ---------------------------------------------------------------------------
// Время: unix timestamp <-> дата/время (UTC), алгоритм Howard Hinnant
// ---------------------------------------------------------------------------
static int64_t daysFromCivil(int y, int m, int d) {
    y -= (m <= 2);
    int64_t era = (y >= 0 ? y : y - 399) / 400;
    unsigned yoe = (unsigned)(y - era * 400);
    unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097LL + (int64_t)doe - 719468;
}

static void civilFromDays(int64_t z, int &y, int &m, int &d) {
    z += 719468;
    int64_t era = (z >= 0 ? z : z - 146096) / 146097;
    unsigned doe = (unsigned)(z - era * 146097);
    unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    int64_t yy = (int64_t)yoe + era * 400;
    unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    unsigned mp = (5 * doy + 2) / 153;
    d = doy - (153 * mp + 2) / 5 + 1;
    m = mp + (mp < 10 ? 3 : -9);
    y = (int)(yy + (m <= 2));
}

static uint32_t toUnixTime(int y, int mo, int d, int h, int mi, int s) {
    int64_t days = daysFromCivil(y, mo, d);
    return (uint32_t)(days * 86400LL + h * 3600 + mi * 60 + s);
}

// Разбирает unix ts обратно в год/месяц/день/час/мин/сек
static void fromUnixTime(uint32_t ts, int &y, int &mo, int &d, int &h, int &mi, int &s) {
    int64_t days = (int64_t)(ts / 86400);
    uint32_t rem = ts % 86400;
    civilFromDays(days, y, mo, d);
    h = rem / 3600;
    mi = (rem % 3600) / 60;
    s = rem % 60;
}

// Текущее время UTC+3 из GPS. Возвращает false, если GPS ещё не дал валидное время.
static bool getLocalTime(int &y, int &mo, int &d, int &h, int &mi, int &s) {
    if (!instance.gps.date.isValid() || !instance.gps.time.isValid()) return false;
    uint32_t utcTs = toUnixTime(instance.gps.date.year(), instance.gps.date.month(), instance.gps.date.day(),
                                 instance.gps.time.hour(), instance.gps.time.minute(), instance.gps.time.second());
    uint32_t localTs = utcTs + UTC_OFFSET_SECONDS;
    fromUnixTime(localTs, y, mo, d, h, mi, s);
    return true;
}

// ---------------------------------------------------------------------------
// Расчёт скорости: haversine между соседними точками, среднее по последним N
// ---------------------------------------------------------------------------
static double haversineMeters(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371000.0;
    double dLat = (lat2 - lat1) * M_PI / 180.0;
    double dLon = (lon2 - lon1) * M_PI / 180.0;
    double a = sin(dLat / 2) * sin(dLat / 2) +
               cos(lat1 * M_PI / 180.0) * cos(lat2 * M_PI / 180.0) *
               sin(dLon / 2) * sin(dLon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return R * c;
}

static void pushSpeedPoint(float lat, float lon, uint32_t ts) {
    speedBuf[speedBufHead] = { lat, lon, ts };
    speedBufHead = (speedBufHead + 1) % SPEED_AVG_POINTS;
    if (speedBufCount < SPEED_AVG_POINTS) speedBufCount++;
}

// Средняя скорость (км/ч) по последним точкам в speedBuf, в хронологическом порядке
static float computeAvgSpeedKmh() {
    if (speedBufCount < 2) return -1.0f; // недостаточно данных -> N/A

    // Восстанавливаем хронологический порядок точек в буфере
    int startIdx = (speedBufHead - speedBufCount + SPEED_AVG_POINTS) % SPEED_AVG_POINTS;
    double totalDist = 0;
    double totalTime = 0;
    Pt prev = speedBuf[startIdx];
    for (int i = 1; i < speedBufCount; i++) {
        int idx = (startIdx + i) % SPEED_AVG_POINTS;
        Pt cur = speedBuf[idx];
        double dt = (double)cur.ts - (double)prev.ts;
        if (dt > 0) {
            double dist = haversineMeters(prev.lat, prev.lon, cur.lat, cur.lon);
            totalDist += dist;
            totalTime += dt;
        }
        prev = cur;
    }
    if (totalTime <= 0) return 0.0f;
    double mps = totalDist / totalTime;
    return (float)(mps * 3.6); // м/с -> км/ч
}

// ---------------------------------------------------------------------------
// SD: работа с файлом лога
// ---------------------------------------------------------------------------
static void closeCurrentFile() {
    if (currentLogFile) {
        currentLogFile.flush();
        currentLogFile.close();
    }
}

// Открывает новый файл log_ГГГГММДД_ЧЧММСС.txt в /sd/WIFIGPS (время - локальное, UTC+3)
static bool openNewLogFile() {
    if (!sdReady) return false;
    if (!SD.exists(SD_ROOT)) SD.mkdir(SD_ROOT);

    int y, mo, d, h, mi, s;
    char path[96];
    if (getLocalTime(y, mo, d, h, mi, s)) {
        snprintf(path, sizeof(path), "%s/log_%04d%02d%02d_%02d%02d%02d.txt",
                 SD_ROOT, y, mo, d, h, mi, s);
    } else {
        // GPS ещё не дал время - используем аптайм устройства, чтобы не потерять файл
        snprintf(path, sizeof(path), "%s/log_uptime_%lu.txt", SD_ROOT, millis() / 1000UL);
    }

    closeCurrentFile();
    currentLogFile = SD.open(path, FILE_WRITE);
    return (bool)currentLogFile;
}

// Пишет одну строку в текущий файл: HH:MM:SS_lat_lon_wifi1,wifi2,...
static void writeLogLine() {
    if (!currentLogFile) return;
    if (!instance.gps.location.isValid()) return; // без валидных координат строку не пишем

    int y, mo, d, h, mi, s;
    char timeStr[16];
    if (getLocalTime(y, mo, d, h, mi, s)) {
        snprintf(timeStr, sizeof(timeStr), "%02d:%02d:%02d", h, mi, s);
    } else {
        strncpy(timeStr, "--:--:--", sizeof(timeStr));
    }

    currentLogFile.printf("%s_%.6f_%.6f_%s\n",
                          timeStr,
                          instance.gps.location.lat(),
                          instance.gps.location.lng(),
                          lastWifiSSIDs.c_str());
    currentLogFile.flush();
    sessionLinesWritten++;
}

// ---------------------------------------------------------------------------
// WiFi сканирование (асинхронное, не блокирует GPS/экран)
// ---------------------------------------------------------------------------
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
            lastWifiCount = n;
            sessionWifiScansTotal += n;

            String ssids = "";
            for (int i = 0; i < n; i++) {
                String ssid = WiFi.SSID(i);
                ssid.replace(",", " ");
                ssid.replace("_", " ");
                if (i > 0) ssids += ",";
                ssids += ssid;
            }
            lastWifiSSIDs = ssids;

            WiFi.scanDelete();
            wifiScanRunning = false;
        } else if (n == WIFI_SCAN_FAILED) {
            wifiScanRunning = false;
        }
    }
}

// ---------------------------------------------------------------------------
// Трек на канвасе (красный на чёрном)
// ---------------------------------------------------------------------------
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
        needFullRedraw = true;
    } else {
        if (lat < minLat) { minLat = lat; needFullRedraw = true; }
        if (lat > maxLat) { maxLat = lat; needFullRedraw = true; }
        if (lon < minLon) { minLon = lon; needFullRedraw = true; }
        if (lon > maxLon) { maxLon = lon; needFullRedraw = true; }
    }

    int32_t prevX = 0, prevY = 0;
    bool hadPrev = (trackCount > 0);
    if (hadPrev) mapLatLonToCanvas(track[trackCount - 1].lat, track[trackCount - 1].lon, prevX, prevY);

    Pt p = { lat, lon, 0 };
    if (trackCount < MAX_TRACK_POINTS) {
        track[trackCount++] = p;
    } else {
        memmove(&track[0], &track[1], sizeof(Pt) * (MAX_TRACK_POINTS - 1));
        track[MAX_TRACK_POINTS - 1] = p;
    }

    if (needFullRedraw) {
        redrawFullTrack();
    } else if (hadPrev) {
        int32_t x1, y1;
        mapLatLonToCanvas(lat, lon, x1, y1);
        drawTrackSegment(prevX, prevY, x1, y1);
    }
}

static void resetTrack() {
    trackCount = 0;
    haveBBox = false;
    if (canvas) lv_canvas_fill_bg(canvas, lv_color_black(), LV_OPA_COVER);
}

// ---------------------------------------------------------------------------
// Экраны LVGL
// ---------------------------------------------------------------------------
static void buildIdleScreen() {
    idleScreen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(idleScreen, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(idleScreen, LV_OPA_COVER, 0);

    idleStatusLabel = lv_label_create(idleScreen);
    lv_obj_set_style_text_color(idleStatusLabel, lv_color_white(), 0);
    lv_obj_set_style_text_font(idleStatusLabel, &lv_font_montserrat_20, 0);
    lv_obj_align(idleStatusLabel, LV_ALIGN_TOP_LEFT, 8, 8);

    idleWifiLabel = lv_label_create(idleScreen);
    lv_obj_set_style_text_color(idleWifiLabel, lv_color_white(), 0);
    lv_obj_align(idleWifiLabel, LV_ALIGN_TOP_LEFT, 8, 40);

    idleTimeLabel = lv_label_create(idleScreen);
    lv_obj_set_style_text_color(idleTimeLabel, lv_color_white(), 0);
    lv_obj_align(idleTimeLabel, LV_ALIGN_TOP_LEFT, 8, 64);

    idleHintLabel = lv_label_create(idleScreen);
    lv_obj_set_style_text_color(idleHintLabel, lv_color_make(255, 60, 60), 0);
    lv_obj_set_style_text_font(idleHintLabel, &lv_font_montserrat_20, 0);
    lv_obj_align(idleHintLabel, LV_ALIGN_BOTTOM_LEFT, 8, -8);
    lv_label_set_text(idleHintLabel, "ENTER - начать запись");
}

static void buildRecScreen() {
    recScreen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(recScreen, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(recScreen, LV_OPA_COVER, 0);

    recTableLabel = lv_label_create(recScreen);
    lv_obj_set_style_text_color(recTableLabel, lv_color_white(), 0);
    lv_obj_align(recTableLabel, LV_ALIGN_TOP_LEFT, 4, 2);

    size_t bufSize = (size_t)SCR_W * TRACK_H * 2; // RGB565
    canvasBuf = ps_malloc(bufSize);
    canvas = lv_canvas_create(recScreen);
    lv_canvas_set_buffer(canvas, canvasBuf, SCR_W, TRACK_H, LV_COLOR_FORMAT_RGB565);
    lv_obj_align(canvas, LV_ALIGN_TOP_LEFT, 0, HEADER_H);
    lv_canvas_fill_bg(canvas, lv_color_black(), LV_OPA_COVER);
}

static void buildStoppedScreen() {
    stoppedScreen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(stoppedScreen, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(stoppedScreen, LV_OPA_COVER, 0);

    lv_obj_t *label = lv_label_create(stoppedScreen);
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_20, 0);
    lv_label_set_text(label, "Приложение остановлено.\nПерезагрузите устройство для повторного запуска.");
    lv_obj_center(label);
}

static void updateIdleScreen() {
    bool fixOk = instance.gps.location.isValid() &&
                 instance.gps.satellites.isValid() &&
                 instance.gps.satellites.value() >= MIN_SATS_FOR_FIX;

    if (fixOk) {
        lv_label_set_text_fmt(idleStatusLabel, "GPS: фикс получен (спутников: %d)",
                               instance.gps.satellites.value());
    } else if (instance.gps.satellites.isValid() && instance.gps.satellites.value() > 0) {
        lv_label_set_text_fmt(idleStatusLabel, "GPS: поиск... (спутников: %d)",
                               instance.gps.satellites.value());
    } else {
        lv_label_set_text(idleStatusLabel, "GPS: поиск спутников...");
    }

    lv_label_set_text_fmt(idleWifiLabel, "WiFi: готов, видно сетей: %d", lastWifiCount);

    int y, mo, d, h, mi, s;
    if (getLocalTime(y, mo, d, h, mi, s)) {
        lv_label_set_text_fmt(idleTimeLabel, "Время (UTC+3): %02d:%02d:%02d  %04d-%02d-%02d",
                               h, mi, s, y, mo, d);
    } else {
        lv_label_set_text(idleTimeLabel, "Время (UTC+3): ожидание данных GPS...");
    }

    if (!sdReady) {
        lv_label_set_text(idleHintLabel, "SD-карта не найдена! Запись недоступна.");
    } else {
        lv_label_set_text(idleHintLabel, "ENTER - начать запись   |   E - выход");
    }
}

static void updateRecScreen() {
    float speed = computeAvgSpeedKmh();
    char speedStr[16];
    if (speed < 0) strcpy(speedStr, "N/A");
    else snprintf(speedStr, sizeof(speedStr), "%.1f км/ч", speed);

    unsigned long fileSizeKb = currentLogFile ? (currentLogFile.size() / 1024UL) : 0;

    if (instance.gps.location.isValid()) {
        lv_label_set_text_fmt(recTableLabel,
            "ЗАПИСЬ   Sat:%d   Lat:%.6f Lon:%.6f\n"
            "WiFi видно:%d   Скорость:%s   Файл:%lu КБ\n"
            "Строк записано:%lu   WiFi просканировано (сумма):%lu",
            instance.gps.satellites.isValid() ? instance.gps.satellites.value() : 0,
            instance.gps.location.lat(), instance.gps.location.lng(),
            lastWifiCount, speedStr, fileSizeKb,
            sessionLinesWritten, sessionWifiScansTotal);
    } else {
        lv_label_set_text_fmt(recTableLabel,
            "ЗАПИСЬ   Sat:%d   Lat:--.------ Lon:--.------ (ждём фикс)\n"
            "WiFi видно:%d   Скорость:%s   Файл:%lu КБ\n"
            "Строк записано:%lu   WiFi просканировано (сумма):%lu",
            instance.gps.satellites.isValid() ? instance.gps.satellites.value() : 0,
            lastWifiCount, speedStr, fileSizeKb,
            sessionLinesWritten, sessionWifiScansTotal);
    }
}

// ---------------------------------------------------------------------------
// Переходы между состояниями
// ---------------------------------------------------------------------------
static void startRecording() {
    if (!sdReady) return; // требование 9: нет SD - не начинаем запись
    sessionLinesWritten = 0;
    sessionWifiScansTotal = 0;
    resetTrack();
    speedBufCount = 0;
    speedBufHead = 0;
    recordingStartMs = millis();
    lastFileRotateMs = recordingStartMs;
    lastWriteMs = 0; // первая запись случится сразу на первом тике handleRecording
    openNewLogFile();
    appState = APP_RECORDING;
    lv_scr_load(recScreen);
}

static void stopRecording() {
    closeCurrentFile();
    appState = APP_IDLE;
    lv_scr_load(idleScreen);
}

static void exitApplication() {
    closeCurrentFile();
    appState = APP_STOPPED;
    lv_scr_load(stoppedScreen);
}

// ---------------------------------------------------------------------------
// Обработка клавиатуры
// ---------------------------------------------------------------------------
static void handleKeyboard() {
    char c;
    int st = instance.kb.getKey(&c);
    if (st != KB_PRESSED) return;

    if (c == 0x0A || c == '\r' || c == '\n') { // ENTER
        if (appState == APP_IDLE) {
            startRecording();
        } else if (appState == APP_RECORDING) {
            stopRecording();
        }
    } else if (c == 'e' || c == 'E') {
        if (appState != APP_STOPPED) {
            exitApplication();
        }
    }
}

// ---------------------------------------------------------------------------
// Логика записи: тикер на запись строки раз в 30 сек + ротация файла каждые 30 мин
// ---------------------------------------------------------------------------
static void handleRecording() {
    instance.gps.loop();

    bool fixOk = instance.gps.location.isValid() &&
                 instance.gps.satellites.isValid() &&
                 instance.gps.satellites.value() >= MIN_SATS_FOR_FIX;

    if (fixOk && instance.gps.location.isUpdated()) {
        addTrackPointAndDraw(instance.gps.location.lat(), instance.gps.location.lng());

        uint32_t ts = toUnixTime(instance.gps.date.year(), instance.gps.date.month(), instance.gps.date.day(),
                                 instance.gps.time.hour(), instance.gps.time.minute(), instance.gps.time.second());
        pushSpeedPoint(instance.gps.location.lat(), instance.gps.location.lng(), ts);
    }

    unsigned long now = millis();

    // Ротация файла каждые 30 минут
    if (now - lastFileRotateMs >= FILE_ROTATE_MS) {
        openNewLogFile();
        lastFileRotateMs = now;
    }

    // Запись строки раз в 30 секунд
    if (lastWriteMs == 0 || now - lastWriteMs >= WRITE_INTERVAL_MS) {
        writeLogLine();
        lastWriteMs = now;
    }
}

// ---------------------------------------------------------------------------
// setup / loop
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);

    instance.begin();
    beginLvglHelper(instance);
    instance.setBrightness(DEVICE_MAX_BRIGHTNESS_LEVEL);

    buildIdleScreen();
    buildRecScreen();
    buildStoppedScreen();
    lv_scr_load(idleScreen);
    lv_timer_handler();

    // SD-карта (LilyGoLib монтирует в /sd)
    int retry = 5;
    do {
        sdReady = instance.installSD();
        if (!sdReady) delay(500);
    } while (!sdReady && --retry > 0);
    if (!sdReady) {
        Serial.println("SD init failed!");
    }

    // WiFi - только сканирование, без подключения к сети
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    appState = APP_IDLE;
}

void loop() {
    if (appState == APP_STOPPED) {
        // Приложение остановлено пользователем ("E") - ничего больше не делаем
        lv_timer_handler();
        delay(20);
        return;
    }

    handleKeyboard();
    handleWifiScan();

    if (appState == APP_RECORDING) {
        handleRecording();
    } else {
        // В режиме ожидания просто читаем GPS, чтобы статус на экране был актуальным
        instance.gps.loop();
    }

    unsigned long now = millis();
    if (now - lastDisplayUpdate >= DISPLAY_UPDATE_MS) {
        if (appState == APP_IDLE) updateIdleScreen();
        else if (appState == APP_RECORDING) updateRecScreen();
        lastDisplayUpdate = now;
    }

    lv_timer_handler();
    delay(2);
}
