/*
 * GPS + WiFi + Meshtastic tracker for LILYGO T-LoRa Pager (ESP32-S3, LilyGoLib)
 * -----------------------------------------------------------------------------
 * Recording (background state, independent of what's on screen):
 *  - ENTER - start recording (requires SD card)
 *  - ENTER again - stop recording (file is flushed and closed)
 *  - "E" - full exit: recording stops, file is closed, device shows a
 *    "stopped" screen and does nothing further (reboot required)
 *
 * Display modes ("S" cycles: Track -> Diagnostics -> Grid -> Track ...):
 *  1. Track: red track on black + track length (km) + status/coords/battery
 *  2. Diagnostics: full-screen, scrollable (rotary encoder up/down) - GPS raw
 *     stats, WiFi list, Meshtastic status, SD, memory, battery
 *  3. Grid: one cell per module (GPS / WiFi / Meshtastic / SD), green if OK,
 *     red if there's a problem/no data, + battery
 *
 * A battery percentage indicator is shown on every screen (top-right corner).
 *
 * Buzzer (via the onboard ES8311 codec + speaker):
 *  - 5 long beeps whenever a Meshtastic sighting is heard
 *  - 1 long beep when GPS acquires a fix (on the searching -> fix transition)
 *  - 1 short beep every 40s while the battery is low
 *
 * SD card: /WIFIGPS folder at the SD root, file log_YYYYMMDD_HHMMSS.txt (new
 * file every 30 minutes of recording). One line written every 30 seconds,
 * ALWAYS (even with no GPS fix):
 *   HH:MM:SS_lat_lon_status_speed_ssid1|bssid1|rssi1,..._M1:node1|rssi1,...
 * status is "ok" or "bad_gps" (lat/lon are 0.000000 for bad_gps). speed is
 * km/h (average of the last 5 fixes) or "NA". Each WiFi network is logged as
 * ssid|bssid|rssi. Each Meshtastic sighting is logged as
 * <channel_tag>:<node_id_hex>|rssi (node_id is the sender's permanent,
 * MAC-derived Meshtastic node number, read from the packet's plaintext
 * header - no decryption needed or performed). Empty lists leave nothing
 * between their surrounding "_" separators.
 *
 * WiFi scanning is asynchronous and never blocks GPS/display. Meshtastic
 * listening is passive (no join, no transmit at all): the SX1262 alternates
 * every ~20s between two known local channels and reads the plaintext header
 * of anything it hears. Since a single radio can only listen to one channel
 * at a time, this only ever catches a share of local traffic - a best-effort
 * supplementary data source, not an exhaustive scanner.
 *
 * Library: LilyGoLib (https://github.com/Xinyuan-LilyGO/LilyGoLib)
 * Board (fqbn): esp32:esp32:tlora_pager
 */

#include <LilyGoLib.h>
#include <LV_Helper.h>
#include <WiFi.h>
#include <SD.h>
#include <math.h>

// ---------------------------------------------------------------------------
// Meshtastic passive-listening settings
// ---------------------------------------------------------------------------
#define MESHTASTIC_SYNC_WORD    0x2B
#define MESHTASTIC_PREAMBLE_LEN 16

struct MeshtasticChannel {
    const char *tag;
    float freqMHz;
    float bandwidthKHz;
    uint8_t spreadingFactor;
    uint8_t codingRate;  // denominator only, e.g. 7 means 4/7
};
static const MeshtasticChannel MESHTASTIC_CHANNELS[] = {
    { "M1", 868.731018, 62.5, 7, 7 },  // user-provided: custom channel, 62.5kHz/SF7/CR4:7
    { "M2", 869.075,    250.0, 9, 5 }, // MEDIUM_FAST preset, frequency overridden to slot 2
};
#define MESHTASTIC_CHANNEL_COUNT 2
#define MESHTASTIC_CHANNEL_SWITCH_MS 20000UL
#define MAX_LORA_ENTRIES_PER_WRITE 8
#define MESHTASTIC_STALE_MS (5UL * 60UL * 1000UL) // "no recent activity" threshold for the grid view

// ---------------------------------------------------------------------------
// General settings
// ---------------------------------------------------------------------------
#define WIFI_SCAN_INTERVAL_MS   15000UL
#define WRITE_INTERVAL_MS       30000UL
#define FILE_ROTATE_MS          (30UL * 60UL * 1000UL)
#define DISPLAY_UPDATE_MS       500UL
#define MAX_TRACK_POINTS        2000
#define MIN_SATS_FOR_FIX        3
#define SPEED_AVG_POINTS        5
#define UTC_OFFSET_SECONDS      (3 * 3600)
#define LOW_BATTERY_PCT         15
#define LOW_BATTERY_BEEP_MS     40000UL

#define SD_ROOT "/WIFIGPS"

#define SCR_W      480
#define SCR_H      222
#define HEADER_H   96
#define TRACK_H    (SCR_H - HEADER_H)

// ---------------------------------------------------------------------------
// Application state
// ---------------------------------------------------------------------------
enum DisplayMode { MODE_TRACK, MODE_DIAG, MODE_GRID };
DisplayMode displayMode = MODE_TRACK;
bool recording = false;
bool appStopped = false;

struct Pt { float lat, lon; uint32_t ts; };
static Pt track[MAX_TRACK_POINTS];
static int trackCount = 0;
static bool haveBBox = false;
static float minLat, maxLat, minLon, maxLon;
static double trackLengthMeters = 0.0;

static Pt speedBuf[SPEED_AVG_POINTS];
static int speedBufCount = 0;
static int speedBufHead = 0;

File currentLogFile;
unsigned long recordingStartMs = 0;
unsigned long lastFileRotateMs = 0;
unsigned long lastWriteMs = 0;
unsigned long sessionLinesWritten = 0;
unsigned long sessionWifiScansTotal = 0;
int lastWifiCount = 0;
String lastWifiSSIDs = "";

#define MAX_LORA_BUFFER 16
String loraBufferIds[MAX_LORA_BUFFER];
int loraBufferRssi[MAX_LORA_BUFFER];
int loraBufferCount = 0;
unsigned long sessionLoraSightingsTotal = 0;
volatile bool loraPacketReady = false;
int currentMeshtasticChannelIdx = 0;
unsigned long lastChannelSwitchMs = 0;

unsigned long bootLoraSightingsTotal = 0;
String lastMeshtasticId = "";
int lastMeshtasticRssi = 0;
unsigned long lastMeshtasticMs = 0;

unsigned long lastDisplayUpdate = 0;
unsigned long lastWifiScanStart = 0;
bool wifiScanRunning = false;
bool sdReady = false;

bool hadGpsFix = false;         // for edge-detecting the "fix acquired" beep
unsigned long lastLowBattBeepMs = 0;
bool audioReady = false;

// LVGL objects
lv_obj_t *trackScreen;
lv_obj_t *trackStatusLabel;
lv_obj_t *canvas;
static void *canvasBuf = nullptr;

lv_obj_t *diagScreen;
lv_obj_t *diagLabel;

lv_obj_t *gridScreen;
lv_obj_t *gridCellGps, *gridCellWifi, *gridCellLora, *gridCellSd;
lv_obj_t *gridLabelGps, *gridLabelWifi, *gridLabelLora, *gridLabelSd;
lv_obj_t *gridBatteryLabel;

lv_obj_t *stoppedScreen;

// Small battery badges added to the track/diag screens (grid has its own gridBatteryLabel)
lv_obj_t *trackBatteryLabel;
lv_obj_t *diagBatteryLabel;

// ---------------------------------------------------------------------------
// Time: unix timestamp <-> date/time (UTC), Howard Hinnant's algorithm
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

static void fromUnixTime(uint32_t ts, int &y, int &mo, int &d, int &h, int &mi, int &s) {
    int64_t days = (int64_t)(ts / 86400);
    uint32_t rem = ts % 86400;
    civilFromDays(days, y, mo, d);
    h = rem / 3600;
    mi = (rem % 3600) / 60;
    s = rem % 60;
}

static bool getLocalTime(int &y, int &mo, int &d, int &h, int &mi, int &s) {
    if (!instance.gps.date.isValid() || !instance.gps.time.isValid()) return false;
    uint32_t utcTs = toUnixTime(instance.gps.date.year(), instance.gps.date.month(), instance.gps.date.day(),
                                 instance.gps.time.hour(), instance.gps.time.minute(), instance.gps.time.second());
    uint32_t localTs = utcTs + UTC_OFFSET_SECONDS;
    fromUnixTime(localTs, y, mo, d, h, mi, s);
    return true;
}

// ---------------------------------------------------------------------------
// Battery (BQ25896 charger IC via instance.ppm - voltage only, no fuel gauge,
// so percentage is a standard LiPo discharge-curve approximation)
// ---------------------------------------------------------------------------
static int batteryPercentFromVoltage(uint16_t mv) {
    struct { uint16_t mv; int pct; } curve[] = {
        {4200, 100}, {4100, 90}, {4000, 80}, {3900, 70}, {3800, 60},
        {3700, 40}, {3600, 20}, {3500, 10}, {3400, 5}, {3300, 0}
    };
    const int n = sizeof(curve) / sizeof(curve[0]);
    if (mv >= curve[0].mv) return 100;
    if (mv <= curve[n - 1].mv) return 0;
    for (int i = 0; i < n - 1; i++) {
        if (mv <= curve[i].mv && mv >= curve[i + 1].mv) {
            float frac = (float)(mv - curve[i + 1].mv) / (float)(curve[i].mv - curve[i + 1].mv);
            return curve[i + 1].pct + (int)(frac * (curve[i].pct - curve[i + 1].pct));
        }
    }
    return 0;
}

static void getBatteryInfo(int &percent, bool &charging) {
    uint16_t mv = instance.ppm.getBattVoltage();
    percent = batteryPercentFromVoltage(mv);
    charging = instance.ppm.isCharging();
}

// Updates any battery label with the current "Batt:NN% [CHG]" text.
static void updateBatteryLabel(lv_obj_t *label) {
    if (!label) return;
    int pct; bool charging;
    getBatteryInfo(pct, charging);
    lv_color_t color = (pct <= LOW_BATTERY_PCT) ? lv_color_make(255, 80, 80) : lv_color_white();
    lv_obj_set_style_text_color(label, color, 0);
    lv_label_set_text_fmt(label, "Batt:%d%%%s", pct, charging ? " CHG" : "");
}

// ---------------------------------------------------------------------------
// Buzzer (ES8311 codec + speaker) - see LilyGoLib examples/peripheral/SimpleTone
// ---------------------------------------------------------------------------
#define BEEP_SAMPLE_RATE 16000
#define BEEP_MAX_MS      400
static int16_t beepBuffer[BEEP_MAX_MS * BEEP_SAMPLE_RATE / 1000];

static void initAudio() {
    instance.powerControl(POWER_SPEAK, true);
#ifdef USING_AUDIO_CODEC
    instance.codec.setVolume(80);
    audioReady = (instance.codec.open(16, 1, BEEP_SAMPLE_RATE) >= 0);
    if (!audioReady) Serial.println("Audio codec open failed - beeps disabled.");
#else
    audioReady = false;
#endif
}

// Blocking (but short) - generates and plays one sine-wave tone.
static void playTone(int freqHz, int durationMs, float volume = 0.6f) {
    if (!audioReady) return;
    if (durationMs > BEEP_MAX_MS) durationMs = BEEP_MAX_MS;
    int samples = durationMs * BEEP_SAMPLE_RATE / 1000;
    for (int i = 0; i < samples; i++) {
        beepBuffer[i] = (int16_t)(32767.0f * sinf(2.0f * PI * freqHz * i / BEEP_SAMPLE_RATE) * volume);
    }
#ifdef USING_AUDIO_CODEC
    instance.codec.write((uint8_t *)beepBuffer, samples * sizeof(int16_t));
#endif
}

static void beepPattern(int count, int toneMs, int gapMs, int freqHz = 1200) {
    for (int i = 0; i < count; i++) {
        playTone(freqHz, toneMs);
        if (i < count - 1) delay(gapMs);
    }
}

static void beepMeshtasticFound()  { beepPattern(5, 120, 90, 1400); }   // 5 long-ish beeps
static void beepGpsFixAcquired()   { beepPattern(1, 350, 0, 1000); }    // 1 long beep
static void beepLowBattery()       { beepPattern(1, 90, 0, 700); }      // 1 short beep

// ---------------------------------------------------------------------------
// Speed: haversine distance between consecutive points, averaged over last N
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

static float computeAvgSpeedKmh() {
    if (speedBufCount < 2) return -1.0f;
    int startIdx = (speedBufHead - speedBufCount + SPEED_AVG_POINTS) % SPEED_AVG_POINTS;
    double totalDist = 0, totalTime = 0;
    Pt prev = speedBuf[startIdx];
    for (int i = 1; i < speedBufCount; i++) {
        int idx = (startIdx + i) % SPEED_AVG_POINTS;
        Pt cur = speedBuf[idx];
        double dt = (double)cur.ts - (double)prev.ts;
        if (dt > 0) {
            totalDist += haversineMeters(prev.lat, prev.lon, cur.lat, cur.lon);
            totalTime += dt;
        }
        prev = cur;
    }
    if (totalTime <= 0) return 0.0f;
    return (float)((totalDist / totalTime) * 3.6);
}

// ---------------------------------------------------------------------------
// SD: log file handling
// ---------------------------------------------------------------------------
static void closeCurrentFile() {
    if (currentLogFile) {
        currentLogFile.flush();
        currentLogFile.close();
    }
}

static bool openNewLogFile() {
    if (!sdReady) return false;
    if (!SD.exists(SD_ROOT)) SD.mkdir(SD_ROOT);

    int y, mo, d, h, mi, s;
    char path[96];
    if (getLocalTime(y, mo, d, h, mi, s)) {
        snprintf(path, sizeof(path), "%s/log_%04d%02d%02d_%02d%02d%02d.txt",
                 SD_ROOT, y, mo, d, h, mi, s);
    } else {
        snprintf(path, sizeof(path), "%s/log_uptime_%lu.txt", SD_ROOT, millis() / 1000UL);
    }

    closeCurrentFile();
    currentLogFile = SD.open(path, FILE_WRITE);
    return (bool)currentLogFile;
}

static String flushLoraBufferToString() {
    String out = "";
    int n = min(loraBufferCount, MAX_LORA_ENTRIES_PER_WRITE);
    for (int i = 0; i < n; i++) {
        if (i > 0) out += ",";
        out += loraBufferIds[i];
        out += "|";
        out += String(loraBufferRssi[i]);
    }
    loraBufferCount = 0;
    return out;
}

static void writeLogLine() {
    if (!currentLogFile) return;

    int y, mo, d, h, mi, s;
    char timeStr[16];
    if (getLocalTime(y, mo, d, h, mi, s)) {
        snprintf(timeStr, sizeof(timeStr), "%02d:%02d:%02d", h, mi, s);
    } else {
        strncpy(timeStr, "--:--:--", sizeof(timeStr));
    }

    char speedStr[16];
    float speed = computeAvgSpeedKmh();
    if (speed < 0) strcpy(speedStr, "NA");
    else snprintf(speedStr, sizeof(speedStr), "%.1f", speed);

    String loraStr = flushLoraBufferToString();

    bool fixOk = instance.gps.location.isValid() &&
                 instance.gps.satellites.isValid() &&
                 instance.gps.satellites.value() >= MIN_SATS_FOR_FIX;

    if (fixOk) {
        currentLogFile.printf("%s_%.6f_%.6f_ok_%s_%s_%s\n",
                              timeStr,
                              instance.gps.location.lat(),
                              instance.gps.location.lng(),
                              speedStr,
                              lastWifiSSIDs.c_str(),
                              loraStr.c_str());
    } else {
        currentLogFile.printf("%s_0.000000_0.000000_bad_gps_%s_%s_%s\n",
                              timeStr,
                              speedStr,
                              lastWifiSSIDs.c_str(),
                              loraStr.c_str());
    }
    currentLogFile.flush();
    sessionLinesWritten++;
}

// ---------------------------------------------------------------------------
// WiFi scanning (asynchronous, never blocks GPS/display)
// ---------------------------------------------------------------------------
static void handleWifiScan() {
    unsigned long now = millis();

    if (!wifiScanRunning && (lastWifiScanStart == 0 || now - lastWifiScanStart >= WIFI_SCAN_INTERVAL_MS)) {
        WiFi.scanNetworks(true, true);
        wifiScanRunning = true;
        lastWifiScanStart = now;
    }

    if (wifiScanRunning) {
        int n = WiFi.scanComplete();
        if (n >= 0) {
            lastWifiCount = n;
            sessionWifiScansTotal += n;

            String entries = "";
            for (int i = 0; i < n; i++) {
                String ssid = WiFi.SSID(i);
                ssid.replace(",", " ");
                ssid.replace("_", " ");
                ssid.replace("|", " ");
                if (i > 0) entries += ",";
                entries += ssid;
                entries += "|";
                entries += WiFi.BSSIDstr(i);
                entries += "|";
                entries += String(WiFi.RSSI(i));
            }
            lastWifiSSIDs = entries;

            WiFi.scanDelete();
            wifiScanRunning = false;
        } else if (n == WIFI_SCAN_FAILED) {
            wifiScanRunning = false;
        }
    }
}

// ---------------------------------------------------------------------------
// Meshtastic passive listening (no join, no transmit - just receive + parse header)
// ---------------------------------------------------------------------------
IRAM_ATTR void onLoraPacket() {
    loraPacketReady = true;
}

static bool tuneToMeshtasticChannel(int idx) {
    const MeshtasticChannel &ch = MESHTASTIC_CHANNELS[idx];
    int state;

    radio.standby();

    state = radio.setFrequency(ch.freqMHz);
    if (state != RADIOLIB_ERR_NONE) { Serial.printf("Meshtastic: setFrequency(%s) failed, %d\n", ch.tag, state); return false; }
    state = radio.setBandwidth(ch.bandwidthKHz);
    if (state != RADIOLIB_ERR_NONE) { Serial.printf("Meshtastic: setBandwidth(%s) failed, %d\n", ch.tag, state); return false; }
    state = radio.setSpreadingFactor(ch.spreadingFactor);
    if (state != RADIOLIB_ERR_NONE) { Serial.printf("Meshtastic: setSpreadingFactor(%s) failed, %d\n", ch.tag, state); return false; }
    state = radio.setCodingRate(ch.codingRate);
    if (state != RADIOLIB_ERR_NONE) { Serial.printf("Meshtastic: setCodingRate(%s) failed, %d\n", ch.tag, state); return false; }
    state = radio.setSyncWord(MESHTASTIC_SYNC_WORD);
    if (state != RADIOLIB_ERR_NONE) { Serial.printf("Meshtastic: setSyncWord(%s) failed, %d\n", ch.tag, state); return false; }
    state = radio.setPreambleLength(MESHTASTIC_PREAMBLE_LEN);
    if (state != RADIOLIB_ERR_NONE) { Serial.printf("Meshtastic: setPreambleLength(%s) failed, %d\n", ch.tag, state); return false; }

    radio.setDio1Action(onLoraPacket);
    state = radio.startReceive();
    if (state != RADIOLIB_ERR_NONE) { Serial.printf("Meshtastic: startReceive(%s) failed, %d\n", ch.tag, state); return false; }

    Serial.printf("Meshtastic listener: tuned to %s (%.6f MHz)\n", ch.tag, ch.freqMHz);
    return true;
}

static bool setupLoraListener() {
    lastChannelSwitchMs = millis();
    return tuneToMeshtasticChannel(currentMeshtasticChannelIdx);
}

static void bufferLoraSighting(const String &id, int rssi) {
    lastMeshtasticId = id;
    lastMeshtasticRssi = rssi;
    lastMeshtasticMs = millis();
    bootLoraSightingsTotal++;
    beepMeshtasticFound();

    for (int i = 0; i < loraBufferCount; i++) {
        if (loraBufferIds[i] == id) {
            if (rssi > loraBufferRssi[i]) loraBufferRssi[i] = rssi;
            return;
        }
    }
    if (loraBufferCount < MAX_LORA_BUFFER) {
        loraBufferIds[loraBufferCount] = id;
        loraBufferRssi[loraBufferCount] = rssi;
        loraBufferCount++;
    }
    sessionLoraSightingsTotal++;
}

// Parses the plaintext Meshtastic packet header (16 bytes, never encrypted):
// to(4) + from(4) + packet_id(4) + flags(1) + channel_hash(1) + next_hop(1) + relay_node(1)
static void parseMeshtasticFrame(uint8_t *data, size_t len, int rssi, int channelIdx) {
    if (len < 8) return;

    char nodeId[9];
    snprintf(nodeId, sizeof(nodeId), "%02X%02X%02X%02X", data[7], data[6], data[5], data[4]);

    String id = String(MESHTASTIC_CHANNELS[channelIdx].tag) + ":" + String(nodeId);
    Serial.printf("Meshtastic: heard %s, rssi=%d dBm, len=%u\n", id.c_str(), rssi, (unsigned)len);
    bufferLoraSighting(id, rssi);
}

static void handleLoraRx() {
    unsigned long now = millis();
    if (now - lastChannelSwitchMs >= MESHTASTIC_CHANNEL_SWITCH_MS) {
        currentMeshtasticChannelIdx = (currentMeshtasticChannelIdx + 1) % MESHTASTIC_CHANNEL_COUNT;
        tuneToMeshtasticChannel(currentMeshtasticChannelIdx);
        lastChannelSwitchMs = now;
        return;
    }

    if (!loraPacketReady) return;
    loraPacketReady = false;

    uint8_t buf[64];
    size_t len = radio.getPacketLength();
    if (len > 0 && len <= sizeof(buf)) {
        int state = radio.readData(buf, len);
        if (state == RADIOLIB_ERR_NONE) {
            int rssi = (int)radio.getRSSI();
            parseMeshtasticFrame(buf, len, rssi, currentMeshtasticChannelIdx);
        }
    }
    radio.startReceive();
}

// ---------------------------------------------------------------------------
// Track on the canvas (red on black)
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
    if (hadPrev) {
        mapLatLonToCanvas(track[trackCount - 1].lat, track[trackCount - 1].lon, prevX, prevY);
        trackLengthMeters += haversineMeters(track[trackCount - 1].lat, track[trackCount - 1].lon, lat, lon);
    }

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
    trackLengthMeters = 0.0;
    if (canvas) lv_canvas_fill_bg(canvas, lv_color_black(), LV_OPA_COVER);
}

// ---------------------------------------------------------------------------
// LVGL screens
// ---------------------------------------------------------------------------
static lv_obj_t *createBatteryLabel(lv_obj_t *parent) {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_obj_align(label, LV_ALIGN_TOP_RIGHT, -6, 4);
    lv_label_set_text(label, "Batt:--%");
    return label;
}

static void buildTrackScreen() {
    trackScreen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(trackScreen, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(trackScreen, LV_OPA_COVER, 0);

    trackStatusLabel = lv_label_create(trackScreen);
    lv_obj_set_style_text_color(trackStatusLabel, lv_color_white(), 0);
    lv_obj_align(trackStatusLabel, LV_ALIGN_TOP_LEFT, 4, 2);

    trackBatteryLabel = createBatteryLabel(trackScreen);

    size_t bufSize = (size_t)SCR_W * TRACK_H * 2;
    canvasBuf = ps_malloc(bufSize);
    canvas = lv_canvas_create(trackScreen);
    lv_canvas_set_buffer(canvas, canvasBuf, SCR_W, TRACK_H, LV_COLOR_FORMAT_RGB565);
    lv_obj_align(canvas, LV_ALIGN_TOP_LEFT, 0, HEADER_H);
    lv_canvas_fill_bg(canvas, lv_color_black(), LV_OPA_COVER);
}

static void buildDiagScreen() {
    diagScreen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(diagScreen, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(diagScreen, LV_OPA_COVER, 0);
    lv_obj_set_scroll_dir(diagScreen, LV_DIR_VER);

    diagBatteryLabel = createBatteryLabel(diagScreen);
    lv_obj_add_flag(diagBatteryLabel, LV_OBJ_FLAG_FLOATING); // stays put while content scrolls

    diagLabel = lv_label_create(diagScreen);
    lv_obj_set_style_text_color(diagLabel, lv_color_make(120, 255, 120), 0);
    lv_obj_set_style_text_font(diagLabel, &lv_font_montserrat_14, 0);
    lv_obj_set_width(diagLabel, SCR_W - 16);
    lv_label_set_long_mode(diagLabel, LV_LABEL_LONG_WRAP);
    lv_obj_align(diagLabel, LV_ALIGN_TOP_LEFT, 6, 26);
}

static void buildGridScreen() {
    gridScreen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(gridScreen, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(gridScreen, LV_OPA_COVER, 0);
    lv_obj_clear_flag(gridScreen, LV_OBJ_FLAG_SCROLLABLE);

    gridBatteryLabel = createBatteryLabel(gridScreen);

    int cellW = SCR_W / 2 - 6;
    int cellH = SCR_H / 2 - 6;
    gridCellGps  = lv_obj_create(gridScreen);
    gridCellWifi = lv_obj_create(gridScreen);
    gridCellLora = lv_obj_create(gridScreen);
    gridCellSd   = lv_obj_create(gridScreen);

    lv_obj_t *cells[4]      = { gridCellGps, gridCellWifi, gridCellLora, gridCellSd };
    lv_obj_t **labels[4]    = { &gridLabelGps, &gridLabelWifi, &gridLabelLora, &gridLabelSd };
    lv_align_t aligns[4]    = { LV_ALIGN_TOP_LEFT, LV_ALIGN_TOP_RIGHT, LV_ALIGN_BOTTOM_LEFT, LV_ALIGN_BOTTOM_RIGHT };
    int xoff[4] = { 4, -4, 4, -4 };
    int yoff[4] = { 4, 4, -4, -4 };

    for (int i = 0; i < 4; i++) {
        lv_obj_set_size(cells[i], cellW, cellH);
        lv_obj_align(cells[i], aligns[i], xoff[i], yoff[i]);
        lv_obj_set_style_bg_color(cells[i], lv_color_make(60, 60, 60), 0);
        lv_obj_set_style_bg_opa(cells[i], LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(cells[i], 0, 0);
        lv_obj_set_style_radius(cells[i], 6, 0);
        lv_obj_clear_flag(cells[i], LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *label = lv_label_create(cells[i]);
        lv_obj_set_style_text_color(label, lv_color_white(), 0);
        lv_obj_align(label, LV_ALIGN_TOP_LEFT, 4, 2);
        *labels[i] = label;
    }
}

static void buildStoppedScreen() {
    stoppedScreen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(stoppedScreen, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(stoppedScreen, LV_OPA_COVER, 0);

    lv_obj_t *label = lv_label_create(stoppedScreen);
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_20, 0);
    lv_label_set_text(label, "Application stopped.\nReboot the device to start again.");
    lv_obj_center(label);
}

// ---------------------------------------------------------------------------
// Screen updates
// ---------------------------------------------------------------------------
static void updateTrackScreen() {
    float speed = computeAvgSpeedKmh();
    char speedStr[16];
    if (speed < 0) strcpy(speedStr, "N/A");
    else snprintf(speedStr, sizeof(speedStr), "%.1f km/h", speed);

    double lengthKm = trackLengthMeters / 1000.0;

    if (instance.gps.location.isValid()) {
        lv_label_set_text_fmt(trackStatusLabel,
            "%s   Sat:%d   Lat:%.6f Lon:%.6f\n"
            "Track length: %.2f km   Speed:%s   Lines:%lu",
            recording ? "RECORDING" : "IDLE",
            instance.gps.satellites.isValid() ? instance.gps.satellites.value() : 0,
            instance.gps.location.lat(), instance.gps.location.lng(),
            lengthKm, speedStr, sessionLinesWritten);
    } else {
        lv_label_set_text_fmt(trackStatusLabel,
            "%s   Sat:%d   Lat:--.------ Lon:--.------ (waiting for fix)\n"
            "Track length: %.2f km   Speed:%s   Lines:%lu",
            recording ? "RECORDING" : "IDLE",
            instance.gps.satellites.isValid() ? instance.gps.satellites.value() : 0,
            lengthKm, speedStr, sessionLinesWritten);
    }
    updateBatteryLabel(trackBatteryLabel);
}

static String formatWifiListForDiag(int maxEntries) {
    if (lastWifiSSIDs.length() == 0) return "  (none)";
    String out = "";
    int start = 0, shown = 0;
    while (start < (int)lastWifiSSIDs.length() && shown < maxEntries) {
        int comma = lastWifiSSIDs.indexOf(',', start);
        String entry = (comma == -1) ? lastWifiSSIDs.substring(start) : lastWifiSSIDs.substring(start, comma);
        int p1 = entry.indexOf('|');
        int p2 = (p1 == -1) ? -1 : entry.indexOf('|', p1 + 1);
        if (p1 != -1 && p2 != -1) {
            String ssid = entry.substring(0, p1);
            if (ssid.length() == 0) ssid = "(hidden)";
            String rssi = entry.substring(p2 + 1);
            out += "  " + ssid + " (" + rssi + " dBm)\n";
            shown++;
        }
        if (comma == -1) break;
        start = comma + 1;
    }
    return (shown == 0) ? "  (none)" : out;
}

static void updateDiagScreen() {
    bool fixOk = instance.gps.location.isValid() &&
                 instance.gps.satellites.isValid() &&
                 instance.gps.satellites.value() >= MIN_SATS_FOR_FIX;

    char gpsLine[80];
    if (instance.gps.location.isValid()) {
        snprintf(gpsLine, sizeof(gpsLine), "Lat:%.6f Lon:%.6f Alt:%.0fm Spd:%.1fkm/h HDOP:%.1f",
                 instance.gps.location.lat(), instance.gps.location.lng(),
                 instance.gps.altitude.isValid() ? instance.gps.altitude.meters() : 0.0,
                 instance.gps.speed.isValid() ? instance.gps.speed.kmph() : 0.0,
                 instance.gps.hdop.isValid() ? instance.gps.hdop.hdop() : 0.0);
    } else {
        snprintf(gpsLine, sizeof(gpsLine), "Lat:--.------ Lon:--.------ (no fix yet)");
    }

    unsigned long meshAgoSec = (lastMeshtasticMs > 0) ? (millis() - lastMeshtasticMs) / 1000 : 0;
    char meshLastLine[64];
    if (lastMeshtasticMs > 0) {
        snprintf(meshLastLine, sizeof(meshLastLine), "%s, rssi %d dBm, %lus ago",
                 lastMeshtasticId.c_str(), lastMeshtasticRssi, meshAgoSec);
    } else {
        strcpy(meshLastLine, "(none heard yet)");
    }

    int battPct; bool battCharging;
    getBatteryInfo(battPct, battCharging);

    lv_label_set_text_fmt(diagLabel,
        "SELF-DIAGNOSTICS   (S or ENTER to exit, scroll wheel to scroll)\n"
        "\n"
        "GPS: %s, sats:%d, chars:%lu, sentences:%lu, failed-cksum:%lu\n"
        "%s\n"
        "\n"
        "WiFi: last scan %d networks\n"
        "%s"
        "\n"
        "Meshtastic: channel %s (%.6f MHz), heard total:%lu\n"
        "  last: %s\n"
        "\n"
        "SD: %s   Battery: %d%%%s\n"
        "Free heap: %lu KB   Free PSRAM: %lu KB",
        fixOk ? "FIX" : "searching",
        instance.gps.satellites.isValid() ? instance.gps.satellites.value() : 0,
        instance.gps.charsProcessed(), instance.gps.sentencesWithFix(), instance.gps.failedChecksum(),
        gpsLine,
        lastWifiCount,
        formatWifiListForDiag(4).c_str(),
        MESHTASTIC_CHANNELS[currentMeshtasticChannelIdx].tag,
        MESHTASTIC_CHANNELS[currentMeshtasticChannelIdx].freqMHz,
        bootLoraSightingsTotal,
        meshLastLine,
        sdReady ? "ready" : "NOT FOUND",
        battPct, battCharging ? " (charging)" : "",
        (unsigned long)(ESP.getFreeHeap() / 1024),
        (unsigned long)(ESP.getFreePsram() / 1024));

    updateBatteryLabel(diagBatteryLabel);
}

static void setCellColor(lv_obj_t *cell, bool ok) {
    lv_obj_set_style_bg_color(cell, ok ? lv_color_make(30, 110, 30) : lv_color_make(140, 30, 30), 0);
}

static void updateGridScreen() {
    bool gpsOk = instance.gps.location.isValid() &&
                 instance.gps.satellites.isValid() &&
                 instance.gps.satellites.value() >= MIN_SATS_FOR_FIX;
    setCellColor(gridCellGps, gpsOk);
    lv_label_set_text_fmt(gridLabelGps, "GPS\n%s\nSat:%d",
        gpsOk ? "FIX" : "NO FIX",
        instance.gps.satellites.isValid() ? instance.gps.satellites.value() : 0);

    bool wifiOk = lastWifiCount > 0;
    setCellColor(gridCellWifi, wifiOk);
    lv_label_set_text_fmt(gridLabelWifi, "WiFi\n%s\n%d networks",
        wifiOk ? "OK" : "NONE SEEN", lastWifiCount);

    bool loraOk = (lastMeshtasticMs > 0) && (millis() - lastMeshtasticMs < MESHTASTIC_STALE_MS);
    setCellColor(gridCellLora, loraOk);
    if (lastMeshtasticMs > 0) {
        lv_label_set_text_fmt(gridLabelLora, "Meshtastic\n%s\n%lus ago",
            loraOk ? "OK" : "STALE", (millis() - lastMeshtasticMs) / 1000);
    } else {
        lv_label_set_text(gridLabelLora, "Meshtastic\nNONE HEARD\n-");
    }

    setCellColor(gridCellSd, sdReady);
    lv_label_set_text_fmt(gridLabelSd, "SD Card\n%s\n%s",
        sdReady ? "READY" : "NOT FOUND",
        recording ? "recording" : "idle");

    updateBatteryLabel(gridBatteryLabel);
}

// ---------------------------------------------------------------------------
// State transitions
// ---------------------------------------------------------------------------
static void loadCurrentModeScreen() {
    switch (displayMode) {
        case MODE_TRACK: lv_scr_load(trackScreen); break;
        case MODE_DIAG:  lv_scr_load(diagScreen);  break;
        case MODE_GRID:  lv_scr_load(gridScreen);  break;
    }
}

static void cycleDisplayMode() {
    displayMode = (DisplayMode)((displayMode + 1) % 3);
    loadCurrentModeScreen();
}

static void startRecording() {
    if (!sdReady) return;
    sessionLinesWritten = 0;
    sessionWifiScansTotal = 0;
    sessionLoraSightingsTotal = 0;
    loraBufferCount = 0;
    resetTrack();
    speedBufCount = 0;
    speedBufHead = 0;
    recordingStartMs = millis();
    lastFileRotateMs = recordingStartMs;
    lastWriteMs = 0;
    openNewLogFile();
    recording = true;
}

static void stopRecording() {
    closeCurrentFile();
    recording = false;
}

static void exitApplication() {
    closeCurrentFile();
    appStopped = true;
    lv_scr_load(stoppedScreen);
}

// ---------------------------------------------------------------------------
// Keyboard + rotary encoder handling
// ---------------------------------------------------------------------------
static void handleKeyboard() {
    char c;
    int st = instance.kb.getKey(&c);
    if (st == KB_PRESSED) {
        if (c == 0x0A || c == '\r' || c == '\n') {
            if (recording) stopRecording();
            else startRecording();
        } else if (c == 'e' || c == 'E') {
            exitApplication();
        } else if (c == 's' || c == 'S') {
            cycleDisplayMode();
        }
    }

    RotaryMsg_t rot = instance.getRotary();
    if (rot.dir != ROTARY_DIR_NONE) {
        if (displayMode == MODE_DIAG) {
            lv_obj_scroll_by(diagScreen, 0, (rot.dir == ROTARY_DIR_DOWN) ? -30 : 30, LV_ANIM_ON);
        }
        instance.clearRotaryMsg();
    }
}

// ---------------------------------------------------------------------------
// Recording logic: write a line every 30 sec + rotate the file every 30 min
// ---------------------------------------------------------------------------
static void handleRecording() {
    instance.gps.loop();

    bool wasFix = hadGpsFix;
    bool fixOk = instance.gps.location.isValid() &&
                 instance.gps.satellites.isValid() &&
                 instance.gps.satellites.value() >= MIN_SATS_FOR_FIX;
    if (fixOk && !wasFix) beepGpsFixAcquired();
    hadGpsFix = fixOk;

    if (fixOk && instance.gps.location.isUpdated()) {
        addTrackPointAndDraw(instance.gps.location.lat(), instance.gps.location.lng());
        uint32_t ts = toUnixTime(instance.gps.date.year(), instance.gps.date.month(), instance.gps.date.day(),
                                 instance.gps.time.hour(), instance.gps.time.minute(), instance.gps.time.second());
        pushSpeedPoint(instance.gps.location.lat(), instance.gps.location.lng(), ts);
    }

    unsigned long now = millis();

    if (now - lastFileRotateMs >= FILE_ROTATE_MS) {
        openNewLogFile();
        lastFileRotateMs = now;
    }

    if (lastWriteMs == 0 || now - lastWriteMs >= WRITE_INTERVAL_MS) {
        writeLogLine();
        lastWriteMs = now;
    }
}

// Tracks the fix edge-detection even while not recording, so the "GPS fix
// acquired" beep also works before you start a recording session.
static void handleGpsFixWatcher() {
    bool fixOk = instance.gps.location.isValid() &&
                 instance.gps.satellites.isValid() &&
                 instance.gps.satellites.value() >= MIN_SATS_FOR_FIX;
    if (fixOk && !hadGpsFix) beepGpsFixAcquired();
    hadGpsFix = fixOk;
}

static void handleLowBatteryBeep() {
    unsigned long now = millis();
    if (now - lastLowBattBeepMs < LOW_BATTERY_BEEP_MS) return;
    int pct; bool charging;
    getBatteryInfo(pct, charging);
    if (pct <= LOW_BATTERY_PCT && !charging) {
        beepLowBattery();
    }
    lastLowBattBeepMs = now;
}

// ---------------------------------------------------------------------------
// setup / loop
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);

    instance.begin();
    beginLvglHelper(instance);
    instance.setBrightness(DEVICE_MAX_BRIGHTNESS_LEVEL);
    instance.enableRotary();

    buildTrackScreen();
    buildDiagScreen();
    buildGridScreen();
    buildStoppedScreen();
    displayMode = MODE_TRACK;
    lv_scr_load(trackScreen);
    lv_timer_handler();

    initAudio();

    int retry = 5;
    do {
        sdReady = instance.installSD();
        if (!sdReady) delay(500);
    } while (!sdReady && --retry > 0);
    if (!sdReady) {
        Serial.println("SD init failed!");
    }

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (!setupLoraListener()) {
        Serial.println("Meshtastic listener init failed - continuing without it.");
    }

    lastLowBattBeepMs = millis();
}

void loop() {
    if (appStopped) {
        lv_timer_handler();
        delay(20);
        return;
    }

    handleKeyboard();
    handleWifiScan();
    handleLoraRx();
    handleLowBatteryBeep();

    if (recording) {
        handleRecording();
    } else {
        instance.gps.loop();
        handleGpsFixWatcher();
    }

    unsigned long now = millis();
    if (now - lastDisplayUpdate >= DISPLAY_UPDATE_MS) {
        switch (displayMode) {
            case MODE_TRACK: updateTrackScreen(); break;
            case MODE_DIAG:  updateDiagScreen();  break;
            case MODE_GRID:  updateGridScreen();  break;
        }
        lastDisplayUpdate = now;
    }

    lv_timer_handler();
    delay(2);
}
