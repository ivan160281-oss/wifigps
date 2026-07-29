/*
 * GPS + WiFi + LoRa tracker for LILYGO T-LoRa Pager (ESP32-S3, LilyGoLib)
 * -----------------------------------------------------------------
 * Behaviour:
 *  - Idle screen (not recording): GPS status, WiFi status, time (UTC+3),
 *    number of currently visible WiFi networks, "ENTER - start recording"
 *  - ENTER (physical T-LoRa Pager keyboard) - start recording
 *  - ENTER while recording - stop recording, file is closed (flushed),
 *    back to idle screen
 *  - "E" - full exit: recording stops, file is closed, device shows a
 *    "stopped" screen and does nothing further (reboot required)
 *  - While recording: table (satellites, coordinates, visible WiFi count,
 *    speed - average of last 5 points, current file size) + red track on
 *    a black background
 *  - SD card: /WIFIGPS folder at the SD root, file log_YYYYMMDD_HHMMSS.txt
 *    (new file every 30 minutes of recording)
 *  - One line written every 30 seconds, ALWAYS (even with no GPS fix):
 *    HH:MM:SS_lat_lon_status_speed_ssid1|bssid1|rssi1,ssid2|bssid2|rssi2,..._M1:node1|rssi1,M2:node2|rssi2,...
 *    status is "ok" when the fix is valid, "bad_gps" otherwise (lat/lon are
 *    written as 0.000000 in that case). speed is km/h (average of the last
 *    5 fixes) or "NA" if not enough data yet. Each WiFi network is logged as
 *    ssid|bssid|rssi (bssid = MAC address, rssi = signal strength in dBm);
 *    networks are comma-separated. Each Meshtastic sighting is logged as
 *    <channel_tag>:<node_id_hex>|rssi (node_id is the sender's permanent,
 *    MAC-derived Meshtastic node number). If a list is empty, nothing sits
 *    between its surrounding "_" separators.
 *  - WiFi scanning is asynchronous and never blocks GPS/display
 *  - LoRa listening is passive Meshtastic sniffing (no join, no transmit at
 *    all): the SX1262 sits in continuous receive, alternating every ~20s
 *    between two known local Meshtastic channels (Moscow region), and reads
 *    the plaintext 16-byte packet header of anything it hears - specifically
 *    the "from" field (a permanent, MAC-derived node number, sent
 *    unencrypted by design), with no need for the channel's PSK/encryption
 *    key. Meshtastic itself uses AES to encrypt only the payload past the
 *    header. Each sighting is logged as M1:<hex node id> or M2:<hex node id>
 *    depending on which of the two channels it was heard on.
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
// These constants are fixed by the Meshtastic protocol itself (same for every
// channel/region) - NOT something to tune per network:
#define MESHTASTIC_SYNC_WORD    0x2B
#define MESHTASTIC_PREAMBLE_LEN 16

// Two known local channels (Moscow region), switched between periodically since
// one SX1262 can only listen to one frequency/SF/BW combination at a time.
struct MeshtasticChannel {
    const char *tag;    // short label used as the id prefix in the log
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
#define MESHTASTIC_CHANNEL_SWITCH_MS 20000UL // how often to hop to the other channel

#define MAX_LORA_ENTRIES_PER_WRITE 8 // cap how many distinct sightings we log per 30s window

// ---------------------------------------------------------------------------
// Settings
// ---------------------------------------------------------------------------
#define WIFI_SCAN_INTERVAL_MS   15000UL   // WiFi scan period (non-blocking)
#define WRITE_INTERVAL_MS       30000UL   // write one line every 30 sec
#define FILE_ROTATE_MS          (30UL * 60UL * 1000UL) // new file every 30 min
#define DISPLAY_UPDATE_MS       500UL     // display refresh period
#define MAX_TRACK_POINTS        2000      // points kept in memory for drawing
#define MIN_SATS_FOR_FIX        3         // min satellites for a valid fix
#define SPEED_AVG_POINTS        5         // speed averaged over last N points
#define UTC_OFFSET_SECONDS      (3 * 3600) // UTC+3

#define SD_ROOT "/WIFIGPS"

// Screen area 480x222
#define SCR_W      480
#define SCR_H      222
#define HEADER_H   96                     // top text block (status/table)
#define TRACK_H    (SCR_H - HEADER_H)

// ---------------------------------------------------------------------------
// Application state
// ---------------------------------------------------------------------------
enum AppState { APP_IDLE, APP_RECORDING, APP_STOPPED };
AppState appState = APP_IDLE;

struct Pt { float lat, lon; uint32_t ts; };
static Pt track[MAX_TRACK_POINTS];
static int trackCount = 0;
static bool haveBBox = false;
static float minLat, maxLat, minLon, maxLon;

// Speed calculation - last points with timestamp (unix ts, sec)
static Pt speedBuf[SPEED_AVG_POINTS];
static int speedBufCount = 0;
static int speedBufHead = 0;

File currentLogFile;
unsigned long recordingStartMs = 0;
unsigned long lastFileRotateMs = 0;
unsigned long lastWriteMs = 0;
unsigned long sessionLinesWritten = 0;
unsigned long sessionWifiScansTotal = 0;   // total networks scanned this session
int lastWifiCount = 0;                     // networks seen in the last completed scan
String lastWifiSSIDs = "";                 // "ssid1,ssid2,ssid3" from the last scan

// LoRa: distinct sightings collected since the last log write (reset every WRITE_INTERVAL_MS)
#define MAX_LORA_BUFFER 16
String loraBufferIds[MAX_LORA_BUFFER];     // e.g. "A:26D1FA3B" or "J:0004A30B001A2BFE"
int loraBufferRssi[MAX_LORA_BUFFER];
int loraBufferCount = 0;
unsigned long sessionLoraSightingsTotal = 0;
volatile bool loraPacketReady = false;
int currentMeshtasticChannelIdx = 0;
unsigned long lastChannelSwitchMs = 0;

unsigned long lastDisplayUpdate = 0;
unsigned long lastWifiScanStart = 0;
bool wifiScanRunning = false;
bool sdReady = false;

// LVGL objects
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

// Splits a unix ts back into year/month/day/hour/min/sec
static void fromUnixTime(uint32_t ts, int &y, int &mo, int &d, int &h, int &mi, int &s) {
    int64_t days = (int64_t)(ts / 86400);
    uint32_t rem = ts % 86400;
    civilFromDays(days, y, mo, d);
    h = rem / 3600;
    mi = (rem % 3600) / 60;
    s = rem % 60;
}

// Current UTC+3 time derived from GPS. Returns false if GPS has no valid time yet.
static bool getLocalTime(int &y, int &mo, int &d, int &h, int &mi, int &s) {
    if (!instance.gps.date.isValid() || !instance.gps.time.isValid()) return false;
    uint32_t utcTs = toUnixTime(instance.gps.date.year(), instance.gps.date.month(), instance.gps.date.day(),
                                 instance.gps.time.hour(), instance.gps.time.minute(), instance.gps.time.second());
    uint32_t localTs = utcTs + UTC_OFFSET_SECONDS;
    fromUnixTime(localTs, y, mo, d, h, mi, s);
    return true;
}

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

// Average speed (km/h) over the points currently in speedBuf, oldest to newest
static float computeAvgSpeedKmh() {
    if (speedBufCount < 2) return -1.0f; // not enough data -> N/A

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
    return (float)(mps * 3.6); // m/s -> km/h
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

// Opens a new log_YYYYMMDD_HHMMSS.txt file in /sd/WIFIGPS (local time, UTC+3)
static bool openNewLogFile() {
    if (!sdReady) return false;
    if (!SD.exists(SD_ROOT)) SD.mkdir(SD_ROOT);

    int y, mo, d, h, mi, s;
    char path[96];
    if (getLocalTime(y, mo, d, h, mi, s)) {
        snprintf(path, sizeof(path), "%s/log_%04d%02d%02d_%02d%02d%02d.txt",
                 SD_ROOT, y, mo, d, h, mi, s);
    } else {
        // GPS has no time yet - fall back to device uptime so we don't lose the file
        snprintf(path, sizeof(path), "%s/log_uptime_%lu.txt", SD_ROOT, millis() / 1000UL);
    }

    closeCurrentFile();
    currentLogFile = SD.open(path, FILE_WRITE);
    return (bool)currentLogFile;
}

// Writes one line to the current file every 30 sec, always - even with no GPS fix.
// Format: HH:MM:SS_lat_lon_status_speed_ssid1|bssid1|rssi1,ssid2|bssid2|rssi2,..._M1:node1|rssi1,...
// status is "ok" when the GPS fix is valid, "bad_gps" otherwise (no fix / not enough satellites).
// speed is km/h (average of the last 5 fixes), or "NA" if not enough data yet.
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
        WiFi.scanNetworks(true /* async */, true /* show_hidden */);
        wifiScanRunning = true;
        lastWifiScanStart = now;
    }

    if (wifiScanRunning) {
        int n = WiFi.scanComplete();
        if (n >= 0) {
            lastWifiCount = n;
            sessionWifiScansTotal += n;

            // Each entry: ssid|bssid|rssi , separated by commas between networks.
            // ssid is sanitized so it can't contain '_', ',' or '|' (our delimiters).
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
// Called from radio interrupt context - keep it minimal, just set a flag.
IRAM_ATTR void onLoraPacket() {
    loraPacketReady = true;
}

// Applies one of the two known channel configs and (re)starts continuous receive.
// Uses individual RadioLib setters on the ALREADY-initialized radio object
// (set up once by instance.begin()) instead of calling radio.begin() again -
// a fresh begin() risks resetting board-specific low-level config (TCXO
// voltage, regulator mode) that only instance.begin() knows how to set
// correctly for this exact hardware, which could silently break receive
// sensitivity entirely.
static bool tuneToMeshtasticChannel(int idx) {
    const MeshtasticChannel &ch = MESHTASTIC_CHANNELS[idx];
    int state;

    radio.standby();

    state = radio.setFrequency(ch.freqMHz);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("Meshtastic listener: setFrequency(%s) failed, code %d\n", ch.tag, state);
        return false;
    }
    state = radio.setBandwidth(ch.bandwidthKHz);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("Meshtastic listener: setBandwidth(%s) failed, code %d\n", ch.tag, state);
        return false;
    }
    state = radio.setSpreadingFactor(ch.spreadingFactor);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("Meshtastic listener: setSpreadingFactor(%s) failed, code %d\n", ch.tag, state);
        return false;
    }
    state = radio.setCodingRate(ch.codingRate);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("Meshtastic listener: setCodingRate(%s) failed, code %d\n", ch.tag, state);
        return false;
    }
    state = radio.setSyncWord(MESHTASTIC_SYNC_WORD);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("Meshtastic listener: setSyncWord(%s) failed, code %d\n", ch.tag, state);
        return false;
    }
    state = radio.setPreambleLength(MESHTASTIC_PREAMBLE_LEN);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("Meshtastic listener: setPreambleLength(%s) failed, code %d\n", ch.tag, state);
        return false;
    }

    radio.setDio1Action(onLoraPacket);
    state = radio.startReceive();
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("Meshtastic listener: startReceive(%s) failed, code %d\n", ch.tag, state);
        return false;
    }

    Serial.printf("Meshtastic listener: tuned to %s (%.6f MHz)\n", ch.tag, ch.freqMHz);
    return true;
}

static bool setupLoraListener() {
    lastChannelSwitchMs = millis();
    return tuneToMeshtasticChannel(currentMeshtasticChannelIdx);
}

// Adds one sighting to the buffer that will be flushed into the next log line.
// Skips duplicates of an id already buffered in this window (keeps the
// strongest RSSI seen for it instead).
static void bufferLoraSighting(const String &id, int rssi) {
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
// We only need "from" - the sender's node number, derived from its Bluetooth
// MAC address and therefore permanent for that physical device. No decryption
// of the payload is needed or attempted.
static void parseMeshtasticFrame(uint8_t *data, size_t len, int rssi, int channelIdx) {
    if (len < 8) return; // too short to even contain to+from

    char nodeId[9];
    // "from" is bytes[4..7], little-endian on the wire -> print MSB first
    snprintf(nodeId, sizeof(nodeId), "%02X%02X%02X%02X", data[7], data[6], data[5], data[4]);

    String id = String(MESHTASTIC_CHANNELS[channelIdx].tag) + ":" + String(nodeId);
    Serial.printf("Meshtastic: heard %s, rssi=%d dBm, len=%u\n", id.c_str(), rssi, (unsigned)len);
    bufferLoraSighting(id, rssi);
}

static void handleLoraRx() {
    // Hop to the other known channel periodically so we get a share of both.
    unsigned long now = millis();
    if (now - lastChannelSwitchMs >= MESHTASTIC_CHANNEL_SWITCH_MS) {
        currentMeshtasticChannelIdx = (currentMeshtasticChannelIdx + 1) % MESHTASTIC_CHANNEL_COUNT;
        tuneToMeshtasticChannel(currentMeshtasticChannelIdx);
        lastChannelSwitchMs = now;
        return; // give the new config a full cycle before reading anything
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
    radio.startReceive(); // resume listening
}

// Builds the "M1:node1|rssi1,M2:node2|rssi2,..." string for the current buffer,
// then clears the buffer for the next collection window.
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
// LVGL screens
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
    lv_label_set_text(idleHintLabel, "ENTER - start recording");
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
    lv_label_set_text(label, "Application stopped.\nReboot the device to start again.");
    lv_obj_center(label);
}

static void updateIdleScreen() {
    bool fixOk = instance.gps.location.isValid() &&
                 instance.gps.satellites.isValid() &&
                 instance.gps.satellites.value() >= MIN_SATS_FOR_FIX;

    if (fixOk) {
        lv_label_set_text_fmt(idleStatusLabel, "GPS: fix acquired (satellites: %d)",
                               instance.gps.satellites.value());
    } else if (instance.gps.satellites.isValid() && instance.gps.satellites.value() > 0) {
        lv_label_set_text_fmt(idleStatusLabel, "GPS: searching... (satellites: %d)",
                               instance.gps.satellites.value());
    } else {
        lv_label_set_text(idleStatusLabel, "GPS: searching for satellites...");
    }

    lv_label_set_text_fmt(idleWifiLabel, "WiFi: ready, networks visible: %d", lastWifiCount);

    int y, mo, d, h, mi, s;
    if (getLocalTime(y, mo, d, h, mi, s)) {
        lv_label_set_text_fmt(idleTimeLabel, "Time (UTC+3): %02d:%02d:%02d  %04d-%02d-%02d",
                               h, mi, s, y, mo, d);
    } else {
        lv_label_set_text(idleTimeLabel, "Time (UTC+3): waiting for GPS data...");
    }

    if (!sdReady) {
        lv_label_set_text(idleHintLabel, "SD card not found! Recording unavailable.");
    } else {
        lv_label_set_text(idleHintLabel, "ENTER - start recording   |   E - exit");
    }
}

static void updateRecScreen() {
    float speed = computeAvgSpeedKmh();
    char speedStr[16];
    if (speed < 0) strcpy(speedStr, "N/A");
    else snprintf(speedStr, sizeof(speedStr), "%.1f km/h", speed);

    unsigned long fileSizeKb = currentLogFile ? (currentLogFile.size() / 1024UL) : 0;

    if (instance.gps.location.isValid()) {
        lv_label_set_text_fmt(recTableLabel,
            "RECORDING   Sat:%d   Lat:%.6f Lon:%.6f\n"
            "WiFi visible:%d   Speed:%s   File:%lu KB\n"
            "Lines written:%lu   WiFi scanned (total):%lu   LoRa sighted (total):%lu",
            instance.gps.satellites.isValid() ? instance.gps.satellites.value() : 0,
            instance.gps.location.lat(), instance.gps.location.lng(),
            lastWifiCount, speedStr, fileSizeKb,
            sessionLinesWritten, sessionWifiScansTotal, sessionLoraSightingsTotal);
    } else {
        lv_label_set_text_fmt(recTableLabel,
            "RECORDING   Sat:%d   Lat:--.------ Lon:--.------ (waiting for fix)\n"
            "WiFi visible:%d   Speed:%s   File:%lu KB\n"
            "Lines written:%lu   WiFi scanned (total):%lu   LoRa sighted (total):%lu",
            instance.gps.satellites.isValid() ? instance.gps.satellites.value() : 0,
            lastWifiCount, speedStr, fileSizeKb,
            sessionLinesWritten, sessionWifiScansTotal, sessionLoraSightingsTotal);
    }
}

// ---------------------------------------------------------------------------
// State transitions
// ---------------------------------------------------------------------------
static void startRecording() {
    if (!sdReady) return; // no SD card -> do not start recording
    sessionLinesWritten = 0;
    sessionWifiScansTotal = 0;
    sessionLoraSightingsTotal = 0;
    loraBufferCount = 0;
    resetTrack();
    speedBufCount = 0;
    speedBufHead = 0;
    recordingStartMs = millis();
    lastFileRotateMs = recordingStartMs;
    lastWriteMs = 0; // first write happens right away on the first handleRecording tick
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
// Keyboard handling
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
// Recording logic: write a line every 30 sec + rotate the file every 30 min
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

    // Rotate the file every 30 minutes
    if (now - lastFileRotateMs >= FILE_ROTATE_MS) {
        openNewLogFile();
        lastFileRotateMs = now;
    }

    // Write a line every 30 seconds
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

    // SD card (LilyGoLib mounts it at /sd)
    int retry = 5;
    do {
        sdReady = instance.installSD();
        if (!sdReady) delay(500);
    } while (!sdReady && --retry > 0);
    if (!sdReady) {
        Serial.println("SD init failed!");
    }

    // WiFi - scan only, never connect to a network
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    // LoRa - passive Meshtastic listening only (no join, no transmit)
    if (!setupLoraListener()) {
        Serial.println("Meshtastic listener init failed - continuing without it.");
    }

    appState = APP_IDLE;
}

void loop() {
    if (appState == APP_STOPPED) {
        // Application stopped by the user ("E") - do nothing further
        lv_timer_handler();
        delay(20);
        return;
    }

    handleKeyboard();
    handleWifiScan();
    handleLoraRx();

    if (appState == APP_RECORDING) {
        handleRecording();
    } else {
        // Idle: still read GPS so the status shown on screen stays current
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
