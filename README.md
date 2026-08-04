# GPS + WiFi + BLE tracker for LILYGO T-LoRa Pager

## How it works

**Recording** (background state, independent of what's on screen):
- Starts **automatically ~60 seconds after boot** (gives GPS/sensors a moment
  to come up) - needs an SD card
- **ENTER** toggles it off/back on manually at any time (file is flushed and
  closed when stopped)
- **E** (any time except when already stopped) - full stop: file is closed,
  screen shows "Application stopped", device does nothing further (reboot
  required to run again)

**WiFi sync** - **U** (idle only, not while recording) - automatically uploads
log files to your server over WiFi, so you don't have to pull the SD card out
by hand every time:
1. Create `/WIFIGPS/sync_config.txt` on the SD card (edit it on a computer,
   plain text, one `key=value` per line). Supports multiple candidate
   networks (e.g. a home network and a car's mobile hotspot) - tried in order
   at sync time, whichever one actually connects is used:
   ```
   ssid1=YourHomeWiFi
   password1=YourHomeWiFiPassword
   ssid2=YourCarHotspot
   password2=YourCarHotspotPassword
   server_url=http://192.168.1.50:5000
   sync_password=YourServerPassword
   ```
   (a bare `ssid=`/`password=` with no number also works, treated as network
   1, for backward compatibility with older single-network config files; use
   the plain-HTTP address of your server - see the server's own README
   for how it's normally run; `server_url` should NOT have a trailing slash.
   `sync_password` must match the server's `WIFIGPS_PASSWORD` - the server
   requires a login for everything now, and since the tracker can't do a
   browser login, it sends this password as a request header instead)
2. Press **U** from the idle screen. The device tries each configured
   network in turn until one connects, then asks the server which log files
   it doesn't already have (via `/api/sync/check`), and uploads only those
   (via `/api/upload`) - so re-syncing later never re-sends files the server
   already has.
3. Progress and a final summary ("Sync done: N uploaded, M failed") are shown
   on screen; press any key to dismiss and return to normal operation
   (WiFi switches back to scan-only mode automatically).

This whole process is blocking (GPS/WiFi-scan pause for its
duration - typically a few seconds to connect plus roughly a second per file)
since it's a deliberate, occasional action, not something meant to run in the
background during a drive.

**Single-screen display** - red track on a black background, plus a header:
- top-right: battery percentage (turns red at 15% or below)
- top-left: **recording indicator** - a filled red ball while recording, a
  hollow/dim grey ball while idle - followed by how many lines have been
  written so far this session
- two colored count chips, same colors as the server's map, so the
  association is visual rather than needing a label: **WiFi** (blue, last
  scan's network count), **BLE** (green, last scan's device count)
- status line: satellite count, coordinates, track length (km)

The old separate Diagnostics and Grid screens (and the "S" key that used to
cycle between all three) have been removed to keep things simple and
responsive - everything that matters day-to-day is on this one screen now.

**Screen sleep (battery saving)** - while running on battery power (not
charging), the display turns off automatically after **3 minutes** with no
key presses. Press **SPACE** to wake it back up - every other key is ignored
while asleep, so a stray press (e.g. in a pocket) can't accidentally toggle
recording or exit. GPS recording and WiFi/BLE scanning all keep
running normally while the screen is off; only the backlight (and the
display redraw work, to save a little more CPU/power) pauses.

**Buzzer feedback** (via the onboard ES8311 codec + speaker, at half the
codec's usual volume):
- **1 short low beep** the moment GPS satellites are found (searching -> fix)
- **3 short high beeps** the moment GPS satellites are lost (fix -> searching)
- **1 short beep every 40 seconds** while the battery is at 15% or below (and
  not charging)

**On the SD card:**
- `/WIFIGPS` folder at the root of the SD card
- a new file every 30 minutes of recording: `log_YYYYMMDD_HHMMSS.txt` (name uses local
  UTC+3 time at file-creation time)
- **circular buffer:** total log data on the SD card is capped at 10 MB. The main way
  space gets freed is that each file is deleted right after a confirmed-successful
  WiFi sync upload (see below); as a fallback, if the cap would still be exceeded (sync
  hasn't run in a while), the oldest files are deleted automatically when a new one starts
- one line every 30 seconds, ALWAYS written (even with no GPS fix), format:
  `HH:MM:SS_lat_lon_status_speed_ssid1|bssid1|rssi1,...__mac1|rssi1,..._heading_steps`
  - `status` is `ok` when the fix is valid or `bad_gps` otherwise (lat/lon are `0.000000`
    in that case)
  - `speed` is always `NA` - the firmware doesn't calculate speed at all anymore; the
    field is kept only so the log format/server parser don't need to change
  - each WiFi network is logged as `ssid|bssid|rssi` (bssid = MAC address, rssi = signal
    strength in dBm)
  - the field between the WiFi and BLE lists is always empty now - LoRa/Meshtastic
    support was removed entirely (no scanning, no listening, nothing recorded), but the
    field's position is kept so the format doesn't shift
  - each BLE sighting is logged as `<mac>|rssi` - passive scan, no pairing/connecting.
    Many phones/wearables rotate their BLE address for privacy every ~15 min, so they
    won't build up repeat sightings; genuinely fixed devices (smart-home gear, beacons)
    will, which is exactly what makes them useful reference points here
  - `heading` is the BHI260AP sensor hub's fused compass heading in degrees, `steps` is
    its cumulative hardware step counter - both `NA` if the sensor isn't detected. This
    is raw data only; the firmware doesn't attempt any dead-reckoning math itself
  - nothing sits between the surrounding `_` separators if a list is empty
- the red track still only uses points with a valid fix - `bad_gps` rows don't affect
  the drawn track or the track length

**About BLE scanning (passive, no pairing/connecting):** uses the classic Arduino BLE
library bundled with arduino-esp32. Because that API is blocking (and only accepts
whole-second durations), a scan cycle is split into two 1-second chunks every 25
seconds, with a keyboard check in between, rather than one longer uninterrupted
block - this keeps key presses responsive even during a scan (see "why button
presses used to feel laggy" below). Unlike the HTTPClient and
BHI260AP sensor code (verified against LilyGoLib's own official examples), the exact
`BLEScan::start()` signature couldn't be verified against a real compile in the
environment this was written in - if the build fails specifically in
`setupBleScanner()`/`handleBleScan()`, the fix is almost certainly a small signature
mismatch for the installed arduino-esp32 core version (worth checking
`BLEScan.h` in the installed core if so).

**Why button presses used to feel laggy:** the single biggest cause was BLE
scanning blocking the main loop for a full 2 seconds at a time - any key
press during that window was missed until the scan finished, sometimes
requiring a second press. Splitting the scan into two 1-second chunks with a
keyboard check in between (see above) cuts that worst case in half. Removing
the old Diagnostics/Grid screens also helps a little, since there's no more
`lv_scr_load()` switching or extra per-frame update work for screens you
aren't looking at.

**About the IMU (BHI260AP sensor hub):** provides a hardware-fused compass heading and
a hardware step counter - no custom sensor-fusion math needed. If your board doesn't
have this sensor populated, the firmware detects that at boot and just logs `NA` for
both fields instead of failing.

## Getting a .bin without installing anything locally

The included `.github/workflows/build.yml` builds the firmware automatically in the
cloud (GitHub Actions) whenever you push to `main`/`master`. It uses the stable ESP32
core 3.3.10.

### Steps
1. Create a GitHub repo, upload the files from here keeping the paths:
   ```
   .github/workflows/build.yml
   gps_wifi_tracker/gps_wifi_tracker.ino
   ```
2. After the commit, open the **Actions** tab - the build starts automatically (~5-8 min)
3. Download `tpager-gps-wifi-tracker-bin.zip` from the **Artifacts** of the finished run
4. It contains 5 files; the simplest is to flash **`gps_wifi_tracker.ino.merged.bin`**
   at address `0x0` (e.g. via https://espressif.github.io/esptool-js/ in the browser,
   no software install needed)

## Technical notes (for developers)

- Library: [LilyGoLib](https://github.com/Xinyuan-LilyGO/LilyGoLib) (official for T-LoRa Pager)
- Board (fqbn): `esp32:esp32:tlora_pager`
- GPS is already built into the library as `instance.gps` (extends TinyGPSPlus); the UART
  is configured automatically inside `instance.begin()` at 38400 baud
- Keyboard: `instance.kb.getKey(&c)` - the board's physical QWERTY keyboard
- Screen sleep: `instance.setBrightness(0)` / `instance.setBrightness(DEVICE_MAX_BRIGHTNESS_LEVEL)` -
  turns the backlight off/on; charging state checked via `instance.ppm.isCharging()`
- Battery: read via `instance.ppm.getBattVoltage()` (BQ25896 charger IC - voltage
  only, no fuel gauge, so the percentage shown is a standard LiPo discharge-curve
  approximation, not a precisely calibrated reading)
- Speaker: `instance.codec` (ES8311 codec) - `instance.powerControl(POWER_SPEAK, true)`
  to turn on audio power (off by default), then `codec.open()`/`codec.write()` to
  play raw PCM sine-wave tones (see `examples/peripheral/SimpleTone` in LilyGoLib).
  `codec.setVolume(40)` sets the overall level (halved from the original 80)
- SD card is mounted by the library at `/sd` via `instance.installSD()`, but paths
  passed to the `SD` object itself must NOT include the `/sd` prefix (the library
  already implies it)
- The screen is drawn with LVGL (v9); the track uses `lv_canvas` with direct line drawing
- Minimum satellites for a valid fix is 3 (`MIN_SATS_FOR_FIX`, adjust if needed)
- The onboard SX1262 (LoRa radio) is entirely unused now - LoRa/Meshtastic support
  was removed from this firmware (no scanning, no listening, no `radio.*` calls
  anywhere in the sketch)
