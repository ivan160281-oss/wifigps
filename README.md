# GPS + WiFi + Meshtastic tracker for LILYGO T-LoRa Pager

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
   plain text, one `key=value` per line):
   ```
   ssid=YourWiFiName
   password=YourWiFiPassword
   server_url=http://192.168.1.50:5000
   sync_password=YourServerPassword
   ```
   (use the plain-HTTP address of your server - see the server's own README
   for how it's normally run; `server_url` should NOT have a trailing slash.
   `sync_password` must match the server's `WIFIGPS_PASSWORD` - the server
   requires a login for everything now, and since the tracker can't do a
   browser login, it sends this password as a request header instead)
2. Press **U** from the idle screen. The device connects to that WiFi network,
   asks the server which log files it doesn't already have (via
   `/api/sync/check`), and uploads only those (via `/api/upload`) - so re-
   syncing later never re-sends files the server already has.
3. Progress and a final summary ("Sync done: N uploaded, M failed") are shown
   on screen; press any key to dismiss and return to normal operation
   (WiFi switches back to scan-only mode automatically).

This whole process is blocking (GPS/WiFi-scan/Meshtastic pause for its
duration - typically a few seconds to connect plus roughly a second per file)
since it's a deliberate, occasional action, not something meant to run in the
background during a drive.

**Single-screen display** - red track on a black background, plus a header:
- top-right: battery percentage (turns red at 15% or below)
- top-left: **recording indicator** - a filled red ball while recording, a
  hollow/dim grey ball while idle - followed by how many lines have been
  written so far this session
- three colored count chips, same colors as the server's map, so the
  association is visual rather than needing a label: **WiFi** (blue, last
  scan's network count), **BLE** (green, last scan's device count),
  **Meshtastic** (purple, total sightings heard since boot - always a count,
  never a "last heard" time or "OK/stale" status)
- status line: satellite count, coordinates, track length (km), speed

The old separate Diagnostics and Grid screens (and the "S" key that used to
cycle between all three) have been removed to keep things simple and
responsive - everything that matters day-to-day is on this one screen now.

**Screen sleep (battery saving)** - while running on battery power (not
charging), the display turns off automatically after **3 minutes** with no
key presses. Press **SPACE** to wake it back up - every other key is ignored
while asleep, so a stray press (e.g. in a pocket) can't accidentally toggle
recording or exit. GPS recording and WiFi/BLE/Meshtastic scanning all keep
running normally while the screen is off; only the backlight (and the
display redraw work, to save a little more CPU/power) pauses.

**Buzzer feedback** (via the onboard ES8311 codec + speaker):
- **1 short low beep** the moment GPS satellites are found (searching -> fix)
- **3 short high beeps** the moment GPS satellites are lost (fix -> searching)
- **1 short beep every 40 seconds** while the battery is at 15% or below (and
  not charging)

(The old "5 beeps whenever a Meshtastic sighting is heard" notification has
been removed - Meshtastic activity is now only shown as a count on the
header chip, no audio interruption.)

**On the SD card:**
- `/WIFIGPS` folder at the root of the SD card
- a new file every 30 minutes of recording: `log_YYYYMMDD_HHMMSS.txt` (name uses local
  UTC+3 time at file-creation time)
- **circular buffer:** total log data on the SD card is capped at 10 MB. The main way
  space gets freed is that each file is deleted right after a confirmed-successful
  WiFi sync upload (see below); as a fallback, if the cap would still be exceeded (sync
  hasn't run in a while), the oldest files are deleted automatically when a new one starts
- one line every 30 seconds, ALWAYS written (even with no GPS fix), format:
  `HH:MM:SS_lat_lon_status_speed_ssid1|bssid1|rssi1,...__M1:node1|rssi1,...__mac1|rssi1,..._heading_steps`
  - `status` is `ok` when the fix is valid or `bad_gps` otherwise (lat/lon are `0.000000`
    in that case)
  - `speed` is km/h (average of the last 5 fixes) or `NA` if not enough data yet
  - each WiFi network is logged as `ssid|bssid|rssi` (bssid = MAC address, rssi = signal
    strength in dBm)
  - each Meshtastic sighting is logged as `<channel_tag>:<node_id>|rssi`, where
    `channel_tag` is `M1` or `M2` (which known local channel it was heard on) and
    `node_id` is the sender's permanent, MAC-derived Meshtastic node number
  - each BLE sighting is logged as `<mac>|rssi` - passive scan, no pairing/connecting.
    Many phones/wearables rotate their BLE address for privacy every ~15 min, so they
    won't build up repeat sightings; genuinely fixed devices (smart-home gear, beacons)
    will, which is exactly what makes them useful reference points here
  - `heading` is the BHI260AP sensor hub's fused compass heading in degrees, `steps` is
    its cumulative hardware step counter - both `NA` if the sensor isn't detected. This
    is raw data only; the firmware doesn't attempt any dead-reckoning math itself
  - nothing sits between the surrounding `_` separators if a list is empty
- the red track and the speed calculation still only use points with a valid fix -
  `bad_gps` rows don't affect the drawn track or the track length

**About the Meshtastic field (passive listening, no network join, no transmit):**
[Meshtastic](https://meshtastic.org/) is an open mesh-messaging protocol run over LoRa
by enthusiasts (not LoRaWAN). Its packet header is never encrypted - only the payload
past the 16-byte header is - so the sender's node id can be read directly, with no
encryption key needed. The SX1262 alternates every ~20 seconds between two known local
channels (Moscow region):
- `M1`: 868.731018 MHz, 62.5 kHz bandwidth, SF7, coding rate 4/7
- `M2`: 869.075 MHz (MEDIUM_FAST preset, frequency slot 2), 250 kHz bandwidth, SF9,
  coding rate 4/5

Both use the Meshtastic protocol's fixed sync word (0x2B) and preamble length (16).
Since a single radio can only listen to one channel/config at a time, alternating
between the two only ever catches a share of the local traffic on each - it's a
best-effort supplementary data source (much longer range than WiFi), not an exhaustive
scanner. There's no existing public database mapping Meshtastic node ids to real-world
coordinates, so (exactly like WiFi) any positioning value has to come from your own
accumulated observations over time. If you know of more local channels in use, add
them to the `MESHTASTIC_CHANNELS` array near the top of the sketch.

**If Meshtastic sightings stay at 0:** open a serial monitor at 115200 baud right
after boot. Every ~20 seconds you should see a line like
`Meshtastic listener: tuned to M1 (868.731018 MHz)` confirming the channel switch
succeeded. When an actual packet is heard, you'll see
`Meshtastic: heard M1:A1B2C3D4, rssi=-88 dBm, len=23`. If you see neither, it's most
likely that there's simply no local Meshtastic traffic on the given channel/frequency
at that moment and place - give it a few minutes in a spot with likely coverage.

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
  play raw PCM sine-wave tones (see `examples/peripheral/SimpleTone` in LilyGoLib)
- SD card is mounted by the library at `/sd` via `instance.installSD()`, but paths
  passed to the `SD` object itself must NOT include the `/sd` prefix (the library
  already implies it)
- The screen is drawn with LVGL (v9); the track uses `lv_canvas` with direct line drawing
- Minimum satellites for a valid fix is 3 (`MIN_SATS_FOR_FIX`, adjust if needed)
