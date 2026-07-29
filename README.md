# GPS + WiFi tracker for LILYGO T-LoRa Pager

## How it works

**Idle screen (not recording):**
- GPS status (searching / fix acquired, satellite count)
- WiFi status (ready, how many networks are visible right now)
- current time UTC+3 (from GPS)
- hint: `ENTER - start recording` (uses the T-LoRa Pager's built-in keyboard)

**ENTER** on the idle screen -> starts recording.
**ENTER** while recording -> stops recording (file is flushed and closed), back to idle screen.
**E** (any time except when already stopped) -> full stop: file is closed, screen shows
"Application stopped", device does nothing further (reboot required to run again).

**While recording, on screen:**
- red track on a black background (bottom part of the screen)
- table: satellite count, coordinates, visible WiFi count, speed (average of the last
  5 points, km/h), current file size in KB
- counters: lines written this session, total networks scanned this session

**On the SD card:**
- `/WIFIGPS` folder at the root of the SD card
- a new file every 30 minutes of recording: `log_YYYYMMDD_HHMMSS.txt` (name uses local
  UTC+3 time at file-creation time)
- one line every 30 seconds, ALWAYS written (even with no GPS fix), format:
  `HH:MM:SS_lat_lon_status_speed_ssid1|bssid1|rssi1,ssid2|bssid2|rssi2,..._M1:node1|rssi1,M2:node2|rssi2,...`
  - `status` is `ok` when the fix is valid or `bad_gps` otherwise (lat/lon are `0.000000`
    in that case)
  - `speed` is km/h (average of the last 5 fixes) or `NA` if not enough data yet
  - each WiFi network is logged as `ssid|bssid|rssi` (bssid = MAC address, rssi = signal
    strength in dBm)
  - each Meshtastic sighting is logged as `<channel_tag>:<node_id>|rssi`, where
    `channel_tag` is `M1` or `M2` (which known local channel it was heard on) and
    `node_id` is the sender's permanent, MAC-derived Meshtastic node number
  - nothing sits between the surrounding `_` separators if a list is empty
- the red track and the speed calculation still only use points with a valid fix -
  `bad_gps` rows don't affect the drawn track

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

**WiFi scanning** is asynchronous, every 15 sec, never blocks GPS or the display.

File: `gps_wifi_tracker/gps_wifi_tracker.ino`

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
- SD card is mounted by the library at `/sd` via `instance.installSD()`
- The screen is drawn with LVGL (v9); the track uses `lv_canvas` with direct line drawing
- Minimum satellites for a valid fix is 3 (`MIN_SATS_FOR_FIX`, adjust if needed)

## If the screen stays black after flashing

This can happen for reasons unrelated to the sketch logic itself. A few things worth
checking on real hardware:
- Confirm the `.bin` was flashed at the correct offset (`0x0` for the merged binary)
- Open a serial monitor (115200 baud) right after reset - `Serial.begin(115200)` runs
  first in `setup()`, so any early crash/reboot loop should be visible there
- Make sure the board didn't silently reset into download/bootloader mode - try a full
  power cycle (unplug USB) rather than just pressing reset
