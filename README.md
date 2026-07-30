# GPS + WiFi + Meshtastic tracker for LILYGO T-LoRa Pager

## How it works

**Recording** (background state, independent of what's on screen):
- **ENTER** - start recording (needs an SD card)
- **ENTER** again - stop recording (file is flushed and closed)
- **E** (any time except when already stopped) - full stop: file is closed,
  screen shows "Application stopped", device does nothing further (reboot
  required to run again)

**Display modes** - **S** cycles between three screens (recording, if active,
keeps running in the background regardless of which screen is shown):

1. **Track** - red track on a black background (bottom part of the screen) +
   status line: recording/idle, satellite count, coordinates, **track length
   in km**, speed, lines written
2. **Diagnostics** - full-screen, scrollable text (use the **rotary
   scroll wheel** on the side of the device to scroll up/down): raw GPS stats
   (characters processed, sentences with a fix, failed checksums, lat/lon/alt/
   speed/HDOP), last WiFi scan (up to 4 networks with RSSI), Meshtastic status
   (current channel, total heard since boot, last node heard + RSSI + seconds
   ago), SD card status, free heap/PSRAM
3. **Grid** - screen split into 4 cells, one per module (GPS / WiFi /
   Meshtastic / SD), colored **green** when everything looks fine and data is
   coming in, **red** when there's a problem (no fix, no networks seen, no
   recent Meshtastic activity, SD not found)

A **battery percentage** badge is shown in the top-right corner on every
screen (turns red at 15% or below).

**Buzzer feedback** (via the onboard ES8311 codec + speaker):
- **5 beeps** whenever a Meshtastic sighting is heard
- **1 long beep** the moment GPS acquires a fix (searching -> fix transition)
- **1 short beep every 40 seconds** while the battery is at 15% or below (and
  not charging)

Note: beeping briefly blocks the main loop (the longest pattern, 5 Meshtastic
beeps, takes ~1 second) - this is a deliberate simplicity/responsiveness
trade-off given how infrequently these events actually happen.

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
- Rotary/scroll wheel: `instance.getRotary()` / `instance.clearRotaryMsg()` -
  enabled once via `instance.enableRotary()`
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
