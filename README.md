# IMU Balance Board Firmware — XIAO MG24 Sense

Seeed Studio XIAO MG24 Sense firmware for instrumented wobble-board balance
assessment. Outputs real-time tilt angles (roll, pitch, yaw) over USB serial
and Bluetooth LE GATT notifications.

Sensor fusion is performed on-device using a Mahony complementary filter fed
by raw accelerometer and gyroscope data from the board's built-in LSM6DS3TR-C.

> This is the **`mg24`** branch. The original ESP32 + LSM6DSO breakout port
> lives on the `master` branch and uses PlatformIO. This branch builds with
> Arduino IDE / arduino-cli instead — see "Why not PlatformIO?" below.

## Hardware

- **Seeed Studio XIAO MG24 Sense** — Silicon Labs EFR32MG24 (Cortex-M33, 78 MHz)
- Built-in **LSM6DS3TR-C** 6-DoF IMU on the internal I2C bus (`Wire1`)

No external wiring is required — the IMU is on-board. The Arduino core wires
the power rail and pulls the address line so the device appears at I²C
address **0x6A**.

The Seeed-Arduino-LSM6DS3 library automatically:

- Remaps `Wire` → `Wire1` when `ARDUINO_XIAO_MG24` is defined
- Drives the IMU power pin (`PD5`) HIGH during `beginCore()`

No code in this firmware touches those pins directly.

## Sensor Fusion

The LSM6DS3TR-C provides only raw accelerometer (±4 g) and gyroscope (±500
dps) data, so orientation is computed on the MG24 using a **Mahony
complementary filter**:

- Gyroscope integration provides fast, low-noise short-term orientation tracking
- Accelerometer provides long-term gravity reference to correct gyro drift
- Adaptive gain gates accel trust: ignored during dynamic motion (high variance)
  and when the accel magnitude deviates significantly from 1 g
- Integral term continuously estimates and removes gyro bias

The filter runs at **208 Hz** (the LSM6DS3 ODR). Outputs are roll, pitch, yaw
in degrees.

**Yaw note:** Without a magnetometer, yaw is gyro-integrated only and will
drift slowly over time. Roll and pitch are stable because they are
gravity-referenced.

### Tuning (`config.h`)

| Constant | Default | Effect |
|---|---|---|
| `MAHONY_KP` | 1.2 | Proportional gain — higher = accel re-levels faster after motion, more noise. Live-tunable via `KP <v>`. |
| `MAHONY_KI` | 0.00005 | Integral gain — higher = removes gyro bias faster |
| `MAHONY_ACCEL_GATE` | 4.0 | Sharpness of magnitude gate |
| `MAHONY_VAR_THRESHOLD` | 0.002 g² | Above this variance = motion detected, accel ignored. Live-tunable via `VAR <v>`. |
| `EMA_ALPHA_DEFAULT` | 0.5 | Output smoothing — higher = less lag, less smooth. Live-tunable via `EMA <a>`. |
| Filter mode | fusion | Boot filter source: fusion / gyro / accel. Live-tunable via `MODE <m>`. |

These are only the *boot* defaults. Live changes via `KP` / `VAR` / `EMA` /
`MODE` (serial or BLE) are non-destructive — they affect the running filter but
do **not** touch flash, so they reset on the next boot. Issue `SAVE` to freeze
the current live tuning to on-chip NVM; that saved set then reloads on every
subsequent boot and survives power cycles.

## Building

### Prerequisites

1. Install **Arduino IDE 2.x** (or `arduino-cli`).
2. Add the SiliconLabs board-manager URL in Preferences:
   `https://siliconlabs.github.io/arduino/package_arduinosilabs_index.json`
3. Open **Tools → Board → Boards Manager**, search for "Silicon Labs", and
   install the `Silicon Labs` core (version 3.0.0; the bundled `sketch.yaml`
   profile pins this version).
4. Open **Sketch → Include Library → Manage Libraries**, search for
   `Seeed Arduino LSM6DS3` and install it.

### Configure the protocol stack

This is the critical step — the firmware uses the Silicon Labs BGAPI
directly, which only links when the right protocol stack variant is selected.

In **Tools → Protocol stack**, choose **`BLE (Silabs)`**. Not "BLE (Arduino)"
and not "Matter". The default for the XIAO MG24 board entry is usually
"Matter", which will produce a duplicate-`sl_bt_on_event` link error.

### Compile and upload

In Arduino IDE:

1. Open `IMUBalanceBoard.ino`.
2. Select **Tools → Board → Silicon Labs → Seeed Studio XIAO MG24 Sense**.
3. Select the USB port the device enumerates as.
4. Click **Upload**.

With `arduino-cli` (a `sketch.yaml` profile is included):

```bash
arduino-cli compile --profile xiao_mg24
arduino-cli compile --profile xiao_mg24 -u -p COM11       # adjust port
arduino-cli monitor -p COM11 -c baudrate=115200
```

### Why not PlatformIO?

The current PlatformIO Seeed-MG24 platform (1.0.0) hardcodes the **Matter**
variant of the Silicon Labs Arduino framework. That variant ships its own
`sl_bt_on_event()` for Matter commissioning, so a sketch that adds its own
custom GATT service produces a link-time duplicate-symbol error.

Switching the variant requires patching
`<platforms>/Seeed Studio/builder/board_build/siliconlab/siliconlab_arduino.py`
to swap every reference to the `matter/` variant directory for `ble_silabs/`
— a global change to the toolchain. Arduino IDE exposes the same switch as a
one-click **Tools → Protocol stack** menu, so this branch uses Arduino IDE.

If you would rather keep PlatformIO, see commit history for an in-progress
attempt; the patching plan is sketched there.

## Bluetooth LE Dashboard

On boot the MG24 advertises a custom GATT service as `IMUBoard-MG24`. Any
Web-Bluetooth client (Chrome / Edge on desktop or Android, Bluefy on iOS)
that subscribes to the angles characteristic will receive notifications at
the configured output rate (default 50 Hz; adjustable 1–50 Hz via `RATE`).

The ready-to-use clients are shipped in this repo under
[`dashboards/`](dashboards/). They run entirely client-side, no server needed.

> **Calibrate once, on the board — never per game.** The per-user tilt range
> (front / back / left / right) is captured a single time in the testing
> dashboard and frozen to the board's NVM with `SAVE`. Every game then **reads
> that range from the board on connect** via the readable command
> characteristic and adapts itself to it. No game asks you to lean-calibrate at
> the start of a session, and the same range follows the board to any phone or
> laptop. (At most, each game has a hideable options menu for fine-tuning.)

- **[dashboards/testingdashboard.html](dashboards/testingdashboard.html)** — the
  full tuning + data dashboard (live tiles, zoomable tilt plot, tilt-vs-time
  chart, session metrics, CSV export, and live Kp / motion-gate / smoothing /
  mode sliders with a **Save to device** button). On connect it **reads the
  board's saved tuning** and syncs every control to it (so what you see is
  what's stored, not UI defaults). It is also where you **capture the tilt
  range** — lean to each edge, tap front / back / left / right, then **Save** —
  which is stored on the board and shared with every game. This is the *only*
  place calibration happens.
- **[dashboards/steadysteps.html](dashboards/steadysteps.html)** — **SteadySteps**,
  the patient-facing game app. A Duolingo-style path of **five levels** trains
  forward/back standing balance over **one shared BLE connection** (connect once;
  every level plays without reconnecting):
  1. **HOLD** — keep the dot in a generous box.
  2. **STEADY** — the box slowly tightens.
  3. **FOLLOW** — the box slowly floats up and down.
  4. **TRACK** — a target glides up and down; keep your dot on it.
  5. **FLIGHT** — a continuous flyer: your dino's height tracks your lean; rise
     over the trees, drop under the other dinos. Distance is the score.

  It **reads the board's saved forward/back range** on connect and maps the
  player's real lean to the dot (centre = midpoint of front/back, full travel =
  their reach). Detection is automatic: with a range saved it goes straight to
  the level path; with none it shows a one-time "set your range in the testing
  dashboard" screen and **advances on its own** once the range is saved — there
  is no calibrate-here step and no recheck button. The **only** calibration
  prompt anywhere is a warning banner that appears solely when the board reports
  it has drifted (health flag) and links to the testing dashboard.

  Levels normally unlock by mastering the previous one. An operator can tap
  **🔓 Unlock all** on the menu and enter the password (`claude1`) to unlock
  every level regardless of progress; the unlock persists in the browser. The
  same unlock reveals a hidden **⚙ Options** panel on the FLIGHT level with live
  difficulty sliders (scroll speed, obstacle spacing, gap size, etc.). Until
  unlocked, no options or tuning UI is shown anywhere.

  > The standalone **Ferra Balance** pro game and the separate Chrome-dino game
  > have been retired — the dino is now FLIGHT (level 5) inside SteadySteps, and
  > all advanced/engineering tuning lives in the testing dashboard.

You can use these two ways:

- **Locally** — double-click the `.html` file in Windows Explorer to open it
  in your default browser.
- **Hosted** — a copy is published over HTTPS at
  **<https://flauga.github.io/IMU-BalanceBoard/>** (served from the
  `gh-pages` branch). HTTPS is required because Web Bluetooth only works in a
  secure context, so this is the way to open it on a phone.

Web Bluetooth support is platform-dependent:

- **Android / desktop** — open the page in **Chrome** (or Edge) and tap
  **Connect**.
- **iPhone / iPad** — Safari and iOS Chrome do **not** support Web Bluetooth.
  Install the free **Bluefy – Web BLE Browser** app and open the page inside
  it instead.

It carries over the full feature set of the ESP32 WiFi dashboard:

- **Live tiles** for roll / pitch / tilt updated every frame. (Yaw is gyro-only
  and drifts, so it is not shown — but it is still recorded in the CSV export.)
- **Zoomable tilt plot** — slider, +/− buttons, and mouse-wheel over the
  canvas all change the radial scale between ±5° and ±90°. The plot draws
  a fading sample trail, the 70 % prediction ellipse, and the current dot.
- **Tilt-vs-time chart** (Chart.js, 20 Hz redraw cap) for roll/pitch.
- **Session metrics** that only accumulate while a session is active:
  Samples, Duration, Max tilt, Avg tilt, Sway path, Mean velocity, Sway area.

### Session lifecycle

| Button | Effect |
|---|---|
| **Start session** | Resets all dashboard-side aggregates (counts, peaks, sway ellipse, CSV buffer) and begins accumulating from the next frame. |
| **Stop & save CSV** | Freezes all metrics on their final values, downloads a `imu_<timestamp>.csv` file containing every recorded sample, and stops growing the CSV buffer. The ellipse stays visible as a summary. |
| **Zero** | Sends `ZERO` to the firmware (so the next frame is referenced from the current pose) **and** clears every dashboard-side aggregate, the trail, the chart, and the dot. Any in-progress session is discarded without saving. |

The session metric math (Welford online mean + covariance for the sway
ellipse, chi² = 2.408 for the 70 % region) is identical to the ESP32
firmware's embedded dashboard.

### CSV format

Columns: `timestamp, elapsed_s, device_ms, roll_deg, pitch_deg, yaw_deg`
— same as the ESP32 dashboard's CSV export.

### GATT layout

| UUID | Properties | Payload |
|---|---|---|
| `6e400001-b5a3-f393-e0a9-e50e24dcca9e` | Service | "IMU Balance Board" |
| `6e400002-b5a3-f393-e0a9-e50e24dcca9e` | Notify (16 B) | `{ uint32 ms; float roll; float pitch; float yaw }` little-endian |
| `6e400003-b5a3-f393-e0a9-e50e24dcca9e` | Write **+ Read** | Write: UTF-8 command string (below). Read: live tuning + limit snapshot (below). |

### Tuning / limit snapshot (command-char read)

The command characteristic is **readable** as well as writable. A read returns a
compact CSV snapshot of the board's current live state so any client can sync
its UI to the board's saved values on connect (rather than overwriting them):

```
TUNE,<mode>,<kp>,<var>,<ema>,<front>,<back>,<left>,<right>,<setmask>,<health>
```

e.g. `TUNE,fusion,1.20,0.0020,0.50,12.0,-10.0,8.0,-9.0,15,0`. `setmask` is a
bitmask of which limits are captured (bit0 front, bit1 back, bit2 left, bit3
right; `15` = all four). All shipped dashboards read this on connect.

`<health>` is a **calibration-health** code (`0` = OK, `1` = gyro-bias drift
suspected) derived at runtime from the magnitude of the Mahony integral
correction term — it flags a sensor that has drifted enough to warrant a re-zero,
and never trips merely from leaning the board. It is a trailing field, so older
clients that read only fields 0–9 ignore it (backward compatible). Every
dashboard shows a re-zero banner while it is non-zero; the testing dashboard also
logs the reason. The tilt **limits** are captured as the average of the last
~0.5 s of a steady hold (a capture taken while the board is still wobbling is
rejected), so a saved edge reflects a settled lean rather than one noisy sample.

The binary frame is identical to the ESP32 firmware's `BIN ON` mode, so
existing Web-Bluetooth dashboards built for that wire format work unchanged
once they swap WebSocket → BLE GATT.

### BLE commands

The command characteristic accepts the same text commands as the UART:

| Command | Description |
|---|---|
| `START` / `STOP` | Begin / pause streaming |
| `ZERO` | Reset orientation reference to current pose (persisted to NVM — survives power cycles) |
| `ZEROCLEAR` | Forget the saved zero; next boot captures the boot pose **only if it passes a plausibility gate** (board flat & still), otherwise the zero is deferred until you run `ZERO` |
| `SAVE` | Freeze current tuning (Kp / VAR / EMA / MODE) **and tilt limits** to NVM — survives power cycles |
| `LIMIT <FRONT\|BACK\|LEFT\|RIGHT>` | Capture the current tilt as that range limit (front/back = pitch, left/right = roll) |
| `LIMITCLEAR` | Forget the captured tilt limits |
| `DEBUG ON` / `DEBUG OFF` | Toggle 1 Hz drift-diagnostic log |
| `RATE <hz>` | Set output rate 1–50 Hz |
| `MODE <fusion\|gyro\|accel>` | Orientation filter source |
| `KP <value>` | Mahony accel gain (fusion mode); higher = faster re-level |
| `VAR <g²>` | Motion-variance gate; higher = accel keeps correcting during motion (less lag) |
| `EMA <alpha>` | Output smoothing 0.01–1.0; higher = less smoothing lag |
| `GYROCAL` | Re-average the gyro bias (~1 s, board must be flat & still; streaming pauses). Queued to the main loop — never blocks the BLE task |
| `SERIAL ON` / `SERIAL OFF` | Toggle per-frame angle prints on the UART |
| `SERIAL DIV <n>` | Print 1 of every n frames on the UART |

The full UART command set — including `SAVE`, `LIMIT …` and `LIMITCLEAR` — is
also accepted over BLE, so the board can be fully driven from a Web-Bluetooth
client with no serial cable.

The testing dashboard exposes all four tuning knobs (Kp / motion gate /
smoothing / filter mode) as live sliders with presets — no reflash needed to
tune — plus a **Save to device** button that issues `SAVE`. Because the command
characteristic is **readable**, the dashboard reads the board's current values
on connect and syncs its controls to them, instead of overwriting the board with
UI defaults. `SAVE` persists the tuning **and** the captured tilt limits together
in one NVM record, so both survive power cycles.

## Serial Commands

Identical to the ESP32 firmware:

| Command | Description |
|---|---|
| `START` / `STOP` | Begin / pause streaming |
| `STATUS` | Show firmware version, BLE status, etc. |
| `RATE <hz>` | Set output rate 1–50 Hz (default 50 Hz) |
| `ZERO` | Reset orientation reference to current pose (persisted to NVM — survives power cycles) |
| `ZEROCLEAR` | Forget the saved zero; next boot captures the boot pose **only if it passes a plausibility gate** (board flat & still), otherwise the zero is deferred until you run `ZERO` |
| `SAVE` | Freeze current tuning (Kp / VAR / EMA / MODE) **and tilt limits** to NVM — survives power cycles |
| `LIMIT <FRONT\|BACK\|LEFT\|RIGHT>` | Capture the current tilt as that range limit |
| `LIMITCLEAR` | Forget the captured tilt limits |
| `DEBUG ON` / `DEBUG OFF` | Toggle 1 Hz drift-diagnostic log |
| `GYROCAL` | Re-average the gyro bias (~1 s, board flat & still; streaming pauses briefly) |
| `SERIAL ON` / `SERIAL OFF` | Toggle per-frame angle prints on the UART |
| `SERIAL DIV <n>` | Print 1 of every n frames (default 5 = ~10 Hz UART) |
| `MODE <fusion\|gyro\|accel>` | Orientation filter source (default fusion) |
| `KP <value>` | Mahony accel gain (fusion mode); higher = faster re-level |
| `VAR <g²>` | Motion-variance gate; higher = less lag during/after motion |
| `EMA <alpha>` | Output smoothing 0.01–1.0; higher = less smoothing lag |
| `HELP` | Show command list |

## Serial Output Format

```
<millis>,<roll_deg>,<pitch_deg>,<yaw_deg>
```

Example:
```
12345,2.34,-1.12,87.50
```

Once per second a diagnostic line is also printed:
```
[STATS] tx=50/s samples=193/s maxGap=21ms clients=1
```

| Field | Meaning |
|---|---|
| `tx` | BLE notifications sent in the last second. The emit rate is retuned to **half the granted connection interval** — expect ≈ 130–145 at a 15 ms interval, ≈ 260 at 7.5 ms; ≈ 50 only if the ceiling clamp (`OUTPUT_INTERVAL_MAX_MS`) is active on a slow link |
| `samples` | IMU samples published by the sensor loop (expected ≈ 190–200) |
| `maxGap` | Worst inter-send gap in ms (expected ≈ emit interval + a few ms; sustained spikes flag stalls) |
| `clients` | Number of connected BLE centrals (0 or 1) |

---

## Dot lag — root-cause findings & fix history

The recurring complaint: the on-screen dot trails the physical board, sometimes
badly, sometimes barely. It has had **four distinct root causes**, fixed in
order. This section is the complete record so nobody re-diagnoses from scratch.

### The latency budget (where the milliseconds actually go)

Every stage between the board moving and the dot moving, with its typical cost
after all fixes:

| # | Stage | Typical cost | Notes |
|---|---|---|---|
| 1 | IMU sampling (208 Hz ODR) | 0–4.8 ms (avg ~2.4) | LSM6DS3, fixed |
| 2 | Sensor anti-alias LPF (100 Hz BW) | ~2 ms group delay | fixed |
| 3 | Mahony fusion | ~0 ms | gyro-integrated — no lag for motion; Kp/VAR only affect settle accuracy |
| 4 | Firmware EMA (`EMA`, default 0.6 @ 208 Hz) | ~3 ms (α 0.6) / ~9 ms (α 0.35) | lever: `EMA` slider; runs at sample rate so it's cheap lag-wise |
| 5 | Wait for a BLE **connection event** | ≤ ½ conn interval (~4–8 ms) | see fix #4 below — used to be up to a *full* interval, phase-drifting |
| 6 | Conn interval itself | 7.5 ms (fast centrals) / 15 ms (Windows) | firmware requests 6–12 × 1.25 ms; Windows won't go below ~15 ms |
| 7 | OS BLE stack → browser JS event | ~5–20 ms | platform-dependent, not tunable |
| 8 | Browser rAF wait | 0–16.7 ms (avg ~8) | frame drawn on next vsync tick |
| 9 | Render-side dot smoothing (α 0.8 / 60 Hz frame) | ~4 ms | lever: smoothing slider; 1.0 = off |
| 10 | Canvas → compositor → display | ~17–33 ms | `desynchronized: true` canvas hint shaves up to one frame on supported Chrome |

**Total, typical: ~55–85 ms motion-to-photon.** Roughly half of that
(stages 7 + 8 + 10) is OS/browser/display plumbing that no firmware change can
remove. Anything visibly worse than ~100 ms means one of the pathologies below
has come back — use the **Link & latency** panel in the testing dashboard to
identify which.

### Fix history (each a different bug, all shipped)

1. **BLE notify batching** — the board fired notifies at 50 Hz but never asked
   for a fast connection interval, so Windows/Chrome defaulted to ~30–50 ms and
   several frames queued up and arrived in one burst: dot freezes, then snaps.
   *Fix:* request a fast interval on connect (`sl_bt_connection_set_parameters`).

2. **Firmware self-throttling** (the "solid 1-second freeze") — the board
   retuned its emit cadence to *whatever* interval the central granted. When the
   OS downshifted to a slow power-saving interval (hundreds of ms), the board
   obediently emitted at a few Hz. Compounded by a 1000 ms supervision timeout
   that tore the link down on any ~1 s RF stall. *Fix:* `OUTPUT_INTERVAL_MAX_MS`
   caps the emit interval at 20 ms (≥ 50 Hz always offered), supervision timeout
   raised to 4 s, and a guarded re-request nudges the interval back up after a
   downshift.

3. **Heavy render-side smoothing** — the dashboards' dot easing (α 0.35,
   frame-rate-dependent) was hiding the batching *and adding ~40 ms of its own
   lag*, more at low fps. *Fix:* frame-rate-independent easing, default α 0.8
   (~4 ms), tunable slider, and the underlying batching fixed at source.

4. **Emit/connection-event phase beat** (found in this pass — the "still lags
   sometimes" residue). The emit timer was set *equal* to the granted connection
   interval (15 ms emit, 15 ms events) on the theory of "one fresh frame per
   event". But the two clocks free-run: they're the same period and **not
   phase-locked**, so the queued frame's wait for its connection event slowly
   swept 0 → 15 → 0 ms as the clocks drifted past each other. The dot lag
   visibly *breathed* — fine for a stretch, then up to a full interval of extra
   lag for many seconds. *Fix:* emit at **half** the granted interval (2 frames
   per event, floor 5 ms, ceiling unchanged). The newest frame at any event is
   now never more than ~half an interval stale, regardless of phase. The radio
   trivially carries two 16-byte notifies per event; the browser drains both and
   renders the latest. Also in this pass: the connection-interval request now
   offers a 6–12 range (7.5–15 ms) instead of pinning 15 ms, so Android/macOS
   centrals that support the BLE floor grant 7.5 ms and halve stage 6.

### Things that were suspected and ruled out

- **The sensor loop stalling** — `[STATS] samples=…` stays ~190–200/s
  throughout every freeze ever logged; the pipeline upstream of the radio never
  starved.
- **UART printing blocking the loop** — the per-frame print is rate-divided
  (default 1-in-5) and ~500 chars/s at 115200 baud is < 5 % of the UART budget.
  It can now be disabled over BLE anyway (`SERIAL OFF`, testing-dashboard
  toggle) to reclaim the headroom while measuring.
- **Mahony tuning** — Kp/VAR change *settling* behaviour (how fast the estimate
  re-levels after motion stops), not transport latency. Don't chase lag with
  the Kp slider; chase it with the Link & latency panel.

### How to measure (testing dashboard → Link & latency)

- **Frame age** — staleness of the newest frame at the moment it's drawn. This
  *is* the data-side share of the perceived lag. Healthy: < 20 ms.
- **Conn interval (est.)** — median inter-notification gap ≈ what the central
  actually granted. 7–8 ms (fast centrals) or ~15 ms (Windows) is right;
  30 ms+ means the OS downshifted and the firmware's re-request isn't being
  honoured.
- **Arrival gap max / Bursts %** — spikes ≫ the median mean OS-side batching.
- **Queue / Draw loop** — a backed-up queue with a low rAF Hz means the browser
  tab is render-bound (close the time-series chart tab overlays, check for
  background throttling), not the link.
- **Verdict** — the same heuristic SteadySteps' Advanced panel uses, so numbers
  are comparable across both pages.

Remaining levers, in order of bang-for-buck: keep the render smoothing slider
high (0.8–1.0), keep `EMA` ≥ 0.6 (the "Responsive" preset), and prefer a
platform that grants 7.5 ms intervals. Below that you're into OS/display
territory (~40 ms floor) that no code in this repo can touch.

---

## Differences from the ESP32 firmware

The MG24 port shares the same Mahony filter, types, and the bulk of the
business logic. Major architectural differences:

- **No FreeRTOS dual-core split.** The ESP32 firmware ran IMU sampling on
  core 0 and WiFi/WebSocket transmit on core 1 with seqlock snapshots, to
  dodge the WiFi driver preempting the Arduino loop. The MG24 is single-core
  Cortex-M33 with a much lighter-weight BLE stack (BGAPI), so a simple
  cooperative `loop()` handles everything at well over 200 Hz polling rate.

- **BLE GATT instead of HTTP + WebSocket.** No embedded web server, no
  served dashboard HTML, no mDNS. The dashboards (under `dashboards/`) are now
  standalone Web-Bluetooth clients.

- **No `wifi_config.h`.** SSID / credentials are gone. The device is paired
  by BLE address.

- **`Wire1` instead of `Wire`.** The LSM6DS3 sits on the MG24's internal I2C
  bus. The Seeed library handles this transparently for our code.

- **Serial baud lowered to 115200.** USB-CDC on the MG24 is not the throughput
  bottleneck the ESP32's hardware UART was, but 115200 is the conventional
  default for Arduino USB CDC.

- **No `ESP.getFreeHeap()` diagnostic.** No equivalent on the MG24 / Silabs
  core that's worth exposing.

- **No deep-sleep / wake-on-motion.** The sibling XIAO nRF52840 Sense port has
  a deep-sleep framework that powers the board down after inactivity and wakes
  it on an LSM6DS3 motion interrupt. On the XIAO MG24 Sense the IMU's INT line
  is **not** routed to a wake-capable GPIO, so wake-on-motion is impossible and
  the sleep framework is deliberately omitted from this build. Everything else
  carried over from the nRF52840 firmware — the testing dashboard, the tilt
  game, and tuning persistence (`SAVE`) — is present here. (Gyro bias is
  recalibrated automatically at boot rather than on demand.)

- **Arduino IDE flat sketch layout.** All `.cpp` / `.h` files live next to
  the `.ino` in the project root (instead of `src/` and `include/` as on the
  ESP32 PlatformIO build) so the Arduino preprocessor picks them up.
