# IMU Balance Board Firmware

ESP32 DevKit V1 + LSM6DSO IMU firmware for instrumented wobble board balance assessment.
Outputs real-time tilt angles (roll, pitch, yaw) over serial and WiFi WebSocket.
Sensor fusion is performed on the ESP32 using a Mahony complementary filter fed by
raw accelerometer and gyroscope data from the LSM6DSO.

## Hardware

- ESP32 DOIT DevKit V1
- SmartElex 6 Degrees of Freedom Breakout — LSM6DSO

Uses **I2C** (the breakout's default protocol).

> **Warning:** The LSM6DSO is a 3.3V device. The ESP32 DevKit V1 has 3.3V I/O on its
> GPIO pins, so no level shifting is needed for I2C.

### LSM6DSO Breakout Pin Reference

| Breakout Pin | Description |
|---|---|
| 3V3 | 3.3V power input |
| GND | Ground |
| SDA | I2C data (default address 0x6B — SA0/SDO pulled high on SmartElex board) |
| SCL | I2C clock |
| INT1 | Programmable interrupt output (not used in this firmware) |
| INT2 | Programmable interrupt output (not used in this firmware) |
| CS | SPI chip select — leave unconnected for I2C mode |
| SDO | Address select / SPI data out — sets address 0x6B when high (default) |

### Wiring (I2C)

| ESP32 GPIO | Function | LSM6DSO Pin |
|---|---|---|
| GPIO 21 | SDA | SDA |
| GPIO 22 | SCL | SCL |
| 3V3 | Power | 3V3 |
| GND | Ground | GND |

Total: **4 wires**. No reset pin. No interrupt pin required.

```
  ESP32 DevKit V1                     SmartElex LSM6DSO breakout
  ~~~~~~~~~~~~~~~                     ~~~~~~~~~~~~~~~~~~~~~~~~~~
  3V3  ——————————————————————————————  3V3
  GND  ——————————————————————————————  GND
  GPIO 21 (SDA) ————————————————————  SDA
  GPIO 22 (SCL) ————————————————————  SCL
```

### I2C Address

The SmartElex breakout pulls SA0/SDO high by default, giving address **0x6B**.
To use 0x6A, cut the address jumper on the back of the board.

## Sensor Fusion

The BNO085 variant of this firmware used on-chip SH2 fusion (quaternion output).
The LSM6DSO provides only raw accelerometer (±4g) and gyroscope (±500 dps) data,
so orientation is computed on the ESP32 using a **Mahony complementary filter**:

- Gyroscope integration provides fast, low-noise short-term orientation tracking
- Accelerometer provides long-term gravity reference to correct gyro drift
- Adaptive gain gates accel trust: ignored during dynamic motion (high variance)
  and when the accel magnitude deviates significantly from 1g
- Integral term continuously estimates and removes gyro bias

The filter runs at **208 Hz** (the LSM6DSO ODR). Outputs are roll, pitch, yaw in degrees.

**Yaw note:** Without a magnetometer, yaw is gyro-integrated only and will drift slowly
over time. Roll and pitch are stable because they are gravity-referenced.

### Tuning (include/config.h)

| Constant | Default | Effect |
|---|---|---|
| `MAHONY_KP` | 2.0 | Proportional gain — higher = accel corrects faster, more noise |
| `MAHONY_KI` | 0.005 | Integral gain — higher = removes bias faster |
| `MAHONY_ACCEL_GATE` | 4.0 | Sharpness of magnitude gate |
| `MAHONY_VAR_THRESHOLD` | 0.002 g² | Above this variance = motion detected, accel ignored |

## Building

Requires [PlatformIO](https://platformio.org/).

```bash
pio run              # compile
pio run -t upload    # flash to ESP32
pio device monitor   # open serial monitor at 921600 baud
```

## WiFi Dashboard

On first boot the ESP32 connects to WiFi (configure credentials in
`include/wifi_config.h`) and serves the dashboard over plain HTTP on **port 80**.
The WebSocket stream runs on **port 81** and the dashboard auto-connects to it.

mDNS (`imuboard.local`) is **disabled by default** — see the Debugging Timeline
below for why. The recommended setup is:

1. Power the ESP32 once and read the boot banner on the serial monitor:
   ```
   ========================================
     Hostname: imuboard500
     MAC:      A8:03:2A:5F:B2:E4
     IP:       192.168.0.10
     Open the dashboard at:
       http://192.168.0.10/   (direct IP)
   ========================================
   ```
2. In your router's DHCP settings, **reserve** that MAC → a known IP (we use
   `192.168.0.10`, `.11`, `.12` for our three boards).
3. Bookmark `http://192.168.0.10/` on your tablet.

To re-enable mDNS, set `MDNS_ENABLED 1` in `include/config.h`.

## Dashboard Metrics

The dashboard displays seven balance metrics. The first four are instantaneous
(updated every frame); the remaining are session aggregates that reset on
**Start Session**.

### Instantaneous

| Metric | Unit | Definition |
|---|---|---|
| **Roll** | ° | Rotation about the board's X-axis (left/right lean). Positive = right side down. Gravity-referenced, no drift. |
| **Pitch** | ° | Rotation about the board's Y-axis (front/back lean). Positive = front edge down. Gravity-referenced, no drift. |
| **Yaw** | ° | Rotation about the vertical Z-axis (heading). Gyro-integrated only — drifts slowly over time. Use `ZERO` to reset. |
| **Tilt** | ° | Magnitude of the off-vertical angle: `√(roll² + pitch²)`. A single number for how far off level the board is, regardless of direction. |

### Session aggregates

| Metric | Unit | Definition |
|---|---|---|
| **Samples** | count | Number of frames received since session start. At the default 50 Hz output rate the count rises ~50/s. |
| **Duration** | mm:ss | Wall-clock time since **Start Session** was pressed. |
| **Max tilt** | ° | Largest single-frame tilt value seen during the session. Sensitive to single spikes. |
| **Avg tilt** | ° | Mean tilt across all samples: `Σ tilt / N`. Lower = steadier balance overall. |
| **Sway path** | ° | Total distance traced in the roll/pitch plane: `Σ √(Δroll² + Δpitch²)`. Think of it as the length of the path a pen would draw on a roll/pitch graph. Lower = less corrective movement. |
| **Mean velocity** | °/s | `sway path / duration`. Average angular speed of the board's tilt — how fast the user is correcting. |
| **Sway area** | °² | Area of the **70 % prediction ellipse** fitted to the entire session's roll/pitch samples. This is the region on the roll/pitch plane that contained the board's tilt for ~70 % of the session. Smaller = a tighter, more consistent stance. |

### About the sway area

The yellow ellipse on the tilt plot is the geometric representation of the sway
area metric. It is computed by:

1. Maintaining a **running mean and 2×2 covariance matrix** of every
   (roll, pitch) sample in the session (Welford's online algorithm — no sample
   storage required).
2. Solving for the eigenvalues of that covariance matrix to find the ellipse's
   principal axes (the two directions of largest spread) and their lengths.
3. Scaling the axes by `√2.408` — the chi-square critical value such that a 2-D
   Gaussian distribution has 70 % of its probability mass inside the ellipse.

Unlike a rolling-window sway area (which would only reflect the last few
seconds), this ellipse represents the **typical region the user occupied across
the entire session**, making it a stable summary of overall stance consistency.
It updates live as more samples arrive, converging on a stable value once the
user has been on the board for a few seconds.

### Session-control behaviour

The three buttons interact with the sway ellipse and other session metrics
as follows:

| Button | Sway area / ellipse | Other session aggregates | Tilt plot + chart |
|---|---|---|---|
| **Start Session** | Reset to empty, begin accumulating | All cleared, begin accumulating | Continues live |
| **Stop & Save CSV** | **Freezes** on the final value and stays visible | Stops updating; final values stay visible | Continues live |
| **Zero** | Cleared and unfrozen — starts accumulating again on next sample | All cleared back to "—" | Cleared (trail, chart, dot reset to default) |

After a Stop, the sway ellipse and number remain on screen as a summary of
the just-finished session — useful for showing the patient or clinician their
result. Hit Zero or Start Session to begin fresh.

## Serial Commands

| Command | Description |
|---|---|
| `START` / `STOP` | Begin / pause streaming |
| `STATUS` | Show firmware version, sensor, heap, WiFi info |
| `RATE <hz>` | Set output rate 1–50 Hz (default 50 Hz) |
| `ZERO` | Reset orientation reference to current pose |
| `DEBUG ON` / `DEBUG OFF` | Toggle 1 Hz drift-diagnostic log |
| `SERIAL ON` / `SERIAL OFF` | Toggle per-frame angle prints on the UART |
| `SERIAL DIV <n>` | Print 1 of every n frames (default 5 = ~10 Hz UART) |
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
[STATS] tx=50/s samples=193/s maxGap=21ms heap=234KB rssi=-52 clients=1
```

| Field | Meaning |
|---|---|
| `tx` | WebSocket frames broadcast in the last second (expected ≈ 50) |
| `samples` | IMU samples published by the sensor task (expected ≈ 190–200) |
| `maxGap` | Worst inter-broadcast gap in ms (expected ≈ 21; spikes flag stalls) |
| `heap` | Free heap in KB |
| `rssi` | WiFi signal strength in dBm |
| `clients` | Number of connected WebSocket clients (0 or 1 — single-client policy) |

---

## Firmware Architecture

### Why this is non-trivial

A naive ESP32-Arduino implementation puts the IMU read, sensor fusion, and
WebSocket broadcast all in `loop()` on core 1. On paper that works at 50 Hz.
In practice it produces multi-second freezes during heavy motion: the WiFi
driver's internal task (priority ~23, on core 1) periodically preempts the
Arduino loop task (priority 1) for radio housekeeping, and the broadcast
stops. The IMU stops getting polled too, because it's on the same task.

The firmware therefore splits work across **three FreeRTOS tasks** with
carefully chosen core affinities and priorities:

| Task | Core | Priority | Job |
|---|---|---|---|
| `imu_sensor` | 0 | `configMAX_PRIORITIES - 2` | Polls IMU at 208 Hz, runs Mahony + EMA, publishes the latest sample to a snapshot |
| `wifi_tx` | 1 | 5 | Runs `_ws->loop()`, serves HTTP requests, broadcasts the latest snapshot at ≈50 Hz |
| Arduino `loop` | 1 | 1 (default) | Serial-command poll, periodic STATUS broadcast, once-per-second `[STATS]` log |

The two key choices:

- **`imu_sensor` on core 0** isolates IMU sampling from the WiFi driver and
  the WebSocket library, both of which live on core 1. Even if core 1 is
  fully preempted for two seconds, the sensor task keeps producing samples.
- **`wifi_tx` at priority 5** sits *above* the Arduino loop task (priority 1)
  but well *below* the WiFi driver (~23). When core 1 schedules out the
  Arduino loop, `wifi_tx` still runs and keeps frames flowing. This is the
  single change that eliminated the visible freezes.

### Inter-task data flow

Two **seqlock**-protected snapshots carry state between tasks. A seqlock is
a wait-free reader pattern: writers increment an atomic counter to odd
(writing), update the payload, then increment to even (stable). Readers
re-try until they see a stable counter on either side of their copy, which
means they got a consistent snapshot. No mutex, no priority inversion, no
producer ever waits.

```
  ┌───────────────────────────┐                          ┌───────────────────────────┐
  │  imu_sensor (core 0)      │                          │  wifi_tx (core 1, p=5)    │
  │  - reads LSM6DSO @ 208 Hz │                          │  - _ws->loop()            │
  │  - Mahony filter          │     g_snap (seqlock)     │  - _pollHttp()            │
  │  - EMA smoothing          │ ───────────────────────► │  - read latest _txSnap    │
  │  - publishes g_snap       │                          │  - broadcastBIN to client │
  └────────────┬──────────────┘                          └─────────────▲─────────────┘
               │                                                       │
               │                                                       │ _txSnap (seqlock,
               │                                                       │  +pub-counter)
               │                                                       │
               │       ┌────────────────────────────────┐              │
               │       │  Arduino loop (core 1, p=1)    │              │
               └──────►│  - reads g_snap @ 50 Hz        ├──────────────┘
                       │  - publishes _txSnap           │
                       │  - serial poll, STATS print    │
                       └────────────────────────────────┘
```

The "publish counter" on `_txSnap` lets `wifi_tx` know whether there's a
fresh sample to send (avoiding duplicate broadcasts) without needing
condition variables.

### Other architecture choices

- **Synchronous `WiFiServer`** for HTTP — `AsyncWebServer` + `AsyncTCP` were
  removed because `AsyncTCP` spawns its own task that intermittently caused
  300+ ms stalls. The dashboard HTML is served in one shot per page load,
  which the synchronous server handles fine.
- **`WebSocketsServer` (links2004)** for the WS protocol — synchronous,
  but driven from the dedicated `wifi_tx` task so its blocking semantics
  no longer matter to the rest of the system.
- **Single-client policy** — the WS server rejects a second connection so
  the broadcast load stays constant. Open a tab elsewhere and the first
  must disconnect before the second can take its slot.
- **Binary WS frames (16 B little-endian, `uint32 ms + 3× float32`)** —
  half the bandwidth of text CSV and avoids `snprintf` cost on the ESP32.
  The dashboard opts in via `BIN ON` immediately on connect.
- **Dashboard-side jitter buffer with device-time playback** — incoming
  frames are kept for 200 ms before rendering, keyed on the ESP32's
  `millis()` timestamp (not browser arrival time). This makes burst-arrivals
  after a brief network hiccup play back smoothly at 50 Hz instead of fast-
  forwarding, and absorbs sub-200 ms jitter without any visible artefact.

---

## Debugging Timeline

This section is here so the next person to touch the firmware doesn't have
to re-derive what works from scratch. The path from "freezes every 5
seconds" to "no observable freezes" was long.

### Symptom

Multi-second freezes in the dashboard, especially during board motion. The
IMU sample timestamps showed gaps of 1–2.5 s, with bursts of catch-up frames
afterwards. Sometimes both cores stalled simultaneously, sometimes only
core 1.

### Suspects investigated, ordered by chronology

| # | Suspect | Verdict | Notes |
|---|---|---|---|
| 1 | I²C bus errors during motion | Innocent | `imuUpd` time stayed ~7 µs even during stalls; sensor task never lost time |
| 2 | `ESPAsyncWebServer` / `AsyncTCP` | **Guilty** | The async TCP task on core 0 caused 300 ms+ stalls every few seconds. Replaced with sync `WiFiServer`. **Major improvement.** |
| 3 | `ESPmDNS` responder | **Guilty** | Incoming multicast queries on the LAN (Apple devices, smart TVs, printers) ran responder work inside the WiFi/lwIP task on core 0 — periodic dual-core stalls. **Set `MDNS_ENABLED 0`; use DHCP-reserved IPs instead.** |
| 4 | WiFi power-save | Partial | `WiFi.setSleep(WIFI_PS_NONE)` + `setTxPower(WIFI_POWER_19_5dBm)` helped a little |
| 5 | WebSocket library's TCP write timeout (default 5000 ms) | Partial | A slow browser fills the TCP send buffer; `WiFiClient::write()` blocks waiting for ACKs. Tried patching the library to set `SO_SNDTIMEO` to 50 ms on accepted clients — helped slightly but didn't eliminate freezes. Superseded by the dedicated TX task (item #10), which made WS blocking semantics irrelevant. Patch was removed |
| 6 | Multiple WS clients (zombie tabs) | Mild | Doubled broadcast load. Added single-client policy that rejects connection #2 |
| 7 | Browser-side render load (Chart.js redraws on every frame) | Mild | Throttled Chart.js redraw to 20 Hz via a `requestAnimationFrame` loop |
| 8 | Bursty arrivals after a brief network gap | Mild | Dashboard's jitter buffer now plays back on device-time, not arrival-time |
| 9 | `loop()` not yielding to async helpers | Mild | Added `vTaskDelay(1)` at end of `loop()` |
| 10 | **WiFi driver preempting the Arduino loop task** | **Root cause of remaining stalls** | The WiFi driver runs at priority ~23 on core 1 and routinely preempts the loop task (priority 1) for radio housekeeping. **Fixed by moving WS broadcast and `_ws->loop()` to a dedicated `wifi_tx` task at priority 5 on core 1** — high enough to survive the preemption pattern |

### Things that were tried but did **not** help

- Lowering `CONFIG_MDNS_TASK_PRIORITY` via PlatformIO build flag. **Doesn't take
  effect** because Arduino-ESP32 ships a pre-built IDF — FreeRTOSConfig.h is
  baked in and ignores `-D` overrides.
- `configGENERATE_RUN_TIME_STATS=1` for `uxTaskGetSystemState()`. Same problem
  as above — the trace facility is compiled out of the pre-built FreeRTOS.
- Bigger TCP send buffer (`SO_SNDBUF`). The stall wasn't a buffer-fill issue;
  it was task preemption. Bigger buffer doesn't help if no task is running to
  drain it.
- WebRTC DataChannel / WebTransport / UDP-to-browser. No ESP32 library exists
  for any of these, and browsers can't read raw UDP. Hard dead end.
- Bluetooth LE notifications. Same 2.4 GHz radio, similar firmware blob, and
  iOS Safari can't connect (no Web Bluetooth on iOS). Worse on every axis.

### What the boot banner means

```
[INIT] Sensor task pinned to core 0.
[WiFi] wifi_tx task on core 1 at priority 5
========================================
  Hostname: imuboard500
  MAC:      A8:03:2A:5F:B2:E4
  IP:       192.168.0.10
========================================
```

Both lines are confirmations that the multi-task architecture started. If
either is missing the system has fallen back to single-task mode and will
freeze under motion — investigate before deploying.

### Live diagnostics

The `[STATS]` line printed once per second is the primary diagnostic. During
a healthy run on a 50 Hz broadcast you should see:

- `tx=50/s` consistently. Drops below ~45 indicate the broadcast is being
  delayed.
- `samples=190-200/s`. The IMU runs at 208 Hz; a small loss to skipped-`dt`
  samples is normal.
- `maxGap=21ms` (frame period). Anything > 100 ms is a stall worth
  investigating; > 500 ms is back-to-the-old-bad-days and means the
  `wifi_tx` task got preempted itself.

A previous debugging build included extra `[STALL]` lines that attributed
each stall to `imuUpd` / `wsLoop` / `send` / `http` durations and tagged
whether one or both cores froze. That instrumentation was removed when the
problem was resolved, but the code is in git history if a new class of
stall ever appears.
