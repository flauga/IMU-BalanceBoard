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

The filter runs at **104 Hz** (the LSM6DSO ODR). Outputs are roll, pitch, yaw in degrees.

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
pio device monitor   # open serial monitor at 115200 baud
```

## WiFi Dashboard

On first boot the ESP32 connects to WiFi (configure credentials in `include/wifi_config.h`)
and serves the dashboard at **http://imuboard.local** (port 80).
The WebSocket stream runs on **port 81**.

From a tablet or phone on the same network, navigate to `http://imuboard.local` —
the page loads and auto-connects. No file copying needed.

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
| **Samples** | count | Number of frames received since session start. At 25 Hz the count rises ~25/s. |
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

## Serial Commands

| Command | Description |
|---|---|
| `START` | Begin streaming angle data |
| `STOP` | Pause streaming |
| `STATUS` | Show firmware version, sensor, heap, WiFi info |
| `RATE <hz>` | Set output rate 1–50 Hz (default 25 Hz) |
| `HELP` | Show command list |

## Serial Output Format

```
<millis>,<roll_deg>,<pitch_deg>,<yaw_deg>
```

Example:
```
12345,2.34,-1.12,87.50
```
