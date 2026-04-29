# IMU Balance Board Firmware

ESP32 DevKit V1 + BNO085 IMU firmware for instrumented wobble board balance assessment.
Outputs real-time tilt angles and COP displacement to serial, with a trial system that computes 18 clinically validated balance metrics.

## Hardware

- ESP32 DOIT DevKit V1
- Adafruit BNO085 breakout (or SparkFun BNO086)

Uses **I2C** (the breakout's default protocol — no solder jumper changes needed).

### BNO085 Breakout Pin Reference

The Adafruit BNO085 breakout has pins on two sides:

**Side 1 (I2C header):** SDA, SCL, INT

| Pin | I2C Function | Description |
|---|---|---|
| SDA | I2C data | Bidirectional data line (has 10K pullup on breakout) |
| SCL | I2C clock | Clock line (has 10K pullup on breakout) |
| INT | Interrupt | BNO085 pulls low when new data is ready |

**Side 2 (config/SPI header):** DI, RST, CS, P0, P1

| Pin | I2C Function | Description |
|---|---|---|
| DI | Address select | LOW = address 0x4A (default), HIGH = 0x4B |
| RST | Reset | Pulse low to hard-reset the BNO085 |
| CS | Chip select | SPI only — leave unconnected for I2C |
| P0 (PS0) | Protocol select 0 | Solder jumper. Both LOW = I2C (default) |
| P1 (PS1) | Protocol select 1 | Solder jumper. See P0 above |
| BT | Boot | Leave unconnected for normal operation |

**Power pins:** VIN (3.3–5V), 3Vo (3.3V regulated output), GND

The board also has two **STEMMA QT / Qwiic** connectors for solderless I2C daisy-chaining.

### Wiring (I2C)

| ESP32 GPIO | Function | BNO085 Pin | Wire Color (suggestion) |
|---|---|---|---|
| GPIO 21 | SDA | SDA | Blue |
| GPIO 22 | SCL | SCL | Yellow |
| GPIO 4 | Reset | RST | White |
| 3V3 | Power | VIN | Red |
| GND | Ground | GND | Black |

Total: **5 wires**. No solder jumper changes needed — the breakout ships in I2C mode.

DI pin left unconnected (floats low = default address 0x4A). INT pin not required for I2C mode (the library polls via I2C). CS, P0, P1, BT left unconnected.

```
  ESP32 DevKit V1                     Adafruit BNO085 breakout
  ~~~~~~~~~~~~~~~                     ~~~~~~~~~~~~~~~~~~~~~~~~
  3V3  ——————————————————————————————  VIN
  GND  ——————————————————————————————  GND
  GPIO 21 (SDA) ————————————————————  SDA
  GPIO 22 (SCL) ————————————————————  SCL
  GPIO 4  (RST) ————————————————————  RST
```

**Alternative:** Use a STEMMA QT / Qwiic cable for SDA+SCL+3V3+GND (4 of the 5 wires), plus one jumper wire for RST.

### Why I2C instead of SPI?

- Board ships in I2C mode — works out of the box, no solder jumper modification
- Only 5 wires vs 8 for SPI
- 100 Hz reporting is well within I2C bandwidth at 400 kHz
- The known BNO085 I2C lockup issue is handled by the RST pin (firmware auto-resets if no data for 2 seconds)

## Building

Requires [PlatformIO](https://platformio.org/).

```bash
pio run              # compile
pio run -t upload    # flash to ESP32
pio device monitor   # open serial monitor at 115200 baud
```

## Serial Commands

| Command | Description |
|---|---|
| `START` | Begin a balance trial |
| `STOP` | Abort a running trial |
| `CALIBRATE` | Re-calibrate level reference (board must be flat and still) |
| `DURATION <N>` | Set trial duration in seconds (5–120, default 30) |
| `RADIUS <R>` | Set wobble board hemisphere radius in mm (default 40) |
| `STATUS` | Show current state, settings, free heap |
| `HELP` | Show command list |

## Serial Output

**Live streaming** (when idle, 10 Hz):
```
LIVE,<millis>,<roll_deg>,<pitch_deg>,<cop_ap_mm>,<cop_ml_mm>
```

**During trial** (10 Hz):
```
TRIAL,<millis>,<roll_deg>,<pitch_deg>,<cop_ap_mm>,<cop_ml_mm>,<progress%>
```

**After trial** — full metrics block with 18 balance metrics covering tilt-domain (degrees), COP-projected (mm), and jerk (smoothness).

## Balance Metrics Computed

| Category | Metrics |
|---|---|
| Tilt (degrees) | RMS tilt, path length, mean velocity, pitch/roll range, mean pitch/roll, 95% ellipse area, time at equilibrium |
| COP (mm) | Sway path, mean velocity, 95% ellipse area, AP/ML range, RMS displacement |
| Jerk (smoothness) | Mean jerk, RMS jerk |
