#include "wifi_manager.h"
#include "wifi_config.h"
#include "serial_command.h"
#include "config.h"
#include <esp_wifi.h>
#if MDNS_ENABLED
#  include <mdns.h>
#endif
#include <functional>
#include <cstdlib>

// Defined in main.cpp
extern void zeroOrientation();
extern void setDriftLog(bool on);

static const struct { const char* ssid; const char* pass; } NETWORKS[] = WIFI_CREDENTIALS;
static constexpr size_t   NETWORK_COUNT      = sizeof(NETWORKS) / sizeof(NETWORKS[0]);
static constexpr uint32_t CONNECT_TIMEOUT_MS = 10000;
static constexpr uint32_t RECONNECT_INTERVAL_MS = 30000;

// Dashboard HTML served over HTTP on port 80.
// Embedded here so the ESP32 can serve it without SPIFFS.
static const char DASHBOARD_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>IMU Balance Board — WiFi</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link href="https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;500&family=IBM+Plex+Sans:wght@400;500;600&display=swap" rel="stylesheet">
<script src="https://cdnjs.cloudflare.com/ajax/libs/Chart.js/4.4.1/chart.umd.js"></script>
<style>
:root {
  --bg:       #0e0e0f;
  --bg2:      #161618;
  --bg3:      #1e1e21;
  --bg4:      #26262a;
  --border:   rgba(255,255,255,.08);
  --border2:  rgba(255,255,255,.14);
  --text:     #e8e8ea;
  --text2:    #888890;
  --text3:    #55555c;
  --green:    #4ade80;
  --green-bg: rgba(74,222,128,.1);
  --red:      #f87171;
  --red-bg:   rgba(248,113,113,.1);
  --amber:    #fbbf24;
  --amber-bg: rgba(251,191,36,.1);
  --blue:     #60a5fa;
  --blue-bg:  rgba(96,165,250,.1);
  --r:        6px;
  --r2:       10px;
  --font:     'IBM Plex Sans', sans-serif;
  --mono:     'IBM Plex Mono', monospace;
}
* { box-sizing: border-box; margin: 0; padding: 0; }
html { background: var(--bg); color: var(--text); font-family: var(--font); font-size: 14px; }
body { max-width: 960px; margin: 0 auto; padding: 20px 16px 60px; }

.header { display: flex; align-items: center; gap: 10px; margin-bottom: 20px; }
.header h1 { font-size: 15px; font-weight: 600; letter-spacing: .04em; }
.header-sub { font-size: 12px; color: var(--text3); font-family: var(--mono); }
.spacer { flex: 1; }
.pill { display: inline-flex; align-items: center; gap: 6px; font-size: 12px; font-weight: 500;
        padding: 4px 10px; border-radius: 20px; background: var(--bg3); border: 1px solid var(--border); color: var(--text2); }
.dot { width: 7px; height: 7px; border-radius: 50%; background: var(--text3); flex-shrink: 0; }
.dot.on  { background: var(--green); box-shadow: 0 0 6px var(--green); }
.dot.rec { background: var(--red); animation: blink 1s infinite; }
@keyframes blink { 0%,100%{opacity:1} 50%{opacity:.25} }

.tabs { display: flex; gap: 1px; margin-bottom: 18px; border-bottom: 1px solid var(--border); }
.tab { font-size: 13px; font-weight: 500; padding: 8px 14px; background: none; border: none;
       color: var(--text2); cursor: pointer; border-bottom: 2px solid transparent; margin-bottom: -1px; transition: color .15s; }
.tab:hover { color: var(--text); }
.tab.active { color: var(--text); border-bottom-color: var(--text); }
.panel { display: none; }
.panel.active { display: block; }

button { font-family: var(--font); font-size: 13px; font-weight: 500; padding: 7px 14px;
          border-radius: var(--r); border: 1px solid var(--border2); background: var(--bg3);
          color: var(--text); cursor: pointer; transition: background .15s, opacity .15s; }
button:hover { background: var(--bg4); }
button:active { opacity: .75; }
button:disabled { opacity: .3; cursor: not-allowed; }
.btn-green { background: var(--green-bg); border-color: rgba(74,222,128,.3); color: var(--green); }
.btn-green:hover { background: rgba(74,222,128,.18); }
.btn-red   { background: var(--red-bg);   border-color: rgba(248,113,113,.3); color: var(--red); }
.btn-red:hover   { background: rgba(248,113,113,.18); }
.btn-amber { background: var(--amber-bg); border-color: rgba(251,191,36,.3);  color: var(--amber); }
.btn-amber:hover { background: rgba(251,191,36,.18); }

input[type=text], input[type=number] {
  font-family: var(--mono); font-size: 13px; padding: 7px 10px; border-radius: var(--r);
  border: 1px solid var(--border2); background: var(--bg3); color: var(--text); outline: none; }
input:focus { border-color: var(--blue); }

.card { background: var(--bg2); border: 1px solid var(--border); border-radius: var(--r2); padding: 14px 16px; margin-bottom: 12px; }
.card-title { font-size: 11px; font-weight: 500; letter-spacing: .08em; text-transform: uppercase; color: var(--text3); margin-bottom: 10px; }

.controls { display: flex; gap: 8px; flex-wrap: wrap; align-items: center; margin-bottom: 10px; }

.metrics { display: grid; grid-template-columns: repeat(4, minmax(0,1fr)); gap: 8px; margin-bottom: 10px; }
@media (max-width: 600px) { .metrics { grid-template-columns: repeat(2, minmax(0,1fr)); } }
.metric { background: var(--bg2); border: 1px solid var(--border); border-radius: var(--r2); padding: 12px 14px; }
.m-label { font-size: 11px; font-weight: 500; text-transform: uppercase; letter-spacing: .07em; color: var(--text3); margin-bottom: 6px; display: flex; align-items: center; gap: 6px; }
.m-tag { width: 8px; height: 8px; border-radius: 2px; flex-shrink: 0; }
.m-val  { font-size: 24px; font-weight: 500; font-family: var(--mono); line-height: 1; }
.m-unit { font-size: 11px; color: var(--text3); font-family: var(--mono); margin-top: 3px; }

.tilt-wrap { display: flex; flex-direction: column; align-items: center; gap: 8px; }
#tiltCanvas { display: block; border-radius: var(--r); background: var(--bg3); border: 1px solid var(--border); }
.scale-row { display: flex; align-items: center; gap: 10px; font-size: 12px; color: var(--text2); align-self: stretch; justify-content: center; }
.scale-row input[type=range] { accent-color: var(--blue); }

.chart-wrap { position: relative; height: 180px; }

.session-stats { display: grid; grid-template-columns: repeat(4, minmax(0,1fr)); gap: 8px; margin-bottom: 12px; }
@media (max-width: 600px) { .session-stats { grid-template-columns: repeat(2, minmax(0,1fr)); } }
.ss-item { background: var(--bg2); border: 1px solid var(--border); border-radius: var(--r2); padding: 12px 14px; }
.ss-label { font-size: 11px; font-weight: 500; text-transform: uppercase; letter-spacing: .07em; color: var(--text3); margin-bottom: 6px; }
.ss-val { font-size: 24px; font-weight: 500; font-family: var(--mono); line-height: 1; color: var(--text); }
.ss-unit { font-size: 11px; color: var(--text3); font-family: var(--mono); margin-top: 3px; }

.replay-bar { display: flex; align-items: center; gap: 10px; padding: 10px 14px;
              background: var(--bg2); border: 1px solid var(--border); border-radius: var(--r2); margin-bottom: 12px; font-size: 12px; color: var(--text2); }
.replay-bar input[type=range] { flex: 1; accent-color: var(--blue); min-width: 80px; }
.replay-time { font-family: var(--mono); color: var(--text); min-width: 90px; text-align: right; font-size: 12px; }
.replay-name { font-family: var(--mono); color: var(--text2); font-size: 11px; flex: 0 1 auto; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; max-width: 180px; }
.speed-sel { font-family: var(--mono); font-size: 12px; padding: 4px 8px; border-radius: var(--r);
             border: 1px solid var(--border2); background: var(--bg3); color: var(--text); outline: none; }

.log { background: var(--bg); border: 1px solid var(--border); border-radius: var(--r);
       height: 90px; overflow-y: auto; padding: 6px 10px; font-family: var(--mono); font-size: 11px; color: var(--text3); }
.log-line { padding: 1px 0; }
.log-ok   { color: var(--green); }
.log-warn { color: var(--amber); }
.log-err  { color: var(--red); }
</style>
</head>
<body>

<div class="header">
  <h1>IMU BALANCE BOARD</h1>
  <span class="header-sub">BNO085 · ESP32 · WiFi</span>
  <div class="spacer"></div>
  <span class="pill" id="rssi-pill" style="display:none" title="WiFi signal strength">
    <span id="rssi-val">–</span> dBm
  </span>
  <span class="pill" id="heap-pill" style="display:none;margin-left:6px" title="ESP32 free heap">
    <span id="heap-val">–</span> KB
  </span>
  <span class="pill" id="hz-pill" style="display:none;margin-left:6px"><span id="hz-val">–</span> Hz</span>
  <span class="pill" style="margin-left:6px"><span class="dot" id="status-dot"></span><span id="status-text">Disconnected</span></span>
</div>

<div class="tabs">
  <button class="tab active" onclick="switchTab('live')">Live</button>
  <button class="tab" onclick="switchTab('about')">About</button>
</div>

<!-- LIVE TAB -->
<div class="panel active" id="panel-live">

  <div class="controls">
    <input type="text" id="host-input" value="imuboard.local" style="width:190px" placeholder="host or IP">
    <button class="btn-green" id="btn-connect">Connect</button>
    <button class="btn-red"   id="btn-disconnect" disabled>Disconnect</button>
    <button class="btn-green" id="btn-record" disabled>▶ Start Session</button>
    <button class="btn-red"   id="btn-stop"   disabled>■ Stop &amp; Save CSV</button>
    <button id="btn-zero" disabled>Zero</button>
    <button class="btn-amber" id="btn-load-replay">⏵ Load CSV…</button>
    <input type="file" id="replay-file" accept=".csv,text/csv" style="display:none">
  </div>

  <!-- Replay bar (shown during replay) -->
  <div class="replay-bar" id="replay-bar" style="display:none">
    <button id="btn-replay-play" class="btn-green">▶</button>
    <button id="btn-replay-stop" class="btn-red">■</button>
    <input type="range" id="replay-seek" min="0" max="1000" value="0" step="1">
    <span class="replay-time" id="replay-time">0.0 / 0.0s</span>
    <select id="replay-speed" class="speed-sel">
      <option value="0.25">0.25×</option>
      <option value="0.5">0.5×</option>
      <option value="1" selected>1×</option>
      <option value="2">2×</option>
      <option value="4">4×</option>
    </select>
    <span class="replay-name" id="replay-name"></span>
  </div>

  <div class="metrics">
    <div class="metric">
      <div class="m-label"><span class="m-tag" style="background:#60a5fa"></span>Roll</div>
      <div class="m-val" style="color:#60a5fa" id="v-roll">–</div>
      <div class="m-unit">degrees</div>
    </div>
    <div class="metric">
      <div class="m-label"><span class="m-tag" style="background:#4ade80"></span>Pitch</div>
      <div class="m-val" style="color:#4ade80" id="v-pitch">–</div>
      <div class="m-unit">degrees</div>
    </div>
    <div class="metric">
      <div class="m-label"><span class="m-tag" style="background:var(--amber)"></span>Yaw</div>
      <div class="m-val" style="color:var(--amber)" id="v-yaw">–</div>
      <div class="m-unit">degrees</div>
    </div>
    <div class="metric">
      <div class="m-label"><span class="m-tag" style="background:var(--text2)"></span>Tilt</div>
      <div class="m-val" id="v-tilt">–</div>
      <div class="m-unit">√(roll²+pitch²) °</div>
    </div>
  </div>

  <div class="card">
    <div style="display:flex;align-items:center;justify-content:space-between;margin-bottom:8px">
      <span class="card-title">Tilt Plot — Roll (X) · Pitch (Y)</span>
      <span style="font-size:11px;color:var(--text3)">Trail: 100 samples · 25 Hz</span>
    </div>
    <div class="tilt-wrap">
      <canvas id="tiltCanvas" width="500" height="500"></canvas>
      <div class="scale-row">
        <span>Scale:</span>
        <input type="range" id="max-angle-slider" min="5" max="90" value="30" step="5" style="width:130px">
        <span id="max-angle-label">±30°</span>
      </div>
    </div>
  </div>

  <div class="card">
    <div style="display:flex;align-items:center;justify-content:space-between;margin-bottom:8px">
      <span class="card-title">Angles over time</span>
      <span style="font-size:11px;color:var(--text3)">last 300 samples</span>
    </div>
    <div class="chart-wrap"><canvas id="timeChart"></canvas></div>
  </div>

  <div class="session-stats">
    <div class="ss-item">
      <div class="ss-label">Samples</div>
      <div class="ss-val" id="ss-n">0</div>
      <div class="ss-unit">count</div>
    </div>
    <div class="ss-item">
      <div class="ss-label">Duration</div>
      <div class="ss-val" id="ss-dur">–</div>
      <div class="ss-unit">mm:ss</div>
    </div>
    <div class="ss-item">
      <div class="ss-label">Max tilt</div>
      <div class="ss-val" id="ss-peak">–</div>
      <div class="ss-unit">degrees</div>
    </div>
    <div class="ss-item">
      <div class="ss-label">Avg tilt</div>
      <div class="ss-val" id="ss-avg">–</div>
      <div class="ss-unit">degrees</div>
    </div>
    <div class="ss-item">
      <div class="ss-label">Sway path</div>
      <div class="ss-val" id="ss-path">–</div>
      <div class="ss-unit">degrees</div>
    </div>
    <div class="ss-item">
      <div class="ss-label">Mean velocity</div>
      <div class="ss-val" id="ss-vel">–</div>
      <div class="ss-unit">°/s</div>
    </div>
    <div class="ss-item">
      <div class="ss-label">Sway area</div>
      <div class="ss-val" id="ss-sway">–</div>
      <div class="ss-unit">°²</div>
    </div>
  </div>

  <div class="log" id="log"></div>
</div>

<!-- ABOUT TAB -->
<div class="panel" id="panel-about">
  <div class="card">
    <div class="card-title">WebSocket Protocol</div>
    <p style="font-size:12px;color:var(--text2);line-height:1.8;font-family:var(--mono)">
      Host: <span style="color:var(--text)" id="about-host">imuboard.local</span> &nbsp; HTTP: <span style="color:var(--text)">80</span> &nbsp; WS: <span style="color:var(--text)">81</span><br>
      Binary frame (default): 16 B LE — uint32 ms · float32 roll · pitch · yaw<br>
      Text frame: <span style="color:var(--text)">&lt;ms&gt;,&lt;roll&gt;,&lt;pitch&gt;,&lt;yaw&gt;</span><br>
      Status frame: <span style="color:var(--text)">[STATUS] {json}</span> — broadcast periodically
    </p>
  </div>
  <div class="card">
    <div class="card-title">CSV format</div>
    <p style="font-size:12px;color:var(--text2);line-height:1.8;font-family:var(--mono)">
      timestamp, elapsed_s, device_ms, roll_deg, pitch_deg, yaw_deg
    </p>
  </div>
  <div class="card">
    <div class="card-title">Hardware</div>
    <p style="font-size:12px;color:var(--text2);line-height:1.7">
      ESP32 DevKit V1 + LSM6DSO (I²C).<br>
      Sensor fusion: Mahony complementary filter at 208 Hz on-device.<br>
      Output rate: 50 Hz over WebSocket.
    </p>
  </div>
</div>

<script>
'use strict';

const WS_PORT   = 81;
const MAX_CHART = 300;
const TRAIL_LEN = 100;

// ── state ──────────────────────────────────────────────────────────────────
let ws             = null;
let userDisconnect = false;   // true when user clicked Disconnect (suppresses auto-reconnect)
let connected      = false;
let recording      = false;
let maxAngle       = 30;
let sessionStart   = 0;
let chartStart     = 0;       // set on first received frame, not on record start
let csvRows        = [];
let sampleCount    = 0;
let peakTilt       = 0;
let sumTilt        = 0;
let swayPath       = 0;
let prevRecRoll    = null;
let prevRecPitch   = null;
let timerHandle    = null;
let sampleTimes    = [];

// Session-wide sway accumulator (Welford online mean + covariance).
// Resets on connect, on record start, and on replay load.
let swayN     = 0;
let swayMx    = 0;  // mean roll
let swayMy    = 0;  // mean pitch
let swayCxx   = 0;  // Σ (r-mx)²
let swayCyy   = 0;  // Σ (p-my)²
let swayCxy   = 0;  // Σ (r-mx)(p-my)
// When true, the accumulator stops growing and the displayed value freezes
// on whatever the final number was. Set by Stop Session; cleared by Start
// Session, Zero, or Connect.
let swayFrozen = false;

// Jitter buffer: playback is keyed on the ESP32's deviceMs timestamp, not
// browser arrival time. This means a network stall (e.g. 1.5 s where no
// frames arrive then 75 burst at once) plays back at the original 50 Hz
// pace — the lag absorbs it, and rendering stays smooth instead of
// "fast-forwarding" through the burst.
//
// JITTER_LAG sets how much cushion we keep. With the dedicated wifi_tx task
// on the ESP32 the big multi-second stalls are gone, so the lag only needs
// to smooth out small jitter from the network and rAF scheduling. 200 ms
// keeps the dashboard feeling responsive while still hiding 100-150 ms
// hiccups. Device-time playback (see updateLive) ensures any burst of
// late frames plays back at 50 Hz, not at burst speed.
const FRAME_MS   = 20;          // nominal 50 Hz inter-frame period
const JITTER_LAG = 200;         // ms of playback cushion behind real time
const FRAME_BUF  = [];          // [{ roll, pitch, yaw, wallT, deviceMs }, ...]
// Hold ~1.5 s of history at 50 Hz — well above JITTER_LAG so an occasional
// gap longer than the lag still has frames buffered when they arrive.
const FRAME_BUF_MAX = 75;
let prevFrame = null;           // kept for backwards compat with updateLive
let currFrame = null;

// Replay state
let replayFrames   = [];    // [{ elapsed, device_ms, roll, pitch, yaw }, ...]
let replayIdx      = 0;
let replayPlaying  = false;
let replayTimer    = null;
let replaySpeed    = 1;
let replayName     = '';

const trailRoll  = new Float32Array(TRAIL_LEN);
const trailPitch = new Float32Array(TRAIL_LEN);
let trailHead = 0, trailFull = false;
let lastRoll = 0, lastPitch = 0;

const chartData = { labels: [], roll: [], pitch: [], yaw: [] };
let chart = null;

const $ = id => document.getElementById(id);
const tiltCanvas = $('tiltCanvas');
const tiltCtx    = tiltCanvas.getContext('2d');

// ── tabs ───────────────────────────────────────────────────────────────────
function switchTab(id) {
  document.querySelectorAll('.tab').forEach((t, i) => {
    t.classList.toggle('active', ['live','about'][i] === id);
  });
  document.querySelectorAll('.panel').forEach((p, i) => {
    p.classList.toggle('active', ['live','about'][i] === id);
  });
}

// ── Chart.js ───────────────────────────────────────────────────────────────
function initChart() {
  if (chart) chart.destroy();
  ['labels','roll','pitch','yaw'].forEach(k => { chartData[k].length = 0; });
  chartStart = 0;
  const ctx = $('timeChart').getContext('2d');
  chart = new Chart(ctx, {
    type: 'line',
    data: {
      labels: chartData.labels,
      datasets: [
        { label: 'Roll',  data: chartData.roll,  borderColor: '#60a5fa', borderWidth: 1.5, pointRadius: 0, tension: .3, backgroundColor: 'transparent' },
        { label: 'Pitch', data: chartData.pitch, borderColor: '#4ade80', borderWidth: 1.5, pointRadius: 0, tension: .3, backgroundColor: 'transparent' },
        { label: 'Yaw',   data: chartData.yaw,   borderColor: '#fbbf24', borderWidth: 1.5, pointRadius: 0, tension: .3, backgroundColor: 'transparent' },
      ]
    },
    options: {
      responsive: true, maintainAspectRatio: false, animation: false,
      plugins: { legend: { display: false } },
      scales: {
        x: { ticks: { color: '#55555c', font: { size: 10, family: 'IBM Plex Mono' }, maxTicksLimit: 8 }, grid: { color: 'rgba(255,255,255,.04)' }, border: { color: 'rgba(255,255,255,.08)' } },
        y: { ticks: { color: '#55555c', font: { size: 10, family: 'IBM Plex Mono' } }, grid: { color: 'rgba(255,255,255,.04)' }, border: { color: 'rgba(255,255,255,.08)' } }
      }
    }
  });
}

// Chart update throttle: pushing data is cheap, but chart.update() does a full
// canvas redraw and is the most expensive thing in the WS message handler. At
// 50 Hz it serialises the browser's event loop and slows WebSocket ACKs back
// to the ESP32, which fills the ESP's TCP send buffer and stalls the
// broadcast loop. Throttling to ~20 Hz keeps the chart looking smooth (no
// visible loss of detail at 300-sample horizon) while freeing the event loop.
const CHART_REDRAW_MS = 50;     // ≈20 Hz
let chartDirty       = false;
let chartLastDrawMs  = 0;

function pushChartPoint(elapsed, roll, pitch, yaw) {
  chartData.labels.push(elapsed.toFixed(1) + 's');
  chartData.roll.push(roll);
  chartData.pitch.push(pitch);
  chartData.yaw.push(yaw);
  if (chartData.labels.length > MAX_CHART) {
    chartData.labels.shift();
    chartData.roll.shift(); chartData.pitch.shift(); chartData.yaw.shift();
  }
  chartDirty = true;
}

// requestAnimationFrame loop: redraw chart at most CHART_REDRAW_MS apart,
// regardless of incoming frame rate. Coalesces multiple WS updates into one
// canvas redraw and yields the main thread back to the WS reader.
(function chartRafLoop() {
  requestAnimationFrame(chartRafLoop);
  if (!chartDirty || !chart) return;
  const now = performance.now();
  if (now - chartLastDrawMs < CHART_REDRAW_MS) return;
  chartLastDrawMs = now;
  chartDirty = false;
  chart.update('none');
}());

// ── Tilt plot ──────────────────────────────────────────────────────────────
function clampToCircle(r, p, max) {
  const mag = Math.sqrt(r * r + p * p);
  if (mag > max) { const s = max / mag; return [r * s, p * s]; }
  return [r, p];
}

// Feed one sample into the session-wide Welford accumulator.
function swayAccumulate(roll, pitch) {
  swayN++;
  const dx = roll  - swayMx;
  const dy = pitch - swayMy;
  swayMx += dx / swayN;
  swayMy += dy / swayN;
  const dx2 = roll  - swayMx;
  const dy2 = pitch - swayMy;
  swayCxx += dx * dx2;
  swayCyy += dy * dy2;
  swayCxy += dx * dy2;
}

function resetSwayAccumulator() {
  swayN = 0; swayMx = 0; swayMy = 0;
  swayCxx = 0; swayCyy = 0; swayCxy = 0;
}

// 70% prediction ellipse from session-wide mean/covariance.
// For a 2D Gaussian, p=0.70 → chi-square critical value ≈ 2.408.
// This is the region the user spent ~70% of their time within.
function computeSwayEllipse() {
  if (swayN < 5) return null;
  const cxx = swayCxx / (swayN - 1);
  const cyy = swayCyy / (swayN - 1);
  const cxy = swayCxy / (swayN - 1);
  const trace = cxx + cyy;
  const det   = cxx * cyy - cxy * cxy;
  const disc  = Math.sqrt(Math.max(0, (trace / 2) ** 2 - det));
  const lam1  = trace / 2 + disc, lam2 = trace / 2 - disc;
  const k     = Math.sqrt(2.408);  // p = 0.70 (was 5.991 for p = 0.95)
  const semiA = Math.sqrt(Math.max(0, lam1)) * k;
  const semiB = Math.sqrt(Math.max(0, lam2)) * k;
  const angle = Math.atan2(2 * cxy, cxx - cyy) / 2;
  return { cx: swayMx, cy: swayMy, semiA, semiB, angle, area: Math.PI * semiA * semiB };
}

function drawTiltPlot(roll, pitch) {
  const W = tiltCanvas.width, H = tiltCanvas.height;
  const cx = W / 2, cy = H / 2;
  const radius = Math.min(W, H) / 2 - 32;
  const scale  = radius / maxAngle;

  tiltCtx.clearRect(0, 0, W, H);
  tiltCtx.fillStyle = '#161618';
  tiltCtx.beginPath(); tiltCtx.arc(cx, cy, radius, 0, Math.PI * 2); tiltCtx.fill();

  const rings = [5, 10, 15, 20, 30, 45, 60, 90].filter(r => r <= maxAngle);
  tiltCtx.font = '9px IBM Plex Mono, monospace';
  tiltCtx.textAlign = 'center';
  for (const r of rings) {
    const rPx = r * scale;
    tiltCtx.beginPath(); tiltCtx.arc(cx, cy, rPx, 0, Math.PI * 2);
    tiltCtx.strokeStyle = r % 10 === 0 ? 'rgba(255,255,255,0.10)' : 'rgba(255,255,255,0.05)';
    tiltCtx.lineWidth = 1; tiltCtx.stroke();
    tiltCtx.fillStyle = 'rgba(255,255,255,0.22)';
    tiltCtx.fillText(`${r}°`, cx, cy - rPx + 11);
  }

  tiltCtx.save();
  tiltCtx.beginPath(); tiltCtx.arc(cx, cy, radius, 0, Math.PI * 2); tiltCtx.clip();

  tiltCtx.strokeStyle = 'rgba(255,255,255,0.14)'; tiltCtx.lineWidth = 1;
  tiltCtx.setLineDash([4, 5]);
  tiltCtx.beginPath(); tiltCtx.moveTo(cx - radius, cy); tiltCtx.lineTo(cx + radius, cy); tiltCtx.stroke();
  tiltCtx.beginPath(); tiltCtx.moveTo(cx, cy - radius); tiltCtx.lineTo(cx, cy + radius); tiltCtx.stroke();
  tiltCtx.setLineDash([]);

  const ellipse = computeSwayEllipse();
  if (ellipse) {
    const ex = cx + ellipse.cx * scale, ey = cy - ellipse.cy * scale;
    const aR = ellipse.semiA * scale, bR = ellipse.semiB * scale;
    const early = swayN < 20;
    tiltCtx.save();
    tiltCtx.translate(ex, ey); tiltCtx.rotate(-ellipse.angle);
    tiltCtx.beginPath(); tiltCtx.ellipse(0, 0, Math.max(aR, 1), Math.max(bR, 1), 0, 0, Math.PI * 2);
    tiltCtx.fillStyle   = early ? 'rgba(251,191,36,0.05)' : 'rgba(251,191,36,0.10)';
    tiltCtx.fill();
    tiltCtx.strokeStyle = early ? 'rgba(251,191,36,0.35)' : 'rgba(251,191,36,0.70)';
    tiltCtx.lineWidth   = 1.5;
    tiltCtx.setLineDash(early ? [4, 4] : []);
    tiltCtx.stroke(); tiltCtx.setLineDash([]);
    tiltCtx.restore();
  }

  const N = trailFull ? TRAIL_LEN : trailHead;
  if (N > 1) {
    const BANDS = 8, bandSize = Math.ceil((N - 1) / BANDS);
    for (let b = 0; b < BANDS; b++) {
      const s = b * bandSize, e = Math.min(s + bandSize, N - 1);
      if (s >= e) break;
      const t = (s + e) / 2 / Math.max(N - 2, 1);
      tiltCtx.beginPath();
      const i0 = trailFull ? (trailHead + s) % TRAIL_LEN : s;
      const [r0, p0] = clampToCircle(trailRoll[i0], trailPitch[i0], maxAngle);
      tiltCtx.moveTo(cx + r0 * scale, cy - p0 * scale);
      for (let i = s + 1; i <= e; i++) {
        const idx = trailFull ? (trailHead + i) % TRAIL_LEN : i;
        const [rc, pc] = clampToCircle(trailRoll[idx], trailPitch[idx], maxAngle);
        tiltCtx.lineTo(cx + rc * scale, cy - pc * scale);
      }
      tiltCtx.strokeStyle = `rgba(148,163,184,${(0.10 + 0.62 * t).toFixed(2)})`;
      tiltCtx.lineWidth   = 0.8 + 1.4 * t;
      tiltCtx.lineJoin = tiltCtx.lineCap = 'round';
      tiltCtx.stroke();
    }
  }

  const [cr, cp] = clampToCircle(roll, pitch, maxAngle);
  const dx = cx + cr * scale, dy = cy - cp * scale;
  tiltCtx.beginPath(); tiltCtx.arc(dx, dy, 14, 0, Math.PI * 2);
  tiltCtx.fillStyle = 'rgba(74,222,128,0.12)'; tiltCtx.fill();
  tiltCtx.beginPath(); tiltCtx.arc(dx, dy, 6, 0, Math.PI * 2);
  tiltCtx.fillStyle = '#4ade80';
  tiltCtx.shadowColor = '#4ade80'; tiltCtx.shadowBlur = 12;
  tiltCtx.fill(); tiltCtx.shadowBlur = 0;
  tiltCtx.restore();

  tiltCtx.beginPath(); tiltCtx.arc(cx, cy, radius, 0, Math.PI * 2);
  tiltCtx.strokeStyle = 'rgba(255,255,255,0.14)'; tiltCtx.lineWidth = 1.5; tiltCtx.stroke();
  tiltCtx.fillStyle = 'rgba(255,255,255,0.35)';
  tiltCtx.font = '11px IBM Plex Sans, sans-serif';
  tiltCtx.textAlign = 'center';
  tiltCtx.fillText('Pitch (+)', cx, cy - radius - 10);
  tiltCtx.fillText('Pitch (−)', cx, cy + radius + 18);
  tiltCtx.textAlign = 'left';  tiltCtx.fillText('Roll (+)', cx + radius + 8, cy + 4);
  tiltCtx.textAlign = 'right'; tiltCtx.fillText('Roll (−)', cx - radius - 8, cy + 4);
}

drawTiltPlot(0, 0);

// ── requestAnimationFrame render loop ─────────────────────────────────────
// Single source of truth for every visible element: dot, trail, metric tiles,
// time chart, sway ellipse. All driven from a buffer playhead running
// JITTER_LAG ms behind real device time so network stalls are absorbed
// invisibly. Live-arrival concerns (Hz pill, CSV recording) stay in
// updateLive() since they need the actual arrival timeline.
//
// The playhead is in DEVICE TIME (the ESP32's millis() timestamp on each
// frame), not browser wall-clock. So when 75 frames burst-arrive after a
// 1 s stall, they still play back at 50 Hz — not at the arrival burst rate.
//
// On first connection we sync device-time to wall-clock by recording the
// offset (deviceMs0 - now0) and then renderDeviceMs = nowT - offset - lag.
let deviceWallOffset   = 0;   // wallT(arrival) - deviceMs(frame) for first frame
let firstFrameSynced   = false;
let lastConsumedDevMs  = 0;   // deviceMs of last frame whose state-updates
                              //   (trail, chart, sway) we've processed.
let bufferStartDevMs   = 0;   // device-ms of the very first frame, used as
                              //   the time-zero for the chart X axis.

(function rafLoop() {
  requestAnimationFrame(rafLoop);
  if (FRAME_BUF.length < 1) return;

  // Sync device time to wall time on first frame
  if (!firstFrameSynced) {
    const f0 = FRAME_BUF[0];
    deviceWallOffset = f0.wallT - f0.deviceMs;
    bufferStartDevMs = f0.deviceMs;
    lastConsumedDevMs = f0.deviceMs - 1;  // so frame 0 gets consumed
    firstFrameSynced = true;
  }

  // Playback position, in device-ms space.
  const nowT = performance.now();
  const sinceStart = nowT - (bufferStartDevMs + deviceWallOffset);
  const effectiveLag = Math.min(JITTER_LAG, sinceStart);
  // renderDevMs is "what deviceMs should we be showing right now?"
  const renderDevMs = nowT - deviceWallOffset - effectiveLag;

  if (FRAME_BUF.length < 2) {
    lastRoll = FRAME_BUF[0].roll; lastPitch = FRAME_BUF[0].pitch;
    drawTiltPlot(lastRoll, lastPitch);
    return;
  }

  // Find the two frames bracketing renderDevMs in device time.
  let lo = 0;
  for (let i = 1; i < FRAME_BUF.length - 1; i++) {
    if (FRAME_BUF[i].deviceMs <= renderDevMs) lo = i;
    else break;
  }
  const hi = Math.min(lo + 1, FRAME_BUF.length - 1);

  const a = FRAME_BUF[lo], b = FRAME_BUF[hi];
  let roll, pitch, yaw;
  if (a.deviceMs !== b.deviceMs) {
    const alpha = Math.max(0, Math.min(1,
      (renderDevMs - a.deviceMs) / (b.deviceMs - a.deviceMs)));
    roll  = a.roll  + alpha * (b.roll  - a.roll);
    pitch = a.pitch + alpha * (b.pitch - a.pitch);
    yaw   = a.yaw   + alpha * (b.yaw   - a.yaw);
  } else {
    roll = a.roll; pitch = a.pitch; yaw = a.yaw;
  }

  // Interpolated values drive everything visible.
  lastRoll = roll; lastPitch = pitch;
  drawTiltPlot(roll, pitch);

  $('v-roll').textContent  = roll.toFixed(2);
  $('v-pitch').textContent = pitch.toFixed(2);
  $('v-yaw').textContent   = yaw.toFixed(2);
  const tilt = Math.sqrt(roll * roll + pitch * pitch);
  $('v-tilt').textContent  = tilt.toFixed(2);

  // Walk every frame the playhead has crossed since the last rAF tick and
  // update sample-paced visuals (trail, sway, chart). Keyed on deviceMs so
  // each real sample is consumed exactly once, at the right pace.
  for (let i = 0; i < FRAME_BUF.length; i++) {
    const f = FRAME_BUF[i];
    if (f.deviceMs <= lastConsumedDevMs) continue;
    if (f.deviceMs > renderDevMs) break;
    trailRoll[trailHead]  = f.roll;
    trailPitch[trailHead] = f.pitch;
    trailHead = (trailHead + 1) % TRAIL_LEN;
    if (trailHead === 0) trailFull = true;
    if (!swayFrozen) swayAccumulate(f.roll, f.pitch);
    const elapsedS = (f.deviceMs - bufferStartDevMs) / 1000;
    pushChartPoint(elapsedS, f.roll, f.pitch, f.yaw);
    lastConsumedDevMs = f.deviceMs;
  }

  // When frozen, leave the displayed sway number untouched — it represents
  // the final value at Stop time. drawTiltPlot still draws the (now-static)
  // ellipse from the unchanged accumulator state.
  if (!swayFrozen) {
    const ellipse = computeSwayEllipse();
    $('ss-sway').textContent = ellipse ? ellipse.area.toFixed(2) : '–';
  }
}());

function clearVisuals() {
  FRAME_BUF.length = 0;
  firstFrameSynced  = false;
  deviceWallOffset  = 0;
  bufferStartDevMs  = 0;
  lastConsumedDevMs = 0;
  prevFrame = null; currFrame = null;
  trailHead = 0; trailFull = false;
  trailRoll.fill(0); trailPitch.fill(0);
  ['labels','roll','pitch','yaw'].forEach(k => { chartData[k].length = 0; });
  chartStart = 0;
  resetSwayAccumulator();
  // Coming out of a frozen state, we need the accumulator to start fresh
  // AND we need the displayed number to follow live samples again.
  swayFrozen = false;
  $('ss-sway').textContent = '–';
  if (chart) chart.update('none');
  drawTiltPlot(lastRoll, lastPitch);
}

function resizeCanvas() {
  const wrap = tiltCanvas.parentElement;
  const maxSize = Math.min(wrap.clientWidth - 16, 600);
  if (maxSize !== tiltCanvas.width) {
    tiltCanvas.width = tiltCanvas.height = maxSize;
    // rAF loop will redraw on next frame; draw once immediately to avoid blank flash
    drawTiltPlot(lastRoll, lastPitch);
  }
}
window.addEventListener('resize', resizeCanvas);
resizeCanvas();

// ── Live update ────────────────────────────────────────────────────────────
function updateLive(t, roll, pitch, yaw) {
  // Push into jitter buffer; rAF loop reads from it with JITTER_LAG.
  // ALL visible state (dot, trail, chart, metric tiles, sway ellipse) is
  // driven from the rAF loop's playback timeline so everything stays in sync.
  //
  // Each frame is tagged with TWO timestamps:
  //   deviceMs : the ESP32's millis() when the sample was captured. Used
  //              for playback timing so bursty arrivals (after a network
  //              stall) play back at the original 50 Hz pace.
  //   wallT    : performance.now() on the browser at arrival. Used only
  //              for the Hz indicator and stall detection.
  // Without deviceMs-based playback, a 1 s stall produces a 50-frame burst
  // arriving within ~50 ms; the buffer would play it back at burst speed
  // and the dashboard would visibly "fast-forward" through the stall.
  const wallT = performance.now();
  FRAME_BUF.push({ roll, pitch, yaw, wallT, deviceMs: t });
  if (FRAME_BUF.length > FRAME_BUF_MAX) FRAME_BUF.shift();
  // Keep legacy refs so replay path still works
  prevFrame = currFrame;
  currFrame = { roll, pitch, wallT };

  const tilt = Math.sqrt(roll * roll + pitch * pitch);

  // Hz via wall-clock receive deltas — measures actual link rate, not the
  // displayed (lagged) rate, so it stays meaningful for diagnostics.
  const now = Date.now();
  sampleTimes.push(now);
  sampleTimes = sampleTimes.filter(ts => now - ts < 2000);
  if (sampleTimes.length >= 2) {
    $('hz-val').textContent = (sampleTimes.length / 2).toFixed(0);
    $('hz-pill').style.display = '';
  }

  if (recording) {
    sampleCount++;
    sumTilt += tilt;
    if (tilt > peakTilt) { peakTilt = tilt; $('ss-peak').textContent = tilt.toFixed(2); }
    $('ss-n').textContent   = String(sampleCount);
    $('ss-avg').textContent = (sumTilt / sampleCount).toFixed(2);

    // Sway path: Σ√(Δroll²+Δpitch²) in degree-space
    if (prevRecRoll !== null) {
      const dr = roll - prevRecRoll, dp = pitch - prevRecPitch;
      swayPath += Math.sqrt(dr * dr + dp * dp);
    }
    prevRecRoll = roll; prevRecPitch = pitch;
    $('ss-path').textContent = swayPath.toFixed(2);
    const recElapsed = (Date.now() - sessionStart) / 1000;
    if (recElapsed > 0) $('ss-vel').textContent = (swayPath / recElapsed).toFixed(2);

    csvRows.push([new Date().toISOString(), recElapsed.toFixed(3), t,
                  roll.toFixed(4), pitch.toFixed(4), yaw.toFixed(4)].join(','));
  }
}

// ── WebSocket ──────────────────────────────────────────────────────────────
function connect() {
  if (ws && ws.readyState <= WebSocket.OPEN) return;
  const host = $('host-input').value.trim();
  if (!host) { log('Enter a hostname or IP first', 'warn'); return; }
  const url = `ws://${host}:${WS_PORT}`;
  log('Connecting to ' + url);
  $('btn-connect').disabled = true;
  userDisconnect = false;

  ws = new WebSocket(url);
  ws.binaryType = 'arraybuffer';

  ws.onopen = () => {
    connected = true;
    setStatus('connected');
    log('Connected to ' + url, 'ok');
    initChart();
    resetSwayAccumulator();
    swayFrozen = false;
    $('ss-sway').textContent = '–';
    // Opt into binary frames: 16 B vs ~30 B per frame, no snprintf cost on
    // the ESP32. The server falls back to text for any client that doesn't
    // ask, so older dashboards keep working.
    try { ws.send('BIN ON'); } catch (_) { /* ignore */ }
  };

  ws.onclose = () => {
    connected = false;
    ws = null;
    $('btn-connect').disabled = false;
    if (recording) stopRecording();
    setStatus('disconnected');
    log('Disconnected');
    if (!userDisconnect) {
      log('Reconnecting in 3s…', 'warn');
      setTimeout(() => { if (!connected) connect(); }, 3000);
    }
  };

  ws.onerror = () => {
    log('WebSocket error — check host and that ESP32 is on same network', 'err');
  };

  ws.onmessage = (ev) => {
    // Binary frame: 16 bytes little-endian — uint32 ms, float32 roll/pitch/yaw
    if (ev.data instanceof ArrayBuffer) {
      if (ev.data.byteLength < 16) return;
      const dv = new DataView(ev.data);
      const t = dv.getUint32(0, true);
      const r = dv.getFloat32(4, true);
      const p = dv.getFloat32(8, true);
      const y = dv.getFloat32(12, true);
      if ([t, r, p, y].some(v => Number.isNaN(v))) return;
      updateLive(t, r, p, y);
      return;
    }

    const line = ev.data.trim();
    if (!line) return;

    if (line.startsWith('[STATUS]')) {
      try {
        const s = JSON.parse(line.slice(8).trim());
        if (typeof s.rssi === 'number') {
          $('rssi-val').textContent = s.rssi;
          // Tint by link quality so the user knows when to move closer.
          let color;
          if      (s.rssi >= -65) color = 'var(--green)';
          else if (s.rssi >= -75) color = 'var(--amber)';
          else                    color = 'var(--red)';
          $('rssi-val').style.color = color;
          $('rssi-pill').style.display = '';
        }
        if (typeof s.heap === 'number') {
          $('heap-val').textContent = Math.round(s.heap / 1024);
          $('heap-pill').style.display = '';
        }
      } catch (_) { /* ignore malformed status */ }
      return;
    }

    const parts = line.split(',');
    if (parts.length === 4) {
      const [t, r, p, y] = parts.map(Number);
      if ([t, r, p, y].some(isNaN)) return;
      updateLive(t, r, p, y);
    }
  };
}

function disconnect() {
  userDisconnect = true;
  if (ws) ws.close();
  // ws is nulled inside onclose
}

function sendCmd(cmd) {
  // Some buttons still call this (e.g., the Zero button sends ZERO). Kept as
  // a minimal helper. Logs only on error so the bottom log stays quiet.
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(cmd);
  } else {
    log('Not connected', 'warn');
  }
}

// ── Status / UI helpers ────────────────────────────────────────────────────
function setStatus(state) {
  const dot = $('status-dot');
  const txt = $('status-text');
  dot.className = 'dot';
  if (state === 'connected') {
    dot.classList.add('on'); txt.textContent = 'Connected';
    $('btn-connect').disabled    = true;
    $('btn-disconnect').disabled = false;
    $('btn-record').disabled     = false;
    $('btn-zero').disabled       = false;
  } else if (state === 'recording') {
    dot.classList.add('rec'); txt.textContent = 'Recording';
  } else {
    txt.textContent = 'Disconnected';
    $('btn-connect').disabled    = false;
    $('btn-disconnect').disabled = true;
    $('btn-record').disabled     = true;
    $('btn-stop').disabled       = true;
    $('btn-zero').disabled       = true;
    $('v-roll').textContent = $('v-pitch').textContent = $('v-yaw').textContent = $('v-tilt').textContent = '–';
    $('hz-pill').style.display = 'none';
    $('rssi-pill').style.display = 'none';
    $('heap-pill').style.display = 'none';
    $('rssi-val').style.color = '';
    sampleTimes.length = 0;
  }
}

function startRecording() {
  if (recording) return;
  csvRows = []; sampleCount = 0; peakTilt = 0; sumTilt = 0;
  swayPath = 0; prevRecRoll = null; prevRecPitch = null;
  // Fresh session: reset the sway accumulator and unfreeze so live samples
  // start contributing again.
  resetSwayAccumulator();
  swayFrozen = false;
  $('ss-sway').textContent = '–';
  sessionStart = Date.now();
  recording = true;
  $('btn-record').disabled = true;
  $('btn-stop').disabled   = false;
  setStatus('recording');
  $('ss-n').textContent    = '0';
  $('ss-dur').textContent  = '–';
  $('ss-peak').textContent = '–';
  $('ss-avg').textContent  = '–';
  $('ss-path').textContent = '–';
  $('ss-vel').textContent  = '–';
  timerHandle = setInterval(() => {
    const s = Math.floor((Date.now() - sessionStart) / 1000);
    $('ss-dur').textContent = `${Math.floor(s / 60)}:${String(s % 60).padStart(2, '0')}`;
  }, 500);
  log('Session started', 'ok');
}

function stopRecording() {
  if (!recording) return;
  recording = false;
  clearInterval(timerHandle);
  $('btn-record').disabled = false;
  $('btn-stop').disabled   = true;
  if (connected) setStatus('connected');
  // Freeze the sway area on whatever the final value was. The number and
  // ellipse remain visible until the user starts a new session or hits Zero.
  swayFrozen = true;
  if (!csvRows.length) { log('No data to save', 'warn'); return; }
  const hdr = 'timestamp,elapsed_s,device_ms,roll_deg,pitch_deg,yaw_deg\n';
  const blob = new Blob([hdr + csvRows.join('\n')], { type: 'text/csv' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = 'imu_' + new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19) + '.csv';
  a.click();
  log('Saved ' + csvRows.length + ' samples → ' + a.download, 'ok');
}

// ── Replay ─────────────────────────────────────────────────────────────────
function parseCsvText(text) {
  const lines = text.split(/\r?\n/).filter(l => l.trim().length);
  if (!lines.length) return [];
  // Detect header
  const first = lines[0].toLowerCase();
  const startIdx = (first.includes('timestamp') || first.includes('roll')) ? 1 : 0;
  const out = [];
  for (let i = startIdx; i < lines.length; i++) {
    const cols = lines[i].split(',');
    if (cols.length < 6) continue;
    const elapsed = parseFloat(cols[1]);
    const ms      = parseFloat(cols[2]);
    const roll    = parseFloat(cols[3]);
    const pitch   = parseFloat(cols[4]);
    const yaw     = parseFloat(cols[5]);
    if ([elapsed, roll, pitch, yaw].some(isNaN)) continue;
    out.push({ elapsed, device_ms: isNaN(ms) ? 0 : ms, roll, pitch, yaw });
  }
  return out;
}

function loadReplayFromFile(file) {
  const reader = new FileReader();
  reader.onload = () => {
    const frames = parseCsvText(String(reader.result || ''));
    if (!frames.length) { log('CSV had no valid samples', 'err'); return; }
    replayFrames = frames;
    replayIdx = 0;
    replayName = file.name;
    replayPlaying = false;
    clearInterval(replayTimer); replayTimer = null;
    $('replay-bar').style.display = '';
    $('replay-name').textContent = file.name;
    $('btn-replay-play').textContent = '▶';
    $('replay-seek').value = 0;
    updateReplayTimeLabel();
    // Reset visuals and session stats so the replay starts clean
    clearVisuals();
    resetSessionStats();
    // Show first frame so user sees something immediately
    const f = replayFrames[0];
    updateLive(f.device_ms, f.roll, f.pitch, f.yaw);
    log('Loaded ' + frames.length + ' samples from ' + file.name, 'ok');
  };
  reader.onerror = () => log('Failed to read CSV file', 'err');
  reader.readAsText(file);
}

function resetSessionStats() {
  sampleCount = 0; peakTilt = 0; sumTilt = 0;
  swayPath = 0; prevRecRoll = null; prevRecPitch = null;
  $('ss-n').textContent    = '0';
  $('ss-dur').textContent  = '–';
  $('ss-peak').textContent = '–';
  $('ss-avg').textContent  = '–';
  $('ss-path').textContent = '–';
  $('ss-vel').textContent  = '–';
}

function updateReplayTimeLabel() {
  const total = replayFrames.length ? replayFrames[replayFrames.length - 1].elapsed : 0;
  const cur   = replayFrames.length ? replayFrames[Math.min(replayIdx, replayFrames.length - 1)].elapsed : 0;
  $('replay-time').textContent = cur.toFixed(1) + ' / ' + total.toFixed(1) + 's';
  if (total > 0) {
    $('replay-seek').value = String(Math.round((cur / total) * 1000));
  }
}

function replayStep() {
  if (!replayPlaying || !replayFrames.length) return;
  if (replayIdx >= replayFrames.length) { replayPause(); return; }
  const f = replayFrames[replayIdx];
  // Fake a "live" frame for all downstream visuals; pass recording=false implicitly
  updateLive(f.device_ms, f.roll, f.pitch, f.yaw);
  // Update synthetic session stats from replay frames directly
  const tilt = Math.sqrt(f.roll * f.roll + f.pitch * f.pitch);
  sampleCount++;
  sumTilt += tilt;
  if (tilt > peakTilt) { peakTilt = tilt; $('ss-peak').textContent = tilt.toFixed(2); }
  if (prevRecRoll !== null) {
    const dr = f.roll - prevRecRoll, dp = f.pitch - prevRecPitch;
    swayPath += Math.sqrt(dr * dr + dp * dp);
  }
  prevRecRoll = f.roll; prevRecPitch = f.pitch;
  $('ss-n').textContent    = String(sampleCount);
  $('ss-avg').textContent  = (sumTilt / sampleCount).toFixed(2);
  $('ss-path').textContent = swayPath.toFixed(2);
  const dur = f.elapsed;
  if (dur > 0) $('ss-vel').textContent = (swayPath / dur).toFixed(2);
  const s = Math.floor(dur);
  $('ss-dur').textContent = `${Math.floor(s / 60)}:${String(s % 60).padStart(2, '0')}`;

  replayIdx++;
  updateReplayTimeLabel();

  if (replayIdx >= replayFrames.length) { replayPause(); return; }

  // Schedule next frame based on CSV elapsed spacing, scaled by speed
  const cur  = replayFrames[replayIdx - 1].elapsed;
  const next = replayFrames[replayIdx].elapsed;
  let dt = Math.max(0, (next - cur) * 1000) / replaySpeed;
  if (!isFinite(dt) || dt > 2000) dt = 40;  // cap for huge gaps
  replayTimer = setTimeout(replayStep, dt);
}

function replayPlay() {
  if (!replayFrames.length) return;
  if (recording) { log('Stop recording before replay', 'warn'); return; }
  if (replayIdx >= replayFrames.length) {
    // Restart from beginning
    replayIdx = 0;
    clearVisuals();
    resetSessionStats();
  }
  replayPlaying = true;
  $('btn-replay-play').textContent = '⏸';
  replayStep();
}

function replayPause() {
  replayPlaying = false;
  clearTimeout(replayTimer); replayTimer = null;
  $('btn-replay-play').textContent = '▶';
}

function replayStop() {
  replayPause();
  replayFrames = [];
  replayIdx = 0;
  replayName = '';
  $('replay-bar').style.display = 'none';
  clearVisuals();
  resetSessionStats();
  log('Replay closed');
}

function replaySeek(fraction) {
  if (!replayFrames.length) return;
  const total = replayFrames[replayFrames.length - 1].elapsed;
  const target = total * fraction;
  // Find nearest frame index by elapsed
  let lo = 0, hi = replayFrames.length - 1;
  while (lo < hi) {
    const mid = (lo + hi) >> 1;
    if (replayFrames[mid].elapsed < target) lo = mid + 1; else hi = mid;
  }
  const wasPlaying = replayPlaying;
  replayPause();
  // Rebuild stats up to this index so metrics stay consistent with replay position
  replayIdx = 0;
  clearVisuals();
  resetSessionStats();
  for (let i = 0; i <= lo && i < replayFrames.length; i++) {
    const f = replayFrames[i];
    const tilt = Math.sqrt(f.roll * f.roll + f.pitch * f.pitch);
    sampleCount++;
    sumTilt += tilt;
    if (tilt > peakTilt) peakTilt = tilt;
    if (prevRecRoll !== null) {
      const dr = f.roll - prevRecRoll, dp = f.pitch - prevRecPitch;
      swayPath += Math.sqrt(dr * dr + dp * dp);
    }
    prevRecRoll = f.roll; prevRecPitch = f.pitch;
    updateLive(f.device_ms, f.roll, f.pitch, f.yaw);
  }
  replayIdx = lo + 1;
  $('ss-n').textContent    = String(sampleCount);
  $('ss-peak').textContent = peakTilt ? peakTilt.toFixed(2) : '–';
  $('ss-avg').textContent  = sampleCount ? (sumTilt / sampleCount).toFixed(2) : '–';
  $('ss-path').textContent = swayPath.toFixed(2);
  const dur = replayFrames[lo].elapsed;
  if (dur > 0) $('ss-vel').textContent = (swayPath / dur).toFixed(2);
  const s = Math.floor(dur);
  $('ss-dur').textContent = `${Math.floor(s / 60)}:${String(s % 60).padStart(2, '0')}`;
  updateReplayTimeLabel();
  if (wasPlaying) replayPlay();
}

function log(msg, cls = '') {
  const el = $('log');
  const d = document.createElement('div');
  d.className = 'log-line' + (cls ? ' log-' + cls : '');
  d.textContent = new Date().toLocaleTimeString('en', { hour12: false }) + '  ' + msg;
  el.appendChild(d); el.scrollTop = el.scrollHeight;
}

// ── Event wiring ───────────────────────────────────────────────────────────
$('btn-connect').addEventListener('click', connect);
$('btn-disconnect').addEventListener('click', disconnect);
$('btn-record').addEventListener('click', startRecording);
$('btn-stop').addEventListener('click', stopRecording);
$('btn-zero').addEventListener('click', () => {
  // Send the firmware-side zero command (resets orientation reference on
  // the ESP32) and reset every dashboard-side derived value so the user
  // gets a true clean slate.
  sendCmd('ZERO');
  // Stop any in-progress session — Zero is destructive and shouldn't run
  // alongside a recording. The CSV up to this point is discarded.
  if (recording) {
    recording = false;
    clearInterval(timerHandle);
    csvRows = [];
    $('btn-record').disabled = false;
    $('btn-stop').disabled   = true;
    if (connected) setStatus('connected');
  }
  // Clear the visual trail + tilt-plot + chart + jitter buffer + sway
  // accumulator (also unfreezes sway so live samples resume contributing).
  clearVisuals();
  // Reset every session-stat tile back to "—".
  resetSessionStats();
  // Blank out the instantaneous metric tiles until the next frame arrives.
  $('v-roll').textContent  = '–';
  $('v-pitch').textContent = '–';
  $('v-yaw').textContent   = '–';
  $('v-tilt').textContent  = '–';
  log('Zeroed — all metrics and visuals cleared');
});

$('host-input').addEventListener('keydown', (e) => {
  if (e.key === 'Enter') connect();
});

$('max-angle-slider').addEventListener('input', () => {
  maxAngle = Number($('max-angle-slider').value);
  $('max-angle-label').textContent = `±${maxAngle}°`;
  drawTiltPlot(lastRoll, lastPitch);
});

// Replay wiring
$('btn-load-replay').addEventListener('click', () => $('replay-file').click());
$('replay-file').addEventListener('change', (e) => {
  const f = e.target.files && e.target.files[0];
  if (f) loadReplayFromFile(f);
  e.target.value = '';  // allow re-loading same file
});
$('btn-replay-play').addEventListener('click', () => {
  if (replayPlaying) replayPause(); else replayPlay();
});
$('btn-replay-stop').addEventListener('click', replayStop);
$('replay-seek').addEventListener('input', (e) => {
  const frac = Number(e.target.value) / 1000;
  replaySeek(frac);
});
$('replay-speed').addEventListener('change', (e) => {
  replaySpeed = Number(e.target.value) || 1;
});

// Auto-connect when the page is served directly from the ESP32 over HTTP.
// The hostname reflects whichever board served this page (imuboard350.local,
// etc.) so the dashboard automatically scopes itself to that device.
if (window.location.protocol === 'http:') {
  $('host-input').value = window.location.hostname;
  $('about-host').textContent = window.location.hostname;
  document.title = 'IMU Balance Board — ' + window.location.hostname;
  connect();
}
</script>
</body>
</html>
)rawliteral";

bool WifiManager::begin() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(true);
    delay(100);

    _wifiOk = _tryConnect();
    if (!_wifiOk) {
        Serial.println("[WiFi] All networks failed — serial-only mode");
        return false;
    }

    _startServer();

    // Spawn the dedicated TX task. Pinned to core 1 (where the Arduino loop
    // lives), priority 5 — above Arduino loop's 1, well below WiFi driver's
    // ~23. Goal: when the Arduino loop task is briefly preempted, this
    // task still gets CPU and keeps the broadcast flowing.
    _wifiTxRun = true;
    BaseType_t ok = xTaskCreatePinnedToCore(
        _txTaskTrampoline, "wifi_tx", 6144, this,
        5,                       // priority
        &_wifiTxHandle,
        1                        // core 1
    );
    if (ok != pdPASS) {
        Serial.println("[WiFi] FATAL: wifi_tx task creation failed");
        _wifiTxRun = false;
    } else {
        Serial.println("[WiFi] wifi_tx task on core 1 at priority 5");
    }
    return true;
}

static const char* wlStatusStr(wl_status_t s) {
    switch (s) {
        case WL_IDLE_STATUS:      return "IDLE";
        case WL_NO_SSID_AVAIL:    return "NO_SSID_AVAIL";
        case WL_SCAN_COMPLETED:   return "SCAN_COMPLETED";
        case WL_CONNECTED:        return "CONNECTED";
        case WL_CONNECT_FAILED:   return "CONNECT_FAILED (auth/assoc)";
        case WL_CONNECTION_LOST:  return "CONNECTION_LOST";
        case WL_DISCONNECTED:     return "DISCONNECTED";
        default:                  return "UNKNOWN";
    }
}

bool WifiManager::_tryConnect() {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(WIFI_PS_NONE);  // hard-disable WiFi modem sleep (stronger than setSleep(false))
    // Max TX power. On marginal links this is the difference between sustained
    // throughput and TCP retransmit storms that stall the dashboard.
    WiFi.setTxPower(WIFI_POWER_19_5dBm);
    // Disable AP roaming / re-scan behaviour — we're a stationary node and
    // background scans can hold both cores for hundreds of ms. esp_wifi_set_*
    // is the ESP-IDF underneath the Arduino WiFi class.
    esp_wifi_set_ps(WIFI_PS_NONE);

    for (size_t i = 0; i < NETWORK_COUNT; i++) {
        Serial.printf("[WiFi] Trying: %s\n", NETWORKS[i].ssid);
        WiFi.begin(NETWORKS[i].ssid, NETWORKS[i].pass);

        uint32_t t = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - t < CONNECT_TIMEOUT_MS) {
            delay(250);
            Serial.print('.');
        }
        Serial.println();

        if (WiFi.status() == WL_CONNECTED) {
            Serial.printf("[WiFi] Connected: %s  IP=%s  RSSI=%d dBm  ch=%d\n",
                          NETWORKS[i].ssid,
                          WiFi.localIP().toString().c_str(),
                          WiFi.RSSI(),
                          WiFi.channel());
            return true;
        }

        Serial.printf("[WiFi] Failed: %s — status=%s\n",
                      NETWORKS[i].ssid, wlStatusStr(WiFi.status()));
        WiFi.disconnect(true, true);
        delay(500);
    }
    return false;
}

void WifiManager::_startServer() {
#if MDNS_ENABLED
    // Native IDF mDNS: runs in its own task so query bursts can never block
    // the sensor task (core 0) or the WS broadcast (core 1). Task priority
    // is set low via CONFIG_MDNS_TASK_PRIORITY in platformio.ini build flags.
    esp_err_t err = mdns_init();
    if (err != ESP_OK) {
        Serial.printf("[WiFi] mDNS init failed: %d\n", (int)err);
    } else {
        mdns_hostname_set(WIFI_HOSTNAME);
        mdns_instance_name_set("IMU Balance Board");
        mdns_service_add(NULL, "_http", "_tcp", WIFI_HTTP_PORT, NULL, 0);
        mdns_service_add(NULL, "_ws",   "_tcp", WIFI_WS_PORT,   NULL, 0);
        Serial.printf("[WiFi] mDNS: %s.local (native, isolated task)\n", WIFI_HOSTNAME);
    }
#else
    Serial.println("[WiFi] mDNS DISABLED (set MDNS_ENABLED 1 in config.h to re-enable)");
#endif

    // Always print the IP + MAC banner. The MAC is the stable hardware ID you
    // give your router when setting up a DHCP reservation for this board.
    Serial.println("========================================");
    Serial.printf("  Hostname: %s\n", WIFI_HOSTNAME);
    Serial.printf("  MAC:      %s   (use this in router's DHCP reservation)\n",
                  WiFi.macAddress().c_str());
    Serial.printf("  IP:       %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("  Open the dashboard at:\n");
#if MDNS_ENABLED
    Serial.printf("    http://%s.local/   (mDNS)\n", WIFI_HOSTNAME);
#endif
    Serial.printf("    http://%s/   (direct IP)\n",
                  WiFi.localIP().toString().c_str());
    Serial.println("========================================");

    // Synchronous HTTP server (no AsyncTCP). The dashboard HTML is the only
    // resource served, and only once per page-load. _pollHttp() runs from
    // poll() on the Arduino loop task — no extra task, no contention with
    // the sensor task or the WebSocket broadcast.
    _http = new WiFiServer(WIFI_HTTP_PORT);
    _http->begin();
    _http->setNoDelay(true);
    Serial.printf("[WiFi] HTTP server on :%u (synchronous)\n", WIFI_HTTP_PORT);

    _ws = new WebSocketsServer(WIFI_WS_PORT);
    _ws->begin();
    // Heartbeat disabled: the WebSockets library's ping/pong path can stall
    // the loop for 100-200 ms on missed pongs. Browsers send their own
    // TCP-level keepalives so we don't lose much by removing it.
    // _ws->enableHeartbeat(15000, 3000, 2);
    _ws->onEvent([this](uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
        this->_onEvent(num, type, payload, length);
    });
    Serial.printf("[WiFi] WS server on :%u\n", WIFI_WS_PORT);
}

void WifiManager::poll() {
    if (!_wifiOk) return;

    // Reconnect if WiFi dropped
    if (WiFi.status() != WL_CONNECTED) {
        uint32_t now = millis();
        if (now - _lastReconnectMs >= RECONNECT_INTERVAL_MS) {
            _lastReconnectMs = now;
            Serial.println("[WiFi] Connection lost — attempting reconnect");
            _clientCount = 0;
            WiFi.disconnect(true);
            delay(100);
            if (_tryConnect()) {
                Serial.println("[WiFi] Reconnected");
            }
        }
        return;
    }

    // NOTE: _ws->loop() and _pollHttp() are no longer called from here.
    // They run on the dedicated wifi_tx task at priority 5 on core 1, so
    // a momentary preemption of the Arduino loop task no longer stalls
    // the broadcast. See _txTaskLoop() below.

    uint32_t now = millis();

    // Periodically push a fresh STATUS to connected clients so the dashboard
    // info pills (RSSI / heap) stay current. Cheap: ~200-byte text frame at 0.5 Hz.
    if (_clientCount > 0 && now - _lastStatusMs >= 2000) {
        _lastStatusMs = now;
        sendStatus();
    }

    // Once-per-second [STATS] diagnostic line on the serial monitor.
    //   tx       — broadcast frames sent in the last second (expected ~50)
    //   samples  — IMU samples published by the sensor task (expected ~190-200)
    //   maxGap   — worst inter-send gap (expected ~21 ms; spikes = stalls)
    //   heap/RSSI — basic health
    if (now - _lastStatsMs >= 1000) {
        extern volatile uint32_t g_sample_count;
        static uint32_t last_samples = 0;
        uint32_t samples_now   = g_sample_count;
        uint32_t samples_delta = samples_now - last_samples;
        last_samples = samples_now;

        Serial.printf("[STATS] tx=%lu/s samples=%lu/s maxGap=%lums heap=%luKB rssi=%d clients=%d\n",
                      (unsigned long)_txFrames,
                      (unsigned long)samples_delta,
                      (unsigned long)_maxGapMs,
                      (unsigned long)(ESP.getFreeHeap() / 1024),
                      WiFi.RSSI(),
                      (int)_clientCount);

        _txFrames    = 0;
        _maxGapMs    = 0;
        _lastStatsMs = now;
    }
}

// Producer side. Called from the Arduino loop task at 50 Hz.
// Just publishes to a one-slot snapshot via seqlock — never calls into the
// WS library, so the producer task cannot be blocked by network I/O.
// The actual broadcast happens on the dedicated wifi_tx task (see below).
void WifiManager::sendFrame(uint32_t ms, float roll, float pitch, float yaw) {
    if (!_wifiOk || !_ws) return;

    uint32_t seq = _txSnapSeq.load(std::memory_order_relaxed);
    _txSnapSeq.store(seq + 1, std::memory_order_release);  // odd: writing
    _txSnap.ms    = ms;
    _txSnap.roll  = roll;
    _txSnap.pitch = pitch;
    _txSnap.yaw   = yaw;
    _txSnapSeq.store(seq + 2, std::memory_order_release);  // even: stable
    _txSnapPubCount.fetch_add(1, std::memory_order_release);
}

// Actual broadcast — runs only on the wifi_tx task.
void WifiManager::_broadcastSnapshot(const TxSnapshot& s) {
    if (_clientCount <= 0) return;

    // Track inter-send gap for the [STATS] line (measured from the perspective
    // of frames actually leaving the device, not frames produced).
    uint32_t nowMs = millis();
    if (_lastTxMs != 0) {
        uint32_t gap = nowMs - _lastTxMs;
        if (gap > _maxGapMs) _maxGapMs = gap;
    }
    _lastTxMs = nowMs;

    // Tally the connected clients' mode preferences.
    uint8_t n_bin = 0, n_txt = 0;
    for (uint8_t i = 0; i < MAX_WS_CLIENTS; i++) {
        if (!_ws->clientIsConnected(i)) continue;
        if (_binMode[i]) n_bin++;
        else             n_txt++;
    }
    if (n_bin == 0 && n_txt == 0) return;

    // Binary frame: 16 bytes little-endian.
    uint8_t bin[16];
    memcpy(bin + 0,  &s.ms,    4);
    memcpy(bin + 4,  &s.roll,  4);
    memcpy(bin + 8,  &s.pitch, 4);
    memcpy(bin + 12, &s.yaw,   4);

    // Text frame (legacy / default): CSV. Built only if needed.
    char txt[64];
    int  txt_len = 0;
    if (n_txt > 0) {
        txt_len = snprintf(txt, sizeof(txt), "%lu,%.2f,%.2f,%.2f",
                           (unsigned long)s.ms, s.roll, s.pitch, s.yaw);
    }

    if (n_bin > 0 && n_txt == 0) {
        _ws->broadcastBIN(bin, sizeof(bin));
    } else if (n_txt > 0 && n_bin == 0) {
        if (txt_len > 0) _ws->broadcastTXT((uint8_t*)txt, (size_t)txt_len);
    } else {
        for (uint8_t i = 0; i < MAX_WS_CLIENTS; i++) {
            if (!_ws->clientIsConnected(i)) continue;
            if (_binMode[i]) {
                _ws->sendBIN(i, bin, sizeof(bin));
            } else if (txt_len > 0) {
                _ws->sendTXT(i, (uint8_t*)txt, (size_t)txt_len);
            }
        }
    }
    _txFrames++;
}

// Trampoline so we can call a member function from xTaskCreatePinnedToCore.
void WifiManager::_txTaskTrampoline(void* arg) {
    static_cast<WifiManager*>(arg)->_txTaskLoop();
}

// The dedicated TX task. Runs on core 1 at priority 5 — higher than the
// Arduino loop task (prio 1), well below the WiFi driver task (prio 23).
// This means brief preemption of the Arduino loop doesn't starve our
// broadcast: this task still gets scheduled and keeps frames flowing.
void WifiManager::_txTaskLoop() {
    while (_wifiTxRun) {
        // Service incoming WS housekeeping (handshake, ping/pong, etc.)
        if (_ws) _ws->loop();
        // Handle any pending HTTP request for the dashboard HTML.
        _pollHttp();

        // Pop the latest published snapshot if we haven't sent it yet.
        // Using a publish-counter ensures we don't re-send the same frame
        // and also that we don't drop frames silently (the producer always
        // overwrites the previous snapshot; we just take whatever is freshest).
        uint32_t pub = _txSnapPubCount.load(std::memory_order_acquire);
        if (pub != _txSnapLastSent) {
            // Consistent read via seqlock.
            TxSnapshot local{};
            for (int tries = 0; tries < 4; tries++) {
                uint32_t s1 = _txSnapSeq.load(std::memory_order_acquire);
                if (s1 & 1) continue;
                local = _txSnap;
                uint32_t s2 = _txSnapSeq.load(std::memory_order_acquire);
                if (s1 == s2) { _txSnapLastSent = pub; break; }
            }
            if (_txSnapLastSent == pub) {
                _broadcastSnapshot(local);
            }
        }

        // Yield. 1 tick = 1 ms — fast enough to keep _ws->loop responsive
        // (which the library expects to be called often).
        vTaskDelay(1);
    }
    _wifiTxHandle = nullptr;
    vTaskDelete(nullptr);
}

void WifiManager::_pollHttp() {
    if (!_http) return;
    WiFiClient client = _http->available();
    if (!client) return;

    // Drain the request line + headers. We don't actually care what the
    // request is — there's only one resource. Bail after ~1 s if peer is slow.
    uint32_t deadline = millis() + 1000;
    bool blank = false;
    String line;
    while (client.connected() && millis() < deadline) {
        if (!client.available()) { delay(1); continue; }
        char c = client.read();
        if (c == '\n') {
            if (blank) break;       // end of headers
            blank = true;
            line = "";
        } else if (c != '\r') {
            blank = false;
            if (line.length() < 256) line += c;
        }
    }

    // Response: HTTP/1.1 200 + Content-Length so the browser closes cleanly.
    // DASHBOARD_HTML is a PROGMEM string literal; sizeof - 1 excludes the
    // trailing NUL.
    const size_t html_len = sizeof(DASHBOARD_HTML) - 1;
    char hdr[160];
    int hdr_len = snprintf(hdr, sizeof(hdr),
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/html; charset=utf-8\r\n"
        "Content-Length: %u\r\n"
        "Connection: close\r\n"
        "Cache-Control: no-store\r\n"
        "\r\n",
        (unsigned)html_len);
    client.write((const uint8_t*)hdr, (size_t)hdr_len);

    // Stream the body in 1 KB chunks so we don't allocate a huge intermediate.
    // PROGMEM access is byte-readable on ESP32 (no need for pgm_read_byte_far).
    constexpr size_t CHUNK = 1024;
    size_t sent = 0;
    while (sent < html_len && client.connected()) {
        size_t n = (html_len - sent) < CHUNK ? (html_len - sent) : CHUNK;
        client.write((const uint8_t*)(DASHBOARD_HTML + sent), n);
        sent += n;
    }
    client.flush();
    client.stop();
}

void WifiManager::sendStatus(int8_t targetClient) {
    if (!_wifiOk || !_ws) return;
    char buf[224];
    snprintf(buf, sizeof(buf),
        "[STATUS] {\"streaming\":%s,\"ip\":\"%s\",\"port\":%u,\"heap\":%lu,"
        "\"version\":\"%s\",\"rssi\":%d,\"ch\":%d,\"ssid\":\"%s\"}",
        (_streaming && *_streaming) ? "true" : "false",
        WiFi.localIP().toString().c_str(),
        WIFI_WS_PORT,
        (unsigned long)ESP.getFreeHeap(),
        FIRMWARE_VERSION,
        WiFi.RSSI(),
        WiFi.channel(),
        WiFi.SSID().c_str());

    size_t len = strlen(buf);
    if (targetClient >= 0) {
        _ws->sendTXT((uint8_t)targetClient, (uint8_t*)buf, len);
    } else {
        _ws->broadcastTXT((uint8_t*)buf, len);
    }
}

void WifiManager::_onEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
    switch (type) {
        case WStype_CONNECTED: {
            IPAddress ip = _ws->remoteIP(num);
            // Single-client policy. If one is already connected, refuse the new
            // arrival so we don't double our broadcast load. The replaced
            // client gets an INFO line + immediate disconnect; the existing
            // session is preserved.
            if (_clientCount > 0) {
                Serial.printf("[WiFi WS] Client #%u from %s REJECTED (slot busy)\n",
                              num, ip.toString().c_str());
                _ws->sendTXT(num, "[INFO] Another client is already connected — disconnecting.");
                _ws->disconnect(num);
                break;
            }
            Serial.printf("[WiFi WS] Client #%u connected from %s\n", num, ip.toString().c_str());
            _clientCount++;
            if (num < MAX_WS_CLIENTS) _binMode[num] = false;
            sendStatus((int8_t)num);
            break;
        }
        case WStype_DISCONNECTED:
            Serial.printf("[WiFi WS] Client #%u disconnected\n", num);
            _clientCount--;
            if (_clientCount < 0) _clientCount = 0;
            if (num < MAX_WS_CLIENTS) _binMode[num] = false;
            break;

        case WStype_TEXT: {
            if (!payload || length == 0) break;
            char cmd[32] = {};
            size_t n = length < sizeof(cmd) - 1 ? length : sizeof(cmd) - 1;
            memcpy(cmd, payload, n);

            if (strcmp(cmd, "START") == 0) {
                if (_streaming) *_streaming = true;
                _ws->sendTXT(num, "[INFO] Streaming started");
                Serial.println("[WiFi WS] START");
            } else if (strcmp(cmd, "STOP") == 0) {
                if (_streaming) *_streaming = false;
                _ws->sendTXT(num, "[INFO] Streaming stopped");
                Serial.println("[WiFi WS] STOP");
            } else if (strcmp(cmd, "STATUS") == 0) {
                sendStatus((int8_t)num);
            } else if (strcmp(cmd, "HELP") == 0) {
                _ws->sendTXT(num, "[INFO] Commands: START STOP STATUS HELP ZERO RATE <hz> DEBUG ON|OFF BIN ON|OFF");
            } else if (strcmp(cmd, "BIN ON") == 0) {
                if (num < MAX_WS_CLIENTS) _binMode[num] = true;
                _ws->sendTXT(num, "[INFO] Binary frame mode ON");
                Serial.printf("[WiFi WS] Client #%u → binary mode\n", num);
            } else if (strcmp(cmd, "BIN OFF") == 0) {
                if (num < MAX_WS_CLIENTS) _binMode[num] = false;
                _ws->sendTXT(num, "[INFO] Binary frame mode OFF");
                Serial.printf("[WiFi WS] Client #%u → text mode\n", num);
            } else if (strcmp(cmd, "ZERO") == 0) {
                zeroOrientation();
                _ws->sendTXT(num, "[INFO] Orientation zeroed");
            } else if (strcmp(cmd, "DEBUG ON") == 0) {
                setDriftLog(true);
                _ws->sendTXT(num, "[INFO] Drift log ON");
            } else if (strcmp(cmd, "DEBUG OFF") == 0) {
                setDriftLog(false);
                _ws->sendTXT(num, "[INFO] Drift log OFF");
            } else if (strncmp(cmd, "RATE ", 5) == 0) {
                int hz = atoi(cmd + 5);
                if (hz < 1)  hz = 1;
                if (hz > 50) hz = 50;
                if (_serial) {
                    _serial->setPrintIntervalMs(1000 / (uint32_t)hz);
                }
                char reply[48];
                snprintf(reply, sizeof(reply), "[INFO] Rate set to %d Hz", hz);
                _ws->sendTXT(num, reply);
                Serial.printf("[WiFi WS] RATE %d Hz\n", hz);
            }
            break;
        }
        default:
            break;
    }
}
