#include "wifi_manager.h"
#include "wifi_config.h"
#include "serial_command.h"
#include "config.h"
#include <ESPAsyncWebServer.h>
#include <functional>
#include <cstdlib>

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

.esp-info { font-size: 11px; color: var(--text3); font-family: var(--mono); padding: 4px 0 10px; min-height: 18px; }

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
  <span class="pill" id="hz-pill" style="display:none"><span id="hz-val">–</span> Hz</span>
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

  <!-- Command row -->
  <div class="controls" id="cmd-row" style="display:none">
    <button onclick="sendCmd('START')"    class="btn-green">START</button>
    <button onclick="sendCmd('STOP')"     class="btn-red">STOP</button>
    <button onclick="sendCmd('STATUS')">STATUS</button>
    <button onclick="sendCmd('HELP')">HELP</button>
    <input  type="text" id="cmd-input" placeholder="RATE 25" style="width:110px">
    <button id="btn-send-cmd">Send</button>
  </div>

  <!-- ESP32 status info line -->
  <div class="esp-info" id="esp-info"></div>

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
      Host: <span style="color:var(--text)">imuboard.local</span> &nbsp; HTTP: <span style="color:var(--text)">80</span> &nbsp; WS: <span style="color:var(--text)">81</span><br>
      Frame: <span style="color:var(--text)">&lt;ms&gt;,&lt;roll&gt;,&lt;pitch&gt;,&lt;yaw&gt;</span><br>
      Status: <span style="color:var(--text)">[STATUS] {json}</span><br>
      Commands: <span style="color:var(--text)">START · STOP · STATUS · HELP · ZERO</span>
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
      ESP32 DevKit V1 + Adafruit BNO085 (I²C).<br>
      Sensor fusion: Game Rotation Vector at 50 Hz (on-chip SH2 firmware).<br>
      Output rate: 10 Hz default. Change with serial command: <span style="font-family:var(--mono);color:var(--text)">RATE 40</span>
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

// Interpolation: two most-recent received frames (wall-clock time at receipt)
let prevFrame = null;  // { roll, pitch, wallT }
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

function pushChartPoint(elapsed, roll, pitch, yaw) {
  chartData.labels.push(elapsed.toFixed(1) + 's');
  chartData.roll.push(roll);
  chartData.pitch.push(pitch);
  chartData.yaw.push(yaw);
  if (chartData.labels.length > MAX_CHART) {
    chartData.labels.shift();
    chartData.roll.shift(); chartData.pitch.shift(); chartData.yaw.shift();
  }
  if (chart) chart.update('none');
}

// ── Tilt plot ──────────────────────────────────────────────────────────────
function clampToCircle(r, p, max) {
  const mag = Math.sqrt(r * r + p * p);
  if (mag > max) { const s = max / mag; return [r * s, p * s]; }
  return [r, p];
}

function computeSwayEllipse() {
  const n = trailFull ? TRAIL_LEN : trailHead;
  if (n < 5) return null;
  let sx = 0, sy = 0;
  for (let i = 0; i < n; i++) {
    const idx = trailFull ? (trailHead + i) % TRAIL_LEN : i;
    sx += trailRoll[idx]; sy += trailPitch[idx];
  }
  const mx = sx / n, my = sy / n;
  let cxx = 0, cyy = 0, cxy = 0;
  for (let i = 0; i < n; i++) {
    const idx = trailFull ? (trailHead + i) % TRAIL_LEN : i;
    const dx = trailRoll[idx] - mx, dy = trailPitch[idx] - my;
    cxx += dx * dx; cyy += dy * dy; cxy += dx * dy;
  }
  cxx /= (n - 1); cyy /= (n - 1); cxy /= (n - 1);
  const trace = cxx + cyy;
  const det   = cxx * cyy - cxy * cxy;
  const disc  = Math.sqrt(Math.max(0, (trace / 2) ** 2 - det));
  const lam1  = trace / 2 + disc, lam2 = trace / 2 - disc;
  const k     = Math.sqrt(5.991);
  const semiA = Math.sqrt(Math.max(0, lam1)) * k;
  const semiB = Math.sqrt(Math.max(0, lam2)) * k;
  const angle = Math.atan2(2 * cxy, cxx - cyy) / 2;
  return { cx: mx, cy: my, semiA, semiB, angle, area: Math.PI * semiA * semiB };
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
    const early = (trailFull ? TRAIL_LEN : trailHead) < 20;
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
// Interpolates between the last two received frames so the dot moves smoothly
// at ~60 fps even when the ESP only sends at 25 Hz.
(function rafLoop() {
  requestAnimationFrame(rafLoop);
  if (!currFrame) return;
  let roll, pitch;
  if (prevFrame && currFrame.wallT > prevFrame.wallT) {
    const span  = currFrame.wallT - prevFrame.wallT;
    const alpha = Math.min(1, (performance.now() - prevFrame.wallT) / span);
    roll  = prevFrame.roll  + alpha * (currFrame.roll  - prevFrame.roll);
    pitch = prevFrame.pitch + alpha * (currFrame.pitch - prevFrame.pitch);
  } else {
    roll = currFrame.roll; pitch = currFrame.pitch;
  }
  lastRoll = roll; lastPitch = pitch;
  drawTiltPlot(roll, pitch);
}());

function clearVisuals() {
  trailHead = 0; trailFull = false;
  trailRoll.fill(0); trailPitch.fill(0);
  ['labels','roll','pitch','yaw'].forEach(k => { chartData[k].length = 0; });
  chartStart = 0;
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
  // Advance interpolation frames (rAF loop reads these)
  prevFrame = currFrame;
  currFrame = { roll, pitch, wallT: performance.now() };

  $('v-roll').textContent  = roll.toFixed(2);
  $('v-pitch').textContent = pitch.toFixed(2);
  $('v-yaw').textContent   = yaw.toFixed(2);
  const tilt = Math.sqrt(roll * roll + pitch * pitch);
  $('v-tilt').textContent  = tilt.toFixed(2);

  // Trail stores actual received samples
  trailRoll[trailHead]  = roll;
  trailPitch[trailHead] = pitch;
  trailHead = (trailHead + 1) % TRAIL_LEN;
  if (trailHead === 0) trailFull = true;

  // Sway area (always visible, not just during recording)
  const ellipse = computeSwayEllipse();
  $('ss-sway').textContent = ellipse ? ellipse.area.toFixed(2) : '–';

  // Chart uses wall-clock elapsed from first frame, not from record start
  if (!chartStart) chartStart = Date.now();
  const elapsed = (Date.now() - chartStart) / 1000;
  pushChartPoint(elapsed, roll, pitch, yaw);

  // Hz via wall-clock receive deltas
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

  ws.onopen = () => {
    connected = true;
    setStatus('connected');
    log('Connected to ' + url, 'ok');
    initChart();
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
    const line = ev.data.trim();
    if (!line) return;

    if (line.startsWith('[STATUS]')) {
      try {
        const s = JSON.parse(line.slice(8).trim());
        const parts = [];
        if (s.ip)      parts.push('IP ' + s.ip);
        if (s.port)    parts.push('port ' + s.port);
        if (s.version) parts.push('fw v' + s.version);
        if (s.heap)    parts.push('heap ' + Math.round(s.heap / 1024) + 'KB');
        $('esp-info').textContent = parts.join(' · ');
      } catch { /* ignore malformed status */ }
      return;
    }
    if (line.startsWith('[INFO]')) {
      log(line.slice(6).trim());
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
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(cmd);
    log('→ ' + cmd);
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
    $('cmd-row').style.display   = '';
  } else if (state === 'recording') {
    dot.classList.add('rec'); txt.textContent = 'Recording';
  } else {
    txt.textContent = 'Disconnected';
    $('btn-connect').disabled    = false;
    $('btn-disconnect').disabled = true;
    $('btn-record').disabled     = true;
    $('btn-stop').disabled       = true;
    $('btn-zero').disabled       = true;
    $('cmd-row').style.display   = 'none';
    $('v-roll').textContent = $('v-pitch').textContent = $('v-yaw').textContent = $('v-tilt').textContent = '–';
    $('hz-pill').style.display = 'none';
    $('esp-info').textContent = '';
    sampleTimes.length = 0;
  }
}

function startRecording() {
  if (recording) return;
  csvRows = []; sampleCount = 0; peakTilt = 0; sumTilt = 0;
  swayPath = 0; prevRecRoll = null; prevRecPitch = null;
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
  sendCmd('ZERO');
  clearVisuals();
  log('Visual history cleared');
});

$('host-input').addEventListener('keydown', (e) => {
  if (e.key === 'Enter') connect();
});

$('btn-send-cmd').addEventListener('click', () => {
  const val = $('cmd-input').value.trim().toUpperCase();
  if (val) { sendCmd(val); $('cmd-input').value = ''; }
});

$('cmd-input').addEventListener('keydown', (e) => {
  if (e.key === 'Enter') $('btn-send-cmd').click();
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

// Auto-connect when the page is served directly from the ESP32 over HTTP
if (window.location.protocol === 'http:') {
  $('host-input').value = window.location.hostname;
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
    return true;
}

bool WifiManager::_tryConnect() {
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
            Serial.printf("[WiFi] Connected: %s\n", WiFi.localIP().toString().c_str());
            return true;
        }

        WiFi.disconnect(true);
        delay(100);
    }
    return false;
}

void WifiManager::_startServer() {
    if (MDNS.begin("imuboard")) {
        MDNS.addService("http", "tcp", WIFI_HTTP_PORT);
        MDNS.addService("ws",   "tcp", WIFI_WS_PORT);
        Serial.println("[WiFi] mDNS: imuboard.local");
    }

    _http = new AsyncWebServer(WIFI_HTTP_PORT);
    _http->on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
        req->send_P(200, "text/html", DASHBOARD_HTML);
    });
    _http->onNotFound([](AsyncWebServerRequest* req) {
        req->send_P(200, "text/html", DASHBOARD_HTML);
    });
    _http->begin();
    Serial.printf("[WiFi] HTTP server on :%u\n", WIFI_HTTP_PORT);

    _ws = new WebSocketsServer(WIFI_WS_PORT);
    _ws->begin();
    _ws->enableHeartbeat(15000, 3000, 2);
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

    if (_ws) _ws->loop();
}

void WifiManager::sendFrame(uint32_t ms, float roll, float pitch, float yaw) {
    if (!_wifiOk || !_ws || _clientCount <= 0) return;
    char buf[64];
    snprintf(buf, sizeof(buf), "%lu,%.2f,%.2f,%.2f", (unsigned long)ms, roll, pitch, yaw);
    _ws->broadcastTXT(buf);
}

void WifiManager::sendStatus(int8_t targetClient) {
    if (!_wifiOk || !_ws) return;
    char buf[160];
    snprintf(buf, sizeof(buf),
        "[STATUS] {\"streaming\":%s,\"ip\":\"%s\",\"port\":%u,\"heap\":%lu,\"version\":\"%s\"}",
        (_streaming && *_streaming) ? "true" : "false",
        WiFi.localIP().toString().c_str(),
        WIFI_WS_PORT,
        (unsigned long)ESP.getFreeHeap(),
        FIRMWARE_VERSION);

    if (targetClient >= 0) {
        _ws->sendTXT((uint8_t)targetClient, buf);
    } else {
        _ws->broadcastTXT(buf);
    }
}

void WifiManager::_onEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
    switch (type) {
        case WStype_CONNECTED: {
            IPAddress ip = _ws->remoteIP(num);
            Serial.printf("[WiFi WS] Client #%u connected from %s\n", num, ip.toString().c_str());
            _clientCount++;
            sendStatus((int8_t)num);
            break;
        }
        case WStype_DISCONNECTED:
            Serial.printf("[WiFi WS] Client #%u disconnected\n", num);
            _clientCount--;
            if (_clientCount < 0) _clientCount = 0;
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
                _ws->sendTXT(num, "[INFO] Commands: START STOP STATUS HELP ZERO RATE <hz>");
            } else if (strcmp(cmd, "ZERO") == 0) {
                _ws->sendTXT(num, "[INFO] ZERO not yet implemented");
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
