/*
  ESP32-C3 + NextPM + Senseair S8  ->  AirGradient Cloud + Local Web Dashboard

  Pins:
    NextPM  UART1  RX=GPIO0  TX=GPIO1   115200 8E1
    S8      UART0  RX=GPIO20 TX=GPIO21  9600   8N1

  Local features:
    - http://<ip>/         web dashboard (auto-refreshes every 2 s)
    - http://<ip>/json     latest values + raw diagnostic bytes
    - http://<ip>/probe    dumps NextPM Modbus registers 0..20 and 128..147 (for debugging)
*/

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClient.h>
#include <WiFiManager.h>
#include <HardwareSerial.h>
#include <Preferences.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Wire.h>
#include <SensirionI2CSgp41.h>
#include <SensirionI2cSht4x.h>
#include <VOCGasIndexAlgorithm.h>
#include <NOxGasIndexAlgorithm.h>
#include <math.h>
#include <string.h>

// -------------------- Pins / UART --------------------
constexpr int NEXTPM_RX_PIN = 0;
constexpr int NEXTPM_TX_PIN = 1;
constexpr int S8_RX_PIN     = 20;
constexpr int S8_TX_PIN     = 21;

HardwareSerial NextPMSerial(1);
HardwareSerial S8Serial(0);

constexpr uint8_t  NEXTPM_MODBUS_ADDR = 0x01;
constexpr float    PM003_FROM_02_05_FRACTION = 0.0f;
constexpr uint32_t POST_PERIOD_MS = 30000;  // AirGradient free tier rate-limits below ~30 s

// I2C pins (AirGradient OpenAir C3 board)
constexpr int I2C_SDA = 7;
constexpr int I2C_SCL = 6;

// SGP41 conditioning duration after boot (datasheet: 10 s)
constexpr uint32_t SGP41_COND_MS = 10000;
// Gas sensors sampling cadence (1 Hz recommended by Sensirion gas index algorithm)
constexpr uint32_t GAS_SAMPLE_MS = 1000;

// -------------------- AirGradient ID -----------------
Preferences prefs;
String gSensorIdFull;

WebServer webServer(80);

// I2C sensors
static SensirionI2CSgp41 sgp41;
static SensirionI2cSht4x sht4x;
static VOCGasIndexAlgorithm vocAlgo;
static NOxGasIndexAlgorithm noxAlgo;
static uint32_t sgpBootMs = 0;
static bool i2cInitialized = false;

// ==================== Shared latest data ==============
struct NextPMSample {
  float pm1 = NAN, pm25 = NAN, pm10 = NAN;
  uint16_t cntPM1_dL = 0, cntPM25_dL = 0, cntPM10_dL = 0;
  uint8_t state = 0xFF;
  bool ok = false;
  String rawHex;
};

struct LatestData {
  // Selected period mirror (for backward-compat + AirGradient POST)
  float pm1 = NAN, pm25 = NAN, pm10 = NAN;
  uint8_t nextpmState = 0xFF;
  bool massOk = false;
  // Cumulative counts from simple protocol (cnt/dL, cut at each PM size) — from selected period
  uint16_t cntPM1_dL = 0, cntPM25_dL = 0, cntPM10_dL = 0;
  // All three NextPM averaging periods
  NextPMSample avg10s;   // cmd 0x11
  NextPMSample avg60s;   // cmd 0x12
  NextPMSample avg15m;   // cmd 0x13
  uint16_t postAvgSec = 60; // which period is the "primary" pushed to AG and mirrored above
  // Bins (Cnt/L) — Modbus (optional / placeholder for when we find the right regs)
  float c02_05 = 0, c05_10 = 0, c10_25 = 0, c25_50 = 0, c50_100 = 0;
  int pm003_dL = -1;
  bool binsOk = false;
  // CO2
  uint16_t co2 = 0;
  bool co2Ok = false;
  // T/RH (SHT4x)
  float atmp = NAN, rhum = NAN;
  bool shtOk = false, shtPresent = false;
  // Gas indexes (SGP41 + Sensirion gas index algorithm)
  int32_t vocIndex = 0, noxIndex = 0;
  uint16_t sgpSrawVoc = 0, sgpSrawNox = 0;
  bool sgpOk = false, sgpPresent = false;
  bool sgpConditioning = true;
  // Cloud POST
  int lastPostCode = 0;
  String lastPostResp;
  // Raw diagnostics (last cycle)
  String nextpmMassRaw;
  String nextpmBinsRaw;
  String s8TxRaw;
  String s8Raw;
  String probeLow;   // regs 0..20
  String probeHigh;  // regs 128..147
  uint32_t lastUpdateMs = 0;
};
static LatestData latest;

// ==================== Helpers =========================
static String bytesToHex(const uint8_t* b, size_t n) {
  String s; s.reserve(n * 3);
  char buf[4];
  for (size_t i = 0; i < n; i++) { snprintf(buf, sizeof(buf), "%02X ", b[i]); s += buf; }
  if (s.length()) s.remove(s.length() - 1);
  return s;
}

static String jsonEscape(const String& s) {
  String r; r.reserve(s.length() + 4);
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '"')      r += "\\\"";
    else if (c == '\\') r += "\\\\";
    else if (c == '\n') r += "\\n";
    else if (c == '\r') r += "\\r";
    else if ((uint8_t)c < 0x20) { char b[8]; snprintf(b, sizeof(b), "\\u%04x", c); r += b; }
    else r += c;
  }
  return r;
}

static String floatOrNull(float v, int dec = 1) {
  if (isnan(v)) return String("null");
  return String(v, dec);
}

String agSerial12() {
  if (WiFi.getMode() != WIFI_STA && WiFi.getMode() != WIFI_AP_STA) {
    WiFi.mode(WIFI_STA);
    delay(10);
  }
  String mac = WiFi.macAddress();
  mac.toLowerCase();
  mac.replace(":", "");
  return mac;
}

String normalizeSensorId(const String& userInput, const String& fallback12hex) {
  String s = userInput; s.trim(); s.toLowerCase();
  if (s.startsWith("airgradient:")) s.remove(0, 12);
  s.replace(":", ""); s.replace(" ", "");
  String hex;
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if ((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f')) hex += c;
  }
  if (hex.length() != 12) return String("airgradient:") + fallback12hex;
  return String("airgradient:") + hex;
}

String loadSavedSensorId() {
  prefs.begin("ag", true);
  String id = prefs.getString("id", "");
  prefs.end();
  return id;
}

void saveSensorId(const String& fullId) {
  prefs.begin("ag", false);
  prefs.putString("id", fullId);
  prefs.end();
}

// ==================== CRC / Checksum ==================
static uint16_t crc16_modbus(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) {
      if (crc & 1) crc = (crc >> 1) ^ 0xA001;
      else         crc >>= 1;
    }
  }
  return crc;
}

static inline uint8_t nextpmChecksum(const uint8_t* b, size_t len) {
  uint32_t s = 0;
  for (size_t i = 0; i < len; i++) s += b[i];
  return (uint8_t)((256 - (s % 256)) & 0xFF);
}

static bool nextpmSendSimpleCmd(uint8_t cmd) {
  uint8_t frame[3] = {0x81, cmd, 0x00};
  frame[2] = nextpmChecksum(frame, 2);
  size_t n = NextPMSerial.write(frame, sizeof(frame));
  NextPMSerial.flush();
  return n == sizeof(frame);
}

// ==================== NextPM: Simple protocol =========
// Simple-protocol mass commands. Response format (16 bytes) is identical across periods:
//   [0]=0x81 [1]=cmd [2]=state
//   [3..4]  cumul. count PM1  cut (>=~0.3 µm) in cnt/(0.1L) = cnt/dL
//   [5..6]  cumul. count PM2.5 cut
//   [7..8]  cumul. count PM10 cut
//   [9..10] mass PM1  (×0.1 µg/m³)
//   [11..12] mass PM2.5
//   [13..14] mass PM10
//   [15] checksum (sum % 256 == 0)
// cmd=0x11 -> 10 s avg, cmd=0x12 -> 60 s avg, cmd=0x13 -> 15 min avg
static bool nextpmReadMassCmd(uint8_t cmd, NextPMSample& s) {
  s = NextPMSample{};
  while (NextPMSerial.available()) NextPMSerial.read();
  if (!nextpmSendSimpleCmd(cmd)) { s.rawHex = "(send fail)"; return false; }

  const uint32_t t0 = millis();
  const uint32_t timeoutMs = 500;
  uint8_t buf[16]; size_t got = 0;
  while (got < sizeof(buf) && millis() - t0 < timeoutMs) {
    if (NextPMSerial.available()) buf[got++] = (uint8_t)NextPMSerial.read();
  }
  s.rawHex = bytesToHex(buf, got);
  if (got != sizeof(buf)) return false;
  if (buf[0] != 0x81 || buf[1] != cmd) return false;
  if (nextpmChecksum(buf, 15) != buf[15]) return false;

  s.state = buf[2];
  auto U16 = [](const uint8_t* p) { return (uint16_t)((p[0] << 8) | p[1]); };
  s.cntPM1_dL  = U16(&buf[3]);
  s.cntPM25_dL = U16(&buf[5]);
  s.cntPM10_dL = U16(&buf[7]);
  s.pm1  = U16(&buf[9])  / 10.0f;
  s.pm25 = U16(&buf[11]) / 10.0f;
  s.pm10 = U16(&buf[13]) / 10.0f;
  s.ok = true;
  return true;
}

// ==================== NextPM: Modbus ==================
// Returns the number of valid words read (>0) or 0 on failure. Always fills rawHex.
static size_t nextpmReadHolding(uint16_t regStart, uint16_t qty, uint16_t* outWords, String& rawHex) {
  while (NextPMSerial.available()) NextPMSerial.read();

  uint8_t req[8];
  req[0] = NEXTPM_MODBUS_ADDR;
  req[1] = 0x03;
  req[2] = (uint8_t)(regStart >> 8);
  req[3] = (uint8_t)(regStart & 0xFF);
  req[4] = (uint8_t)(qty >> 8);
  req[5] = (uint8_t)(qty & 0xFF);
  uint16_t crc = crc16_modbus(req, 6);
  req[6] = (uint8_t)(crc & 0xFF);
  req[7] = (uint8_t)((crc >> 8) & 0xFF);

  if (NextPMSerial.write(req, sizeof(req)) != sizeof(req)) { rawHex = "(send fail)"; return 0; }
  NextPMSerial.flush();

  const uint32_t t0 = millis();
  const uint32_t timeoutMs = 500;
  uint8_t buf[260]; size_t got = 0;
  while (got < sizeof(buf) && millis() - t0 < timeoutMs) {
    if (NextPMSerial.available()) buf[got++] = (uint8_t)NextPMSerial.read();
  }
  rawHex = bytesToHex(buf, got);
  if (got < 5) return 0;
  if (buf[0] != NEXTPM_MODBUS_ADDR) return 0;
  // Modbus exception: func = 0x83
  if ((buf[1] & 0x7F) != 0x03) return 0;
  if (buf[1] & 0x80) return 0; // exception

  uint8_t byteCount = buf[2];
  if (byteCount != qty * 2) return 0;
  if (got < (size_t)(3 + byteCount + 2)) return 0;

  uint16_t crcCalc = crc16_modbus(buf, 3 + byteCount);
  uint16_t crcRx   = (uint16_t)buf[3 + byteCount] | ((uint16_t)buf[3 + byteCount + 1] << 8);
  if (crcCalc != crcRx) return 0;

  for (uint16_t i = 0; i < qty; i++) {
    outWords[i] = ((uint16_t)buf[3 + 2 * i] << 8) | buf[3 + 2 * i + 1];
  }
  return qty;
}

static bool nextpmReadBinnedCounts(float& c02_05, float& c05_10, float& c10_25,
                                   float& c25_50, float& c50_100, String& rawHex) {
  uint16_t words[10] = {0};
  if (nextpmReadHolding(128, 10, words, rawHex) != 10) return false;

  auto U32_LSW_MSW = [](uint16_t lsw, uint16_t msw) -> uint32_t {
    return (uint32_t)lsw | ((uint32_t)msw << 16);
  };

  c02_05  = (float)U32_LSW_MSW(words[0], words[1]);
  c05_10  = (float)U32_LSW_MSW(words[2], words[3]);
  c10_25  = (float)U32_LSW_MSW(words[4], words[5]);
  c25_50  = (float)U32_LSW_MSW(words[6], words[7]);
  c50_100 = (float)U32_LSW_MSW(words[8], words[9]);
  return true;
}

// ==================== Senseair S8 =====================
static bool s8ReadCO2(uint16_t& co2ppm, String& txHex, String& rawHex) {
  co2ppm = 0;
  while (S8Serial.available()) S8Serial.read();

  const uint8_t cmd[8] = {0xFE, 0x04, 0x00, 0x03, 0x00, 0x01, 0xD5, 0xC5};
  txHex = bytesToHex(cmd, sizeof(cmd));
  if (S8Serial.write(cmd, sizeof(cmd)) != sizeof(cmd)) { rawHex = "(send fail)"; return false; }
  S8Serial.flush();

  const uint32_t t0 = millis();
  const uint32_t timeoutMs = 500;
  uint8_t resp[32]; size_t got = 0;
  while (got < sizeof(resp) && millis() - t0 < timeoutMs) {
    if (S8Serial.available()) resp[got++] = (uint8_t)S8Serial.read();
  }
  rawHex = bytesToHex(resp, got);
  if (got < 7) return false;
  if (resp[0] != 0xFE || resp[1] != 0x04 || resp[2] != 0x02) return false;

  uint16_t crcCalc = crc16_modbus(resp, 5);
  uint16_t crcRx   = (uint16_t)resp[5] | ((uint16_t)resp[6] << 8);
  if (crcCalc != crcRx) return false;

  co2ppm = ((uint16_t)resp[3] << 8) | resp[4];
  return true;
}

// ==================== Wi-Fi / Portal ==================
bool ensureWifiConnected() {
  if (WiFi.status() == WL_CONNECTED) return true;

  WiFi.mode(WIFI_STA);
  WiFiManager wm;

  char idbuf[40];
  strncpy(idbuf, gSensorIdFull.c_str(), sizeof(idbuf));
  idbuf[sizeof(idbuf) - 1] = 0;

  WiFiManagerParameter idParam(
      "sensorid",
      "Sensor ID (airgradient:xxxxxxxxxxxx or 12 hex)",
      idbuf, sizeof(idbuf) - 1);
  wm.addParameter(&idParam);

  String apName = "airgradient-" + agSerial12().substring(6);
  const char* apPass = "cleanair";
  wm.setConfigPortalTimeout(180);

  Serial.printf("Portail Wi-Fi SSID=%s pass=%s\n", apName.c_str(), apPass);
  bool ok = wm.autoConnect(apName.c_str(), apPass);

  String entered = String(idParam.getValue());
  String normalized = normalizeSensorId(entered, agSerial12());
  if (normalized != gSensorIdFull) {
    gSensorIdFull = normalized;
    saveSensorId(gSensorIdFull);
    Serial.printf("[ID] Nouveau Sensor ID: %s\n", gSensorIdFull.c_str());
  }

  if (ok) {
    Serial.printf("Wi-Fi OK: SSID=%s IP=%s RSSI=%d\n",
                  WiFi.SSID().c_str(), WiFi.localIP().toString().c_str(), WiFi.RSSI());
  } else {
    Serial.println("Wi-Fi non configure (timeout portail).");
  }
  return ok;
}

// ==================== POST -> AirGradient =============
static bool postToAirGradient(float pm01, float pm02, float pm10,
                              int pm003Count_dL, int rco2ppm) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[POST] No Wi-Fi.");
    return false;
  }

  String url = String("http://hw.airgradient.com/sensors/") + gSensorIdFull + "/measures";

  String payload = String("{\"wifi\":") + WiFi.RSSI();
  if (!isnan(pm01))       payload += String(",\"pm01\":") + String(pm01, 1);
  if (!isnan(pm02))       payload += String(",\"pm02\":") + String(pm02, 1);
  if (!isnan(pm10))       payload += String(",\"pm10\":") + String(pm10, 1);
  if (pm003Count_dL >= 0) payload += String(",\"pm003Count\":") + pm003Count_dL;
  if (rco2ppm >= 0)       payload += String(",\"rco2\":") + rco2ppm;
  if (latest.shtOk) {
    payload += String(",\"atmp\":") + String(latest.atmp, 2);
    payload += String(",\"rhum\":") + String(latest.rhum, 2);
  }
  if (latest.sgpOk && !latest.sgpConditioning) {
    payload += String(",\"tvoc_index\":") + String(latest.vocIndex);
    payload += String(",\"nox_index\":")  + String(latest.noxIndex);
    payload += String(",\"tvoc_raw\":")   + String(latest.sgpSrawVoc);
    payload += String(",\"nox_raw\":")    + String(latest.sgpSrawNox);
  }
  payload += "}";

  WiFiClient client;
  HTTPClient http;
  http.begin(client, url);
  http.addHeader("content-type", "application/json");
  int code = http.POST(payload);
  String resp = http.getString();
  http.end();

  latest.lastPostCode = code;
  latest.lastPostResp = resp;
  Serial.printf("[POST] %d %s\n", code, resp.c_str());
  return (code >= 200 && code < 300);
}

// ==================== Web Handlers ====================
static const char DASH_HTML[] PROGMEM = R"HTML(<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>OpenAir NextPM</title>
<style>body{font-family:system-ui,sans-serif;margin:0;padding:16px;background:#111;color:#eee}
h1{margin:0 0 12px;font-size:18px}.row{display:grid;grid-template-columns:repeat(auto-fit,minmax(130px,1fr));gap:8px;margin-bottom:12px}
.card{background:#1e1e1e;border-radius:8px;padding:10px 12px}.u{color:#888;font-size:12px}
.v{font-size:26px;font-weight:600;line-height:1.1}.ok{color:#4caf50}.err{color:#f44336}.warn{color:#ffa000}
.box{background:#1e1e1e;border-radius:8px;padding:10px 12px;margin:8px 0}
pre{background:#000;color:#9cf;padding:8px;border-radius:4px;overflow-x:auto;font-size:11px;margin:4px 0;white-space:pre-wrap;word-break:break-all}
.meta{color:#777;font-size:11px}a{color:#7af}</style></head>
<body><h1>OpenAir NextPM — Local Dashboard</h1>
<div id="root">Loading…</div>
<div class="meta">Auto-refresh 2s · <a href="/json">/json</a> · <a href="/probe">/probe</a> (Modbus scan) · <a href="/s8scan">/s8scan</a> (S8 RX-pin sweep)</div>
<script>
function esc(s){return (s==null?'':''+s).replace(/[&<>]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;'}[c]));}
function card(name,val,unit,ok){return '<div class="card"><div class="u">'+name+'</div><div class="v '+(ok===true?'ok':ok===false?'err':'')+'">'+val+'</div><div class="u">'+unit+'</div></div>';}
async function refresh(){
 try {
  const r = await fetch('/json',{cache:'no-store'}); const d = await r.json();
  let h = '<div class="row">';
  h += card('PM1',  d.pm1 ==null?'—':d.pm1.toFixed(1),  'µg/m³', d.massOk);
  h += card('PM2.5',d.pm25==null?'—':d.pm25.toFixed(1), 'µg/m³', d.massOk);
  h += card('PM10', d.pm10==null?'—':d.pm10.toFixed(1), 'µg/m³', d.massOk);
  h += card('PM0.3≈', d.pm003_dL>=0?d.pm003_dL:'—',     '/dL',   d.binsOk);
  h += card('CO₂',   d.co2Ok?d.co2:'—',                 'ppm',   d.co2Ok);
  h += card('Temp',  d.shtOk && d.atmp!=null?d.atmp.toFixed(1):'—','°C',    d.shtOk);
  h += card('RH',    d.shtOk && d.rhum!=null?d.rhum.toFixed(0):'—','%',     d.shtOk);
  h += card('TVOC',  d.sgpOk && !d.sgpConditioning?d.tvoc_index:'—','index',d.sgpOk && !d.sgpConditioning);
  h += card('NOx',   d.sgpOk && !d.sgpConditioning?d.nox_index:'—','index', d.sgpOk && !d.sgpConditioning);
  h += card('RSSI',  d.rssi,                            'dBm',   true);
  h += '</div>';
  const per = d.postAvgSec;
  const perLabel = per==10?'10 s':per==900?'15 min':'60 s';
  const fmt = s => s==null?'—':s.toFixed(1);
  const row = (lbl,s,isSel)=> '<tr'+(isSel?' style="background:#1b3a1b;color:#cfc"':'')
    +'><td>'+lbl+(isSel?' ★':'')+'</td><td>'+fmt(s.pm1)+'</td><td>'+fmt(s.pm25)+'</td><td>'+fmt(s.pm10)
    +'</td><td>'+s.cntPM1_dL+'</td><td>'+(s.ok?'<span class=ok>ok</span>':'<span class=err>fail</span>')+'</td></tr>';
  h += '<div class="box"><b>NextPM</b> — période primaire : <b>'+perLabel+'</b>'
     + '  <small>(<a href="/setperiod?sec=10">10s</a> · <a href="/setperiod?sec=60">60s</a> · <a href="/setperiod?sec=900">15m</a>)</small>'
     + '<table style="width:100%;margin-top:6px;font-size:12px;border-collapse:collapse"><thead><tr style="color:#888">'
     + '<th style="text-align:left">Moyennage</th><th>PM1</th><th>PM2.5</th><th>PM10</th><th>cnt≥0.3µm /dL</th><th>ok</th></tr></thead><tbody>'
     + row('10 s',  d.pm_10s_pm1!=null?{pm1:d.pm_10s_pm1,pm25:d.pm_10s_pm25,pm10:d.pm_10s_pm10,cntPM1_dL:d.pm_10s_cntPM1_dL,ok:d.pm_10s_ok}:{pm1:null,pm25:null,pm10:null,cntPM1_dL:0,ok:false}, per==10)
     + row('60 s',  {pm1:d.pm_60s_pm1,pm25:d.pm_60s_pm25,pm10:d.pm_60s_pm10,cntPM1_dL:d.pm_60s_cntPM1_dL,ok:d.pm_60s_ok}, per==60)
     + row('15 min',{pm1:d.pm_15m_pm1,pm25:d.pm_15m_pm25,pm10:d.pm_15m_pm10,cntPM1_dL:d.pm_15m_cntPM1_dL,ok:d.pm_15m_ok}, per==900)
     + '</tbody></table>'
     + '<div class="u" style="margin-top:4px">state=0x'+d.nextpmState.toString(16).padStart(2,'0')
     + ' · modbus-bins '+(d.binsOk?'<span class=ok>OK</span>':'<span class=err>FAIL</span>')
     + '</div></div>';
  h += '<div class="box"><b>S8 CO₂:</b> '+(d.co2Ok?'<span class=ok>OK '+d.co2+' ppm</span>':'<span class=err>read failed</span>')+'</div>';
  h += '<div class="box"><b>Last cloud POST:</b> HTTP '+d.lastPostCode+' <span class="meta">'+esc(d.lastPostResp)+'</span></div>';
  h += '<div class="box"><b>Raw diagnostics</b>'
     + '<div class="u">NextPM mass resp:</div><pre>'+esc(d.nextpmMassRaw)+'</pre>'
     + '<div class="u">NextPM bins resp (reg 128..137):</div><pre>'+esc(d.nextpmBinsRaw)+'</pre>'
     + '<div class="u">S8 tx → rx:</div><pre>'+esc(d.s8TxRaw)+'\n'+esc(d.s8Raw)+'</pre>'
     + '</div>';
  h += '<div class="meta">Sensor ID: '+esc(d.sensorId)+' · uptime '+d.uptimeSec+'s · age '+d.dataAgeSec+'s</div>';
  document.getElementById('root').innerHTML = h;
 } catch(e){ document.getElementById('root').innerText = 'Error: '+e.message; }
}
refresh(); setInterval(refresh, 2000);
</script></body></html>)HTML";

static void handleRoot() {
  webServer.sendHeader("Cache-Control", "no-store");
  webServer.send_P(200, "text/html", DASH_HTML);
}

static void handleJson() {
  String j = "{";
  j += "\"pm1\":"    + floatOrNull(latest.pm1);
  j += ",\"pm25\":"  + floatOrNull(latest.pm25);
  j += ",\"pm10\":"  + floatOrNull(latest.pm10);
  j += ",\"c02_05\":"  + String((uint32_t)latest.c02_05);
  j += ",\"c05_10\":"  + String((uint32_t)latest.c05_10);
  j += ",\"c10_25\":"  + String((uint32_t)latest.c10_25);
  j += ",\"c25_50\":"  + String((uint32_t)latest.c25_50);
  j += ",\"c50_100\":" + String((uint32_t)latest.c50_100);
  j += ",\"pm003_dL\":" + String(latest.pm003_dL);
  j += ",\"cntPM1_dL\":"  + String(latest.cntPM1_dL);
  j += ",\"cntPM25_dL\":" + String(latest.cntPM25_dL);
  j += ",\"cntPM10_dL\":" + String(latest.cntPM10_dL);
  j += ",\"postAvgSec\":" + String(latest.postAvgSec);
  // All three NextPM averaging periods
  auto emitSample = [&](const char* pfx, const NextPMSample& s) {
    j += String(",\"") + pfx + "_ok\":" + (s.ok ? "true" : "false");
    j += String(",\"") + pfx + "_pm1\":"  + floatOrNull(s.pm1);
    j += String(",\"") + pfx + "_pm25\":" + floatOrNull(s.pm25);
    j += String(",\"") + pfx + "_pm10\":" + floatOrNull(s.pm10);
    j += String(",\"") + pfx + "_cntPM1_dL\":"  + String(s.cntPM1_dL);
    j += String(",\"") + pfx + "_cntPM25_dL\":" + String(s.cntPM25_dL);
    j += String(",\"") + pfx + "_cntPM10_dL\":" + String(s.cntPM10_dL);
    j += String(",\"") + pfx + "_state\":" + String(s.state);
  };
  emitSample("pm_10s", latest.avg10s);
  emitSample("pm_60s", latest.avg60s);
  emitSample("pm_15m", latest.avg15m);
  j += ",\"co2\":"      + String(latest.co2);
  j += ",\"co2Ok\":"    + String(latest.co2Ok ? "true" : "false");
  j += ",\"atmp\":"    + floatOrNull(latest.atmp, 2);
  j += ",\"rhum\":"    + floatOrNull(latest.rhum, 2);
  j += ",\"shtOk\":"   + String(latest.shtOk ? "true" : "false");
  j += ",\"shtPresent\":" + String(latest.shtPresent ? "true" : "false");
  j += ",\"tvoc_index\":" + String(latest.vocIndex);
  j += ",\"nox_index\":"  + String(latest.noxIndex);
  j += ",\"tvoc_raw\":"   + String(latest.sgpSrawVoc);
  j += ",\"nox_raw\":"    + String(latest.sgpSrawNox);
  j += ",\"sgpOk\":"      + String(latest.sgpOk ? "true" : "false");
  j += ",\"sgpPresent\":" + String(latest.sgpPresent ? "true" : "false");
  j += ",\"sgpConditioning\":" + String(latest.sgpConditioning ? "true" : "false");
  j += ",\"massOk\":"   + String(latest.massOk ? "true" : "false");
  j += ",\"binsOk\":"   + String(latest.binsOk ? "true" : "false");
  j += ",\"nextpmState\":" + String(latest.nextpmState);
  j += ",\"rssi\":"     + String(WiFi.RSSI());
  j += ",\"lastPostCode\":" + String(latest.lastPostCode);
  j += ",\"lastPostResp\":\"" + jsonEscape(latest.lastPostResp) + "\"";
  j += ",\"nextpmMassRaw\":\"" + jsonEscape(latest.nextpmMassRaw) + "\"";
  j += ",\"nextpmBinsRaw\":\"" + jsonEscape(latest.nextpmBinsRaw) + "\"";
  j += ",\"s8TxRaw\":\""  + jsonEscape(latest.s8TxRaw) + "\"";
  j += ",\"s8Raw\":\""    + jsonEscape(latest.s8Raw) + "\"";
  j += ",\"sensorId\":\"" + jsonEscape(gSensorIdFull) + "\"";
  j += ",\"uptimeSec\":"  + String((uint32_t)(millis() / 1000));
  j += ",\"dataAgeSec\":" + String((uint32_t)((millis() - latest.lastUpdateMs) / 1000));
  j += "}";
  webServer.sendHeader("Cache-Control", "no-store");
  webServer.send(200, "application/json", j);
}

// Dump register ranges to help identify where NextPM actually exposes counts
static void handleProbe() {
  String out = "NextPM Modbus probe\n======================\n";

  struct Block { uint16_t start; uint16_t qty; };
  Block blocks[] = { {0,40}, {40,40}, {80,40}, {120,40}, {160,40}, {200,30} };

  for (auto& b : blocks) {
    uint16_t w[40] = {0};
    String raw;
    size_t got = nextpmReadHolding(b.start, b.qty, w, raw);
    char hdr[96];
    snprintf(hdr, sizeof(hdr), "\n[read %u..%u qty=%u] got=%u words\n",
             b.start, b.start + b.qty - 1, b.qty, (unsigned)got);
    out += hdr;
    out += "raw: " + raw + "\n";
    if (got == b.qty) {
      for (uint16_t i = 0; i < b.qty; i++) {
        if (w[i] == 0) continue; // skip zeros for readability
        char line[72];
        snprintf(line, sizeof(line), "  reg[%3u] = 0x%04X  (%u)\n", b.start + i, w[i], w[i]);
        out += line;
      }
    }
    delay(80); // breathe between Modbus requests
  }

  webServer.sendHeader("Cache-Control", "no-store");
  webServer.send(200, "text/plain", out);
}

// GPIO scan: try each candidate pin as UART0 RX for S8 and see if any bytes come back.
// Responds with a table of pin -> bytes received.
static void handleS8Scan() {
  // Candidate free GPIOs on ESP32-C3 that are brought out on most boards.
  // Skip: 0/1 (NextPM), 8/9 (strap/boot), 18/19 (USB D-/D+).
  const int candidates[] = { 2, 3, 4, 5, 6, 7, 10, 20 };
  const int candTxDefault = 21;

  String out = "S8 UART RX-pin scan\n===================\n";
  out += "TX fixed on GPIO" + String(candTxDefault) + "\n";
  out += "Command: FE 04 00 03 00 01 D5 C5\n\n";

  S8Serial.end();
  delay(50);

  const uint8_t cmd[8] = {0xFE, 0x04, 0x00, 0x03, 0x00, 0x01, 0xD5, 0xC5};

  for (int i = 0; i < (int)(sizeof(candidates)/sizeof(candidates[0])); i++) {
    int rxp = candidates[i];
    S8Serial.end();
    delay(20);
    S8Serial.begin(9600, SERIAL_8N1, rxp, candTxDefault);
    S8Serial.setTimeout(50);
    while (S8Serial.available()) S8Serial.read();
    S8Serial.write(cmd, sizeof(cmd));
    S8Serial.flush();

    uint32_t t0 = millis();
    uint8_t buf[32]; size_t got = 0;
    while (got < sizeof(buf) && millis() - t0 < 400) {
      if (S8Serial.available()) buf[got++] = (uint8_t)S8Serial.read();
    }
    char line[96];
    snprintf(line, sizeof(line), "RX=GPIO%-2d  got=%2u bytes:  ", rxp, (unsigned)got);
    out += line + bytesToHex(buf, got) + "\n";
  }

  // Restore default
  S8Serial.end();
  delay(20);
  S8Serial.begin(9600, SERIAL_8N1, S8_RX_PIN, S8_TX_PIN);
  S8Serial.setTimeout(50);

  webServer.sendHeader("Cache-Control", "no-store");
  webServer.send(200, "text/plain", out);
}

// ==================== I2C sensors init + read ========
static void initI2CSensors() {
  Wire.end();
  delay(10);
  if (!Wire.begin(I2C_SDA, I2C_SCL, 100000)) {
    Serial.println("[I2C] Wire.begin failed");
    return;
  }
  i2cInitialized = true;

  // Probe SHT4x at 0x44
  Wire.beginTransmission(0x44);
  bool sht44 = (Wire.endTransmission(true) == 0);
  if (sht44) {
    sht4x.begin(Wire, 0x44);
    uint32_t sn = 0;
    if (sht4x.serialNumber(sn) == 0) {
      latest.shtPresent = true;
      Serial.printf("[SHT4x] detected, serial=0x%08X\n", sn);
    } else {
      Serial.println("[SHT4x] 0x44 ACKs but serialNumber() failed");
    }
  }

  // Probe SGP41 at 0x59
  Wire.beginTransmission(0x59);
  bool sgpAck = (Wire.endTransmission(true) == 0);
  if (sgpAck) {
    sgp41.begin(Wire);
    uint16_t testResult = 0;
    uint16_t error = sgp41.executeSelfTest(testResult);  // ~320 ms
    if (error == 0) {
      latest.sgpPresent = true;
      sgpBootMs = millis();
      Serial.printf("[SGP41] self-test OK (result=0x%04X), entering 10 s conditioning\n", testResult);
    } else {
      Serial.printf("[SGP41] self-test FAIL err=%u\n", error);
    }
  }
}

// Read SHT4x, update latest.atmp/rhum. Returns true on success.
static bool sampleSHT() {
  if (!latest.shtPresent) return false;
  float t = NAN, rh = NAN;
  uint16_t err = sht4x.measureHighPrecision(t, rh);
  if (err != 0) { latest.shtOk = false; return false; }
  latest.atmp = t;
  latest.rhum = rh;
  latest.shtOk = true;
  return true;
}

// Sample SGP41. During first SGP41_COND_MS use executeConditioning, then measureRawSignals.
static bool sampleSGP() {
  if (!latest.sgpPresent) return false;

  // Compensation words: default 25 °C / 50 % RH if no SHT
  uint16_t compRh = 0x8000; // 50 %
  uint16_t compT  = 0x6666; // 25 °C
  if (latest.shtOk && !isnan(latest.atmp) && !isnan(latest.rhum)) {
    float rh = constrain(latest.rhum, 0.0f, 100.0f);
    float t  = constrain(latest.atmp, -45.0f, 130.0f);
    compRh = (uint16_t) lroundf(rh * 65535.0f / 100.0f);
    compT  = (uint16_t) lroundf((t + 45.0f) * 65535.0f / 175.0f);
  }

  uint16_t srawVoc = 0, srawNox = 0;
  bool conditioning = (millis() - sgpBootMs) < SGP41_COND_MS;
  latest.sgpConditioning = conditioning;

  uint16_t err;
  if (conditioning) {
    err = sgp41.executeConditioning(compRh, compT, srawVoc);
    srawNox = 0;
  } else {
    err = sgp41.measureRawSignals(compRh, compT, srawVoc, srawNox);
  }
  if (err != 0) { latest.sgpOk = false; return false; }

  latest.sgpSrawVoc = srawVoc;
  latest.sgpSrawNox = srawNox;
  latest.sgpOk = true;

  // Feed gas index algorithm (must be called at ~1 Hz steady cadence)
  int32_t voc = vocAlgo.process((int32_t)srawVoc);
  int32_t nox = conditioning ? 0 : noxAlgo.process((int32_t)srawNox);
  latest.vocIndex = voc;
  latest.noxIndex = nox;
  return true;
}

// ==================== I2C bus scan ====================
// Sweep candidate (SDA,SCL) pairs (excluding NextPM 0/1, S8 20/21, USB 18/19)
// and for each, run a standard I2C address scan (0x08..0x77). Report any hits.
static String knownAddrLabel(uint8_t a) {
  switch (a) {
    case 0x44: case 0x45: return "SHT3x/SHT4x (T/RH)";
    case 0x59:            return "SGP41 (TVOC/NOx)";
    case 0x58:            return "SGP30";
    case 0x62:            return "SCD40/41 (CO2)";
    case 0x61:            return "SCD30 (CO2)";
    case 0x76: case 0x77: return "BMP/BME 280 or 680";
    case 0x29:            return "VEML7700 / TSL2591";
    case 0x10:            return "VEML6075";
    case 0x38:            return "AHT10/20 (T/RH)";
    case 0x40:            return "Si7021 / HTU21";
    case 0x3C: case 0x3D: return "SSD1306 OLED";
    default:              return "";
  }
}

static void handleI2CScan() {
  // Candidate SDA,SCL pairs to try.
  const int pairs[][2] = {
    { 7, 6},  { 6, 7},
    { 5, 4},  { 4, 5},
    { 3, 2},  { 2, 3},
    {10, 8},  { 8,10},
    { 7,10},  {10, 7}
  };
  const int N = (int)(sizeof(pairs) / sizeof(pairs[0]));

  String out = "I2C bus scan\n============\n";

  for (int p = 0; p < N; p++) {
    int sda = pairs[p][0], scl = pairs[p][1];
    out += "\n[SDA=GPIO" + String(sda) + " SCL=GPIO" + String(scl) + "]\n";

    Wire.end();
    delay(10);
    // Some boards need a small pre-toggle to release any stuck slave.
    if (!Wire.begin(sda, scl, 100000)) {
      out += "  Wire.begin failed\n";
      continue;
    }
    Wire.setTimeOut(30);

    int found = 0;
    for (uint8_t a = 0x08; a <= 0x77; a++) {
      Wire.beginTransmission(a);
      uint8_t err = Wire.endTransmission(true);
      if (err == 0) {
        found++;
        char line[80];
        snprintf(line, sizeof(line), "  0x%02X  %s\n", a, knownAddrLabel(a).c_str());
        out += line;
      }
    }
    if (found == 0) out += "  (no devices)\n";
  }

  Wire.end();
  webServer.sendHeader("Cache-Control", "no-store");
  webServer.send(200, "text/plain", out);
}

// ==================== Averaging period management ====
static uint16_t loadSavedPostAvgSec() {
  prefs.begin("ag", true);
  uint16_t v = prefs.getUShort("avgsec", 60);
  prefs.end();
  if (v != 10 && v != 60 && v != 900) v = 60;
  return v;
}

static void savePostAvgSec(uint16_t v) {
  prefs.begin("ag", false);
  prefs.putUShort("avgsec", v);
  prefs.end();
}

static void handleSetPeriod() {
  if (!webServer.hasArg("sec")) {
    webServer.send(400, "text/plain",
      "missing ?sec=10 | 60 | 900\n"
      "current: " + String(latest.postAvgSec) + " s\n");
    return;
  }
  long v = webServer.arg("sec").toInt();
  if (v != 10 && v != 60 && v != 900) {
    webServer.send(400, "text/plain", "sec must be 10, 60, or 900\n");
    return;
  }
  latest.postAvgSec = (uint16_t)v;
  savePostAvgSec(latest.postAvgSec);
  webServer.send(200, "text/plain",
    "Averaging period set to " + String(latest.postAvgSec) + " s.\n"
    "Applied to next POST cycle.\n");
}

// ==================== ID management endpoints ========
static void handleSetId() {
  if (!webServer.hasArg("id")) {
    webServer.send(400, "text/plain",
      "missing ?id=<12hex or airgradient:12hex>\n"
      "current: " + gSensorIdFull + "\n");
    return;
  }
  String newId = normalizeSensorId(webServer.arg("id"), agSerial12());
  gSensorIdFull = newId;
  saveSensorId(gSensorIdFull);
  webServer.send(200, "text/plain",
    "Sensor ID set to: " + gSensorIdFull + "\n"
    "Next POST (within 10 s) will use this ID.\n");
}

static void handleClearId() {
  prefs.begin("ag", false);
  prefs.remove("id");
  prefs.end();
  gSensorIdFull = String("airgradient:") + agSerial12();
  saveSensorId(gSensorIdFull);
  webServer.send(200, "text/plain",
    "Sensor ID reset to MAC-derived: " + gSensorIdFull + "\n");
}

static void handleMacInfo() {
  String staMacLower = agSerial12();
  String apMac = WiFi.softAPmacAddress(); apMac.toLowerCase(); apMac.replace(":", "");

  String out = "Stored Sensor ID   : " + gSensorIdFull + "\n";
  out += "STA MAC (ag serial): " + staMacLower + "\n";
  out += "AP  MAC            : " + apMac + "\n";
  out += "\nTo align the stored ID with the chip MAC, call:\n";
  out += "  http://" + WiFi.localIP().toString() + "/setid?id=" + staMacLower + "\n";
  out += "Or to clear NVS and re-derive from MAC:\n";
  out += "  http://" + WiFi.localIP().toString() + "/clearid\n";
  webServer.send(200, "text/plain", out);
}

// ==================== Setup / Loop ====================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nBoot");

  WiFi.mode(WIFI_STA);

  gSensorIdFull = loadSavedSensorId();
  if (gSensorIdFull.length() == 0) {
    gSensorIdFull = String("airgradient:") + agSerial12();
    saveSensorId(gSensorIdFull);
  }
  latest.postAvgSec = loadSavedPostAvgSec();
  Serial.printf("DeviceID (%s) postAvgSec=%u s\n", gSensorIdFull.c_str(), latest.postAvgSec);

  NextPMSerial.begin(115200, SERIAL_8E1, NEXTPM_RX_PIN, NEXTPM_TX_PIN);
  NextPMSerial.setTimeout(60);
  S8Serial.begin(9600, SERIAL_8N1, S8_RX_PIN, S8_TX_PIN);
  S8Serial.setTimeout(50);

  Serial.printf("NextPM: RX=%d TX=%d (115200 8E1)\n", NEXTPM_RX_PIN, NEXTPM_TX_PIN);
  Serial.printf("S8   : RX=%d TX=%d (9600  8N1)\n", S8_RX_PIN, S8_TX_PIN);
  Serial.printf("I2C  : SDA=%d SCL=%d\n", I2C_SDA, I2C_SCL);

  initI2CSensors();

  ensureWifiConnected();

  if (WiFi.status() == WL_CONNECTED) {
    if (MDNS.begin("openair-nextpm")) {
      MDNS.addService("http", "tcp", 80);
      Serial.println("mDNS: openair-nextpm.local");
    }
    webServer.on("/", handleRoot);
    webServer.on("/json", handleJson);
    webServer.on("/probe", handleProbe);
    webServer.on("/s8scan", handleS8Scan);
    webServer.on("/setid", handleSetId);
    webServer.on("/clearid", handleClearId);
    webServer.on("/macinfo", handleMacInfo);
    webServer.on("/i2cscan", handleI2CScan);
    webServer.on("/setperiod", handleSetPeriod);
    webServer.begin();
    Serial.printf("HTTP server up: http://%s/\n", WiFi.localIP().toString().c_str());
  }
}

void loop() {
  static uint32_t tLastPost = 0, tTick = 0, tLastGas = 0;
  webServer.handleClient();

  if (millis() - tTick > 2000) {
    tTick = millis();
    Serial.printf("[tick] ip=%s rssi=%d\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
  }

  // Gas sensors at 1 Hz (required for Sensirion gas index algorithm)
  if (i2cInitialized && millis() - tLastGas >= GAS_SAMPLE_MS) {
    tLastGas = millis();
    sampleSHT();
    sampleSGP();
  }

  if (millis() - tLastPost > POST_PERIOD_MS) {
    tLastPost = millis();

    // Query all 3 averaging periods (10 s, 60 s, 15 min). Small gap between requests.
    nextpmReadMassCmd(0x11, latest.avg10s); delay(80);
    nextpmReadMassCmd(0x12, latest.avg60s); delay(80);
    nextpmReadMassCmd(0x13, latest.avg15m);

    // Pick the primary (mirrored to top-level + used in AirGradient POST)
    const NextPMSample* sel = &latest.avg60s;
    if      (latest.postAvgSec == 10)  sel = &latest.avg10s;
    else if (latest.postAvgSec == 900) sel = &latest.avg15m;
    else                               sel = &latest.avg60s;

    float pm1 = sel->pm1, pm25 = sel->pm25, pm10 = sel->pm10;
    uint16_t cnt1 = sel->cntPM1_dL, cnt25 = sel->cntPM25_dL, cnt10 = sel->cntPM10_dL;
    bool okMass = sel->ok;

    latest.nextpmMassRaw = sel->rawHex;
    latest.massOk = okMass;
    int pm003_dL = -1;
    if (okMass) {
      latest.pm1 = pm1; latest.pm25 = pm25; latest.pm10 = pm10; latest.nextpmState = sel->state;
      latest.cntPM1_dL = cnt1; latest.cntPM25_dL = cnt25; latest.cntPM10_dL = cnt10;
      pm003_dL = (int)cnt1;
      latest.pm003_dL = pm003_dL;
      Serial.printf("NextPM [%us]: PM1=%.1f PM2.5=%.1f PM10=%.1f state=0x%02X | 10s=%.1f/%.1f/%.1f 60s=%.1f/%.1f/%.1f 15m=%.1f/%.1f/%.1f\n",
                    latest.postAvgSec, pm1, pm25, pm10, sel->state,
                    latest.avg10s.pm1, latest.avg10s.pm25, latest.avg10s.pm10,
                    latest.avg60s.pm1, latest.avg60s.pm25, latest.avg60s.pm10,
                    latest.avg15m.pm1, latest.avg15m.pm25, latest.avg15m.pm10);
    } else {
      Serial.printf("NextPM mass FAIL (sel %us) raw=[%s]\n", latest.postAvgSec, latest.nextpmMassRaw.c_str());
    }

    // Optional: still try the Modbus bins so we can see raw response.
    float c02_05 = 0, c05_10 = 0, c10_25 = 0, c25_50 = 0, c50_100 = 0;
    String rawBins;
    bool okBins = nextpmReadBinnedCounts(c02_05, c05_10, c10_25, c25_50, c50_100, rawBins);
    latest.nextpmBinsRaw = rawBins;
    latest.binsOk = okBins;
    if (okBins) {
      latest.c02_05 = c02_05; latest.c05_10 = c05_10; latest.c10_25 = c10_25;
      latest.c25_50 = c25_50; latest.c50_100 = c50_100;
    }

    uint16_t co2 = 0;
    String s8Tx, s8Rx;
    bool okCO2 = s8ReadCO2(co2, s8Tx, s8Rx);
    latest.s8TxRaw = s8Tx; latest.s8Raw = s8Rx;
    latest.co2Ok = okCO2;
    if (okCO2) {
      latest.co2 = co2;
      Serial.printf("CO2=%u ppm\n", co2);
    } else {
      Serial.printf("S8 FAIL tx=[%s] rx=[%s]\n", s8Tx.c_str(), s8Rx.c_str());
    }

    latest.lastUpdateMs = millis();

    if (okMass || okBins || okCO2) {
      postToAirGradient(okMass ? pm1 : NAN,
                        okMass ? pm25 : NAN,
                        okMass ? pm10 : NAN,
                        pm003_dL,
                        okCO2 ? (int)co2 : -1);
    }
  }
}
