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
#include <WiFiClientSecure.h>
#include <WiFiManager.h>
#include <Update.h>
#include <esp_ota_ops.h>
#include <esp_task_wdt.h>
#include <esp_system.h>
#include <time.h>
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

// Optional compile-time provisioning (gitignored secrets.h). Lets a station
// connect + authenticate without the captive portal — handy for plug-and-play
// deployment. Falls back to the portal when absent or when the hardcoded Wi-Fi
// is unreachable. NEVER publish an OTA binary built with a real token here:
// the token would be extractable from the public .bin.
#if __has_include("secrets.h")
  #include "secrets.h"
#endif
#ifndef WIFI_SSID_DEFAULT
  #define WIFI_SSID_DEFAULT ""
#endif
#ifndef WIFI_PASS_DEFAULT
  #define WIFI_PASS_DEFAULT ""
#endif
#ifndef DEVICE_TOKEN_DEFAULT
  #define DEVICE_TOKEN_DEFAULT ""
#endif

// Forward decl so the Arduino .ino auto-prototypes see the type name
struct NextPMSample;

// -------------------- Pins / UART --------------------
constexpr int NEXTPM_RX_PIN = 0;
constexpr int NEXTPM_TX_PIN = 1;
constexpr int S8_RX_PIN     = 20;
constexpr int S8_TX_PIN     = 21;

HardwareSerial NextPMSerial(1);
HardwareSerial S8Serial(0);

constexpr uint8_t  NEXTPM_MODBUS_ADDR = 0x01;
constexpr float    PM003_FROM_02_05_FRACTION = 0.0f;
constexpr uint32_t POST_PERIOD_MS = 30000;

// -------------------- AirSentinels backend ----------
// The station POSTs straight to our own backend (PocketBase behind Traefik TLS).
// HTTPS only — Traefik 301/308-redirects :80 -> :443, so plain HTTP won't work.
constexpr char AIRSENTINELS_HOST[] = "station.airsentinels.fr";
constexpr char AIRSENTINELS_URL[]  = "https://station.airsentinels.fr/api/openair/ingest";

// Pinned root CA: ISRG Root X1 (Let's Encrypt). Valid until 2035-06-04.
// Pinning the *root* (not the rotating intermediate) protects the device token
// from MITM without needing firmware updates when LE rotates intermediates.
static const char ISRG_ROOT_X1_PEM[] = R"CERT(-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)CERT";

// Device token (shared secret) sent in the X-Device-Token header. Persisted in
// NVS, entered once via the Wi-Fi captive portal. Empty until provisioned.
String gDeviceToken;
// NTP clock state. Until synced we omit "ts" and let the server stamp ingestion.
static bool gTimeSynced = false;

// -------------------- OTA (pull-based) ---------------
// Bump on every release. The OTA manifest's "version" is compared against this;
// if different, the station downloads and flashes the new image.
constexpr char FW_VERSION[]       = "2026.06.25-1";
constexpr char OTA_MANIFEST_URL[] = "https://station.airsentinels.fr/firmware/openair-nextpm.json";
constexpr uint32_t OTA_CHECK_PERIOD_MS    = 6UL * 3600UL * 1000UL;  // re-check every 6 h
constexpr uint32_t OTA_FIRST_CHECK_MS     = 45UL * 1000UL;          // first check 45 s after boot
constexpr uint32_t OTA_CONFIRM_TIMEOUT_MS = 5UL * 60UL * 1000UL;    // a fresh image must POST OK within 5 min
static bool     gOtaPendingConfirm = false;  // booted a freshly OTA'd image, not yet confirmed
static uint32_t gBootMs = 0;

// -------------------- Reliability (Phase 2a) ---------
constexpr uint32_t WDT_TIMEOUT_S      = 120;        // reboot if loop or gas task hangs this long
constexpr uint32_t WIFI_CHECK_MS      = 5000;       // reconnect cadence when disconnected
constexpr uint32_t NTP_RESYNC_OK_MS   = 6UL * 3600UL * 1000UL;  // resync every 6 h once synced
constexpr uint32_t NTP_RETRY_MS       = 5UL * 60UL * 1000UL;    // retry every 5 min until synced
constexpr int      OUTBOX_MAX         = 60;         // ~30 min of 30 s readings buffered offline
constexpr uint32_t BACKOFF_MIN_MS     = 30000;      // first backoff after a server error
constexpr uint32_t BACKOFF_MAX_MS     = 300000;     // cap at 5 min
constexpr int      DRAIN_PER_CYCLE    = 15;         // max buffered POSTs flushed per cycle

// Offline outbox (FIFO ring of pre-serialized JSON payloads).
static String   gOutbox[OUTBOX_MAX];
static int      gObHead = 0, gObCount = 0;
static uint32_t gBackoffUntilMs = 0, gBackoffMs = 0;
static uint32_t gPostOk = 0, gPostFail = 0;   // lifetime counters (telemetry)
static uint32_t gCycleJitterMs = 0;           // ±jitter so co-located stations don't POST in lockstep

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
static SemaphoreHandle_t i2cMutex = nullptr;

// Rolling window for gas-index smoothing (POST stability)
static constexpr int GAS_WIN = 10;  // 10 samples @ 1 Hz = 10 s
static int32_t vocHist[GAS_WIN] = {0};
static int32_t noxHist[GAS_WIN] = {0};
static int     gasHistIdx = 0;
static int     gasHistCount = 0;
static uint32_t gasMissedTicks = 0;   // diagnostic: how many 1 Hz ticks we were late (should stay 0)

static inline void i2cLock()   { if (i2cMutex) xSemaphoreTake(i2cMutex, portMAX_DELAY); }
static inline void i2cUnlock() { if (i2cMutex) xSemaphoreGive(i2cMutex); }

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
  int32_t vocIndexAvg = 0, noxIndexAvg = 0;  // rolling 10 s avg used in POST
  uint16_t sgpSrawVoc = 0, sgpSrawNox = 0;
  bool sgpOk = false, sgpPresent = false;
  bool sgpConditioning = true;
  uint32_t gasSamples = 0;  // diagnostic: total 1 Hz ticks taken
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

// device_serial sent to AirSentinels = the bare 12-hex, stripped of the legacy
// "airgradient:" prefix. Honors a manual override set via /setid.
String deviceSerial12() {
  String s = gSensorIdFull;
  s.toLowerCase();
  if (s.startsWith("airgradient:")) s.remove(0, 12);
  s.replace(":", ""); s.replace(" ", "");
  if (s.length() != 12) return agSerial12();
  return s;
}

String loadSavedToken() {
  prefs.begin("ag", true);
  String t = prefs.getString("tok", "");
  prefs.end();
  return t;
}

void saveToken(const String& t) {
  prefs.begin("ag", false);
  prefs.putString("tok", t);
  prefs.end();
}

// Short label for the last reset cause — distinguishes a clean OTA reboot from a
// watchdog/panic/brownout in the fleet telemetry.
const char* resetReasonStr() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  return "poweron";
    case ESP_RST_SW:       return "sw";       // ESP.restart() (OTA, rollback)
    case ESP_RST_PANIC:    return "panic";
    case ESP_RST_INT_WDT:  return "int_wdt";
    case ESP_RST_TASK_WDT: return "task_wdt"; // our watchdog fired
    case ESP_RST_WDT:      return "wdt";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_DEEPSLEEP:return "deepsleep";
    default:               return "other";
  }
}

// ISO8601 UTC timestamp, e.g. 2026-06-25T07:35:00Z. Returns "" if NTP not synced.
String isoUtcNow() {
  if (!gTimeSynced) return String("");
  time_t now = time(nullptr);
  if (now < 1700000000) return String("");  // sanity: clock not set
  struct tm tmv;
  gmtime_r(&now, &tmv);
  char buf[24];
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tmv);
  return String(buf);
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

// NextPM granulometry via the SIMPLE protocol (cmd 0x25/0x26/0x27 = 10s/60s/15min).
// These are the real size bins — the Modbus registers (128..137) are empty on this
// firmware revision. Response is 24 bytes:
//   [0]=0x81 [1]=cmd [2]=state  then 5 × 4-byte counts (Nb/L), each MSW(2B)+LSW(2B),
//   big-endian within each 16-bit word, [23]=checksum.
// Bins: 0.3-0.5, 0.5-1, 1-2.5, 2.5-5, 5-10 µm.
static bool nextpmReadBinsCmd(uint8_t cmd, float& b03_05, float& b05_1, float& b1_25,
                              float& b25_5, float& b5_10, String& rawHex) {
  while (NextPMSerial.available()) NextPMSerial.read();
  if (!nextpmSendSimpleCmd(cmd)) { rawHex = "(send fail)"; return false; }

  const uint32_t t0 = millis();
  const uint32_t timeoutMs = 500;
  uint8_t buf[24]; size_t got = 0;
  while (got < sizeof(buf) && millis() - t0 < timeoutMs) {
    if (NextPMSerial.available()) buf[got++] = (uint8_t)NextPMSerial.read();
  }
  rawHex = bytesToHex(buf, got);
  if (got != sizeof(buf)) return false;
  if (buf[0] != 0x81 || buf[1] != cmd) return false;
  if (nextpmChecksum(buf, 23) != buf[23]) return false;

  auto U32 = [](const uint8_t* p) -> uint32_t {
    uint32_t msw = ((uint32_t)p[0] << 8) | p[1];
    uint32_t lsw = ((uint32_t)p[2] << 8) | p[3];
    return (msw << 16) | lsw;
  };
  b03_05 = (float)U32(&buf[3]);
  b05_1  = (float)U32(&buf[7]);
  b1_25  = (float)U32(&buf[11]);
  b25_5  = (float)U32(&buf[15]);
  b5_10  = (float)U32(&buf[19]);
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

  // Plug-and-play: try hardcoded creds first (from secrets.h), before the portal.
  if (strlen(WIFI_SSID_DEFAULT) > 0) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID_DEFAULT, WIFI_PASS_DEFAULT);
    Serial.printf("Wi-Fi: trying hardcoded SSID=%s ...\n", WIFI_SSID_DEFAULT);
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) delay(200);
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("Wi-Fi OK (hardcoded): IP=%s RSSI=%d\n",
                    WiFi.localIP().toString().c_str(), WiFi.RSSI());
      return true;
    }
    Serial.println("Hardcoded Wi-Fi failed — falling back to captive portal");
  }

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

  char tokbuf[80];
  strncpy(tokbuf, gDeviceToken.c_str(), sizeof(tokbuf));
  tokbuf[sizeof(tokbuf) - 1] = 0;
  WiFiManagerParameter tokParam(
      "devtoken",
      "AirSentinels device token",
      tokbuf, sizeof(tokbuf) - 1);
  wm.addParameter(&tokParam);

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

  String enteredTok = String(tokParam.getValue());
  enteredTok.trim();
  if (enteredTok.length() > 0 && enteredTok != gDeviceToken) {
    gDeviceToken = enteredTok;
    saveToken(gDeviceToken);
    Serial.println("[TOKEN] AirSentinels device token saved");
  }

  if (ok) {
    Serial.printf("Wi-Fi OK: SSID=%s IP=%s RSSI=%d\n",
                  WiFi.SSID().c_str(), WiFi.localIP().toString().c_str(), WiFi.RSSI());
  } else {
    Serial.println("Wi-Fi non configure (timeout portail).");
  }
  return ok;
}

// ==================== Outbox (offline buffer) ========
static void otaConfirmSuccess();  // defined in the OTA section below
static bool outboxEmpty() { return gObCount == 0; }
static void outboxPush(const String& s) {
  int idx = (gObHead + gObCount) % OUTBOX_MAX;
  if (gObCount < OUTBOX_MAX) { gOutbox[idx] = s; gObCount++; }
  else { gOutbox[gObHead] = s; gObHead = (gObHead + 1) % OUTBOX_MAX; }  // overwrite oldest
}
static String& outboxPeek() { return gOutbox[gObHead]; }
static void outboxPop() {
  if (gObCount > 0) { gOutbox[gObHead] = String(); gObHead = (gObHead + 1) % OUTBOX_MAX; gObCount--; }
}
static void resetBackoff() { gBackoffMs = 0; gBackoffUntilMs = 0; }
static void applyBackoff() {
  gBackoffMs = gBackoffMs ? min(gBackoffMs * 2, BACKOFF_MAX_MS) : BACKOFF_MIN_MS;
  gBackoffUntilMs = millis() + gBackoffMs;
  Serial.printf("[POST] backoff %u s (outbox=%d)\n", gBackoffMs / 1000, gObCount);
}

// ==================== POST -> AirSentinels ===========
// Builds the native AirSentinels payload from `latest`. device_serial +
// X-Device-Token identify the unit; ts is sent only when NTP is synced (else the
// server stamps ingestion time). Trailing health telemetry lets the fleet see
// uptime / heap / buffer depth / reset cause without a serial console.
static String buildPayload() {
  String payload = String("{\"device_serial\":\"") + deviceSerial12() + "\"";
  String ts = isoUtcNow();
  if (ts.length()) payload += String(",\"ts\":\"") + ts + "\"";

  payload += String(",\"rssi\":") + WiFi.RSSI();
  payload += String(",\"postAvgSec\":") + latest.postAvgSec;
  payload += String(",\"fw_version\":\"") + FW_VERSION + "\"";

  // Primary mirror (selected window) — only when this cycle's mass read was OK,
  // so we never POST stale PM from a previous successful cycle.
  if (latest.massOk) {
    if (!isnan(latest.pm1))  payload += String(",\"pm1\":")  + String(latest.pm1, 1);
    if (!isnan(latest.pm25)) payload += String(",\"pm25\":") + String(latest.pm25, 1);
    if (!isnan(latest.pm10)) payload += String(",\"pm10\":") + String(latest.pm10, 1);
    if (latest.pm003_dL >= 0) payload += String(",\"pm003_dL\":") + latest.pm003_dL;
    payload += String(",\"cntPM1_dL\":")  + latest.cntPM1_dL;
    payload += String(",\"cntPM25_dL\":") + latest.cntPM25_dL;
    payload += String(",\"cntPM10_dL\":") + latest.cntPM10_dL;
  }
  // Modbus binned counts (usually empty on this FW revision, kept for forward-compat)
  if (latest.binsOk) {
    payload += String(",\"c02_05\":")  + String((uint32_t)latest.c02_05);
    payload += String(",\"c05_10\":")  + String((uint32_t)latest.c05_10);
    payload += String(",\"c10_25\":")  + String((uint32_t)latest.c10_25);
    payload += String(",\"c25_50\":")  + String((uint32_t)latest.c25_50);
    payload += String(",\"c50_100\":") + String((uint32_t)latest.c50_100);
  }

  // All three NextPM averaging windows
  auto emitWin = [&](const char* pfx, const NextPMSample& s) {
    payload += String(",\"") + pfx + "_ok\":" + (s.ok ? "true" : "false");
    if (s.ok) {
      payload += String(",\"") + pfx + "_pm1\":"  + String(s.pm1, 1);
      payload += String(",\"") + pfx + "_pm25\":" + String(s.pm25, 1);
      payload += String(",\"") + pfx + "_pm10\":" + String(s.pm10, 1);
      payload += String(",\"") + pfx + "_cntPM1_dL\":"  + s.cntPM1_dL;
      payload += String(",\"") + pfx + "_cntPM25_dL\":" + s.cntPM25_dL;
      payload += String(",\"") + pfx + "_cntPM10_dL\":" + s.cntPM10_dL;
    }
  };
  emitWin("pm_10s", latest.avg10s);
  emitWin("pm_60s", latest.avg60s);
  emitWin("pm_15m", latest.avg15m);

  if (latest.co2Ok) payload += String(",\"co2\":") + latest.co2;
  if (latest.shtOk) {
    payload += String(",\"atmp\":") + String(latest.atmp, 2);
    payload += String(",\"rhum\":") + String(latest.rhum, 2);
  }
  if (latest.sgpOk && !latest.sgpConditioning) {
    payload += String(",\"tvoc_index\":")     + String(latest.vocIndex);
    payload += String(",\"nox_index\":")      + String(latest.noxIndex);
    payload += String(",\"tvoc_index_avg\":") + String(latest.vocIndexAvg);
    payload += String(",\"nox_index_avg\":")  + String(latest.noxIndexAvg);
    payload += String(",\"tvoc_raw\":")       + String(latest.sgpSrawVoc);
    payload += String(",\"nox_raw\":")        + String(latest.sgpSrawNox);
  }
  payload += String(",\"sensor_ok\":") + ((latest.massOk && latest.co2Ok) ? "true" : "false");
  payload += String(",\"sgpConditioning\":") + (latest.sgpConditioning ? "true" : "false");

  // Health telemetry
  payload += String(",\"uptime_s\":") + (millis() / 1000);
  payload += String(",\"heap_free\":") + (uint32_t)ESP.getFreeHeap();
  payload += String(",\"outbox_depth\":") + gObCount;
  payload += String(",\"post_fail\":") + gPostFail;
  payload += String(",\"reset_reason\":\"") + resetReasonStr() + "\"";
  payload += String(",\"ip\":\"") + WiFi.localIP().toString() + "\"";
  payload += "}";
  return payload;
}

// Single HTTPS POST of one payload. Returns the HTTP status (or a negative
// transport error). Explicit timeouts so a half-open TLS connection can't wedge
// the loop (the watchdog is the last resort).
static int sendPayload(const String& payload) {
  WiFiClientSecure client;
  client.setCACert(ISRG_ROOT_X1_PEM);
  HTTPClient http;
  http.setConnectTimeout(8000);
  http.setTimeout(8000);
  if (!http.begin(client, AIRSENTINELS_URL)) {
    latest.lastPostCode = -2; latest.lastPostResp = "begin fail";
    return -2;
  }
  http.addHeader("Content-Type", "application/json");
  http.addHeader("X-Device-Token", gDeviceToken);
  int code = http.POST(payload);
  String resp = http.getString();
  http.end();
  latest.lastPostCode = code;
  latest.lastPostResp = resp;
  return code;
}

// One POST cycle: queue the fresh reading, then drain the outbox FIFO. A server
// error (or transport failure) stops the drain and arms exponential backoff, so
// data survives a flaky 4G link instead of being dropped.
static void doPostCycle() {
  if (gDeviceToken.length() < 8) {
    Serial.println("[POST] No device token — set it via the captive portal.");
    latest.lastPostCode = -1; latest.lastPostResp = "no token";
    return;
  }
  outboxPush(buildPayload());                 // strict FIFO: this reading goes last
  if (WiFi.status() != WL_CONNECTED) return;  // keep buffering until WiFi returns
  if (millis() < gBackoffUntilMs) return;     // in backoff window: buffer only

  int drained = 0;
  bool anyOk = false;
  while (!outboxEmpty() && drained < DRAIN_PER_CYCLE) {
    esp_task_wdt_reset();
    int code = sendPayload(outboxPeek());
    if (code >= 200 && code < 300) {
      outboxPop(); drained++; anyOk = true; gPostOk++;
    } else {
      gPostFail++;
      Serial.printf("[POST] %d (outbox=%d) — stop drain\n", code, gObCount);
      applyBackoff();
      break;
    }
  }
  if (anyOk) {
    Serial.printf("[POST] flushed %d, outbox=%d\n", drained, gObCount);
    if (outboxEmpty()) resetBackoff();        // fully caught up
    otaConfirmSuccess();                       // a real POST proves a fresh image works
  }
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
.meta{color:#777;font-size:11px}a{color:#7af}
.tabs{display:inline-flex;gap:4px;background:#222;border-radius:8px;padding:4px;margin:0 0 8px}
.tab{padding:6px 12px;border-radius:6px;cursor:pointer;user-select:none;font-size:13px;color:#aaa}
.tab.active{background:#2a5a2a;color:#cfc;font-weight:600}
.tab .star{color:#ffa000;margin-left:4px}</style></head>
<body><h1>OpenAir NextPM — Local Dashboard</h1>
<div id="root">Loading…</div>
<div class="meta">Auto-refresh 2s · <a href="/json">/json</a> · <a href="/probe">/probe</a> · <a href="/s8scan">/s8scan</a> · <a href="/i2cscan">/i2cscan</a></div>
<script>
function esc(s){return (s==null?'':''+s).replace(/[&<>]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;'}[c]));}
function card(name,val,unit,ok){return '<div class="card"><div class="u">'+name+'</div><div class="v '+(ok===true?'ok':ok===false?'err':'')+'">'+val+'</div><div class="u">'+unit+'</div></div>';}

// Local display preference (independent from postAvgSec which drives the cloud POST)
let viewPeriod = parseInt(localStorage.getItem('viewPeriod')||'60');
if (![10,60,900].includes(viewPeriod)) viewPeriod = 60;
let cachedPostAvg = 60;

async function setPrimary(sec) {
  // Switch both the local view AND the cloud primary (so AirGradient matches).
  viewPeriod = sec;
  localStorage.setItem('viewPeriod', String(sec));
  try { await fetch('/setperiod?sec='+sec, {cache:'no-store'}); } catch(e){}
  refresh();
}
async function setViewOnly(sec) {
  viewPeriod = sec;
  localStorage.setItem('viewPeriod', String(sec));
  refresh();
}

function pickSample(d, sec) {
  if (sec==10)  return {pm1:d.pm_10s_pm1, pm25:d.pm_10s_pm25, pm10:d.pm_10s_pm10, cnt:d.pm_10s_cntPM1_dL, ok:d.pm_10s_ok, state:d.pm_10s_state};
  if (sec==900) return {pm1:d.pm_15m_pm1, pm25:d.pm_15m_pm25, pm10:d.pm_15m_pm10, cnt:d.pm_15m_cntPM1_dL, ok:d.pm_15m_ok, state:d.pm_15m_state};
  return           {pm1:d.pm_60s_pm1, pm25:d.pm_60s_pm25, pm10:d.pm_60s_pm10, cnt:d.pm_60s_cntPM1_dL, ok:d.pm_60s_ok, state:d.pm_60s_state};
}

async function refresh(){
 try {
  const r = await fetch('/json',{cache:'no-store'}); const d = await r.json();
  cachedPostAvg = d.postAvgSec;
  const sel = pickSample(d, viewPeriod);

  // Period tabs (click = switch view + cloud primary; shift-click = view only)
  const mkTab = (sec, lbl) => {
    const active = viewPeriod===sec ? ' active':'';
    const star   = cachedPostAvg===sec ? '<span class="star" title="Also the period sent to AirGradient">★</span>' : '';
    return '<span class="tab'+active+'" onclick="event.shiftKey?setViewOnly('+sec+'):setPrimary('+sec+')">'+lbl+star+'</span>';
  };
  let h = '<div class="tabs">' + mkTab(10,'10 s') + mkTab(60,'60 s') + mkTab(900,'15 min') + '</div>';
  h += '<div class="meta" style="margin-bottom:8px">★ = période envoyée au cloud · click = switch local + cloud · shift+click = switch local seulement</div>';

  h += '<div class="row">';
  h += card('PM1',   sel.pm1 ==null?'—':sel.pm1.toFixed(1),  'µg/m³', sel.ok);
  h += card('PM2.5', sel.pm25==null?'—':sel.pm25.toFixed(1), 'µg/m³', sel.ok);
  h += card('PM10',  sel.pm10==null?'—':sel.pm10.toFixed(1), 'µg/m³', sel.ok);
  h += card('PM0.3≈',sel.ok?sel.cnt:'—',                     '/dL',   sel.ok);
  h += card('CO₂',   d.co2Ok?d.co2:'—',                      'ppm',   d.co2Ok);
  h += card('Temp',  d.shtOk && d.atmp!=null?d.atmp.toFixed(1):'—','°C',    d.shtOk);
  h += card('RH',    d.shtOk && d.rhum!=null?d.rhum.toFixed(0):'—','%',     d.shtOk);
  h += card('TVOC',  d.sgpOk && !d.sgpConditioning?d.tvoc_index_avg:'—','index 10s',d.sgpOk && !d.sgpConditioning);
  h += card('NOx',   d.sgpOk && !d.sgpConditioning?d.nox_index_avg :'—','index 10s', d.sgpOk && !d.sgpConditioning);
  h += card('RSSI',  d.rssi,                                 'dBm',   true);
  h += '</div>';

  const fmt = s => s==null?'—':s.toFixed(1);
  const row = (lbl,s,sec)=> '<tr'+(viewPeriod==sec?' style="background:#1b3a1b;color:#cfc"':'')
    +'><td>'+lbl+(cachedPostAvg==sec?' ★':'')+'</td><td>'+fmt(s.pm1)+'</td><td>'+fmt(s.pm25)+'</td><td>'+fmt(s.pm10)
    +'</td><td>'+s.cntPM1_dL+'</td><td>'+(s.ok?'<span class=ok>ok</span>':'<span class=err>fail</span>')+'</td></tr>';
  h += '<div class="box"><b>NextPM</b> — vue actuelle : <b>'+(viewPeriod==10?'10 s':viewPeriod==900?'15 min':'60 s')+'</b>'
     + ' · cloud : <b>'+(cachedPostAvg==10?'10 s':cachedPostAvg==900?'15 min':'60 s')+'</b>'
     + '<table style="width:100%;margin-top:6px;font-size:12px;border-collapse:collapse"><thead><tr style="color:#888">'
     + '<th style="text-align:left">Moyennage</th><th>PM1</th><th>PM2.5</th><th>PM10</th><th>cnt≥0.3µm /dL</th><th>ok</th></tr></thead><tbody>'
     + row('10 s',  {pm1:d.pm_10s_pm1,pm25:d.pm_10s_pm25,pm10:d.pm_10s_pm10,cntPM1_dL:d.pm_10s_cntPM1_dL,ok:d.pm_10s_ok}, 10)
     + row('60 s',  {pm1:d.pm_60s_pm1,pm25:d.pm_60s_pm25,pm10:d.pm_60s_pm10,cntPM1_dL:d.pm_60s_cntPM1_dL,ok:d.pm_60s_ok}, 60)
     + row('15 min',{pm1:d.pm_15m_pm1,pm25:d.pm_15m_pm25,pm10:d.pm_15m_pm10,cntPM1_dL:d.pm_15m_cntPM1_dL,ok:d.pm_15m_ok}, 900)
     + '</tbody></table>'
     + '<div class="u" style="margin-top:4px">state=0x'+d.nextpmState.toString(16).padStart(2,'0')
     + ' · gas samples=' + d.gas_samples + ' · missed ticks=' + d.gas_missed
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
  j += ",\"tvoc_index\":"     + String(latest.vocIndex);
  j += ",\"nox_index\":"      + String(latest.noxIndex);
  j += ",\"tvoc_index_avg\":" + String(latest.vocIndexAvg);
  j += ",\"nox_index_avg\":"  + String(latest.noxIndexAvg);
  j += ",\"gas_samples\":"    + String(latest.gasSamples);
  j += ",\"gas_missed\":"     + String(gasMissedTicks);
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
  if (!i2cMutex) i2cMutex = xSemaphoreCreateMutex();
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

// Read SHT4x (I2C mutex held by caller)
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

// Sample SGP41 at 1 Hz (I2C mutex held by caller). During first SGP41_COND_MS
// use executeConditioning, then measureRawSignals.
static bool sampleSGP() {
  if (!latest.sgpPresent) return false;

  // Compensation words: default 25 °C / 50 % RH if no SHT
  uint16_t compRh = 0x8000;
  uint16_t compT  = 0x6666;
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
  latest.gasSamples++;

  int32_t voc = vocAlgo.process((int32_t)srawVoc);
  int32_t nox = conditioning ? 0 : noxAlgo.process((int32_t)srawNox);
  latest.vocIndex = voc;
  latest.noxIndex = nox;

  // Rolling 10 s average (suppresses display sawtooth)
  vocHist[gasHistIdx] = voc;
  noxHist[gasHistIdx] = nox;
  gasHistIdx = (gasHistIdx + 1) % GAS_WIN;
  if (gasHistCount < GAS_WIN) gasHistCount++;
  int64_t vs = 0, ns = 0;
  for (int i = 0; i < gasHistCount; i++) { vs += vocHist[i]; ns += noxHist[i]; }
  latest.vocIndexAvg = (int32_t)(vs / gasHistCount);
  latest.noxIndexAvg = (int32_t)(ns / gasHistCount);
  return true;
}

// Dedicated 1 Hz FreeRTOS task so I/O-heavy activities in loop() never
// stretch the gas-index algorithm cadence (which is what makes TVOC/NOx
// look like a sawtooth).
static void gasSensorTask(void*) {
  esp_task_wdt_add(NULL);   // a wedged I2C read here also triggers a recovery reboot
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(GAS_SAMPLE_MS);
  for (;;) {
    esp_task_wdt_reset();
    if (i2cInitialized) {
      i2cLock();
      sampleSHT();
      sampleSGP();
      i2cUnlock();
    }
    // Count a "miss" only when we're more than half a tick behind schedule —
    // i.e. the sampling couldn't keep up with the 1 Hz cadence.
    TickType_t now = xTaskGetTickCount();
    TickType_t scheduled = lastWake + period;
    if ((int32_t)(now - scheduled) > (int32_t)(period / 2)) {
      gasMissedTicks++;
    }
    vTaskDelayUntil(&lastWake, period);
  }
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

  // Block the gas task while we mess with the bus pins.
  i2cLock();
  for (int p = 0; p < N; p++) {
    int sda = pairs[p][0], scl = pairs[p][1];
    out += "\n[SDA=GPIO" + String(sda) + " SCL=GPIO" + String(scl) + "]\n";

    Wire.end();
    delay(10);
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

  // Restore normal I2C config for the gas task
  Wire.end();
  delay(10);
  Wire.begin(I2C_SDA, I2C_SCL, 100000);
  i2cUnlock();
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

// Rotate the AirSentinels device token over the LAN (no reflash). Persisted NVS.
static void handleSetToken() {
  if (!webServer.hasArg("token")) {
    webServer.send(400, "text/plain",
      String("missing ?token=...\ncurrent: ") + (gDeviceToken.length() ? "set" : "MISSING") + "\n");
    return;
  }
  String t = webServer.arg("token"); t.trim();
  if (t.length() < 8) { webServer.send(400, "text/plain", "token too short (>=8)\n"); return; }
  gDeviceToken = t;
  saveToken(gDeviceToken);
  webServer.send(200, "text/plain",
    "Token saved (" + String(t.length()) + " chars). Next POST uses it.\n");
}

// Switch the station to a new Wi-Fi over the LAN (e.g. rotate a hotspot password
// across the fleet). Creds persist to NVS; the device will reconnect on its own.
static void handleSetWifi() {
  if (!webServer.hasArg("ssid")) {
    webServer.send(400, "text/plain", "missing ?ssid=...&pass=...\n");
    return;
  }
  String ssid = webServer.arg("ssid");
  String pass = webServer.hasArg("pass") ? webServer.arg("pass") : "";
  webServer.send(200, "text/plain",
    "Switching to SSID '" + ssid + "'. The device changes networks now — "
    "reconnect to its new IP (see /macinfo on the new network or the dashboard telemetry).\n");
  delay(200);
  WiFi.persistent(true);
  WiFi.begin(ssid.c_str(), pass.c_str());  // saved to NVS, used on reconnect
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

// Diagnostic: send one simple-protocol command and dump the raw response, to
// check which channels a given NextPM unit/firmware actually supports.
// Read-only allowlist so we never hit a config/sleep/write command by mistake.
//   /nextpmcmd?cmd=26   -> sends {0x81,0x26,chk}, dumps up to 32 bytes
static void handleNextpmCmd() {
  if (!webServer.hasArg("cmd")) {
    webServer.send(400, "text/plain",
      "missing ?cmd=<hex>. Allowed (read-only): 11 12 13 (mass) 25 26 27 (bins) 16 (T/RH)\n");
    return;
  }
  long cmd = strtol(webServer.arg("cmd").c_str(), nullptr, 16);
  const long allowed[] = { 0x11, 0x12, 0x13, 0x25, 0x26, 0x27, 0x16 };
  bool ok = false;
  for (long a : allowed) if (a == cmd) ok = true;
  if (!ok) { webServer.send(400, "text/plain", "cmd not in read-only allowlist\n"); return; }

  while (NextPMSerial.available()) NextPMSerial.read();
  bool sent = nextpmSendSimpleCmd((uint8_t)cmd);
  const uint32_t t0 = millis();
  uint8_t buf[32]; size_t got = 0;
  while (got < sizeof(buf) && millis() - t0 < 700) {
    if (NextPMSerial.available()) buf[got++] = (uint8_t)NextPMSerial.read();
  }
  String out = "cmd=0x" + String((uint32_t)cmd, HEX) + " sent=" + (sent ? "ok" : "FAIL") +
               " got=" + String((unsigned)got) + " bytes\n" + bytesToHex(buf, got) + "\n";
  webServer.sendHeader("Cache-Control", "no-store");
  webServer.send(200, "text/plain", out);
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

// ==================== OTA pull ========================
// Minimal JSON field extractor (no ArduinoJson dependency). Handles
// "key":"string" and "key":number. Returns "" if not found.
static String jsonField(const String& json, const char* key) {
  String k = String("\"") + key + "\"";
  int i = json.indexOf(k);
  if (i < 0) return String("");
  i = json.indexOf(':', i + k.length());
  if (i < 0) return String("");
  i++;
  while (i < (int)json.length() && (json[i] == ' ' || json[i] == '\t')) i++;
  if (i >= (int)json.length()) return String("");
  if (json[i] == '"') {
    int j = json.indexOf('"', i + 1);
    if (j < 0) return String("");
    return json.substring(i + 1, j);
  }
  int j = i;
  while (j < (int)json.length() &&
         json[j] != ',' && json[j] != '}' && json[j] != ' ' && json[j] != '\n' && json[j] != '\r') j++;
  return json.substring(i, j);
}

// Called once at boot to reconcile NVS OTA marker with the running version.
static void otaInitBootState() {
  prefs.begin("ag", true);
  String tried = prefs.getString("otaver", "");
  prefs.end();
  if (tried.length() == 0) return;
  if (tried == String(FW_VERSION)) {
    // We're running exactly the version we just installed → success path, but
    // we only *confirm* (cancel rollback) after a real POST succeeds.
    gOtaPendingConfirm = true;
    Serial.printf("[OTA] running freshly installed %s — awaiting POST to confirm\n", FW_VERSION);
  } else {
    // Running a different version than the one we tried to install: the new
    // image never took (or the bootloader already rolled us back). Clear it.
    prefs.begin("ag", false); prefs.remove("otaver"); prefs.end();
    Serial.printf("[OTA] cleared stale install marker (%s, running %s)\n", tried.c_str(), FW_VERSION);
  }
}

// Confirm the freshly installed image is healthy (called after a successful POST).
static void otaConfirmSuccess() {
  if (!gOtaPendingConfirm) return;
  gOtaPendingConfirm = false;
  prefs.begin("ag", false); prefs.remove("otaver"); prefs.end();
  const esp_partition_t* running = esp_ota_get_running_partition();
  esp_ota_img_states_t st;
  if (running && esp_ota_get_state_partition(running, &st) == ESP_OK &&
      st == ESP_OTA_IMG_PENDING_VERIFY) {
    esp_ota_mark_app_valid_cancel_rollback();
    Serial.println("[OTA] image confirmed valid (rollback cancelled)");
  } else {
    Serial.println("[OTA] image confirmed (no pending-verify state)");
  }
}

// If a fresh image never produced a successful POST within the timeout, revert
// to the previous slot. Works even when bootloader rollback isn't compiled in,
// by manually re-pointing the boot partition to the other OTA slot.
static void otaRollbackIfStuck() {
  if (!gOtaPendingConfirm) return;
  if (millis() - gBootMs < OTA_CONFIRM_TIMEOUT_MS) return;
  Serial.println("[OTA] fresh image failed to confirm in time — rolling back");
  prefs.begin("ag", false); prefs.remove("otaver"); prefs.end();
  const esp_partition_t* running = esp_ota_get_running_partition();
  esp_ota_img_states_t st;
  if (running && esp_ota_get_state_partition(running, &st) == ESP_OK &&
      st == ESP_OTA_IMG_PENDING_VERIFY) {
    esp_ota_mark_app_invalid_rollback_and_reboot();  // does not return
  }
  // Fallback: point boot at the other slot (holds the previous firmware) and reboot.
  const esp_partition_t* other = esp_ota_get_next_update_partition(NULL);
  if (other) esp_ota_set_boot_partition(other);
  delay(100);
  ESP.restart();
}

static void otaCheckAndApply() {
  if (WiFi.status() != WL_CONNECTED) return;
  // Don't chase a new update until the current fresh image is confirmed.
  if (gOtaPendingConfirm) return;

  WiFiClientSecure client;
  client.setCACert(ISRG_ROOT_X1_PEM);
  HTTPClient http;
  http.setConnectTimeout(8000);
  http.setTimeout(8000);
  if (!http.begin(client, OTA_MANIFEST_URL)) return;
  int code = http.GET();
  if (code != 200) { Serial.printf("[OTA] manifest HTTP %d\n", code); http.end(); return; }
  String body = http.getString();
  http.end();

  String version = jsonField(body, "version");
  String url     = jsonField(body, "url");
  String md5     = jsonField(body, "md5");
  long   size    = jsonField(body, "size").toInt();
  if (version.length() == 0 || url.length() == 0) { Serial.println("[OTA] bad manifest"); return; }
  if (version == String(FW_VERSION)) { Serial.printf("[OTA] up to date (%s)\n", FW_VERSION); return; }

  Serial.printf("[OTA] update %s -> %s, downloading %s\n", FW_VERSION, version.c_str(), url.c_str());
  WiFiClientSecure dlc;
  dlc.setCACert(ISRG_ROOT_X1_PEM);
  HTTPClient dl;
  dl.setConnectTimeout(8000);
  dl.setTimeout(20000);   // larger read window for the ~1.3 MB image
  if (!dl.begin(dlc, url)) { Serial.println("[OTA] dl begin fail"); return; }
  int dcode = dl.GET();
  if (dcode != 200) { Serial.printf("[OTA] bin HTTP %d\n", dcode); dl.end(); return; }

  int len = dl.getSize();
  if (size > 0) len = (int)size;
  if (!Update.begin(len > 0 ? len : UPDATE_SIZE_UNKNOWN)) {
    Serial.printf("[OTA] Update.begin fail: %s\n", Update.errorString());
    dl.end(); return;
  }
  if (md5.length() == 32) Update.setMD5(md5.c_str());  // verified during write

  esp_task_wdt_reset();  // the stream write can take ~10-20 s
  WiFiClient* stream = dl.getStreamPtr();
  size_t written = Update.writeStream(*stream);
  dl.end();
  esp_task_wdt_reset();
  Serial.printf("[OTA] wrote %u bytes\n", (unsigned)written);

  if (!Update.end(true)) {
    Serial.printf("[OTA] Update.end fail: %s\n", Update.errorString());
    return;  // MD5/size mismatch or write error → nothing flashed to boot slot
  }
  prefs.begin("ag", false); prefs.putString("otaver", version); prefs.end();
  Serial.println("[OTA] update OK — rebooting into new image");
  delay(200);
  ESP.restart();
}

// ==================== Setup / Loop ====================
void setup() {
  Serial.begin(115200);
  delay(200);
  gBootMs = millis();
  Serial.printf("\nBoot — firmware %s (reset=%s)\n", FW_VERSION, resetReasonStr());

  // Task watchdog: reboot if the loop or gas task stops feeding it (I2C lockup,
  // UART stall, wedged TLS handshake). The Arduino core may already have TWDT
  // running — reconfigure rather than fail.
  esp_task_wdt_config_t wdtCfg = { .timeout_ms = WDT_TIMEOUT_S * 1000,
                                   .idle_core_mask = 0, .trigger_panic = true };
  if (esp_task_wdt_init(&wdtCfg) == ESP_ERR_INVALID_STATE) esp_task_wdt_reconfigure(&wdtCfg);
  esp_task_wdt_add(NULL);

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);

  gSensorIdFull = loadSavedSensorId();
  if (gSensorIdFull.length() == 0) {
    gSensorIdFull = String("airgradient:") + agSerial12();
    saveSensorId(gSensorIdFull);
  }
  latest.postAvgSec = loadSavedPostAvgSec();
  gDeviceToken = loadSavedToken();
  if (gDeviceToken.length() == 0 && strlen(DEVICE_TOKEN_DEFAULT) > 0) {
    gDeviceToken = DEVICE_TOKEN_DEFAULT;
    saveToken(gDeviceToken);
    Serial.println("[TOKEN] using compile-time default");
  }
  otaInitBootState();
  Serial.printf("DeviceID (%s) serial=%s postAvgSec=%u s token=%s\n",
                gSensorIdFull.c_str(), deviceSerial12().c_str(), latest.postAvgSec,
                gDeviceToken.length() ? "set" : "MISSING");

  NextPMSerial.begin(115200, SERIAL_8E1, NEXTPM_RX_PIN, NEXTPM_TX_PIN);
  NextPMSerial.setTimeout(60);
  S8Serial.begin(9600, SERIAL_8N1, S8_RX_PIN, S8_TX_PIN);
  S8Serial.setTimeout(50);

  Serial.printf("NextPM: RX=%d TX=%d (115200 8E1)\n", NEXTPM_RX_PIN, NEXTPM_TX_PIN);
  Serial.printf("S8   : RX=%d TX=%d (9600  8N1)\n", S8_RX_PIN, S8_TX_PIN);
  Serial.printf("I2C  : SDA=%d SCL=%d\n", I2C_SDA, I2C_SCL);

  initI2CSensors();
  if (i2cInitialized) {
    xTaskCreatePinnedToCore(gasSensorTask, "gas", 4096, nullptr, 2, nullptr, tskNO_AFFINITY);
  }

  ensureWifiConnected();

  if (WiFi.status() == WL_CONNECTED) {
    // NTP so we can stamp ingestion with real UTC time. Non-blocking: the gas
    // task and POST loop keep running; isoUtcNow() returns "" until synced and
    // the server stamps in the meantime.
    configTime(0, 0, "pool.ntp.org", "time.google.com");
    struct tm tmv;
    if (getLocalTime(&tmv, 5000)) {
      gTimeSynced = true;
      Serial.printf("NTP synced: %04d-%02d-%02dT%02d:%02d:%02dZ\n",
                    tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday,
                    tmv.tm_hour, tmv.tm_min, tmv.tm_sec);
    } else {
      Serial.println("NTP not synced yet (server will stamp ts)");
    }

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
    webServer.on("/settoken", handleSetToken);
    webServer.on("/setwifi", handleSetWifi);
    webServer.on("/nextpmcmd", handleNextpmCmd);
    webServer.on("/macinfo", handleMacInfo);
    webServer.on("/i2cscan", handleI2CScan);
    webServer.on("/setperiod", handleSetPeriod);
    webServer.begin();
    Serial.printf("HTTP server up: http://%s/\n", WiFi.localIP().toString().c_str());
  }
}

void loop() {
  static uint32_t tLastPost = 0, tTick = 0, tLastGas = 0, tLastOta = 0,
                  tWifiChk = 0, tNtp = 0;
  esp_task_wdt_reset();
  webServer.handleClient();

  if (millis() - tTick > 2000) {
    tTick = millis();
    Serial.printf("[tick] ip=%s rssi=%d heap=%u outbox=%d\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI(),
                  (unsigned)ESP.getFreeHeap(), gObCount);
  }

  // Gas sampling lives in its own FreeRTOS task (see gasSensorTask).
  (void)tLastGas;

  // WiFi keep-alive: non-blocking reconnect attempts when the link drops (4G
  // hotspot reboot, range). Never re-opens the blocking captive portal here.
  if (millis() - tWifiChk > WIFI_CHECK_MS) {
    tWifiChk = millis();
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[WiFi] down — reconnecting");
      if (strlen(WIFI_SSID_DEFAULT) > 0) WiFi.begin(WIFI_SSID_DEFAULT, WIFI_PASS_DEFAULT);
      else WiFi.reconnect();
    }
  }

  // NTP: resync periodically (clock drift over weeks); retry faster until synced.
  uint32_t ntpDue = gTimeSynced ? NTP_RESYNC_OK_MS : NTP_RETRY_MS;
  if (WiFi.status() == WL_CONNECTED && millis() - tNtp > ntpDue) {
    tNtp = millis();
    configTime(0, 0, "pool.ntp.org", "time.google.com");
    struct tm tmv;
    if (getLocalTime(&tmv, 3000)) gTimeSynced = true;
  }

  // OTA: first check shortly after boot, then on a slow cadence. Roll back a
  // fresh image that never managed a successful POST.
  otaRollbackIfStuck();
  uint32_t otaDue = (tLastOta == 0) ? OTA_FIRST_CHECK_MS : OTA_CHECK_PERIOD_MS;
  if (WiFi.status() == WL_CONNECTED && millis() - tLastOta > otaDue) {
    tLastOta = millis();
    otaCheckAndApply();
  }

  if (millis() - tLastPost > POST_PERIOD_MS + gCycleJitterMs) {
    tLastPost = millis();
    gCycleJitterMs = esp_random() % 5000;  // 0–5 s, re-rolled each cycle: desyncs a shared hotspot

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

    // Granulometry bins via the simple protocol (0x25/0x26/0x27), matching the
    // selected averaging window. Fields c02_05.. carry the 0.3-0.5 .. 5-10 µm bins.
    float c02_05 = 0, c05_10 = 0, c10_25 = 0, c25_50 = 0, c50_100 = 0;
    String rawBins;
    uint8_t binCmd = (latest.postAvgSec == 10) ? 0x25 : (latest.postAvgSec == 900) ? 0x27 : 0x26;
    delay(80);
    bool okBins = nextpmReadBinsCmd(binCmd, c02_05, c05_10, c10_25, c25_50, c50_100, rawBins);
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
      doPostCycle();   // buffers + drains the outbox; confirms OTA on success
    }
  }
}
