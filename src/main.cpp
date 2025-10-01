explain what this code is about
/*
  ESP32-C3 + NextPM (PM2) + Senseair S8 (PM1) -> AirGradient Cloud
  JSON payload: wifi, pm01, pm02, pm10 (µg/m3), pm003Count (pcs/dL), rco2 (ppm)

  NextPM:
    - UART 115200 8E1
    - Simple protocol 0x12: PM1/PM2.5/PM10 (µg/m3) 1‑min average
    - Modbus 0x03 registers 128..137: 5 counting channels (Cnt/L) 10‑s average
      0.2–0.5, 0.5–1.0, 1.0–2.5, 2.5–5.0, 5.0–10.0 µm (LSW then MSW)

  S8:
    - UART 9600 8N1
    - CO2 read via standard Modbus‑like frame (register 0x0003)

  Cloud:
    POST http://hw.airgradient.com/sensors/airgradient:<serial>/measures
    <serial> = STA MAC in lowercase without ':'
*/

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClient.h>
#include <WiFiManager.h>
#include <HardwareSerial.h>
#include <Preferences.h>
#include <math.h>
#include <string.h>

// -------------------- Pins / UART --------------------
// NextPM on UART1 (MCU RX pin receives from sensor; TX pin goes to sensor)
constexpr int NEXTPM_RX_PIN = 0;
constexpr int NEXTPM_TX_PIN = 1;
// Senseair S8 on UART0
constexpr int S8_RX_PIN     = 20;
constexpr int S8_TX_PIN     = 21;

HardwareSerial NextPMSerial(1);    // UART1 -> NextPM (115200 8E1)
HardwareSerial S8Serial(0);        // UART0 -> Senseair S8 (9600 8N1)

// -------------------- Settings -----------------------
// NextPM Modbus address (1 by default)
constexpr uint8_t NEXTPM_MODBUS_ADDR = 0x01;

// Include a fraction of the 0.2–0.5 channel in PM >= 0.3 (0.0 = conservative)
constexpr float PM003_FROM_02_05_FRACTION = 0.0f;

// Periods
constexpr uint32_t POST_PERIOD_MS = 10000; // 10 s

// -------------------- AirGradient ID (configurable) --
Preferences prefs;
String gSensorIdFull;  // "airgradient:xxxxxxxxxxxx"

// Build 12‑hex STA MAC string "aabbccddeeff"
String agSerial12() {
  if (WiFi.getMode() != WIFI_STA && WiFi.getMode() != WIFI_AP_STA) {
    WiFi.mode(WIFI_STA);
    delay(10);
  }
  String mac = WiFi.macAddress(); // "AA:BB:CC:DD:EE:FF"
  mac.toLowerCase();
  mac.replace(":", "");
  return mac; // "aabbccddeeff"
}

// Normalize user input into "airgradient:xxxxxxxxxxxx". If empty/invalid -> fallback MAC.
String normalizeSensorId(const String& userInput, const String& fallback12hex) {
  String s = userInput; s.trim(); s.toLowerCase();
  if (s.startsWith("airgradient:")) s.remove(0, 12);
  s.replace(":", ""); s.replace(" ", "");

  String hex;
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if ((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f')) hex += c;
  }
  if (hex.length() != 12) {
    return String("airgradient:") + fallback12hex;
  }
  return String("airgradient:") + hex;
}

// Load saved Sensor ID from NVS (Preferences). Returns empty string if not set.
String loadSavedSensorId() {
  prefs.begin("ag", true);
  String id = prefs.getString("id", "");
  prefs.end();
  return id;
}

// Save Sensor ID to NVS (Preferences).
void saveSensorId(const String& fullId) {
  prefs.begin("ag", false);
  prefs.putString("id", fullId);
  prefs.end();
}

// -------------------- Wi‑Fi / Portal -----------------
// Ensure Wi‑Fi connection; opens captive portal with a custom "Sensor ID" field when needed.
bool ensureWifiConnected() {
  if (WiFi.status() == WL_CONNECTED) return true;

  WiFi.mode(WIFI_STA);
  WiFiManager wm;

  // Custom "Sensor ID" text field
  char idbuf[40];
  strncpy(idbuf, gSensorIdFull.c_str(), sizeof(idbuf));
  idbuf[sizeof(idbuf)-1] = 0;

  WiFiManagerParameter idParam(
      "sensorid",
      "Sensor ID (airgradient:xxxxxxxxxxxx or 12 hex)",
      idbuf,
      sizeof(idbuf)-1
  );
  wm.addParameter(&idParam);

  // Shorten SSID by using the tail of the MAC for readability
  String apName = "airgradient-" + agSerial12().substring(6);
  const char* apPass = "cleanair";
  wm.setConfigPortalTimeout(180);

  Serial.printf("Portail Wi‑Fi SSID=%s pass=%s (tu peux saisir/éditer le Sensor ID)\n",
                apName.c_str(), apPass);

  bool ok = wm.autoConnect(apName.c_str(), apPass);

  // Read the entered value and normalize it (even if connection failed)
  String entered = String(idParam.getValue());
  String normalized = normalizeSensorId(entered, agSerial12());
  if (normalized != gSensorIdFull) {
    gSensorIdFull = normalized;
    saveSensorId(gSensorIdFull);
    Serial.printf("[ID] Nouveau Sensor ID: %s\n", gSensorIdFull.c_str());
  } else {
    Serial.printf("[ID] Sensor ID conservé: %s\n", gSensorIdFull.c_str());
  }

  if (ok) {
    Serial.printf("Wi‑Fi connecté: SSID=%s IP=%s RSSI=%d dBm\n",
                  WiFi.SSID().c_str(), WiFi.localIP().toString().c_str(), WiFi.RSSI());
  } else {
    Serial.println("Wi‑Fi non configuré (timeout portail).");
  }
  return ok;
}

// ==================== CRC / Checksum utils ============
static uint16_t crc16_modbus(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i=0;i<len;i++) {
    crc ^= data[i];
    for (int b=0;b<8;b++) {
      if (crc & 1) crc = (crc >> 1) ^ 0xA001;
      else         crc >>= 1;
    }
  }
  return crc; // Low byte first in frames
}

// Simple protocol checksum: sum % 256 == 0
static inline uint8_t nextpmChecksum(const uint8_t* b, size_t len) {
  uint32_t s = 0; for (size_t i=0;i<len;i++) s += b[i];
  return (uint8_t)((256 - (s % 256)) & 0xFF);
}

// Send a simple‑protocol command to NextPM
static bool nextpmSendSimpleCmd(uint8_t cmd) {
  uint8_t frame[3] = {0x81, cmd, 0x00};
  frame[2] = nextpmChecksum(frame, 2);
  size_t n = NextPMSerial.write(frame, sizeof(frame));
  NextPMSerial.flush();
  return n == sizeof(frame);
}

// ==================== NextPM: Simple protocol =========
// Read 1‑min mass averages in µg/m3 (cmd 0x12)
static bool nextpmReadMass_1min(float& pm1ug, float& pm25ug, float& pm10ug, uint8_t* outState=nullptr) {
  pm1ug = pm25ug = pm10ug = NAN;
  while (NextPMSerial.available()) NextPMSerial.read();

  if (!nextpmSendSimpleCmd(0x12)) return false;

  const uint32_t t0 = millis();
  const uint32_t timeoutMs = 250;
  uint8_t buf[16]; size_t got = 0;
  while (got < sizeof(buf) && millis() - t0 < timeoutMs) {
    if (NextPMSerial.available()) buf[got++] = (uint8_t)NextPMSerial.read();
  }
  if (got != sizeof(buf)) return false;
  if (buf[0] != 0x81 || buf[1] != 0x12) return false;
  if (nextpmChecksum(buf, 15) != buf[15]) return false;

  if (outState) *outState = buf[2];

  auto U16 = [](const uint8_t* p){ return (uint16_t)((p[0]<<8) | p[1]); };
  // Counts per mL: bytes [3..8], masses x0.1 µg/m3: bytes [9..14]
  pm1ug  = U16(&buf[9])  / 10.0f;
  pm25ug = U16(&buf[11]) / 10.0f;
  pm10ug = U16(&buf[13]) / 10.0f;
  return true;
}

// ==================== NextPM: Modbus ==================
// Read 'qty' holding registers starting at regStart (1‑based)
static bool nextpmReadHolding(uint16_t regStart, uint16_t qty, uint16_t* outWords) {
  while (NextPMSerial.available()) NextPMSerial.read();

  uint8_t req[8];
  req[0] = NEXTPM_MODBUS_ADDR;
  req[1] = 0x03; // Read Holding Registers
  req[2] = (uint8_t)(regStart >> 8);
  req[3] = (uint8_t)(regStart & 0xFF);
  req[4] = (uint8_t)(qty >> 8);
  req[5] = (uint8_t)(qty & 0xFF);
  uint16_t crc = crc16_modbus(req, 6);
  req[6] = (uint8_t)(crc & 0xFF);        // CRC low
  req[7] = (uint8_t)((crc >> 8) & 0xFF); // CRC high

  if (NextPMSerial.write(req, sizeof(req)) != sizeof(req)) return false;
  NextPMSerial.flush();

  // Response: addr, func, byteCount (=qty*2), ...data..., CRClo, CRChi
  const uint32_t t0 = millis();
  const uint32_t timeoutMs = 300;

  uint8_t hdr[3]; size_t got = 0;
  while (got < 3 && millis() - t0 < timeoutMs) {
    if (NextPMSerial.available()) hdr[got++] = (uint8_t)NextPMSerial.read();
  }
  if (got != 3) return false;
  if (hdr[0] != NEXTPM_MODBUS_ADDR || hdr[1] != 0x03) return false;
  uint8_t byteCount = hdr[2];
  if (byteCount != qty*2) return false;

  const size_t rem = byteCount + 2; // data + CRC
  uint8_t buf[256];
  if (rem > sizeof(buf)) return false;
  got = 0;
  while (got < rem && millis() - t0 < timeoutMs) {
    if (NextPMSerial.available()) buf[got++] = (uint8_t)NextPMSerial.read();
  }
  if (got != rem) return false;

  // CRC check
  uint8_t crcFrame[3+256];
  memcpy(crcFrame, hdr, 3);
  memcpy(crcFrame+3, buf, byteCount);
  uint16_t crcCalc = crc16_modbus(crcFrame, 3+byteCount);
  uint16_t crcRx   = (uint16_t)buf[byteCount] | ((uint16_t)buf[byteCount+1] << 8);
  if (crcCalc != crcRx) return false;

  // Unpack big‑endian words into outWords[0..qty-1]
  for (uint16_t i=0;i<qty;i++) {
    outWords[i] = ((uint16_t)buf[2*i] << 8) | buf[2*i+1];
  }
  return true;
}

// Read the 5 counting channels (Cnt/L, 10 s avg), registers 128..137
// Returns true on success; fills c02_05 ... c50_100 as floats (Cnt/L)
static bool nextpmReadBinnedCounts(float& c02_05, float& c05_10, float& c10_25,
                                   float& c25_50, float& c50_100) {
  uint16_t words[10] = {0};
  if (!nextpmReadHolding(128, 10, words)) return false;

  auto U32_from_words_LSW_MSW = [](uint16_t lsw, uint16_t msw) -> uint32_t {
    return (uint32_t)lsw | ((uint32_t)msw << 16);
  };

  uint32_t q02_05 = U32_from_words_LSW_MSW(words[0], words[1]);
  uint32_t q05_10 = U32_from_words_LSW_MSW(words[2], words[3]);
  uint32_t q10_25 = U32_from_words_LSW_MSW(words[4], words[5]);
  uint32_t q25_50 = U32_from_words_LSW_MSW(words[6], words[7]);
  uint32_t q50_100= U32_from_words_LSW_MSW(words[8], words[9]);

  // Convert to float Cnt/L
  c02_05  = (float)q02_05;
  c05_10  = (float)q05_10;
  c10_25  = (float)q10_25;
  c25_50  = (float)q25_50;
  c50_100 = (float)q50_100;
  return true;
}

// ==================== Senseair S8 =====================
// Read CO2 via Modbus‑like standard command (register 0x0003)
static bool s8ReadCO2(uint16_t& co2ppm) {
  co2ppm = 0;
  while (S8Serial.available()) S8Serial.read();

  const uint8_t cmd[8] = {0xFE, 0x04, 0x00, 0x03, 0x00, 0x01, 0xD5, 0xC5};
  if (S8Serial.write(cmd, sizeof(cmd)) != sizeof(cmd)) return false;
  S8Serial.flush();

  const uint32_t t0 = millis();
  const uint32_t timeoutMs = 200;
  uint8_t resp[7]; size_t got = 0;
  while (got < sizeof(resp) && millis() - t0 < timeoutMs) {
    if (S8Serial.available()) resp[got++] = (uint8_t)S8Serial.read();
  }
  if (got != sizeof(resp)) return false;
  if (resp[0] != 0xFE || resp[1] != 0x04 || resp[2] != 0x02) return false;

  // CRC check
  uint16_t crcCalc = crc16_modbus(resp, 5);
  uint16_t crcRx   = (uint16_t)resp[5] | ((uint16_t)resp[6] << 8);
  if (crcCalc != crcRx) return false;

  co2ppm = ((uint16_t)resp[3] << 8) | resp[4];
  return true;
}

// ==================== POST -> AirGradient =============
// Build JSON with only valid fields and POST it to AirGradient Cloud
static bool postToAirGradient(float pm01, float pm02, float pm10,
                              int pm003Count_dL, int rco2ppm) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[POST] Pas de Wi‑Fi. Portail…");
    if (!ensureWifiConnected()) return false;
  }

  String url = String("http://hw.airgradient.com/sensors/") + gSensorIdFull + "/measures";

  // Build JSON: include only valid values
  String payload = String("{\"wifi\":") + WiFi.RSSI();
  if (!isnan(pm01))           payload += String(",\"pm01\":") + String(pm01, 1);
  if (!isnan(pm02))           payload += String(",\"pm02\":") + String(pm02, 1);
  if (!isnan(pm10))           payload += String(",\"pm10\":") + String(pm10, 1);
  if (pm003Count_dL >= 0)     payload += String(",\"pm003Count\":") + pm003Count_dL;
  if (rco2ppm >= 0)           payload += String(",\"rco2\":") + rco2ppm;
  payload += "}";

  WiFiClient client;
  HTTPClient http;
  http.begin(client, url);
  http.addHeader("content-type", "application/json");

  int code = http.POST(payload);
  String resp = http.getString();
  http.end();

  Serial.printf("[POST] %s -> %d %s\n", url.c_str(), code, resp.c_str());
  return (code >= 200 && code < 300);
}

// ==================== Setup / Loop ====================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nBoot");

  WiFi.mode(WIFI_STA);

  // Load ID from NVS or initialize with MAC‑based default
  gSensorIdFull = loadSavedSensorId();
  if (gSensorIdFull.length() == 0) {
    gSensorIdFull = String("airgradient:") + agSerial12();
    saveSensorId(gSensorIdFull);
  }
  Serial.printf("DeviceID (%s)\n", gSensorIdFull.c_str());

  // Sensor UARTs
  NextPMSerial.begin(115200, SERIAL_8E1, NEXTPM_RX_PIN, NEXTPM_TX_PIN);
  NextPMSerial.setTimeout(60); // >50 ms recommended
  S8Serial.begin(9600, SERIAL_8N1, S8_RX_PIN, S8_TX_PIN);
  S8Serial.setTimeout(50);

  Serial.printf("NextPM: RX=%d TX=%d (115200 8E1)\n", NEXTPM_RX_PIN, NEXTPM_TX_PIN);
  Serial.printf("S8   : RX=%d TX=%d (9600  8N1)\n", S8_RX_PIN, S8_TX_PIN);

  ensureWifiConnected();
}

void loop() {
  static uint32_t tLastPost = 0, tTick = 0;

  // Heartbeat every second
  if (millis() - tTick > 1000) { tTick = millis(); Serial.println("[tick]"); }

  // Periodic data read + POST
  if (millis() - tLastPost > POST_PERIOD_MS) {
    tLastPost = millis();

    // 1) 1‑min mass averages (µg/m3)
    float pm1 = NAN, pm25 = NAN, pm10 = NAN;
    uint8_t st = 0xFF;
    bool okMass = nextpmReadMass_1min(pm1, pm25, pm10, &st);
    if (okMass) {
      Serial.printf("Mass 1min: PM1=%.1f PM2.5=%.1f PM10=%.1f  state=0x%02X\n", pm1, pm25, pm10, st);
    } else {
      Serial.println("NextPM mass: lecture échouée");
    }

    // 2) 10‑s binned counts (Cnt/L) -> PM >= 0.3 proxy (Cnt/dL)
    float c02_05=0, c05_10=0, c10_25=0, c25_50=0, c50_100=0;
    bool okBins = nextpmReadBinnedCounts(c02_05, c05_10, c10_25, c25_50, c50_100);
    int pm003_dL = -1;
    if (okBins) {
      double pm003_L = PM003_FROM_02_05_FRACTION * c02_05 + c05_10 + c10_25 + c25_50 + c50_100;
      pm003_dL = (int)lround(pm003_L / 10.0); // per dL
      Serial.printf("Counts 10s (Nb/L): 0.2-0.5=%.0f 0.5-1=%.0f 1-2.5=%.0f 2.5-5=%.0f 5-10=%.0f  => PM0.3≈ %d /dL\n",
                    c02_05, c05_10, c10_25, c25_50, c50_100, pm003_dL);
    } else {
      Serial.println("NextPM bins: lecture échouée");
    }

    // 3) CO2 from S8
    uint16_t co2 = 0; bool okCO2 = s8ReadCO2(co2);
    if (okCO2) Serial.printf("CO2=%u ppm\n", co2); else Serial.println("S8: lecture échouée");

    // 4) POST to cloud (only if at least one sensor has a valid value)
    if (okMass || okBins || okCO2) {
      postToAirGradient(pm1, pm25, pm10, pm003_dL, okCO2 ? (int)co2 : -1);
    } else {
      Serial.println("[POST] Aucun capteur n’a de valeur valide pour l’instant.");
    }
  }
}
